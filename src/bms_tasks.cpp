// ============================================================================
// bms_tasks.cpp — FreeRTOS task implementations
//
// Task priority map (higher number = higher priority on ESP32 FreeRTOS):
//   task_watchdog  : 5   — must preempt all others to feed WDT
//   task_fsm       : 4   — drives state machine
//   task_measure   : 3   — time-critical ADC reads
//   task_balance   : 2   — periodic DCC refresh
//   task_daq       : 1   — non-critical CAN transmit
// ============================================================================

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_task_wdt.h"
#include "bms_config.h"
#include "bms_types.h"
#include "bms_fault.h"
#include "bms_fsm.h"
#include "ltc_spi.h"
#include "bms_hardware.h"
#include "bms_measurements.h"
#include "bms_balance.h"
#include "bms_telemetry.h"
#include "bms_tasks.h"
#include "bms_wifi.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>


// ── Shared RTOS primitives ────────────────────────────────────────────────────
EventGroupHandle_t  g_event_group         = nullptr;
SemaphoreHandle_t   g_meas_mutex          = nullptr;
QueueHandle_t       g_meas_queue          = nullptr;
TaskHandle_t        g_balance_task_handle = nullptr;

// ── Fault snapshot ────────────────────────────────────────────────────────────
// Fix #4: g_snapshot_mutex guards both g_fault_snapshot (large struct) and
// g_fault_snapshot_pending. volatile on the flag alone is insufficient —
// task_daq can read a partially-written struct if the flag becomes visible
// before the struct write completes. Always hold the mutex when touching either.
fault_snapshot_t     g_fault_snapshot         = {};
volatile bool        g_fault_snapshot_pending = false;
SemaphoreHandle_t    g_snapshot_mutex         = nullptr;

// ── External FSM context (owned in main.cpp) ──────────────────────────────────
extern BmsFsm g_fsm;

// ── Watchdog checkin timestamps ───────────────────────────────────────────────
static volatile uint32_t s_wdt_checkin_measure = 0;
static volatile uint32_t s_wdt_checkin_balance = 0;
static volatile uint32_t s_wdt_checkin_daq     = 0;

#define DAQ_TIMEOUT_MS  (MEASURE_PERIOD_MS * 5)

// ============================================================================
// task_fsm
// ============================================================================
void task_fsm(void *pvParameters) {

    for (;;) {
        // Fix #1: process event bits BEFORE fsm_run() so faults set by
        // task_measure during the previous tick are reflected in g_faultRegister
        // and fsm.fault_reg before the FSM executes this tick's state handler.
        EventBits_t bits = xEventGroupGetBits(g_event_group);

        if (bits & EVT_FAULT_ANY) {
            if (bits & EVT_FAULT_OV)         fault_set(static_cast<uint16_t>(Fault::CELL_OV));
            if (bits & EVT_FAULT_UV)         fault_set(static_cast<uint16_t>(Fault::CELL_UV));
            if (bits & EVT_FAULT_OC)         fault_set(static_cast<uint16_t>(Fault::OC_DSG));
            if (bits & EVT_FAULT_OT)         fault_set(static_cast<uint16_t>(Fault::OT));
            if (bits & EVT_FAULT_LTC)        fault_set(static_cast<uint16_t>(Fault::SPI));
            if (bits & EVT_FAULT_ADS)        fault_set(static_cast<uint16_t>(Fault::ADS_ID));  // Fix #3
            if (bits & EVT_FAULT_BAL_OT)     fault_set(static_cast<uint16_t>(Fault::OT));
            if (bits & EVT_FAULT_TASK_STALL) fault_set(static_cast<uint16_t>(Fault::TASK_STALL));

            xEventGroupClearBits(g_event_group, EVT_FAULT_ANY);
            fsm_set_state(g_fsm, BmsState::FAULT);
        }

        if (bits & EVT_BALANCE_DONE) {
            xEventGroupClearBits(g_event_group, EVT_BALANCE_DONE);
            fsm_set_state(g_fsm, BmsState::NORMAL);
        }

        fsm_run(g_fsm);

        xEventGroupSetBits(g_event_group, EVT_MEASURE_START);
        vTaskDelay(pdMS_TO_TICKS(MEASURE_PERIOD_MS));
    }
}

// ============================================================================
// task_measure
// ============================================================================
void task_measure(void *pvParameters) {
    for (;;) {
        xEventGroupWaitBits(
            g_event_group,
            EVT_MEASURE_START,
            pdTRUE, pdFALSE,
            portMAX_DELAY
        );

        ltc_wakeup_idle();

        // Take a local snapshot of g_meas to work with, then write back
        measurement_data_t meas = {};

        // Copy current balance mask in so ltc_read_voltages can apply it
        if (xSemaphoreTake(g_meas_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            memcpy(meas.balance_cells, g_meas.balance_cells,
                   sizeof(meas.balance_cells));
            xSemaphoreGive(g_meas_mutex);
        }

        // Read all measurements into local meas struct
        if (!meas_cell_data(&meas)) {
            // meas_cell_data sets the SPI fault bit internally
            xEventGroupSetBits(g_event_group, EVT_FAULT_LTC);
            continue;
        }

        // Publish to g_meas and the DAQ queue under mutex
        if (xSemaphoreTake(g_meas_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_meas = meas;
            xSemaphoreGive(g_meas_mutex);
        }

        if (xQueueSend(g_meas_queue, &meas, 0) == errQUEUE_FULL) {
            measurement_data_t discard;
            xQueueReceive(g_meas_queue, &discard, 0);
            xQueueSend(g_meas_queue, &meas, 0);
        }

        // ── Protection checks ─────────────────────────────────────────────────
        if (meas_check_overvoltage(&meas))
            xEventGroupSetBits(g_event_group, EVT_FAULT_OV);

        if (meas_check_undervoltage(&meas))
            xEventGroupSetBits(g_event_group, EVT_FAULT_UV);

        if (meas_check_overcurrent(&meas))
            xEventGroupSetBits(g_event_group, EVT_FAULT_OC);

        uint8_t ot_ch = 0;
        if (meas_check_overtemp(&meas, &ot_ch))
            xEventGroupSetBits(g_event_group, EVT_FAULT_OT);

        // ── Sleep idle timeout — reset activity timestamp on any real current ─
        // 0.5 A deadband filters ADC noise so a resting pack doesn't stay awake.
        // g_fsm.last_activity_ms is read by state_normal() to gate sleep entry.
        if (meas.current_a > 0.5f || meas.current_a < -0.5f) {
            g_fsm.last_activity_ms = millis();
        }

        s_wdt_checkin_measure = millis();
    }
}

// ============================================================================
// task_balance
// ============================================================================
void task_balance(void *pvParameters) {
    for (;;) {
        // Fix #2: block here with ulTaskNotifyTake so the task re-arms itself
        // for each BALANCE entry without ever calling vTaskSuspend.
        // vTaskSuspend + xTaskNotifyGive does NOT resume a suspended task —
        // the notification value increments but the task stays suspended,
        // silently killing balancing on every re-entry after the first.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Fix #6: reset checkin timestamp on each BALANCE entry so the watchdog
        // does not see a stale timestamp from the previous balance session and
        // immediately fire EVT_FAULT_TASK_STALL before the first iteration runs.
        s_wdt_checkin_balance = millis();

        while (fsm_get_state(g_fsm) == BmsState::BALANCE) {
            // Checkin at top of loop so every iteration — including the final
            // one that calls break — is counted.
            s_wdt_checkin_balance = millis();

            measurement_data_t meas_snap = {};
            if (xSemaphoreTake(g_meas_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                meas_snap = g_meas;
                xSemaphoreGive(g_meas_mutex);
            }

            // Compute mask into meas_snap, then write back balance_cells to g_meas
            balance_compute_mask(&meas_snap);

            if (xSemaphoreTake(g_meas_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                memcpy(g_meas.balance_cells, meas_snap.balance_cells,
                       sizeof(g_meas.balance_cells));
                xSemaphoreGive(g_meas_mutex);
            }

            if (balance_apply(&meas_snap) != LTC_OK) {
                xEventGroupSetBits(g_event_group, EVT_FAULT_LTC);
                balance_stop(&meas_snap);
                break;
            }

            uint8_t ot_ch = 0;
            if (meas_check_balance_overtemp(&meas_snap, &ot_ch)) {
                balance_stop(&meas_snap);
                xEventGroupSetBits(g_event_group, EVT_FAULT_BAL_OT);
                break;
            }

            if (balance_satisfied(&meas_snap)) {
                balance_stop(&meas_snap);
                xEventGroupSetBits(g_event_group, EVT_BALANCE_DONE);
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(BAL_REFRESH_MS));
        }
        // No vTaskSuspend here — task loops back to ulTaskNotifyTake and blocks.
    }
}

// ============================================================================
// task_daq
// ============================================================================
void task_daq(void *pvParameters) {
    telemetry_init();

    for (;;) {
        measurement_data_t meas = {};

        if (xQueueReceive(g_meas_queue, &meas, portMAX_DELAY) == pdTRUE) {

            telemetry_send(&meas, fsm_get_state(g_fsm));

            wifi_push_telemetry(&meas, fsm_get_state(g_fsm), fault_get());

            // Fix #4: take snapshot mutex before checking the flag and reading
            // the struct. The write side (fsm_on_enter FAULT) holds this same
            // mutex across the struct write + flag set, so we can never observe
            // the flag true while the struct is only partially written.
            if (xSemaphoreTake(g_snapshot_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (g_fault_snapshot_pending) {
                    fault_snapshot_t snap_local = g_fault_snapshot;  // copy under lock
                    g_fault_snapshot_pending = false;
                    xSemaphoreGive(g_snapshot_mutex);
                    telemetry_send_fault(&snap_local);               // transmit outside lock
                } else {
                    xSemaphoreGive(g_snapshot_mutex);
                }
            }

            s_wdt_checkin_daq = millis();
        }
    }
}

// ============================================================================
// task_watchdog
// ============================================================================
void task_watchdog(void *pvParameters) {
    // Register this task with the hardware WDT before calling esp_task_wdt_reset().
    // Without esp_task_wdt_add() the reset call is a no-op or causes a panic
    // depending on ESP-IDF WDT configuration.
    esp_task_wdt_add(NULL);

    s_wdt_checkin_measure = millis();
    s_wdt_checkin_balance = millis();
    s_wdt_checkin_daq     = millis();

    for (;;) {
        uint32_t now = millis();

        if ((now - s_wdt_checkin_measure) > (MEASURE_PERIOD_MS * 2)) {
            Serial.println("[wdt] task_measure stall detected");
            xEventGroupSetBits(g_event_group, EVT_FAULT_TASK_STALL);
        }

        if (fsm_get_state(g_fsm) == BmsState::BALANCE) {
            if ((now - s_wdt_checkin_balance) > (BAL_REFRESH_MS * 2)) {
                Serial.println("[wdt] task_balance stall detected");
                xEventGroupSetBits(g_event_group, EVT_FAULT_TASK_STALL);
            }
        }

        if ((now - s_wdt_checkin_daq) > DAQ_TIMEOUT_MS) {
            Serial.println("[wdt] task_daq stall detected");
            xEventGroupSetBits(g_event_group, EVT_FAULT_TASK_STALL);
        }

        if (fsm_get_state(g_fsm) != BmsState::FAULT) {
            esp_task_wdt_reset();
        }

        vTaskDelay(pdMS_TO_TICKS(WDT_FEED_PERIOD_MS));
    }
}
