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
#include "bms_fault.cpp"
#include "bms_fault.h"
#include "bms_config.h"
#include "bms_fsm.cpp"
#include "ltc_spi.cpp"
#include "bms_hardware.cpp"
#include "bms_measurements.cpp"
#include "bms_balance.cpp"
#include "bms_telemetry.cpp"


// ── Shared RTOS primitives ────────────────────────────────────────────────────
EventGroupHandle_t  g_event_group         = nullptr;
SemaphoreHandle_t   g_meas_mutex          = nullptr;
QueueHandle_t       g_meas_queue          = nullptr;
TaskHandle_t        g_balance_task_handle = nullptr;

// ── Fault snapshot ────────────────────────────────────────────────────────────
fault_snapshot_t     g_fault_snapshot         = {};
volatile bool        g_fault_snapshot_pending = false;

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
    g_event_group = xEventGroupCreate();
    g_meas_mutex  = xSemaphoreCreateMutex();
    g_meas_queue  = xQueueCreate(2, sizeof(measurement_data_t));

    xTaskCreatePinnedToCore(
        task_balance, "bal",
        4096, nullptr, 2,
        &g_balance_task_handle, 1
    );
    vTaskSuspend(g_balance_task_handle);

    for (;;) {
        fsm_run(g_fsm);

        EventBits_t bits = xEventGroupGetBits(g_event_group);

        if (bits & EVT_FAULT_ANY) {
            if (bits & EVT_FAULT_OV)         fault_set(static_cast<uint16_t>(Fault::CELL_OV));
            if (bits & EVT_FAULT_UV)         fault_set(static_cast<uint16_t>(Fault::CELL_UV));
            if (bits & EVT_FAULT_OC)         fault_set(static_cast<uint16_t>(Fault::OC_DSG));
            if (bits & EVT_FAULT_OT)         fault_set(static_cast<uint16_t>(Fault::OT));
            if (bits & EVT_FAULT_LTC)        fault_set(static_cast<uint16_t>(Fault::SPI));
            if (bits & EVT_FAULT_TASK_STALL) fault_set(static_cast<uint16_t>(Fault::SPI));

            xEventGroupClearBits(g_event_group, EVT_FAULT_ANY);
            fsm_set_state(g_fsm, BmsState::FAULT);
        }

        if (bits & EVT_BALANCE_DONE) {
            xEventGroupClearBits(g_event_group, EVT_BALANCE_DONE);
            fsm_set_state(g_fsm, BmsState::NORMAL);
        }

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

        s_wdt_checkin_measure = millis();
    }
}

// ============================================================================
// task_balance
// ============================================================================
void task_balance(void *pvParameters) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (fsm_get_state(g_fsm) == BmsState::BALANCE) {

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

            s_wdt_checkin_balance = millis();
            vTaskDelay(pdMS_TO_TICKS(BAL_REFRESH_MS));
        }

        vTaskSuspend(nullptr);
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

            if (g_fault_snapshot_pending) {
                telemetry_send_fault(&g_fault_snapshot);
                g_fault_snapshot_pending = false;
            }

            s_wdt_checkin_daq = millis();
        }
    }
}

// ============================================================================
// task_watchdog
// ============================================================================
void task_watchdog(void *pvParameters) {
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