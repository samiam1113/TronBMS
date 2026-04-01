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
#include "bms_tasks.h"
#include "bms_fault.h"
#include "bms_fsm.h"
#include "ltc_spi.h"
#include "bms_hardware.h"
#include "bms_measurements.h"
#include "bms_balance.h"
#include "bms_telemetry.h"

// ── Shared RTOS primitives (definitions) ─────────────────────────────────────
EventGroupHandle_t  g_event_group           = nullptr;
SemaphoreHandle_t   g_meas_mutex            = nullptr;
QueueHandle_t       g_meas_queue            = nullptr;
TaskHandle_t        g_balance_task_handle   = nullptr;

// ── Shared data ───────────────────────────────────────────────────────────────
measurement_data_t   g_latest_meas          = {};
fault_snapshot_t     g_fault_snapshot       = {};
volatile bool        g_fault_snapshot_pending = false;

// ── External FSM context (defined/owned in main.cpp) ─────────────────────────
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
    // Initialise RTOS primitives
    g_event_group = xEventGroupCreate();
    g_meas_mutex  = xSemaphoreCreateMutex();
    g_meas_queue  = xQueueCreate(2, sizeof(measurement_data_t));

    // Create task_balance in suspended state
    xTaskCreatePinnedToCore(
        task_balance, "bal",
        4096, nullptr, 2,
        &g_balance_task_handle, 1
    );
    vTaskSuspend(g_balance_task_handle);

    for (;;) {
        // Run one FSM tick for the current state
        fsm_run(g_fsm);

        // ── Check for any fault event ─────────────────────────────────────
        EventBits_t bits = xEventGroupGetBits(g_event_group);
        if (bits & EVT_FAULT_ANY) {
            // Snapshot and log before state transition clears context
            BmsState cur = fsm_get_state(g_fsm);
            fault_capture_context(&g_fault_snapshot,
                                  (bms_state_t)static_cast<uint8_t>(cur));
            g_fault_snapshot_pending = true;
            fault_log_write(&g_fault_snapshot);

            xEventGroupClearBits(g_event_group, EVT_FAULT_ANY);
            fsm_set_state(g_fsm, BmsState::FAULT);
        }

        // ── Check for balance done ────────────────────────────────────────
        if (bits & EVT_BALANCE_DONE) {
            xEventGroupClearBits(g_event_group, EVT_BALANCE_DONE);
            fsm_set_state(g_fsm, BmsState::NORMAL);
        }

        // ── Signal measure task ───────────────────────────────────────────
        xEventGroupSetBits(g_event_group, EVT_MEASURE_START);

        vTaskDelay(pdMS_TO_TICKS(MEASURE_PERIOD_MS));
    }
}

// ============================================================================
// task_measure
// ============================================================================
void task_measure(void *pvParameters) {
    for (;;) {
        // Block until FSM signals a measurement cycle
        xEventGroupWaitBits(
            g_event_group,
            EVT_MEASURE_START,
            pdTRUE,     // clear on exit
            pdFALSE,
            portMAX_DELAY
        );

        ltc_wakeup_idle();

        // Read voltages
        bool volt_ok;
        {
            bool bal_snap[NUM_CELLS];
            if (xSemaphoreTake(g_meas_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                memcpy(bal_snap, g_balanceCells, sizeof(bal_snap));
                xSemaphoreGive(g_meas_mutex);
            } else {
                memset(bal_snap, 0, sizeof(bal_snap));
            }
            volt_ok = ltc_read_voltages(bal_snap);
        }
        if (!volt_ok) {
            xEventGroupSetBits(g_event_group, EVT_FAULT_LTC);
            continue;
        }

        // Read temperatures
        float temps_local[10];
        {
            bool bal_snap[NUM_CELLS];
            if (xSemaphoreTake(g_meas_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                memcpy(bal_snap, g_balanceCells, sizeof(bal_snap));
                xSemaphoreGive(g_meas_mutex);
            } else {
                memset(bal_snap, 0, sizeof(bal_snap));
            }
            if (!ltc_read_temperatures(temps_local, bal_snap)) {
                xEventGroupSetBits(g_event_group, EVT_FAULT_LTC);
                continue;
            }
        }

        // Read current
        float amps = ads_read_current();
        // ADS returns 0.0f on comms error only in a hard-fail scenario;
        // we trust it unless we add a status return in future.

        // ── Assemble measurement struct and publish ───────────────────────
        measurement_data_t meas = {};
        meas.current_a = amps;
        for (int i = 0; i < 4; i++) meas.temps[i] = temps_local[i];
        for (int ic = 0; ic < TOTAL_IC; ic++) {
            for (int c = 0; c < CELLS_PER_IC; c++) {
                meas.cell_v[ic][c] = g_cellV[ic * CELLS_PER_IC + c];
            }
        }

        if (xSemaphoreTake(g_meas_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_latest_meas = meas;
            g_amps        = amps;
            xSemaphoreGive(g_meas_mutex);
        }

        // Post to queue for task_daq (overwrite oldest if full)
        if (xQueueSend(g_meas_queue, &meas, 0) == errQUEUE_FULL) {
            measurement_data_t discard;
            xQueueReceive(g_meas_queue, &discard, 0);
            xQueueSend(g_meas_queue, &meas, 0);
        }

        // ── Protection checks ─────────────────────────────────────────────
        if (meas_check_overvoltage(&meas))
            xEventGroupSetBits(g_event_group, EVT_FAULT_OV);

        if (meas_check_undervoltage(&meas))
            xEventGroupSetBits(g_event_group, EVT_FAULT_UV);

        if (meas_check_overcurrent(&meas))
            xEventGroupSetBits(g_event_group, EVT_FAULT_OC);

        uint8_t ot_ch = 0;
        if (meas_check_overtemp(&meas, &ot_ch))
            xEventGroupSetBits(g_event_group, EVT_FAULT_OT);

        // Checkin
        s_wdt_checkin_measure = millis();
    }
}

// ============================================================================
// task_balance
// ============================================================================
void task_balance(void *pvParameters) {
    for (;;) {
        // Wait for FSM to resume this task (via vTaskResume) when entering
        // STATE_BALANCE. Suspend self when done or on fault.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (fsm_get_state(g_fsm) == BmsState::BALANCE) {

            measurement_data_t meas_snap = {};
            balance_state_t bal = {};

            if (xSemaphoreTake(g_meas_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                meas_snap = g_latest_meas;
                xSemaphoreGive(g_meas_mutex);
            }

            // Compute DCC mask
            balance_compute_mask(&meas_snap, &bal);

            // Apply to hardware
            if (balance_apply(&bal) != LTC_OK) {
                xEventGroupSetBits(g_event_group, EVT_FAULT_LTC);
                balance_stop();
                break;
            }

            // Balance overtemp check
            uint8_t ot_ch = 0;
            if (meas_check_balance_overtemp(&meas_snap, &ot_ch)) {
                balance_stop();
                xEventGroupSetBits(g_event_group, EVT_FAULT_BAL_OT);
                break;
            }

            // Check if balanced
            if (balance_satisfied(&meas_snap)) {
                balance_stop();
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

        // Block until a new measurement is available
        if (xQueueReceive(g_meas_queue, &meas, portMAX_DELAY) == pdTRUE) {

            bms_state_t state = (bms_state_t)static_cast<uint8_t>(
                fsm_get_state(g_fsm)
            );
            telemetry_send(&meas, state);

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

        // task_measure stall check
        if ((now - s_wdt_checkin_measure) > (MEASURE_PERIOD_MS * 2)) {
            Serial.println("[wdt] task_measure stall detected");
            xEventGroupSetBits(g_event_group, EVT_FAULT_TASK_STALL);
        }

        // task_balance stall check — only relevant in BALANCE state
        if (fsm_get_state(g_fsm) == BmsState::BALANCE) {
            if ((now - s_wdt_checkin_balance) > (BAL_REFRESH_MS * 2)) {
                Serial.println("[wdt] task_balance stall detected");
                xEventGroupSetBits(g_event_group, EVT_FAULT_TASK_STALL);
            }
        }

        // task_daq stall check
        if ((now - s_wdt_checkin_daq) > DAQ_TIMEOUT_MS) {
            Serial.println("[wdt] task_daq stall detected");
            xEventGroupSetBits(g_event_group, EVT_FAULT_TASK_STALL);
        }

        // Feed hardware WDT only when NOT in FAULT state
        if (fsm_get_state(g_fsm) != BmsState::FAULT) {
            esp_task_wdt_reset();
        }
        // In FAULT: stop feeding — hardware WDT will reset the system

        vTaskDelay(pdMS_TO_TICKS(WDT_FEED_PERIOD_MS));
    }
}
