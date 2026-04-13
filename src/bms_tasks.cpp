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

// ── State name lookup ─────────────────────────────────────────────────────────
static const char *state_name(BmsState s) {
    switch (s) {
        case BmsState::INIT:     return "INIT";
        case BmsState::STARTUP:  return "STARTUP";
        case BmsState::NORMAL:   return "NORMAL";
        case BmsState::CHARGING: return "CHARGING";
        case BmsState::BALANCE:  return "BALANCE";
        case BmsState::SLEEP:    return "SLEEP";
        case BmsState::FAULT:    return "FAULT";
        case BmsState::DRIVE:    return "DRIVE";
        default:                 return "UNKNOWN";
    }
}

static void print_fault_reg(uint16_t reg) {
    if (reg == 0) { Serial.print("NONE"); return; }
    if (reg & static_cast<uint16_t>(Fault::CELL_OV))    Serial.print("CELL_OV ");
    if (reg & static_cast<uint16_t>(Fault::CELL_UV))    Serial.print("CELL_UV ");
    if (reg & static_cast<uint16_t>(Fault::OT))         Serial.print("OT ");
    if (reg & static_cast<uint16_t>(Fault::OC_CHG))     Serial.print("OC_CHG ");
    if (reg & static_cast<uint16_t>(Fault::OC_DSG))     Serial.print("OC_DSG ");
    if (reg & static_cast<uint16_t>(Fault::SPI))        Serial.print("SPI ");
    if (reg & static_cast<uint16_t>(Fault::ADS_ID))     Serial.print("ADS_ID ");
    if (reg & static_cast<uint16_t>(Fault::GATE))       Serial.print("GATE ");
    if (reg & static_cast<uint16_t>(Fault::TASK_STALL)) Serial.print("TASK_STALL ");
}

static void print_pack_summary(const measurement_data_t &meas, BmsState state) {
    Serial.println(); 
    float vmin = meas.cell_v[0][0], vmax = meas.cell_v[0][0], vsum = 0.0f;
    int bal_count = 0;
    int min_ic = 0, min_c = 0, max_ic = 0, max_c = 0;
    for (int ic = 0; ic < TOTAL_IC; ic++)
        for (int c = 0; c < CELLS_PER_IC; c++) {
            float v = meas.cell_v[ic][c];
            if (v < vmin) { vmin = v; min_ic = ic; min_c = c; }
            if (v > vmax) { vmax = v; max_ic = ic; max_c = c; }
            vsum += v;
            if (meas.balance_cells[ic][c]) bal_count++;
        }
    float soc = constrain((vsum / 82.0f) * 100.0f, 0.0f, 100.0f);
    float tmax = -99.0f;
    for (int i = 0; i < NUM_TEMP_SENSORS; i++)
        if (meas.temps[i] > -50.0f && meas.temps[i] > tmax) tmax = meas.temps[i];

    Serial.printf(
        "[pack] %-8s  SoC=%5.1f%%  pack=%6.3fV"
        "  min=IC%d-C%02d:%6.4fV  max=IC%d-C%02d:%6.4fV"
        "  delta=%5.1fmV  I=%+7.3fA  bal=%d\n",
        state_name(state), soc, vsum,
        min_ic + 1, min_c + 1, vmin,
        max_ic + 1, max_c + 1, vmax,
        (vmax - vmin) * 1000.0f, meas.current_a, bal_count);

    Serial.print("[pack] temps: ");
    for (int i = 0; i < NUM_TEMP_SENSORS; i++) {
        if (meas.temps[i] < -50.0f)
            Serial.printf("S%d=OPEN  ", i);
        else
            Serial.printf("S%d=%5.1f°C%s  ", i, meas.temps[i],
                          meas.temps[i] >= TEMP_CUTOFF_C ? "[CUT]" :
                          meas.temps[i] >= TEMP_WARN_C   ? "[WRN]" : "");
    }
    Serial.print("  fault=");
    print_fault_reg(fault_get());
    Serial.println();
}

static void print_cell_detail(const measurement_data_t &meas) {
    Serial.println(); 
    for (int ic = 0; ic < TOTAL_IC; ic++) {
        Serial.printf("[cells] IC%d: ", ic + 1);
        for (int c = 0; c < CELLS_PER_IC; c++)
            Serial.printf("C%02d=%6.4fV%s  ", c + 1, meas.cell_v[ic][c],
                          meas.balance_cells[ic][c] ? "[B]" : "   ");
        Serial.println();
    }
    Serial.print("[temps] ");
    for (int i = 0; i < NUM_TEMP_SENSORS; i++) {
        if (meas.temps[i] < -50.0f) Serial.printf("S%d=OPEN  ", i);
        else Serial.printf("S%d=%5.1f°C%s  ", i, meas.temps[i],
                           meas.temps[i] >= TEMP_CUTOFF_C ? "[CUT]" :
                           meas.temps[i] >= TEMP_WARN_C   ? "[WRN]" : "");
    }
    Serial.println();
}

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
    BmsState s_prev_state = BmsState::INIT;
    uint32_t s_last_detail_ms = 0;
    Serial.println("[fsm] task started.");

    for (;;) {
        // Fix #1: process event bits BEFORE fsm_run() so faults set by
        // task_measure during the previous tick are reflected in g_faultRegister
        // and fsm.fault_reg before the FSM executes this tick's state handler.
        EventBits_t bits = xEventGroupGetBits(g_event_group);

        if (bits & EVT_FAULT_ANY) {
            Serial.println(); 
            Serial.printf("[fsm] FAULT bits=0x%04lX  from state=%s\n",
                          (uint32_t)bits, state_name(fsm_get_state(g_fsm)));
            if (bits & EVT_FAULT_OV)         { fault_set(static_cast<uint16_t>(Fault::CELL_OV));    Serial.println("[fsm]  CELL_OV"); }
            if (bits & EVT_FAULT_UV)         { fault_set(static_cast<uint16_t>(Fault::CELL_UV));    Serial.println("[fsm]  CELL_UV"); }
            if (bits & EVT_FAULT_OC)         { fault_set(static_cast<uint16_t>(Fault::OC_DSG));     Serial.println("[fsm]  OC_DSG"); }
            if (bits & EVT_FAULT_OT)         { fault_set(static_cast<uint16_t>(Fault::OT));         Serial.println("[fsm]  OT"); }
            if (bits & EVT_FAULT_LTC)        { fault_set(static_cast<uint16_t>(Fault::SPI));        Serial.println("[fsm]  SPI"); }
            if (bits & EVT_FAULT_ADS)        { fault_set(static_cast<uint16_t>(Fault::ADS_ID));     Serial.println("[fsm]  ADS_ID"); }
            if (bits & EVT_FAULT_BAL_OT)     { fault_set(static_cast<uint16_t>(Fault::OT));         Serial.println("[fsm]  BAL_OT"); }
            if (bits & EVT_FAULT_TASK_STALL) { fault_set(static_cast<uint16_t>(Fault::TASK_STALL)); Serial.println("[fsm]  TASK_STALL"); }
            Serial.printf("[fsm] fault_reg=0x%04X  active: ", fault_get());
            print_fault_reg(fault_get());
            Serial.println();
            xEventGroupClearBits(g_event_group, EVT_FAULT_ANY);
            fsm_set_state(g_fsm, BmsState::FAULT);
        }

       if (bits & EVT_BALANCE_DONE) {
            Serial.println("[fsm] EVT_BALANCE_DONE — returning to NORMAL.");
            xEventGroupClearBits(g_event_group, EVT_BALANCE_DONE);
            fsm_set_state(g_fsm, BmsState::NORMAL);
        }

        fsm_run(g_fsm);

// State transition log + cell detail on change
        BmsState cur = fsm_get_state(g_fsm);
        if (cur != s_prev_state) {
            Serial.println(); 
            Serial.printf("[fsm] STATE %s → %s  t=%lums\n",
                          state_name(s_prev_state), state_name(cur), millis());
            s_prev_state = cur;
            if (xSemaphoreTake(g_meas_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                measurement_data_t snap = g_meas;
                xSemaphoreGive(g_meas_mutex);
                print_cell_detail(snap);
            }
        }
        // Full cell detail every 10 seconds
        if ((millis() - s_last_detail_ms) >= 5000) {
            s_last_detail_ms = millis();
            if (xSemaphoreTake(g_meas_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                measurement_data_t snap = g_meas;
                xSemaphoreGive(g_meas_mutex);
                print_cell_detail(snap);
            }
        }

        xEventGroupSetBits(g_event_group, EVT_MEASURE_START);
        vTaskDelay(pdMS_TO_TICKS(MEASURE_PERIOD_MS));
    }
}
// ============================================================================
// task_serial — serial command handler
// ============================================================================
void task_serial(void *pvParameters) {
    Serial.println("[serial] Command handler ready. Type ? for help.");
    for (;;) {
        if (Serial.available()) {
            char cmd = Serial.read();
            while (Serial.available()) Serial.read();
            switch (cmd) {
                case 'c': {
                    Serial.println("[serial] CMD: Force CHARGING — closing CHG gate.");
                    fault_clear(static_cast<uint16_t>(Fault::CELL_OV));
                    fault_clear(static_cast<uint16_t>(Fault::CELL_UV));
                    fault_clear(static_cast<uint16_t>(Fault::OC_CHG));
                    fault_clear(static_cast<uint16_t>(Fault::SPI));
                    g_fsm.fault_reg = 0;
                    fsm_set_state(g_fsm, BmsState::CHARGING);
                    break;
                }
                case 'd': {
                    Serial.println("[serial] CMD: Force DRIVE — closing DSG gate.");
                    fault_clear(static_cast<uint16_t>(Fault::CELL_OV));
                    fault_clear(static_cast<uint16_t>(Fault::CELL_UV));
                    fault_clear(static_cast<uint16_t>(Fault::OC_DSG));
                    fault_clear(static_cast<uint16_t>(Fault::SPI));
                    g_fsm.fault_reg = 0;
                    fsm_set_state(g_fsm, BmsState::DRIVE);
                    break;
                }
                case 'a': {
                    Serial.println("[serial] CMD: Force NORMAL — clears faults, auto FSM resumes.");
                    fault_clear(static_cast<uint16_t>(Fault::CELL_OV));
                    fault_clear(static_cast<uint16_t>(Fault::CELL_UV));
                    fault_clear(static_cast<uint16_t>(Fault::OT));
                    fault_clear(static_cast<uint16_t>(Fault::OC_CHG));
                    fault_clear(static_cast<uint16_t>(Fault::OC_DSG));
                    fault_clear(static_cast<uint16_t>(Fault::SPI));
                    fault_clear(static_cast<uint16_t>(Fault::ADS_ID));
                    fault_clear(static_cast<uint16_t>(Fault::GATE));
                    fault_clear(static_cast<uint16_t>(Fault::TASK_STALL));
                    g_fsm.fault_reg = 0;
                    balance_stop(&g_meas);
                    fsm_set_state(g_fsm, BmsState::NORMAL);
                    break;
                }
                case 'n': {
                    Serial.println("[serial] CMD: Force NORMAL.");
                    fsm_set_state(g_fsm, BmsState::NORMAL);
                    break;
                }
                case 'b': {
                    Serial.println("[serial] CMD: Force BALANCE.");
                    fsm_set_state(g_fsm, BmsState::BALANCE);
                    break;
                }
                case 's': {
                    Serial.println("[serial] CMD: Force SLEEP.");
                    balance_stop(&g_meas);
                    fsm_set_state(g_fsm, BmsState::SLEEP);
                    break;
                }
                case 'f': {
                    Serial.println("[serial] CMD: Force FAULT.");
                    fsm_set_state(g_fsm, BmsState::FAULT);
                    break;
                }
                case 'i': {
                    Serial.println("[serial] CMD: Force INIT.");
                    fsm_set_state(g_fsm, BmsState::INIT);
                    break;
                }
                case 'x': {
                    Serial.println("[serial] CMD: Clearing fault register.");
                    fault_clear(static_cast<uint16_t>(Fault::CELL_OV));
                    fault_clear(static_cast<uint16_t>(Fault::CELL_UV));
                    fault_clear(static_cast<uint16_t>(Fault::OT));
                    fault_clear(static_cast<uint16_t>(Fault::OC_CHG));
                    fault_clear(static_cast<uint16_t>(Fault::OC_DSG));
                    fault_clear(static_cast<uint16_t>(Fault::SPI));
                    fault_clear(static_cast<uint16_t>(Fault::ADS_ID));
                    fault_clear(static_cast<uint16_t>(Fault::GATE));
                    fault_clear(static_cast<uint16_t>(Fault::TASK_STALL));
                    g_fsm.fault_reg = 0;
                    Serial.printf("[serial] Fault register cleared: 0x%04X\n", fault_get());
                    break;
                }
                case '?': {
                    Serial.println("[serial] ── Commands ──────────────────────────");
                    Serial.println("[serial]   c = force CHARGING (closes CHG gate)");
                    Serial.println("[serial]   d = force DRIVE (closes DSG gate)");
                    Serial.println("[serial]   a = force NORMAL (clears faults, auto FSM)");
                    Serial.println("[serial]   n = force NORMAL (no fault clear)");
                    Serial.println("[serial]   b = force BALANCE");
                    Serial.println("[serial]   s = force SLEEP");
                    Serial.println("[serial]   f = force FAULT");
                    Serial.println("[serial]   i = force INIT");
                    Serial.println("[serial]   x = clear fault register only");
                    Serial.println("[serial]   ? = show this help");
                    Serial.println("[serial] ─────────────────────────────────────");
                    break;
                }
                default:
                    Serial.printf("[serial] Unknown command '%c' — type ? for help.\n", cmd);
                    break;
            }
        }

        // Stack high-water mark every 10 seconds
        static uint32_t s_hwm_ms = 0;
        if ((millis() - s_hwm_ms) >= 10000) {
            s_hwm_ms = millis();
            Serial.printf("[hwm] serial=%u  fsm=%u  meas=%u  bal=%u  daq=%u  wdt=%u\n",
                uxTaskGetStackHighWaterMark(NULL),
                uxTaskGetStackHighWaterMark(xTaskGetHandle("fsm")),
                uxTaskGetStackHighWaterMark(xTaskGetHandle("meas")),
                uxTaskGetStackHighWaterMark(xTaskGetHandle("bal")),
                uxTaskGetStackHighWaterMark(xTaskGetHandle("daq")),
                uxTaskGetStackHighWaterMark(xTaskGetHandle("wdt")));
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
// ============================================================================
// task_measure
// ============================================================================
void task_measure(void *pvParameters) {
    static uint32_t s_last_print_ms = 0;
    static float    s_last_current  = 0.0f;
    static BmsState s_last_state    = BmsState::INIT;
    Serial.println("[meas] task started.");
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
            Serial.println("[meas] ERROR: SPI read failed.");
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
            Serial.println("[meas] WARN: DAQ queue full — frame dropped.");
        }

        // OV check — print each offending cell
        bool ov = false;
        for (int ic = 0; ic < TOTAL_IC; ic++)
            for (int c = 0; c < CELLS_PER_IC; c++)
                if (meas.cell_v[ic][c] >= CELL_OV_V) {
                    Serial.printf("[meas] FAULT OV: IC%d-C%02d=%.4fV (limit=%.4fV)\n",
                                  ic+1, c+1, meas.cell_v[ic][c], CELL_OV_V);
                    ov = true;
                }
        if (ov) xEventGroupSetBits(g_event_group, EVT_FAULT_OV);

        // UV check — print each offending cell
        bool uv = false;
        for (int ic = 0; ic < TOTAL_IC; ic++)
            for (int c = 0; c < CELLS_PER_IC; c++)
                if (meas.cell_v[ic][c] <= CELL_UV_V) {
                    Serial.printf("[meas] FAULT UV: IC%d-C%02d=%.4fV (limit=%.4fV)\n",
                                  ic+1, c+1, meas.cell_v[ic][c], CELL_UV_V);
                    uv = true;
                }
        if (uv) xEventGroupSetBits(g_event_group, EVT_FAULT_UV);

        // OC check
        if (meas_check_overcurrent(&meas)) {
            Serial.printf("[meas] FAULT OC: I=%.3fA\n", meas.current_a);
            xEventGroupSetBits(g_event_group, EVT_FAULT_OC);
        }

        // OT check
        uint8_t ot_ch = 0;
        if (meas_check_overtemp(&meas, &ot_ch)) {
            Serial.printf("[meas] FAULT OT: sensor=%d  T=%.1f°C (limit=%d°C)\n",
                          ot_ch, meas.temps[ot_ch], TEMP_CUTOFF_C);
            xEventGroupSetBits(g_event_group, EVT_FAULT_OT);
        }

        // ── Sleep idle timeout — reset activity timestamp on any real current ─
        // 0.5 A deadband filters ADC noise so a resting pack doesn't stay awake.
        // g_fsm.last_activity_ms is read by state_normal() to gate sleep entry.
        if (meas.current_a > 0.5f || meas.current_a < -0.5f) {
            g_fsm.last_activity_ms = millis();
        }
        // Current direction change
        if ((s_last_current > -0.5f) && (meas.current_a < -0.5f))
            Serial.printf("[meas] → CHARGING  I=%.3fA\n", meas.current_a);
        if ((s_last_current < -0.5f) && (meas.current_a > -0.5f))
            Serial.printf("[meas] → NOT CHARGING  I=%.3fA\n", meas.current_a);
        if ((s_last_current < 0.5f) && (meas.current_a > 0.5f))
            Serial.printf("[meas] → DISCHARGING  I=%.3fA\n", meas.current_a);
        s_last_current = meas.current_a;

        // Throttled pack summary every 1 second
        BmsState cur_state = fsm_get_state(g_fsm);
        if (cur_state != s_last_state || (millis() - s_last_print_ms) >= 5000) {
            s_last_print_ms = millis();
            s_last_state = cur_state;
            print_pack_summary(meas, cur_state);
        }
        s_wdt_checkin_measure = millis();
    }
}

// ============================================================================
// task_balance
// ============================================================================
void task_balance(void *pvParameters) {
    Serial.println("[bal] task started — waiting for notify.");
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        s_wdt_checkin_balance = millis();

        Serial.println();
        Serial.println("[bal] BALANCE session starting.");
        uint32_t session_start = millis();
        uint32_t iter = 0;

        while (fsm_get_state(g_fsm) == BmsState::BALANCE) {
            s_wdt_checkin_balance = millis();
            iter++;

            measurement_data_t meas_snap = {};
            if (xSemaphoreTake(g_meas_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                meas_snap = g_meas;
                xSemaphoreGive(g_meas_mutex);
            }

            balance_compute_mask(&meas_snap);

            int flagged = 0;
            for (int ic = 0; ic < TOTAL_IC; ic++)
                for (int c = 0; c < CELLS_PER_IC; c++)
                    if (meas_snap.balance_cells[ic][c]) flagged++;

            static uint32_t s_last_bal_print_ms = 0;
            if ((millis() - s_last_bal_print_ms) >= 5000) {
                s_last_bal_print_ms = millis();
                Serial.println();
                Serial.printf("[bal] iter=%lu  flagged=%d  elapsed=%lums\n",
                              iter, flagged, millis() - session_start);
            }

            if (xSemaphoreTake(g_meas_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                memcpy(g_meas.balance_cells, meas_snap.balance_cells,
                       sizeof(g_meas.balance_cells));
                xSemaphoreGive(g_meas_mutex);
            }

            if (balance_apply(&meas_snap) != LTC_OK) {
                Serial.println("[bal] ERROR: balance_apply() SPI fail.");
                xEventGroupSetBits(g_event_group, EVT_FAULT_LTC);
                balance_stop(&meas_snap);
                break;
            }

            uint8_t ot_ch = 0;
            if (meas_check_balance_overtemp(&meas_snap, &ot_ch)) {
                Serial.printf("[bal] OVERTEMP sensor=%d — stopping.\n", ot_ch);
                balance_stop(&meas_snap);
                xEventGroupSetBits(g_event_group, EVT_FAULT_BAL_OT);
                break;
            }

            if (balance_satisfied(&meas_snap)) {
                Serial.printf("[bal] SATISFIED  iters=%lu  duration=%lums\n",
                              iter, millis() - session_start);
                balance_stop(&meas_snap);
                xEventGroupSetBits(g_event_group, EVT_BALANCE_DONE);
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(BAL_REFRESH_MS));
        }

        Serial.println();
        Serial.printf("[bal] Session ended  iters=%lu  duration=%lums\n",
                      iter, millis() - session_start);
    }
}

// ============================================================================
// task_daq
// ============================================================================
void task_daq(void *pvParameters) {
    telemetry_init();
    uint32_t frame_count = 0;
    Serial.println("[daq] task started — CAN active.");

    for (;;) {
        measurement_data_t meas = {};
        if (xQueueReceive(g_meas_queue, &meas, portMAX_DELAY) == pdTRUE) {
            frame_count++;
            BmsState state = fsm_get_state(g_fsm);

            telemetry_send(&meas, state);
            wifi_push_telemetry(&meas, state, fault_get());

            if (frame_count % 50 == 0)
                Serial.printf("[daq] %lu frames  state=%s  fault=0x%04X\n",
                              frame_count, state_name(state), fault_get());

            if (xSemaphoreTake(g_snapshot_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (g_fault_snapshot_pending) {
                    fault_snapshot_t snap_local = g_fault_snapshot;
                    g_fault_snapshot_pending = false;
                    xSemaphoreGive(g_snapshot_mutex);
                    Serial.printf("[daq] Fault snapshot tx: code=%d  state=%s\n",
                                  static_cast<uint8_t>(snap_local.code),
                                  state_name(snap_local.state_at_fault));
                    telemetry_send_fault(&snap_local);
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
            Serial.printf("[wdt] task_measure STALL  last=%lums ago\n",
                          now - s_wdt_checkin_measure);
            xEventGroupSetBits(g_event_group, EVT_FAULT_TASK_STALL);
        }
        if (fsm_get_state(g_fsm) == BmsState::BALANCE) {
            if ((now - s_wdt_checkin_balance) > (BAL_REFRESH_MS * 2)) {
                Serial.printf("[wdt] task_balance STALL  last=%lums ago\n",
                              now - s_wdt_checkin_balance);
                xEventGroupSetBits(g_event_group, EVT_FAULT_TASK_STALL);
            }
        }
        if ((now - s_wdt_checkin_daq) > DAQ_TIMEOUT_MS) {
            Serial.printf("[wdt] task_daq STALL  last=%lums ago\n",
                          now - s_wdt_checkin_daq);
            xEventGroupSetBits(g_event_group, EVT_FAULT_TASK_STALL);
        }
        esp_task_wdt_reset();

        vTaskDelay(pdMS_TO_TICKS(WDT_FEED_PERIOD_MS));
    }
}