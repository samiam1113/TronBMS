// ============================================================================
// bms_fsm.cpp — BMS Finite State Machine Implementation
//
// State transition summary:
//
//   INIT
//     entry : ltc_wakeup_sleep(), ads_reset()
//     do    : configureADS131M02() + ID verify
//     exit  : → STARTUP (all OK) | → FAULT (comms failure)
//
//   STARTUP
//     entry : ltc_wakeup_idle()
//     do    : ltc_comms_test(), ads_checkid(), gate_driver_selftest(),
//             ltc_write_config() (REFON=1, DCC=0)
//     exit  : → NORMAL (all tests pass) | → FAULT
//
//   NORMAL
//     entry : contactors closed (if no faults), isoSPI active
//     do    : protection checks, idle timeout, charge/balance condition
//     exit  : → CHARGING (current < -0.5A) | → SLEEP | → FAULT
//
//   CHARGING
//     entry : close CHG gate, reset idle timer
//     do    : selective per-cell balancing while charging continues
//             - Balance cell if cell >= 3.8V AND cell > (min + 25mV)
//             - Open gate if any cell hits 4.1V and others >25mV behind,
//               drain to within 3mV of lowest, close gate and continue
//             - Emergency: spread >= 75mV → open gate, drain, resume
//             - Done: all cells within 3mV of 4.1V → open gate, → SLEEP
//     exit  : → SLEEP (charge done or charger removed) | → FAULT
//
//   BALANCE
//     entry : notify task_balance
//     do    : recompute + apply balance mask each tick, protection checks
//     exit  : → NORMAL (balanced) | → FAULT
//
//   SLEEP
//     entry : balance_stop(), ltc_wakeup_sleep(), open contactors
//     do    : poll ADS current for wakeup condition
//     exit  : → INIT (current detected)
//
//   FAULT
//     entry : open contactors, balance_stop(), snapshot + log fault context
//     do    : poll fault_reg for clearance
//     exit  : → INIT (all faults cleared)
//
// ============================================================================

#include "bms_fsm.h"
#include "ltc_spi.h"
#include "bms_hardware.h"
#include "bms_measurements.h"
#include "bms_balance.h"
#include "bms_fault.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <Arduino.h>

// ── Charging thresholds ───────────────────────────────────────────────────────
static constexpr float CHG_CELL_TARGET_V       = 4.100f;  // Full charge target per cell
static constexpr float CHG_DONE_WINDOW_V       = 0.003f;  // All within 3mV of 4.1V = done
static constexpr float CHG_BAL_START_V         = 3.800f;  // Min cell voltage to start balancing
static constexpr float CHG_BAL_DELTA_V         = 0.025f;  // 25mV above lowest triggers balance
static constexpr float CHG_EMERGENCY_DELTA_V   = 0.075f;  // 75mV spread → emergency gate open
static constexpr float CHG_DRAIN_DONE_V        = 0.003f;  // Drain until within 3mV of lowest
static constexpr float CHG_GATE_OPEN_DELTA_V   = 0.025f;  // Gate open threshold when cell at 4.1V

// ── Pack SoC calculation ──────────────────────────────────────────────────────
// 82V pack = 100% (20S × 4.1V)
static constexpr float PACK_FULL_V             = 82.0f;

// Forward declarations — defined in bms_tasks.cpp
extern SemaphoreHandle_t g_meas_mutex;
extern SemaphoreHandle_t g_snapshot_mutex;
extern measurement_data_t g_meas;
extern fault_snapshot_t  g_fault_snapshot;
extern volatile bool     g_fault_snapshot_pending;
extern TaskHandle_t      g_balance_task_handle;

static void fsm_on_enter(BmsFsm &fsm, BmsState new_state);

// ============================================================================
// Internal helpers
// ============================================================================

static void contactor_open_chg(BmsFsm &fsm) {
    digitalWrite(GATE_CHG_PIN, LOW);
    fsm.chg_open = true;
}

static void contactor_open_dsg(BmsFsm &fsm) {
    digitalWrite(GATE_DSCHG_PIN, LOW);
    fsm.dsg_open = true;
}

static void contactor_close_chg(BmsFsm &fsm) {
    if (fsm.fault_reg & FAULT_BLOCKS_CHG) return;
    digitalWrite(GATE_CHG_PIN, HIGH);
    fsm.chg_open = false;
}

static void contactor_close_dsg(BmsFsm &fsm) {
    if (fsm.fault_reg & FAULT_BLOCKS_DSG) return;
    digitalWrite(GATE_DSCHG_PIN, HIGH);
    fsm.dsg_open = false;
}

static void contactors_update(BmsFsm &fsm) {
    (fsm.fault_reg & FAULT_BLOCKS_CHG) ? contactor_open_chg(fsm)
                                       : contactor_close_chg(fsm);
    (fsm.fault_reg & FAULT_BLOCKS_DSG) ? contactor_open_dsg(fsm)
                                       : contactor_close_dsg(fsm);
}

static void fsm_fault_set(BmsFsm &fsm, Fault bit) {
    fault_set(static_cast<uint16_t>(bit));
    fsm.fault_reg = fault_get();
    contactors_update(fsm);
}

static bool state_timeout(const BmsFsm &fsm, uint32_t ms) {
    return (millis() - fsm.state_entry_ms) >= ms;
}

// Scan meas and return min/max cell voltages
static void pack_min_max(const measurement_data_t &meas, float &vmin, float &vmax) {
    vmin = meas.cell_v[0][0];
    vmax = meas.cell_v[0][0];
    for (int ic = 0; ic < TOTAL_IC; ic++) {
        for (int c = 0; c < CELLS_PER_IC; c++) {
            float v = meas.cell_v[ic][c];
            if (v < vmin) vmin = v;
            if (v > vmax) vmax = v;
        }
    }
}

// Sum all cell voltages for SoC calculation
static float pack_sum_voltage(const measurement_data_t &meas) {
    float sum = 0.0f;
    for (int ic = 0; ic < TOTAL_IC; ic++)
        for (int c = 0; c < CELLS_PER_IC; c++)
            sum += meas.cell_v[ic][c];
    return sum;
}

static float pack_soc_percent(const measurement_data_t &meas) {
    float soc = (pack_sum_voltage(meas) / PACK_FULL_V) * 100.0f;
    if (soc > 100.0f) soc = 100.0f;
    if (soc <   0.0f) soc =   0.0f;
    return soc;
}

static bool cells_need_charge_balance(const measurement_data_t &meas) {
    float vmin, vmax;
    pack_min_max(meas, vmin, vmax);
    return (vmax - vmin) >= CHG_BAL_DELTA_V;
}

// ============================================================================
// Charging state helpers
// ============================================================================

// Apply a selective balance mask: only flag cells that are >= CHG_BAL_START_V
// AND more than CHG_BAL_DELTA_V above the pack minimum.
// Does NOT flag all cells — only the specific high ones.
static void charging_balance_compute_selective(measurement_data_t *meas) {
    float vmin, vmax;
    pack_min_max(*meas, vmin, vmax);

    for (int ic = 0; ic < TOTAL_IC; ic++) {
        for (int c = 0; c < CELLS_PER_IC; c++) {
            float v = meas->cell_v[ic][c];
            // Balance this cell if it is at least CHG_BAL_START_V (3.8V)
            // and more than CHG_BAL_DELTA_V (25mV) above the pack minimum
            bool bal = (v >= CHG_BAL_START_V) && (v > (vmin + CHG_BAL_DELTA_V));
            meas->balance_cells[ic][c] = bal;
        }
    }
    (void)vmax;
}

// Apply an emergency drain mask: flag any cell that is more than
// CHG_DRAIN_DONE_V (3mV) above the pack minimum.
static void charging_emergency_mask(measurement_data_t *meas) {
    float vmin, vmax;
    pack_min_max(*meas, vmin, vmax);
    for (int ic = 0; ic < TOTAL_IC; ic++)
        for (int c = 0; c < CELLS_PER_IC; c++)
            meas->balance_cells[ic][c] = (meas->cell_v[ic][c] > (vmin + CHG_DRAIN_DONE_V));
    (void)vmax;
}

// Returns true when the emergency drain is complete:
// all cells within CHG_DRAIN_DONE_V of the minimum.
static bool charging_drain_satisfied(const measurement_data_t &meas) {
    float vmin, vmax;
    pack_min_max(meas, vmin, vmax);
    return (vmax - vmin) <= CHG_DRAIN_DONE_V;
}

// Returns true when charging is complete:
// all cells within CHG_DONE_WINDOW_V of CHG_CELL_TARGET_V.
static bool charging_done(const measurement_data_t &meas) {
    for (int ic = 0; ic < TOTAL_IC; ic++)
        for (int c = 0; c < CELLS_PER_IC; c++)
            if (meas.cell_v[ic][c] < (CHG_CELL_TARGET_V - CHG_DONE_WINDOW_V))
                return false;
    return true;
}

// Returns true if any cell has reached CHG_CELL_TARGET_V while other cells
// are still more than CHG_GATE_OPEN_DELTA_V behind.
static bool charging_needs_gate_open_balance(const measurement_data_t &meas) {
    float vmin, vmax;
    pack_min_max(meas, vmin, vmax);
    bool any_at_target = (vmax >= CHG_CELL_TARGET_V);
    bool others_behind = ((vmax - vmin) > CHG_GATE_OPEN_DELTA_V);
    return any_at_target && others_behind;
}

// ============================================================================
// State handlers
// ============================================================================

// ----------------------------------------------------------------------------
// INIT — wake hardware, verify ADS131 comms
// ----------------------------------------------------------------------------
static void state_init(BmsFsm &fsm) {
    ltc_wakeup_sleep();

    ads_reset();
vTaskDelay(pdMS_TO_TICKS(10));  // let ADS settle after reset

    fsm_set_state(fsm, BmsState::STARTUP);
}

// ----------------------------------------------------------------------------
// STARTUP — full self-test sequence
// ----------------------------------------------------------------------------
static void state_startup(BmsFsm &fsm) {
    if (!ltc_comms_test()) {
        fsm_fault_set(fsm, Fault::SPI);
        fsm_set_state(fsm, BmsState::FAULT);
        return;
    }

    if (!gate_driver_selftest()) {
        fsm_fault_set(fsm, Fault::GATE);
        fsm_set_state(fsm, BmsState::FAULT);
        return;
    }

    LtcConfig cfg[TOTAL_IC] = {};
    for (int ic = 0; ic < TOTAL_IC; ic++) {
        cfg[ic].refon  = true;
        cfg[ic].adcopt = false;
        cfg[ic].dcto   = 0x00;
        cfg[ic].dcc    = 0x0000;
        cfg[ic].vuv    = (uint16_t)((CELL_UV_RAW / 16u) - 1u);
        cfg[ic].vov    = (uint16_t) (CELL_OV_RAW / 16u);
        for (int g = 0; g < 5; g++) cfg[ic].gpio_pulldown[g] = false;
    }
    ltc_write_config(cfg);

    fsm_set_state(fsm, BmsState::NORMAL);
}

// ----------------------------------------------------------------------------
// NORMAL — monitor pack, react to faults, decide charge/balance/sleep
// ----------------------------------------------------------------------------
static void state_normal(BmsFsm &fsm) {
    const measurement_data_t &meas = g_meas;

    contactors_update(fsm);

    if (fsm.fault_reg != 0) {
        fsm_set_state(fsm, BmsState::FAULT);
        return;
    }

    if ((millis() - fsm.last_activity_ms) >= SLEEP_IDLE_TIMEOUT_MS) {
        fsm_set_state(fsm, BmsState::SLEEP);
        return;
    }

    if (!balance_satisfied(&meas)) {
        fsm_set_state(fsm, BmsState::BALANCE);
    }
}

// ----------------------------------------------------------------------------
// CHARGING — monitor charging, selective per-cell balance, detect completion
//
// Sub-state machine inside CHARGING (tracked via fsm.chg_open):
//   chg_open=false : normal charging, selective balance active
//   chg_open=true  : gate-open drain phase (4.1V cell or emergency)
// ----------------------------------------------------------------------------
static void state_charging(BmsFsm &fsm) {
    measurement_data_t meas_snap;
    if (xSemaphoreTake(g_meas_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        meas_snap = g_meas;
        xSemaphoreGive(g_meas_mutex);
    } else {
        return;
    }

    // Print SoC periodically (every ~5 seconds at 100ms tick rate)
    static uint32_t s_soc_print_ms = 0;
    if ((millis() - s_soc_print_ms) >= 5000) {
        float soc = pack_soc_percent(meas_snap);
        float vmin, vmax;
        pack_min_max(meas_snap, vmin, vmax);
        Serial.printf("[chg] SoC: %.1f%%  min: %.4fV  max: %.4fV  delta: %.1fmV  I: %.2fA\n",
                      soc, vmin, vmax, (vmax - vmin) * 1000.0f, meas_snap.current_a);
        s_soc_print_ms = millis();
    }

    // ── Fault check ───────────────────────────────────────────────────────────
    if (fsm.fault_reg != 0) {
        balance_stop(&meas_snap);
        if (xSemaphoreTake(g_meas_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            memcpy(g_meas.balance_cells, meas_snap.balance_cells,
                   sizeof(g_meas.balance_cells));
            xSemaphoreGive(g_meas_mutex);
        }
        fsm_set_state(fsm, BmsState::FAULT);
        return;
    }

    float vmin, vmax;
    pack_min_max(meas_snap, vmin, vmax);
    float spread = vmax - vmin;

    // ── Gate-open drain phase ─────────────────────────────────────────────────
    // Either a 4.1V cell was detected with others behind, or emergency spread.
    // Drain until all within 3mV of minimum, then re-close gate.
    if (fsm.chg_open) {
        // Compute drain mask and apply
        charging_emergency_mask(&meas_snap);
        if (xSemaphoreTake(g_meas_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            memcpy(g_meas.balance_cells, meas_snap.balance_cells,
                   sizeof(g_meas.balance_cells));
            xSemaphoreGive(g_meas_mutex);
        }
        balance_apply(&meas_snap);

        if (charging_drain_satisfied(meas_snap)) {
            // Drain complete
            balance_stop(&meas_snap);
            if (xSemaphoreTake(g_meas_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                memcpy(g_meas.balance_cells, meas_snap.balance_cells,
                       sizeof(g_meas.balance_cells));
                xSemaphoreGive(g_meas_mutex);
            }

            // Check if charging is fully done
            if (charging_done(meas_snap)) {
                Serial.println("[chg] Charge complete — all cells within 3mV of 4.1V.");
                contactor_open_chg(fsm);
                fsm_set_state(fsm, BmsState::SLEEP);
                return;
            }

            // Not done — re-close charge gate and continue
            Serial.println("[chg] Drain complete — resuming charge.");
            contactor_close_chg(fsm);
        }
        return;
    }

    // ── Normal charging phase ─────────────────────────────────────────────────

    // Check if charger has been removed
    const float amps = meas_snap.current_a;
    if (amps > 999.0f) {
        Serial.println("[chg] Charger removed — transitioning to SLEEP.");
        balance_stop(&meas_snap);
        if (xSemaphoreTake(g_meas_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            memcpy(g_meas.balance_cells, meas_snap.balance_cells,
                   sizeof(g_meas.balance_cells));
            xSemaphoreGive(g_meas_mutex);
        }
        fsm_set_state(fsm, BmsState::SLEEP);
        return;
    }

    // Emergency: spread >= 75mV → open gate immediately and drain
    if (spread >= CHG_EMERGENCY_DELTA_V) {
        Serial.printf("[chg] EMERGENCY: spread %.1fmV >= 75mV — opening charge gate.\n",
                      spread * 1000.0f);
        contactor_open_chg(fsm);
        // fsm.chg_open is now true — drain phase runs next tick
        return;
    }

    // 4.1V cell detected with others >25mV behind → open gate to drain
    if (charging_needs_gate_open_balance(meas_snap)) {
        Serial.printf("[chg] Cell at 4.1V with others >25mV behind (spread %.1fmV) — opening gate.\n",
                      spread * 1000.0f);
        contactor_open_chg(fsm);
        return;
    }

    // Charge done: all within 3mV of 4.1V
    if (charging_done(meas_snap)) {
        Serial.println("[chg] Charge complete — all cells within 3mV of 4.1V.");
        balance_stop(&meas_snap);
        if (xSemaphoreTake(g_meas_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            memcpy(g_meas.balance_cells, meas_snap.balance_cells,
                   sizeof(g_meas.balance_cells));
            xSemaphoreGive(g_meas_mutex);
        }
        contactor_open_chg(fsm);
        fsm_set_state(fsm, BmsState::SLEEP);
        return;
    }

    // Selective balance while charging continues
    // Only flag cells >= 3.8V that are >25mV above the pack minimum
    charging_balance_compute_selective(&meas_snap);

    // Write balance mask back to g_meas so task_measure sees it
    if (xSemaphoreTake(g_meas_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(g_meas.balance_cells, meas_snap.balance_cells,
               sizeof(g_meas.balance_cells));
        xSemaphoreGive(g_meas_mutex);
    }

    // Apply DCC bits — charge gate stays closed, balance runs simultaneously
    balance_apply(&meas_snap);
}

// ----------------------------------------------------------------------------
// BALANCE — active cell balancing from NORMAL state, protection checks
// ----------------------------------------------------------------------------
static void state_balance(BmsFsm &fsm) {
    measurement_data_t meas_snap;
    if (xSemaphoreTake(g_meas_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        meas_snap = g_meas;
        xSemaphoreGive(g_meas_mutex);
    } else {
        return;
    }

    uint8_t bal_ot_ch = 0;
    meas_check_balance_overtemp(&meas_snap, &bal_ot_ch);

    contactors_update(fsm);

    if (fsm.fault_reg != 0) {
        fsm_set_state(fsm, BmsState::FAULT);
        return;
    }
    // Completion and fault exit handled by task_fsm event group processing
}

// ----------------------------------------------------------------------------
// SLEEP — LTC asleep, poll ADS131 current for wakeup
// ----------------------------------------------------------------------------
static void state_sleep(BmsFsm &fsm) {
    if ((millis() - fsm.state_entry_ms) < 3000) return;

    const float amps = ads_read_current();
    if (amps > 1.0f || amps < -1.0f) {
        fsm.last_activity_ms = millis();
        fsm_set_state(fsm, BmsState::INIT);
    }
}

// ----------------------------------------------------------------------------
// FAULT — safe state, contactors open, wait for fault register to clear
// ----------------------------------------------------------------------------
static void state_fault(BmsFsm &fsm) {
    contactor_open_chg(fsm);
    contactor_open_dsg(fsm);
    fsm.fault_reg = fault_get();
    if (fsm.fault_reg == 0) {
        fsm_set_state(fsm, BmsState::INIT);
    }
}

static void state_drive(BmsFsm &fsm) {
    measurement_data_t meas_snap;
    if (xSemaphoreTake(g_meas_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        meas_snap = g_meas;
        xSemaphoreGive(g_meas_mutex);
    } else return;

    contactors_update(fsm);

    if (fsm.fault_reg != 0) {
        fsm_set_state(fsm, BmsState::FAULT);
        return;
    }

    // Return to NORMAL when no discharge current
    if (meas_snap.current_a < 0.5f) {
        fsm_set_state(fsm, BmsState::NORMAL);
        return;
    }

    // Update activity timestamp
    fsm.last_activity_ms = millis();

    // Periodic status
    static uint32_t s_drive_print_ms = 0;
    if ((millis() - s_drive_print_ms) >= 5000) {
        s_drive_print_ms = millis();
        float vmin, vmax;
        float vsum = 0.0f;
        vmin = vmax = meas_snap.cell_v[0][0];
        for (int ic = 0; ic < TOTAL_IC; ic++)
            for (int c = 0; c < CELLS_PER_IC; c++) {
                float v = meas_snap.cell_v[ic][c];
                if (v < vmin) vmin = v;
                if (v > vmax) vmax = v;
                vsum += v;
            }
        float soc = constrain((vsum / 82.0f) * 100.0f, 0.0f, 100.0f);
        Serial.printf("[drive] SoC=%5.1f%%  pack=%.3fV  min=%.4fV  max=%.4fV  delta=%.1fmV  I=%.2fA\n",
                      soc, vsum, vmin, vmax, (vmax-vmin)*1000.0f, meas_snap.current_a);
    }
}

// ============================================================================
// State transition — entry actions run exactly once per state change
// ============================================================================
static void fsm_on_enter(BmsFsm &fsm, BmsState new_state) {
    fsm.prev_state     = fsm.state;
    fsm.state          = new_state;
    fsm.state_entry_ms = millis();

    switch (new_state) {

        case BmsState::INIT:
            break;

        case BmsState::STARTUP:
            ltc_wakeup_idle();
            break;

        case BmsState::NORMAL:
            fsm.last_activity_ms = millis();
            contactors_update(fsm);
            break;

        case BmsState::CHARGING:
            fsm.from_charging    = false;
            fsm.last_activity_ms = millis();
            contactor_close_chg(fsm);
            contactors_update(fsm);
            Serial.println("[chg] Entering CHARGING state.");
            break;

        case BmsState::BALANCE:
            if (g_balance_task_handle) {
                xTaskNotifyGive(g_balance_task_handle);
            }
            break;

        case BmsState::SLEEP:
            balance_stop(&g_meas);
            ltc_wakeup_sleep();
            contactor_open_chg(fsm);
            contactor_open_dsg(fsm);
            Serial.println("[fsm] Entering SLEEP.");
            break;

        case BmsState::FAULT:
            contactor_open_chg(fsm);
            contactor_open_dsg(fsm);
            balance_stop(nullptr);
            if (xSemaphoreTake(g_snapshot_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                fault_capture_context(&g_fault_snapshot, fsm.prev_state);
                fault_log_write(&g_fault_snapshot);
                g_fault_snapshot_pending = true;
                xSemaphoreGive(g_snapshot_mutex);
            } else {
                fault_capture_context(&g_fault_snapshot, fsm.prev_state);
                fault_log_write(&g_fault_snapshot);
            }
            Serial.printf("[fsm] FAULT entered from state %d  fault_reg=0x%04X\n",
                          static_cast<uint8_t>(fsm.prev_state), fsm.fault_reg);
            break;
        case BmsState::DRIVE:
            contactor_close_dsg(fsm);
            contactors_update(fsm);
            fsm.last_activity_ms = millis();
            Serial.println("[fsm] Entering DRIVE state — DSG gate closed.");
            break;
    }
}

// ============================================================================
// Public API
// ============================================================================

void fsm_init(BmsFsm &fsm) {
    fsm = BmsFsm{};
    fsm_on_enter(fsm, BmsState::INIT);
}

void fsm_set_state(BmsFsm &fsm, BmsState new_state) {
    fsm_on_enter(fsm, new_state);
}

BmsState fsm_get_state(const BmsFsm &fsm) {
    return fsm.state;
}

void fsm_run(BmsFsm &fsm) {
    switch (fsm.state) {
        case BmsState::INIT:     state_init(fsm);     break;
        case BmsState::STARTUP:  state_startup(fsm);  break;
        case BmsState::NORMAL:   state_normal(fsm);   break;
        case BmsState::CHARGING: state_charging(fsm); break;
        case BmsState::BALANCE:  state_balance(fsm);  break;
        case BmsState::SLEEP:    state_sleep(fsm);    break;
        case BmsState::FAULT:    state_fault(fsm);    break;
        case BmsState::DRIVE:    state_drive(fsm);    break;
    }
}