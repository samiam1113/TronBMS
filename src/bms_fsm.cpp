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
//     do    : protection checks, idle timeout, balance condition
//     exit  : → BALANCE | → SLEEP | → FAULT
//
//   BALANCE
//     entry : notify task_balance (ADCV now issued inside ltc_read_voltages)
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

// Forward declarations — defined in bms_tasks.cpp
extern SemaphoreHandle_t g_meas_mutex;
extern SemaphoreHandle_t g_snapshot_mutex;   // Fix #4
extern measurement_data_t g_meas;

// ── Fault snapshot globals — defined in bms_tasks.cpp ────────────────────────
extern fault_snapshot_t  g_fault_snapshot;
extern volatile bool     g_fault_snapshot_pending;
extern TaskHandle_t      g_balance_task_handle;

// Forward declaration — defined below fsm_on_enter
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

// FSM-local fault_set — sets bit in fsm.fault_reg and re-evaluates contactors.
// Distinct from the global fault_set() in bms_fault.cpp which sets g_faultRegister.
static void fsm_fault_set(BmsFsm &fsm, Fault bit) {
    fault_set(static_cast<uint16_t>(bit));  // write to g_faultRegister first
    fsm.fault_reg = fault_get();            // then mirror into fsm
    contactors_update(fsm);
}
static bool state_timeout(const BmsFsm &fsm, uint32_t ms) {
    return (millis() - fsm.state_entry_ms) >= ms;
}

static bool cells_need_charge_balance(const measurement_data_t &meas) {
    float vmin = meas.cell_v[0][0];
    float vmax = meas.cell_v[0][0];
    for (int ic = 0; ic < TOTAL_IC; ic++) {
        for (int c = 0; c < CELLS_PER_IC; c++) {
            float v = meas.cell_v[ic][c];
            if (v < vmin) vmin = v;
            if (v > vmax) vmax = v;
        }
    }
    return (vmax - vmin) >= CELL_IMBALANCE_THRESHOLD_V;
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
    if (!ads_configure()) {
        fsm_fault_set(fsm, Fault::ADS_ID);
        fsm_set_state(fsm, BmsState::FAULT);
        return;
    }

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

    if (!ads_checkid()) {
        fsm_fault_set(fsm, Fault::ADS_ID);
        fsm_set_state(fsm, BmsState::FAULT);
        return;
    }

    if (!gate_driver_selftest()) {
        fsm_fault_set(fsm, Fault::GATE);
        fsm_set_state(fsm, BmsState::FAULT);
        return;
    }

    // REFON=1, all DCC bits cleared — both ICs identical
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
// NORMAL — monitor pack, react to faults, decide balance or sleep
// ----------------------------------------------------------------------------
static void state_normal(BmsFsm &fsm) {
    // Take a snapshot of g_meas for this tick's checks
    // (g_meas is written by task_measure; we read it here without a mutex
    //  because individual float reads on Xtensa are atomic and the FSM
    //  runs at lower priority than task_measure — acceptable for protection)
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
    
    const float amps = ads_read_current();
    if (amps < -0.5f) {
    fsm_set_state(fsm, BmsState::CHARGING);
    return;
    }

    if (!balance_satisfied(&meas)) {
        fsm_set_state(fsm, BmsState::BALANCE);
    }

    
}

// ----------------------------------------------------------------------------
// BALANCE — active cell balancing, protection checks every tick
// ----------------------------------------------------------------------------
static void state_balance(BmsFsm &fsm) {
    // FSM only monitors for fault/completion conditions each tick.
    // All DCC mask computation and hardware writes are owned by task_balance.

    measurement_data_t meas_snap;
    if (xSemaphoreTake(g_meas_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        meas_snap = g_meas;
        xSemaphoreGive(g_meas_mutex);
    } else {
        return;
    }

    // Check balance overtemp — sets g_balance_overtemp which task_balance reads
    uint8_t bal_ot_ch = 0;
    meas_check_balance_overtemp(&meas_snap, &bal_ot_ch);

    contactors_update(fsm);

    if (fsm.fault_reg != 0) {
        // Signal task_balance to stop by setting state before it checks
        fsm_set_state(fsm, BmsState::FAULT);
        return;
    }

    if (fsm.from_charging && !cells_need_charge_balance(meas_snap)) {
    fsm.from_charging = false;
    const float amps = ads_read_current();
    // CORRECT — still charging only if current is sufficiently negative
    if (amps < -0.5f) {
        fsm_set_state(fsm, BmsState::CHARGING);
    } else {
        fsm_set_state(fsm, BmsState::SLEEP);
    }
}
    // Completion (EVT_BALANCE_DONE) and fault exit are handled by
    // task_fsm's event group processing — no action needed here
}

// ----------------------------------------------------------------------------
// SLEEP — LTC asleep, poll ADS131 current for wakeup
// ----------------------------------------------------------------------------
static void state_sleep(BmsFsm &fsm) {
    // Wait for LTC reference to settle before polling current
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

// ----------------------------------------------------------------------------
// CHARGING — monitor for charge faults and balance condition
// ----------------------------------------------------------------------------

static void state_charging(BmsFsm &fsm) {
    measurement_data_t meas_snap;
    if (xSemaphoreTake(g_meas_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        meas_snap = g_meas;
        xSemaphoreGive(g_meas_mutex);
    } else {
        return;
    }

    contactors_update(fsm);

    if (fsm.fault_reg != 0) {
        fsm_set_state(fsm, BmsState::FAULT);
        return;
    }

    // Negative current = charging. If it rises above -0.5 A, charger has stopped.
    const float amps = ads_read_current();
    if (amps > -0.5f) {
        fsm_set_state(fsm, BmsState::SLEEP);
        return;
    }

    // Cell spread >= 50 mV — pause charging and balance
    if (cells_need_charge_balance(meas_snap)) {
        fsm.from_charging = true;
        contactor_open_chg(fsm);
        fsm_set_state(fsm, BmsState::BALANCE);
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
            fsm.last_activity_ms = millis();  // reset idle timer on every NORMAL entry
            contactors_update(fsm);
            break;

        case BmsState::CHARGING:
            fsm.from_charging = false;      // clear return flag on fresh entry
            fsm.last_activity_ms = millis(); // reset idle timer — we are active
            contactor_close_chg(fsm);       // ensure CHG gate is closed
            // DSG gate follows normal fault logic
            contactors_update(fsm);
            break;

        case BmsState::BALANCE:
            // ADCV is now issued inside ltc_read_voltages — no need to start
            // a conversion here. Just notify task_balance to begin.
            if (g_balance_task_handle) {
                xTaskNotifyGive(g_balance_task_handle);
            }
            break;

        case BmsState::SLEEP:
            balance_stop(&g_meas);
            ltc_wakeup_sleep();
            contactor_open_chg(fsm);
            contactor_open_dsg(fsm);
            break;

        case BmsState::FAULT:
            contactor_open_chg(fsm);
            contactor_open_dsg(fsm);
            balance_stop(nullptr);  // clears DCC hardware only, no struct write
            // Fix #4: hold g_snapshot_mutex across the full struct write AND the
            // flag set. task_daq checks the flag under the same mutex, so it can
            // never observe pending=true while the struct is only partially written.
            if (xSemaphoreTake(g_snapshot_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                fault_capture_context(&g_fault_snapshot, fsm.prev_state);
                fault_log_write(&g_fault_snapshot);
                g_fault_snapshot_pending = true;
                xSemaphoreGive(g_snapshot_mutex);
            } else {
                // Mutex timeout at fault entry — still capture and log,
                // skip the pending flag so DAQ doesn't race the partial write.
                fault_capture_context(&g_fault_snapshot, fsm.prev_state);
                fault_log_write(&g_fault_snapshot);
            }
            break;

        
    }
}

// ============================================================================
// Public API
// ============================================================================

void fsm_init(BmsFsm &fsm) {
    fsm = BmsFsm{};   // zero-initialise to defaults
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
        case BmsState::INIT:     state_init(fsm);    break;
        case BmsState::STARTUP:  state_startup(fsm); break;
        case BmsState::NORMAL:   state_normal(fsm);  break;
        case BmsState::CHARGING: state_charging(fsm); break;
        case BmsState::BALANCE:  state_balance(fsm); break;
        case BmsState::SLEEP:    state_sleep(fsm);   break;
        case BmsState::FAULT:    state_fault(fsm);   break;
    }
}