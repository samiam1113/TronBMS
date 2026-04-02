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
//     entry : ltc_start_adc_conversion()
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
#include "ltc_spi.cpp"
#include "bms_hardware.cpp"
#include "bms_measurements.cpp"
#include "bms_balance.cpp"
#include "bms_fault.cpp"

#include <Arduino.h>

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
    fsm.fault_reg |= bit;
    contactors_update(fsm);
}

static bool state_timeout(const BmsFsm &fsm, uint32_t ms) {
    return (millis() - fsm.state_entry_ms) >= ms;
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
    if (!configureADS131M02()) {
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

    if (meas_check_overvoltage(&meas))  fsm_fault_set(fsm, Fault::CELL_OV);
    if (meas_check_undervoltage(&meas)) fsm_fault_set(fsm, Fault::CELL_UV);
    if (meas_check_overcurrent(&meas))  fsm_fault_set(fsm, Fault::OC_DSG);

    uint8_t ot_ch = 0;
    if (meas_check_overtemp(&meas, &ot_ch)) fsm_fault_set(fsm, Fault::OT);

    contactors_update(fsm);

    if (fsm.fault_reg != 0) {
        fsm_set_state(fsm, BmsState::FAULT);
        return;
    }

    if (state_timeout(fsm, SLEEP_IDLE_TIMEOUT_MS) &&
        (millis() - fsm.last_activity_ms) >= SLEEP_IDLE_TIMEOUT_MS) {
        fsm_set_state(fsm, BmsState::SLEEP);
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
    const measurement_data_t &meas = g_meas;

    if (meas_check_overvoltage(&meas))  fsm_fault_set(fsm, Fault::CELL_OV);
    if (meas_check_undervoltage(&meas)) fsm_fault_set(fsm, Fault::CELL_UV);
    if (meas_check_overcurrent(&meas))  fsm_fault_set(fsm, Fault::OC_DSG);

    uint8_t ot_ch = 0;
    if (meas_check_overtemp(&meas, &ot_ch)) fsm_fault_set(fsm, Fault::OT);

    uint8_t bal_ot_ch = 0;
    meas_check_balance_overtemp(&meas, &bal_ot_ch);  // sets g_balance_overtemp

    contactors_update(fsm);

    if (fsm.fault_reg != 0) {
        balance_stop(&g_meas);
        fsm_set_state(fsm, BmsState::FAULT);
        return;
    }

    // balance_compute_mask writes into g_meas.balance_cells
    balance_compute_mask(&g_meas);
    // balance_apply reads g_meas.balance_cells and pushes to LTC;
    // skips the write if g_balance_overtemp is set
    balance_apply(&g_meas);

    if (balance_satisfied(&meas)) {
        balance_stop(&g_meas);
        fsm_set_state(fsm, BmsState::NORMAL);
    }
}

// ----------------------------------------------------------------------------
// SLEEP — LTC asleep, poll ADS131 current for wakeup
// ----------------------------------------------------------------------------
static void state_sleep(BmsFsm &fsm) {
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

    // Sync with bms_fault module — external callers clear bits via fault_clear()
    fsm.fault_reg = g_faultRegister;

    if (fsm.fault_reg == 0) {
        fsm_set_state(fsm, BmsState::INIT);
    }
}

// ============================================================================
// State transition — entry actions run exactly once per state change
// ============================================================================
static void fsm_on_enter(BmsFsm &fsm, BmsState new_state) {
    fsm.prev_state     = fsm.state;
    fsm.state          = new_state;
    fsm.state_entry_ms = millis();

    fault_snapshot_t     g_fault_snapshot         = {};

    switch (new_state) {

        case BmsState::INIT:
            break;

        case BmsState::STARTUP:
            ltc_wakeup_idle();
            break;

        case BmsState::NORMAL:
            contactors_update(fsm);
            break;

        case BmsState::BALANCE:
            ltc_start_adc_conversion(true, false);
            break;

        case BmsState::SLEEP:
            balance_stop(&g_meas);
            ltc_wakeup_sleep();
            delay(3000);
            contactor_open_chg(fsm);
            contactor_open_dsg(fsm);
            break;

        case BmsState::FAULT:
            contactor_open_chg(fsm);
            contactor_open_dsg(fsm);
            balance_stop(&g_meas);
            fault_capture_context(&g_fault_snapshot, fsm.prev_state);
            fault_log_write(&g_fault_snapshot);
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
        case BmsState::INIT:    state_init(fsm);    break;
        case BmsState::STARTUP: state_startup(fsm); break;
        case BmsState::NORMAL:  state_normal(fsm);  break;
        case BmsState::BALANCE: state_balance(fsm); break;
        case BmsState::SLEEP:   state_sleep(fsm);   break;
        case BmsState::FAULT:   state_fault(fsm);   break;
    }
}