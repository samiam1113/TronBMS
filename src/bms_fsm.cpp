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
//     do    : ltc_comms_test(),!ads_checkid(), gate_driver_selftest(),
//             ltc_write_config() (REFON=1, DCC=0)
//     exit  : → NORMAL (all tests pass) | → FAULT
//
//   NORMAL
//     entry : contactors closed (if no faults), isoSPI active
//     do    : protection checks, idle timeout, balance condition
//     exit  : → BALANCE | → SLEEP | → FAULT
//
//   BALANCE
//     entry : ltc_start_adc_conversion() (REFUP already on from STARTUP)
//     do    : recompute + apply balance mask each tick, protection checks
//     exit  : → NORMAL (balanced) | → FAULT
//
//   SLEEP
//     entry : balance_stop(), ltc_sleep_cmd(), open contactors
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
#include "ltc_spi.cpp"            // ltc_wakeup_sleep/idle, ltc_write_config,
                                // ltc_comms_test, ltc_start_adc_conversion,
                                // ltc_sleep_cmd
#include "bms_hardware.cpp"       // ads_reset, configureADS131M02,!ads_checkid,
                                // ads_read_current,
                                // gate_driver_selftest,
                                // gate_driver_enable, gate_driver_disable
#include "bms_measurements.cpp"   // meas_check_overvoltage/undervoltage/
                                // overcurrent/overtemp/balance_overtemp
#include "bms_balance.cpp"        // balance_compute_mask, balance_satisfied,
                                // balance_apply, balance_stop
#include "bms_fault.cpp"          // fault_get, fault_capture_context,
                                // fault_log_write
 
#include <Arduino.h>            // millis(), digitalWrite()
 
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
    if (fsm.fault_reg & FAULT_BLOCKS_CHG) return;  // safety gate
    digitalWrite(GATE_CHG_PIN, HIGH);
    fsm.chg_open = false;
}
 
static void contactor_close_dsg(BmsFsm &fsm) {
    if (fsm.fault_reg & FAULT_BLOCKS_DSG) return;
    digitalWrite(GATE_DSCHG_PIN, HIGH);
    fsm.dsg_open = false;
}
 
/** Re-evaluate both contactors against the current fault register. */
static void contactors_update(BmsFsm &fsm) {
    (fsm.fault_reg & FAULT_BLOCKS_CHG) ? contactor_open_chg(fsm)
                                       : contactor_close_chg(fsm);
    (fsm.fault_reg & FAULT_BLOCKS_DSG) ? contactor_open_dsg(fsm)
                                       : contactor_close_dsg(fsm);
}
 
/** Set a fault bit and immediately re-evaluate contactors. */
static void fault_set(BmsFsm &fsm, Fault bit) {
    fsm.fault_reg |= bit;
    contactors_update(fsm);
}
 
/** Returns true if the FSM has been in the current state longer than ms. */
static bool state_timeout(const BmsFsm &fsm, uint32_t ms) {
    return (millis() - fsm.state_entry_ms) >= ms;
}
 
// ============================================================================
// State handlers — one function per state, called each FSM tick
// ============================================================================
 
// ----------------------------------------------------------------------------
// INIT
//   Brings hardware out of reset and verifies basic comms.
//
//   Sequence per design outline:
//     1. ltc_wakeup_sleep()  — CS pulse → LTC STANDBY + tREFUP, also wakes
//                              IC2 via the isoSPI Port B pulse (isoSPI ready)
//     2. ads_reset()         — assert/deassert ADS131 /RESET pin
//     3. configureADS131M02()     — WREG CLOCK + GAIN1, readback verify
//
//   On failure → FAULT.
// ----------------------------------------------------------------------------
static void state_init(BmsFsm &fsm) {
    // Step 1: LTC SLEEP → STANDBY, isoSPI chain wakeup (both ICs)
    ltc_wakeup_sleep();
 
    // Step 2 & 3: ADS131 reset + configure
    ads_reset();
    if (!configureADS131M02()) {
        fault_set(fsm, Fault::ADS_ID);
        fsm_set_state(fsm, BmsState::FAULT);
        return;
    }
 
    fsm_set_state(fsm, BmsState::STARTUP);
}
 
// ----------------------------------------------------------------------------
// STARTUP
//   Verifies all sub-systems are healthy before allowing operation.
//
//   Sequence per design outline:
//     1. ltc_comms_test()       — write/read config, verify PEC round-trip
//     2.!ads_checkid()          — read ID register vs ADS_EXPECTED_ID
//     3. gate_driver_selftest() — pulse gate output, check feedback pin
//     4. ltc_write_config()     — REFON=1, DCC=0 (active standby, no balance)
// ----------------------------------------------------------------------------
static void state_startup(BmsFsm &fsm) {
    // 1. LTC comms loopback
    if (!ltc_comms_test()) {
        fault_set(fsm, Fault::SPI);
        fsm_set_state(fsm, BmsState::FAULT);
        return;
    }
 
    // 2. ADS ID verify
    if (!ads_checkid()) {
        fault_set(fsm, Fault::ADS_ID);
        fsm_set_state(fsm, BmsState::FAULT);
        return;
    }
 
    // 3. Gate driver self-test
    if (!gate_driver_selftest()) {
        fault_set(fsm, Fault::GATE);
        fsm_set_state(fsm, BmsState::FAULT);
        return;
    }
 
    // 4. LTC active standby — REFON=1, all DCC bits cleared
    const LtcConfig bal_none[NUM_CELLS] = {};
    ltc_write_config(bal_none);
 
    fsm_set_state(fsm, BmsState::NORMAL);
}
 
// ----------------------------------------------------------------------------
// NORMAL
//   Monitors the pack each tick, reacts to faults, and decides when to
//   balance or sleep.
//
//   Per tick:
//     • Run all protection checks (each writes into fsm.fault_reg)
//     • Re-evaluate contactors
//     • Any fault → FAULT
//     • Cells unbalanced and no fault → BALANCE
//     • No current for SLEEP_IDLE_TIMEOUT_MS → SLEEP
// ----------------------------------------------------------------------------
static void state_normal(BmsFsm &fsm) {
    // Protection — each check calls fault_set/clear which updates fault_reg
    meas_check_overvoltage(fsm);
    meas_check_undervoltage(fsm);
    meas_check_overcurrent(fsm);
    meas_check_overtemp(fsm);
    contactors_update(fsm);
 
    if (fsm.fault_reg != 0) {
        fsm_set_state(fsm, BmsState::FAULT);
        return;
    }
 
    // Idle timeout — last_activity_ms is updated by task_measure
    // whenever |amps| exceeds a small threshold (~1 A)
    if (state_timeout(fsm, SLEEP_IDLE_TIMEOUT_MS) &&
        (millis() - fsm.last_activity_ms) >= SLEEP_IDLE_TIMEOUT_MS) {
        fsm_set_state(fsm, BmsState::SLEEP);
        return;
    }
 
    // Balance needed — balance_satisfied() checks max−min delta vs BAL_THRESHOLD_UV
    if (!balance_satisfied()) {
        fsm_set_state(fsm, BmsState::BALANCE);
    }
}
 
// ----------------------------------------------------------------------------
// BALANCE
//   Brings all cells to within BAL_THRESHOLD_UV of the minimum cell.
//
//   Per tick:
//     • Protection checks first — safety always wins
//     • Check balance overtemp (softer threshold than OT fault)
//     • Recompute mask from latest g_cellRaw, apply via ltc_write_config()
//     • balance_satisfied() → NORMAL
//     • Fault → FAULT
// ----------------------------------------------------------------------------
static void state_balance(BmsFsm &fsm) {
    meas_check_overvoltage(fsm);
    meas_check_undervoltage(fsm);
    meas_check_overcurrent(fsm);
    meas_check_overtemp(fsm);
    meas_check_balance_overtemp(fsm);   // softer threshold, pauses balance_apply()
    contactors_update(fsm);
 
    if (fsm.fault_reg != 0) {
        balance_stop();
        fsm_set_state(fsm, BmsState::FAULT);
        return;
    }
 
    // Recompute mask from current raw cell counts and push to LTC
    // balance_apply() skips the WRCFGA write if balance_overtemp is active
    balance_compute_mask();
    balance_apply();
 
    if (balance_satisfied()) {
        balance_stop();
        fsm_set_state(fsm, BmsState::NORMAL);
    }
}
 
// ----------------------------------------------------------------------------
// SLEEP
//   Pack is idle — LTC is in SLEEP mode, contactors open.
//   ADS131 is left running (its idle current is low at OSR=4096) so we can
//   detect a load or charger without a full LTC wakeup on every poll.
//
//   Wakeup condition: |current| > 1 A → re-init from INIT.
// ----------------------------------------------------------------------------
static void state_sleep(BmsFsm &fsm) {
    const float amps = ads_read_current();
 
    if (amps > 1.0f || amps < -1.0f) {
        fsm.last_activity_ms = millis();
        fsm_set_state(fsm, BmsState::INIT);
    }
    // else: stay here — task_fsm delays WDT_FEED_PERIOD_MS between ticks
    // so we are not busy-polling at full CPU speed
}
 
// ----------------------------------------------------------------------------
// FAULT
//   Safe state: contactors open, no balancing.
//   Stays here until all fault bits are cleared externally (e.g. via a CAN
//   reset command handled in task_daq, or a manual BMS reset).
// ----------------------------------------------------------------------------
static void state_fault(BmsFsm &fsm) {
    // Enforce safe state every tick — no path out of FAULT opens contactors
    contactor_open_chg(fsm);
    contactor_open_dsg(fsm);
 
    // Sync fault_reg with bms_fault module (external callers may have cleared
    // bits via fault_clear() in bms_fault.cpp)
    fsm.fault_reg = static_cast<uint16_t>(fault_get());
 
    if (fsm.fault_reg == 0) {
        fsm_set_state(fsm, BmsState::INIT);
    }
}
 
// ============================================================================
// State transition — entry actions run exactly once on each state change
// ============================================================================
static void fsm_on_enter(BmsFsm &fsm, BmsState new_state) {
    fsm.prev_state    = fsm.state;
    fsm.state         = new_state;
    fsm.state_entry_ms = millis();
 
    switch (new_state) {
 
        case BmsState::INIT:
            // Nothing — state_init() owns the full wakeup sequence
            break;
 
        case BmsState::STARTUP:
            // LTC is already in STANDBY from INIT; idle wakeup keeps it there
            // without the full tREFUP wait
            ltc_wakeup_idle();
            break;
 
        case BmsState::NORMAL:
            // Close contactors if the fault register is clean
            contactors_update(fsm);
            break;
 
        case BmsState::BALANCE:
            // Trigger ADCV so conversions are running when state_balance() fires.
            // REFON=1 was set in STARTUP and stays on while LTC is not sleeping.
            ltc_start_adc_conversion();
            break;
 
        case BmsState::SLEEP:
            balance_stop();             // clear all DCC bits before sleeping
            ltc_wakeup_sleep();         // ensure LTC is awake to receive cmd
            delay(3000);          // send LTC SLEEP (cmd 0x0005)
            contactor_open_chg(fsm);
            contactor_open_dsg(fsm);
            break;
 
        case BmsState::FAULT:
            contactor_open_chg(fsm);
            contactor_open_dsg(fsm);
            balance_stop();
            fault_capture_context();                    // snapshot measurements
            printFaultState(fsm.fault_reg);             
            break;
    }
}
