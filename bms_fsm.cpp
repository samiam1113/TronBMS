// ============================================================================
// bms_fsm.cpp — BMS Finite State Machine
//
// State transition summary:
//
//   INIT
//     entry : ltc_wakeup_sleep(), ads_reset()
//     do    : ads_configure() + ID verify
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
//     entry : ltc_start_adc_conversion() (REFUP already on from STARTUP)
//     do    : recompute + apply balance mask each tick, protection checks
//     exit  : → NORMAL (balanced) | → FAULT
//
//   SLEEP
//     entry : balance_stop(), ltc_sleep, open contactors
//     do    : poll ADS current for wakeup condition
//     exit  : → INIT (current detected)
//
//   FAULT
//     entry : open contactors, balance_stop(), snapshot + log fault context
//     do    : poll fault_reg for clearance
//     exit  : → INIT (all faults cleared)
//
// ============================================================================

#include <Arduino.h>
#include "bms_fsm.h"
#include "bms_fault.h"
#include "ltc_spi.h"
#include "bms_hardware.h"
#include "bms_measurements.h"
#include "bms_balance.h"

// Forward declarations of state handlers
static void state_init(BmsFsm &fsm);
static void state_startup(BmsFsm &fsm);
static void state_normal(BmsFsm &fsm);
static void state_balance(BmsFsm &fsm);
static void state_sleep(BmsFsm &fsm);
static void state_fault(BmsFsm &fsm);
static void fsm_on_enter(BmsFsm &fsm, BmsState new_state);

// ============================================================================
// Public API
// ============================================================================

void fsm_init(BmsFsm &fsm) {
    fsm.state            = BmsState::INIT;
    fsm.prev_state       = BmsState::INIT;
    fsm.fault_reg        = 0;
    fsm.state_entry_ms   = millis();
    fsm.last_activity_ms = millis();
    fsm.chg_open         = true;
    fsm.dsg_open         = true;
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

BmsState fsm_get_state(const BmsFsm &fsm) {
    return fsm.state;
}

void fsm_set_state(BmsFsm &fsm, BmsState new_state) {
    if (fsm.state == new_state) return;
    Serial.printf("[fsm] %d → %d\n",
                  static_cast<int>(fsm.state),
                  static_cast<int>(new_state));
    fsm_on_enter(fsm, new_state);
}

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
    if (fsm.fault_reg & Fault::CELL_OV) return;
    if (fsm.fault_reg & Fault::PACK_OV) return;
    if (fsm.fault_reg & Fault::OT)      return;
    if (fsm.fault_reg & Fault::OC_CHG)  return;
    if (fsm.fault_reg & Fault::SPI)     return;
    if (fsm.fault_reg & Fault::GATE)    return;
    digitalWrite(GATE_CHG_PIN, HIGH);
    fsm.chg_open = false;
}

static void contactor_close_dsg(BmsFsm &fsm) {
    if (fsm.fault_reg & Fault::CELL_UV) return;
    if (fsm.fault_reg & Fault::PACK_UV) return;
    if (fsm.fault_reg & Fault::OT)      return;
    if (fsm.fault_reg & Fault::OC_DSG)  return;
    if (fsm.fault_reg & Fault::SPI)     return;
    if (fsm.fault_reg & Fault::GATE)    return;
    digitalWrite(GATE_DSCHG_PIN, HIGH);
    fsm.dsg_open = false;
}

static void contactors_update(BmsFsm &fsm) {
    (fsm.fault_reg & FAULT_BLOCKS_CHG) ? contactor_open_chg(fsm)
                                       : contactor_close_chg(fsm);
    (fsm.fault_reg & FAULT_BLOCKS_DSG) ? contactor_open_dsg(fsm)
                                       : contactor_close_dsg(fsm);
}

static bool state_timeout(const BmsFsm &fsm, uint32_t ms) {
    return (millis() - fsm.state_entry_ms) >= ms;
}

// ============================================================================
// State handlers
// ============================================================================

// ----------------------------------------------------------------------------
// INIT — bring hardware out of reset, verify ADS comms
// ----------------------------------------------------------------------------
static void state_init(BmsFsm &fsm) {
    ltc_wakeup_sleep();

    ads_reset();
    delay(2);

    if (!ads_configure()) {
        fsm.fault_reg |= Fault::ADS_ID;
        fsm_set_state(fsm, BmsState::FAULT);
        return;
    }

    fsm_set_state(fsm, BmsState::STARTUP);
}

// ----------------------------------------------------------------------------
// STARTUP — self-test all sub-systems
// ----------------------------------------------------------------------------
static void state_startup(BmsFsm &fsm) {
    if (!ltc_comms_test()) {
        fsm.fault_reg |= Fault::SPI;
        fsm_set_state(fsm, BmsState::FAULT);
        return;
    }

    if (!ads_checkid()) {
        fsm.fault_reg |= Fault::ADS_ID;
        fsm_set_state(fsm, BmsState::FAULT);
        return;
    }

    if (!gate_driver_selftest()) {
        fsm.fault_reg |= Fault::GATE;
        fsm_set_state(fsm, BmsState::FAULT);
        return;
    }

    // LTC active standby — REFON=1, all DCC cleared, UV/OV thresholds set
    LtcConfig cfg[2];
    for (int ic = 0; ic < 2; ic++) {
        memset(&cfg[ic], 0, sizeof(LtcConfig));
        cfg[ic].refon  = true;
        cfg[ic].adcopt = false;
        cfg[ic].dcto   = 0x00;
        cfg[ic].dcc    = 0x0000;
        cfg[ic].vuv    = (uint16_t)((CELL_UV_RAW / 16u) - 1u);
        cfg[ic].vov    = (uint16_t) (CELL_OV_RAW / 16u);
    }
    ltc_write_config(cfg);

    fsm_set_state(fsm, BmsState::NORMAL);
}

// ----------------------------------------------------------------------------
// NORMAL — steady-state monitoring
// ----------------------------------------------------------------------------
static void state_normal(BmsFsm &fsm) {
    // Sync fault register from global (task_measure writes via fault_set)
    fsm.fault_reg = g_faultRegister;
    contactors_update(fsm);

    if (fsm.fault_reg != 0) {
        fsm_set_state(fsm, BmsState::FAULT);
        return;
    }

    // Idle sleep timeout
    if ((millis() - fsm.last_activity_ms) >= SLEEP_IDLE_TIMEOUT_MS) {
        fsm_set_state(fsm, BmsState::SLEEP);
        return;
    }

    // Check if balancing is needed
    measurement_data_t snap;
    extern measurement_data_t g_latest_meas;
    snap = g_latest_meas;   // shallow copy — safe for float array

    if (!balance_satisfied(&snap)) {
        fsm_set_state(fsm, BmsState::BALANCE);
    }
}

// ----------------------------------------------------------------------------
// BALANCE — active cell balancing
// ----------------------------------------------------------------------------
static void state_balance(BmsFsm &fsm) {
    fsm.fault_reg = g_faultRegister;
    contactors_update(fsm);

    if (fsm.fault_reg != 0) {
        balance_stop();
        fsm_set_state(fsm, BmsState::FAULT);
        return;
    }

    // task_balance owns the DCC mask computation and apply each BAL_REFRESH_MS;
    // the FSM just monitors for completion or fault via event group bits.
    // Balance completion EVT_BALANCE_DONE is handled in task_fsm loop.
}

// ----------------------------------------------------------------------------
// SLEEP — low-power idle, poll current for wakeup
// ----------------------------------------------------------------------------
static void state_sleep(BmsFsm &fsm) {
    const float amps = ads_read_current();

    if (amps > 1.0f || amps < -1.0f) {
        fsm.last_activity_ms = millis();
        fsm_set_state(fsm, BmsState::INIT);
    }
}

// ----------------------------------------------------------------------------
// FAULT — safe state, contactors open, wait for fault clearance
// ----------------------------------------------------------------------------
static void state_fault(BmsFsm &fsm) {
    contactor_open_chg(fsm);
    contactor_open_dsg(fsm);

    // Sync from global — external caller may have cleared faults
    fsm.fault_reg = g_faultRegister;

    if (fsm.fault_reg == 0) {
        Serial.println("[fsm] All faults cleared — returning to INIT");
        fsm_set_state(fsm, BmsState::INIT);
    }
}

// ============================================================================
// Entry actions — run exactly once on each state transition
// ============================================================================
static void fsm_on_enter(BmsFsm &fsm, BmsState new_state) {
    fsm.prev_state    = fsm.state;
    fsm.state         = new_state;
    fsm.state_entry_ms = millis();

    switch (new_state) {

        case BmsState::INIT:
            // state_init() owns the full wakeup sequence
            break;

        case BmsState::STARTUP:
            // LTC already in STANDBY from INIT; idle wake is sufficient
            ltc_wakeup_idle();
            break;

        case BmsState::NORMAL:
            contactors_update(fsm);
            break;

        case BmsState::BALANCE: {
            // Start ADC conversions — REFON=1 already set in STARTUP
            ltc_start_adc_conversion(true, false);
            // Resume task_balance via task notification
            extern TaskHandle_t g_balance_task_handle;
            if (g_balance_task_handle) {
                vTaskResume(g_balance_task_handle);
                xTaskNotifyGive(g_balance_task_handle);
            }
            break;
        }

        case BmsState::SLEEP:
            balance_stop();
            ltc_wakeup_sleep();         // ensure IC is awake to receive cmd
            // Send LTC SLEEP command (0x0005) via raw SPI
            {
                extern SPIClass *hspi;
                uint8_t sleep_cmd[4] = {0x00, 0x05, 0x00, 0x00};
                uint16_t crc = ltc_pec15_calc(sleep_cmd, 2);
                sleep_cmd[2] = (crc >> 8) & 0xFF;
                sleep_cmd[3] =  crc       & 0xFF;
                hspi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
                digitalWrite(HSPI_SS, LOW);
                hspi->transfer(sleep_cmd, 4);
                digitalWrite(HSPI_SS, HIGH);
                hspi->endTransaction();
            }
            contactor_open_chg(fsm);
            contactor_open_dsg(fsm);
            break;

        case BmsState::FAULT:
            contactor_open_chg(fsm);
            contactor_open_dsg(fsm);
            balance_stop();
            // Context capture is done in task_fsm before calling fsm_set_state
            Serial.printf("[fsm] FAULT entered — fault_reg=0x%04X\n",
                          fsm.fault_reg);
            break;
    }
}