#pragma once
 
#include <stdint.h>
#include "bms_config.h"
#include "bms_state.h"   // BmsState
 
// ============================================================================
// bms_fsm.h — BMS Finite State Machine
//
// States:   INIT → STARTUP → NORMAL ↔ BALANCE
//                                   → SLEEP
//           any  → FAULT
//
// Thread safety: BmsFsm::state is written only by task_fsm (highest-priority
//                FSM task) and read by other tasks. On Xtensa/ESP32 a 32-bit
//                aligned read is atomic, but use fsm_get_state() so the
//                intent is clear and a mutex can be swapped in later if needed.
// ============================================================================
 


struct BmsFsm {
    BmsState         state            = BmsState::INIT;
    BmsState         prev_state       = BmsState::INIT;
    uint16_t         fault_reg        = 0;
    uint32_t         state_entry_ms   = 0;
    uint32_t         last_activity_ms = 0;
    bool             chg_open         = true;
    bool             dsg_open         = true;
};

// ----------------------------------------------------------------------------
// Fault register  (bitfield — use fault_set/clear helpers, not raw OR)
// ----------------------------------------------------------------------------
enum class Fault : uint16_t {
    NONE    = 0,
    CELL_OV = (1u << 0),    // Cell over-voltage
    CELL_UV = (1u << 1),    // Cell under-voltage
    PACK_OV = (1u << 2),    // Pack over-voltage
    PACK_UV = (1u << 3),    // Pack under-voltage
    OT      = (1u << 4),    // Over-temperature (any sensor)
    OC_CHG  = (1u << 5),    // Charge over-current
    OC_DSG  = (1u << 6),    // Discharge over-current
    SPI     = (1u << 7),    // SPI / PEC comms failure
    ADS_ID  = (1u << 8),    // ADS131 ID mismatch on startup
    GATE    = (1u << 9),    // Gate driver self-test failure
};
 
// Bitwise helpers — let Fault values OR/AND with each other and with uint16_t
// without needing manual casts everywhere in the FSM.
constexpr uint16_t operator|(Fault a, Fault b) {
    return static_cast<uint16_t>(a) | static_cast<uint16_t>(b);
}
constexpr uint16_t operator|(uint16_t a, Fault b) {
    return a | static_cast<uint16_t>(b);
}
inline uint16_t& operator|=(uint16_t &a, Fault b) {
    a |= static_cast<uint16_t>(b); return a;
}
inline uint16_t& operator&=(uint16_t &a, Fault b) {
    a &= static_cast<uint16_t>(b); return a;
}
constexpr bool operator&(uint16_t a, Fault b) {
    return (a & static_cast<uint16_t>(b)) != 0;
}
 
// Faults that block the charge path
static constexpr uint16_t FAULT_BLOCKS_CHG =
    Fault::CELL_OV | Fault::PACK_OV | Fault::OT |
    Fault::OC_CHG  | Fault::SPI     | Fault::GATE;
 
// Faults that block the discharge path
static constexpr uint16_t FAULT_BLOCKS_DSG =
    Fault::CELL_UV | Fault::PACK_UV | Fault::OT |
    Fault::OC_DSG  | Fault::SPI     | Fault::GATE;
 
// ----------------------------------------------------------------------------
// Timing constants  (move to bms_config.h once you centralise them)
// ----------------------------------------------------------------------------
static constexpr uint32_t MEASURE_PERIOD_MS     = 100;    // Voltage + current sample rate
static constexpr uint32_t BAL_REFRESH_MS        = 500;    // Max interval between WRCFGA refreshes
                                                           //   (LTC DCC watchdog resets at ~2 s)
static constexpr uint32_t SLEEP_IDLE_TIMEOUT_MS = 30000;  // Enter sleep after 30 s idle
static constexpr uint32_t WDT_FEED_PERIOD_MS    = 500;    // task_watchdog wake period
 
// ----------------------------------------------------------------------------
// FSM context — pass by reference throughout, one instance in main.cpp
// ----------------------------------------------------------------------------
struct BmsFsm {
    BmsState state           = BmsState::INIT;
    BmsState prev_state      = BmsState::INIT;
    uint16_t fault_reg       = 0;       // bitmask of Fault values
    uint32_t state_entry_ms  = 0;       // millis() when current state was entered
    uint32_t last_activity_ms = 0;      // updated by task_measure on any current
    bool     chg_open        = true;    // true = CHG contactor open
    bool     dsg_open        = true;    // true = DSG contactor open
};
 
// ----------------------------------------------------------------------------
// Public API
// ----------------------------------------------------------------------------
 
/**
 * fsm_init — initialise context, enter INIT state.
 * Call once from setup() before starting RTOS tasks.
 */
void fsm_init(BmsFsm &fsm);
 
/**
 * fsm_run — execute one FSM tick.
 * Called by task_fsm every WDT_FEED_PERIOD_MS ms.
 */
void fsm_run(BmsFsm &fsm);
 
/** Thread-safe state accessor for other tasks. */
BmsState fsm_get_state(const BmsFsm &fsm);
 
/**
 * fsm_set_state — transition to a new state and run entry actions.
 * Internal to bms_fsm.cpp; external callers should use fault_set()
 * in bms_fault.cpp which will call this.
 */
void fsm_set_state(BmsFsm &fsm, BmsState new_state);