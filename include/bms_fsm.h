#pragma once

#include <stdint.h>
#include "bms_config.h"
#include "bms_state.h"

// ============================================================================
// bms_fsm.h — BMS Finite State Machine
//
// States:   INIT → STARTUP → NORMAL ↔ BALANCE
//                                    → SLEEP
//           any  → FAULT
//
// Thread safety: BmsFsm::state is written only by task_fsm and read by other
// tasks. On Xtensa/ESP32 a 32-bit-aligned read is atomic, but use
// fsm_get_state() so the intent is clear and a mutex can be swapped in later.
// ============================================================================

// ----------------------------------------------------------------------------
// Fault register — bitmask (use Fault:: constants, not raw values)
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
    TASK_STALL = (1u << 10), // FreeRTOS task missed watchdog checkin
};

// Bitwise helpers so Fault values OR/AND with each other and uint16_t
// without manual casts throughout the FSM.
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
// Timing constants
// ----------------------------------------------------------------------------
static constexpr uint32_t MEASURE_PERIOD_MS     = 100;    // Voltage + current sample rate
static constexpr uint32_t BAL_REFRESH_MS        = 500;    // Max interval between WRCFGA refreshes
                                                           // (LTC DCC watchdog resets at ~2 s)
static constexpr uint32_t SLEEP_IDLE_TIMEOUT_MS = 30000;  // Enter sleep after 30 s idle
static constexpr uint32_t WDT_FEED_PERIOD_MS    = 500;    // task_watchdog wake period

// ----------------------------------------------------------------------------
// FSM context — one instance in main.cpp, passed by reference throughout
// ----------------------------------------------------------------------------
struct BmsFsm {
    BmsState state            = BmsState::INIT;
    BmsState prev_state       = BmsState::INIT;
    uint16_t fault_reg        = 0;        // bitmask of Fault values
    uint32_t state_entry_ms   = 0;        // millis() when current state was entered
    uint32_t last_activity_ms = 0;        // updated by task_measure on any current
    bool     chg_open         = true;     // true = CHG contactor open
    bool     dsg_open         = true;     // true = DSG contactor open
};

// ----------------------------------------------------------------------------
// Public API
// ----------------------------------------------------------------------------

/**
 * fsm_init — zero-initialise context and enter INIT state.
 * Call once from setup() before starting RTOS tasks.
 */
void fsm_init(BmsFsm &fsm);

/**
 * fsm_run — execute one FSM tick.
 * Called by task_fsm every MEASURE_PERIOD_MS ms.
 */
void fsm_run(BmsFsm &fsm);

/**
 * fsm_get_state — thread-safe state accessor for other tasks.
 */
BmsState fsm_get_state(const BmsFsm &fsm);

/**
 * fsm_set_state — transition to a new state and run entry actions.
 * Primarily internal to bms_fsm.cpp; task_fsm may call this directly
 * after evaluating event group bits.
 */
void fsm_set_state(BmsFsm &fsm, BmsState new_state);