#pragma once

#include <stdint.h>
#include "bms_config.h"
#include "bms_types.h"   // measurement_data_t
#include "bms_state.h"   // BmsState

// ============================================================================
// bms_fault.h — Fault register, snapshot types, and NVS logging API
// ============================================================================

// ----------------------------------------------------------------------------
// Fault code — labels a snapshot with the primary fault that triggered it
// ----------------------------------------------------------------------------
enum class FaultCode : uint8_t {
    NONE        = 0,
    OVERVOLTAGE,
    UNDERVOLTAGE,
    OVERCURRENT,
    OVERTEMP,
    BAL_OVERTEMP,
    SPI_LTC,
    SPI_ADS,
    STARTUP,
};

// ----------------------------------------------------------------------------
// Fault snapshot — persisted to NVS on fault entry
// ----------------------------------------------------------------------------
struct fault_snapshot_t {
    FaultCode        code;
    BmsState         state_at_fault;
    measurement_data_t meas;   // Full pack state at moment of fault
};

// ----------------------------------------------------------------------------
// Global pack measurement — single source of truth written by task_measure,
// read by fault_capture_context and anyone else needing current data.
// Protected by g_meas_mutex in bms_tasks.cpp.
// ----------------------------------------------------------------------------
extern measurement_data_t g_meas;

// ----------------------------------------------------------------------------
// Fault register — bitmask of active Fault enum values
// ----------------------------------------------------------------------------
extern volatile uint16_t g_faultRegister;

// ----------------------------------------------------------------------------
// API
// ----------------------------------------------------------------------------
void     fault_set(uint16_t bit);
void     fault_clear(uint16_t bit);
uint16_t fault_get(void);

void fault_capture_context(fault_snapshot_t *snap, BmsState state_at_fault);
void fault_log_write(const fault_snapshot_t *snap);
bool fault_log_read(fault_snapshot_t *snap_out);