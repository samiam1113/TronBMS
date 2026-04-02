#pragma once

#include "bms_types.h"
#include "bms_state.h"
#include "bms_fault.h"

// ============================================================================
// bms_telemetry.h — CAN telemetry via ESP32 TWAI peripheral
//
// Frame map:
//   0x100 + n  (n = 0..9)  — cell voltages, 2 per frame (4-byte float each)
//                             0x100–0x104: IC1 cells 0–9
//                             0x105–0x109: IC2 cells 0–9
//   0x110 + n  (n = 0..2)  — temperatures, 2 per frame (4-byte float each)
//                             0x110: temps[0,1]
//                             0x111: temps[2,3]
//                             0x112: temps[4] + 0.0f padding
//   0x120                  — pack current (float) + BMS state (uint32_t)
//   0x130                  — fault code (uint32_t) + state at fault (uint32_t)
// ============================================================================

/**
 * telemetry_init — install and start the TWAI (CAN) driver.
 * Call once from setup() or task init before any telemetry_send* calls.
 */
void telemetry_init(void);

/**
 * telemetry_send — transmit cell voltages, temperatures, current, and BMS state.
 */
void telemetry_send(measurement_data_t *meas, BmsState state);

/**
 * telemetry_send_fault — transmit fault code and state-at-fault snapshot frame.
 */
void telemetry_send_fault(fault_snapshot_t *snapshot);