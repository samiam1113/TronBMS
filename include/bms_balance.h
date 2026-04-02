#ifndef BMS_BALANCE_H
#define BMS_BALANCE_H

#include "bms_types.h"
#include "bms_config.h"

// ============================================================================
// bms_balance.h — Cell balancing API
//
// All functions operate on measurement_data_t* — no direct global access.
// balance_apply reads g_balance_overtemp (set by meas_check_balance_overtemp)
// to suppress DCC writes during a thermal pause.
// ============================================================================

// ── Global balance overtemp flag ─────────────────────────────────────────────
// Set/cleared by meas_check_balance_overtemp() in bms_measurements.cpp.
// Read by balance_apply() to gate DCC writes.
extern bool g_balance_overtemp;

// ── API ───────────────────────────────────────────────────────────────────────

/**
 * balance_compute_mask — scan meas->cell_raw and set meas->balance_cells[ic][c]
 * true for any cell whose raw count exceeds (global_min + BAL_THRESHOLD_UV).
 * Writes only to meas->balance_cells; does not touch hardware.
 */
void balance_compute_mask(measurement_data_t *meas);

/**
 * balance_satisfied — return true if (max_raw - min_raw) < BAL_THRESHOLD_UV.
 * Pure read; does not modify meas.
 */
bool balance_satisfied(const measurement_data_t *meas);

/**
 * balance_apply — push meas->balance_cells mask to LTC hardware via WRCFGA.
 * Skipped (returns LTC_OK) if g_balance_overtemp is set.
 * Returns LTC_OK on success, LTC_ERR_SPI on comms failure.
 */
ltc_status_t balance_apply(const measurement_data_t *meas);

/**
 * balance_stop — clear meas->balance_cells and push zeroed DCC to hardware.
 * Safe to call with meas == nullptr (skips the struct clear, still writes LTC).
 */
void balance_stop(measurement_data_t *meas);

#endif // BMS_BALANCE_H