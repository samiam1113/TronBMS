#ifndef BMS_MEASUREMENTS_H
#define BMS_MEASUREMENTS_H

#include "bms_types.h"
#include "bms_config.h"

// ============================================================================
// bms_measurements.h — Pack measurement and protection check API
//
// All check functions:
//   - take const measurement_data_t* (no side effects on the struct)
//   - return true if the fault condition is active, false otherwise
//   - do NOT call fault_set/fault_clear — that is the caller's responsibility
//
// meas_cell_data is the only function that reads hardware; all others are
// pure calculations over already-populated measurement_data_t fields.
// ============================================================================

/**
 * meas_cell_data — read voltages, temperatures, and current into *meas.
 * Calls ltc_read_voltages, ltc_read_temperatures, and ads_read_current.
 * Sets Fault::SPI via fault_set() on comms failure; clears it on success.
 * Returns false on any hardware read error.
 */
bool meas_cell_data(measurement_data_t *meas);

// ── Protection checks ─────────────────────────────────────────────────────────

/**
 * meas_check_overvoltage — true if any cell_v >= CELL_OV_V or pack sum >= PACK_OV_V.
 */
bool meas_check_overvoltage(const measurement_data_t *meas);

/**
 * meas_check_undervoltage — true if any cell_v <= CELL_UV_V or pack sum <= PACK_UV_V.
 */
bool meas_check_undervoltage(const measurement_data_t *meas);

/**
 * meas_check_overcurrent — true if current exceeds continuous or peak+timer thresholds.
 * Maintains an internal timer for peak current detection; timer resets when
 * current drops below CURR_DSCHG_PEAK_A / CURR_CHG_PEAK_A.
 */
bool meas_check_overcurrent(const measurement_data_t *meas);

/**
 * meas_check_overtemp — true if any sensor >= TEMP_CUTOFF_C.
 * Writes the index of the first triggering sensor to *ot_ch_out (0xFF if none).
 */
bool meas_check_overtemp(const measurement_data_t *meas, uint8_t *ot_ch_out);

/**
 * meas_check_balance_overtemp — true if any sensor >= TEMP_WARN_C.
 * Sets/clears g_balance_overtemp with 3°C hysteresis on the clear path.
 * Writes the triggering sensor index to *ot_ch_out (0xFF if none).
 */
bool meas_check_balance_overtemp(const measurement_data_t *meas, uint8_t *ot_ch_out);

// ── Cell voltage utilities ────────────────────────────────────────────────────

/**
 * meas_min_cell_voltage — lowest cell_v across the entire pack (volts).
 */
float meas_min_cell_voltage(const measurement_data_t *meas);

/**
 * meas_max_cell_voltage — highest cell_v across the entire pack (volts).
 */
float meas_max_cell_voltage(const measurement_data_t *meas);

#endif // BMS_MEASUREMENTS_H