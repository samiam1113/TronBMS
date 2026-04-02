// ============================================================================
// bms_measurements.cpp — Protection checks and cell data helpers
//
// All check functions take const measurement_data_t* and return bool.
//   true  = fault condition active
//   false = fault condition not active
//
// No fault_set/fault_clear calls here — that is the caller's responsibility
// (task_measure in bms_tasks.cpp sets event bits, task_fsm sets fault reg).
// ============================================================================

#include "Arduino.h"
#include <SPI.h>
#include "bms_config.h"
#include "bms_types.h"
#include "bms_fault.h"
#include "bms_fsm.h"

// ── LTC / ADS drivers (defined in ltc_spi.cpp / bms_hardware.cpp) ────────────
extern bool  ltc_read_voltages(measurement_data_t *meas);
extern bool  ltc_read_temperatures(measurement_data_t *meas);
extern float ads_read_current();

// ── Module-level peak current timer ──────────────────────────────────────────
static uint32_t s_oc_peak_start_ms = 0;
static bool     s_oc_peak_is_charge = false;

// ── Global balance overtemp flag — read by bms_balance.cpp ───────────────────
bool g_balance_overtemp = false;

// ============================================================================
// meas_cell_data — read voltages + temps + current into g_meas
// Called by task_measure before protection checks.
// ============================================================================
bool meas_cell_data(measurement_data_t *meas) {
    if (!ltc_read_voltages(meas)) {
        fault_set(static_cast<uint16_t>(Fault::SPI));
        Serial.println("[meas] SPI read failed — voltages");
        return false;
    }

    if (!ltc_read_temperatures(meas)) {
        fault_set(static_cast<uint16_t>(Fault::SPI));
        Serial.println("[meas] SPI read failed — temperatures");
        return false;
    }

    meas->current_a = ads_read_current();

    fault_clear(static_cast<uint16_t>(Fault::SPI));
    return true;
}

// ============================================================================
// Voltage helpers
// ============================================================================
float meas_min_cell_voltage(const measurement_data_t *meas) {
    float mn = meas->cell_v[0][0];
    for (int ic = 0; ic < TOTAL_IC; ic++)
        for (int c = 0; c < CELLS_PER_IC; c++)
            if (meas->cell_v[ic][c] < mn) mn = meas->cell_v[ic][c];
    return mn;
}

float meas_max_cell_voltage(const measurement_data_t *meas) {
    float mx = meas->cell_v[0][0];
    for (int ic = 0; ic < TOTAL_IC; ic++)
        for (int c = 0; c < CELLS_PER_IC; c++)
            if (meas->cell_v[ic][c] > mx) mx = meas->cell_v[ic][c];
    return mx;
}

// ============================================================================
// Protection checks
// ============================================================================

bool meas_check_overvoltage(const measurement_data_t *meas) {
    bool  cell_ov = false;
    float pack_v  = 0.0f;

    for (int ic = 0; ic < TOTAL_IC; ic++) {
        for (int c = 0; c < CELLS_PER_IC; c++) {
            float v = meas->cell_v[ic][c];
            pack_v += v;
            if (v >= CELL_OV_V) {
                cell_ov = true;
                Serial.printf("[meas] Cell OV: IC%d cell%d = %.4f V\n", ic + 1, c + 1, v);
            }
        }
    }

    if (cell_ov) return true;

    if (pack_v >= PACK_OV_V) {
        Serial.printf("[meas] Pack OV: %.2f V\n", pack_v);
        return true;
    }

    return false;
}

bool meas_check_undervoltage(const measurement_data_t *meas) {
    bool  cell_uv = false;
    float pack_v  = 0.0f;

    for (int ic = 0; ic < TOTAL_IC; ic++) {
        for (int c = 0; c < CELLS_PER_IC; c++) {
            float v = meas->cell_v[ic][c];
            pack_v += v;
            if (v <= CELL_UV_V) {
                cell_uv = true;
                Serial.printf("[meas] Cell UV: IC%d cell%d = %.4f V\n", ic + 1, c + 1, v);
            }
        }
    }

    if (cell_uv) return true;

    if (pack_v <= PACK_UV_V) {
        Serial.printf("[meas] Pack UV: %.2f V\n", pack_v);
        return true;
    }

    return false;
}

bool meas_check_overcurrent(const measurement_data_t *meas) {
    const float a = meas->current_a;

    if (a > 0.0f) {
        // ── Discharge ────────────────────────────────────────────────────────
        if (a >= CURR_DSCHG_CONT_A) {
            Serial.printf("[meas] OC_DSG cont: %.1f A (limit %.0f A)\n",
                          a, CURR_DSCHG_CONT_A);
            s_oc_peak_start_ms = 0;   // reset so peak timer is clean on next entry
            return true;
        }
        if (a >= CURR_DSCHG_PEAK_A) {
            // Reset timer if we just crossed from charge direction
            if (s_oc_peak_is_charge) {
                s_oc_peak_start_ms  = 0;
                s_oc_peak_is_charge = false;
            }
            if (s_oc_peak_start_ms == 0) {
                s_oc_peak_start_ms = millis();
            } else if ((millis() - s_oc_peak_start_ms) > (uint32_t)CURR_PEAK_MS) {
                Serial.printf("[meas] OC_DSG peak: %.1f A held > %d ms\n",
                              a, CURR_PEAK_MS);
                s_oc_peak_start_ms = 0;
                return true;
            }
        } else {
            s_oc_peak_start_ms = 0;
        }

    } else if (a < 0.0f) {
        // ── Charge ───────────────────────────────────────────────────────────
        const float abs_a = -a;

        if (abs_a >= CURR_CHG_CONT_A) {
            Serial.printf("[meas] OC_CHG cont: %.1f A (limit %.0f A)\n",
                          abs_a, CURR_CHG_CONT_A);
            s_oc_peak_start_ms = 0;
            return true;
        }
        if (abs_a >= CURR_CHG_PEAK_A) {
            // Reset timer if we just crossed from discharge direction
            if (!s_oc_peak_is_charge) {
                s_oc_peak_start_ms  = 0;
                s_oc_peak_is_charge = true;
            }
            if (s_oc_peak_start_ms == 0) {
                s_oc_peak_start_ms = millis();
            } else if ((millis() - s_oc_peak_start_ms) > (uint32_t)CURR_PEAK_MS) {
                Serial.printf("[meas] OC_CHG peak: %.1f A held > %d ms\n",
                              abs_a, CURR_PEAK_MS);
                s_oc_peak_start_ms = 0;
                return true;
            }
        } else {
            s_oc_peak_start_ms = 0;
        }

    } else {
        s_oc_peak_start_ms = 0;
    }

    return false;
}

bool meas_check_overtemp(const measurement_data_t *meas, uint8_t *ot_ch_out) {
    if (ot_ch_out) *ot_ch_out = 0xFF;

    float sum     = 0.0f;
    int   valid_n = 0;

    for (int i = 0; i < NUM_TEMP_SENSORS; i++) {
        const float t = meas->temps[i];
        if (t < -50.0f) continue;

        sum += t;
        valid_n++;

        if (t >= (float)TEMP_CUTOFF_C) {
            Serial.printf("[meas] OT cutoff: sensor %d = %.1f °C\n", i, t);
            if (ot_ch_out) *ot_ch_out = (uint8_t)i;
            return true;
        }
    }

    if (valid_n > 0) {
        float avg = sum / (float)valid_n;
        if (avg >= (float)TEMP_WARN_C)
            Serial.printf("[meas] Temp WARN: avg = %.1f °C\n", avg);
    }

    return false;
}

bool meas_check_balance_overtemp(const measurement_data_t *meas, uint8_t *ot_ch_out) {
    static const float BAL_OT_CLEAR_HYST_C = 3.0f;

    if (ot_ch_out) *ot_ch_out = 0xFF;

    for (int i = 0; i < NUM_TEMP_SENSORS; i++) {
        const float t = meas->temps[i];
        if (t < -50.0f) continue;
        if (t >= (float)TEMP_WARN_C) {
            Serial.printf("[meas] Balance OT pause: sensor %d = %.1f °C\n", i, t);
            if (ot_ch_out) *ot_ch_out = (uint8_t)i;
            g_balance_overtemp = true;
            return true;
        }
    }

    bool all_cool = true;
    for (int i = 0; i < NUM_TEMP_SENSORS; i++) {
        const float t = meas->temps[i];
        if (t < -50.0f) continue;
        if (t >= ((float)TEMP_WARN_C - BAL_OT_CLEAR_HYST_C)) {
            all_cool = false;
            break;
        }
    }
    if (all_cool) g_balance_overtemp = false;

    return false;
}