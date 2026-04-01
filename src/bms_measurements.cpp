#include "Arduino.h"
#include <SPI.h>
#include "bms_config.h"


// ── External globals 
extern float    g_cellV[NUM_CELLS];
extern uint16_t g_cellRaw[NUM_CELLS];
extern float    g_tempC[10];
extern float    g_amps;
extern uint16_t g_faultRegister;
extern bool     g_balanceCells[NUM_CELLS];
 
// ── Fault helpers 
extern void fault_set(uint16_t bit);
extern void fault_clear(uint16_t bit);
 
// ── LTC / ADS drivers 
extern bool  ltc_read_voltages(bool bal[NUM_CELLS]);
extern bool  ltc_read_temperatures(float tempC[10], bool bal[NUM_CELLS]);
extern float ads_read_current();
 
// ── Module-level peak current timer ──────────────────────────────────────────
// Tracks how long a peak-level overcurrent has been sustained.
// Reset to 0 whenever current drops back below the peak threshold.
static uint32_t s_oc_peak_start_ms = 0;

void meas_run_cycle() {
    meas_cell_data();
}

void meas_check_overvoltage() {
    static const float CELL_OV_HYST_V = 0.05f;
    static const float PACK_OV_HYST_V = 1.00f;
 
    bool  cell_ov = false;
    float pack_v  = 0.0f;
 
    for (int i = 0; i < NUM_CELLS; i++) {
        pack_v += g_cellV[i];
        if (g_cellV[i] >= CELL_OV_V) {
            cell_ov = true;
            Serial.printf("[meas] Cell OV: cell %2d = %.4f V\n", i + 1, g_cellV[i]);
        }
    }
 
    // Per-cell OV
    if (cell_ov) {
        fault_set(FAULT_CELL_OV);
    } else if (meas_max_cell_voltage() < (CELL_OV_V - CELL_OV_HYST_V)) {
        fault_clear(FAULT_CELL_OV);
    }
 
    // Pack OV
    if (pack_v >= PACK_OV_V) {
        fault_set(FAULT_PACK_OV);
        Serial.printf("[meas] Pack OV: %.2f V\n", pack_v);
    } else if (pack_v < (PACK_OV_V - PACK_OV_HYST_V)) {
        fault_clear(FAULT_PACK_OV);
    }
}

void meas_check_undervoltage() {
    static const float CELL_UV_HYST_V = 0.10f;
    static const float PACK_UV_HYST_V = 1.00f;
 
    bool  cell_uv = false;
    float pack_v  = 0.0f;
 
    for (int i = 0; i < NUM_CELLS; i++) {
        pack_v += g_cellV[i];
        if (g_cellV[i] <= CELL_UV_V) {
            cell_uv = true;
            Serial.printf("[meas] Cell UV: cell %2d = %.4f V\n", i + 1, g_cellV[i]);
        }
    }
 
    // Per-cell UV
    if (cell_uv) {
        fault_set(FAULT_CELL_UV);
    } else if (meas_min_cell_voltage() > (CELL_UV_V + CELL_UV_HYST_V)) {
        fault_clear(FAULT_CELL_UV);
    }
 
    // Pack UV
    if (pack_v <= PACK_UV_V) {
        fault_set(FAULT_PACK_UV);
        Serial.printf("[meas] Pack UV: %.2f V\n", pack_v);
    } else if (pack_v > (PACK_UV_V + PACK_UV_HYST_V)) {
        fault_clear(FAULT_PACK_UV);
    }
}

void meas_check_overcurrent() {
    static const float DSG_CONT_HYST_A = 5.0f;
    static const float CHG_CONT_HYST_A = 2.0f;
 
    g_amps = ads_read_current();
    const float a = g_amps;
 
    if (a > 0.0f) {
        // ── Discharge ────────────────────────────────────────────────────────
        if (a >= CURR_DSCHG_CONT_A) {
            fault_set(FAULT_OC_DSG);
            Serial.printf("[meas] OC_DSG cont: %.1f A (limit %.0f A)\n",
                          a, CURR_DSCHG_CONT_A);
        } else if (a < (CURR_DSCHG_CONT_A - DSG_CONT_HYST_A)) {
            fault_clear(FAULT_OC_DSG);
        }
 
        if (a >= CURR_DSCHG_PEAK_A) {
            if (s_oc_peak_start_ms == 0) {
                s_oc_peak_start_ms = millis();
            } else if ((millis() - s_oc_peak_start_ms) > (uint32_t)CURR_PEAK_MS) {
                fault_set(FAULT_OC_DSG);
                Serial.printf("[meas] OC_DSG peak: %.1f A held > %d ms\n",
                              a, CURR_PEAK_MS);
            }
        } else {
            s_oc_peak_start_ms = 0;
        }
 
    } else if (a < 0.0f) {
        // ── Charge ───────────────────────────────────────────────────────────
        const float abs_a = -a;
 
        if (abs_a >= CURR_CHG_CONT_A) {
            fault_set(FAULT_OC_CHG);
            Serial.printf("[meas] OC_CHG cont: %.1f A (limit %.0f A)\n",
                          abs_a, CURR_CHG_CONT_A);
        } else if (abs_a < (CURR_CHG_CONT_A - CHG_CONT_HYST_A)) {
            fault_clear(FAULT_OC_CHG);
        }
 
        if (abs_a >= CURR_CHG_PEAK_A) {
            if (s_oc_peak_start_ms == 0) {
                s_oc_peak_start_ms = millis();
            } else if ((millis() - s_oc_peak_start_ms) > (uint32_t)CURR_PEAK_MS) {
                fault_set(FAULT_OC_CHG);
                Serial.printf("[meas] OC_CHG peak: %.1f A held > %d ms\n",
                              abs_a, CURR_PEAK_MS);
            }
        } else {
            s_oc_peak_start_ms = 0;
        }
 
    } else {
        // Near-zero current — reset peak timer
        s_oc_peak_start_ms = 0;
    }
}


void meas_check_overtemp() {
    static const float OT_CLEAR_HYST_C = 5.0f;
 
    bool  any_cutoff = false;
    float sum        = 0.0f;
    int   valid_n    = 0;
 
    for (int i = 0; i < 10; i++) {
        const float t = g_tempC[i];
        if (t < -50.0f) continue;   // open/short NTC — skip
 
        sum += t;
        valid_n++;
 
        if (t >= (float)TEMP_CUTOFF_C) {
            any_cutoff = true;
            Serial.printf("[meas] OT cutoff: sensor %d = %.1f °C\n", i, t);
        }
    }
 
    if (any_cutoff) {
        fault_set(FAULT_OT);
    } else {
        bool all_clear = true;
        for (int i = 0; i < 10; i++) {
            if (g_tempC[i] < -50.0f) continue;
            if (g_tempC[i] >= ((float)TEMP_CUTOFF_C - OT_CLEAR_HYST_C)) {
                all_clear = false;
                break;
            }
        }
        if (all_clear) fault_clear(FAULT_OT);
    }
 
    // Warning log — average threshold, no fault bit
    if (valid_n > 0) {
        float avg = sum / (float)valid_n;
        if (avg >= (float)TEMP_WARN_C) {
            Serial.printf("[meas] Temp WARN: avg = %.1f °C (warn at %.0f °C)\n",
                          avg, (float)TEMP_WARN_C);
        }
    }
}

bool g_balance_overtemp = false;
 
void meas_check_balance_overtemp() {
    static const float BAL_OT_CLEAR_HYST_C = 3.0f;
 
    bool any_warm = false;
    for (int i = 0; i < 10; i++) {
        if (g_tempC[i] < -50.0f) continue;
        if (g_tempC[i] >= (float)TEMP_WARN_C) {
            any_warm = true;
            Serial.printf("[meas] Balance OT pause: sensor %d = %.1f °C\n",
                          i, g_tempC[i]);
            break;
        }
    }
 
    if (any_warm) {
        g_balance_overtemp = true;
    } else {
        bool all_cool = true;
        for (int i = 0; i < 10; i++) {
            if (g_tempC[i] < -50.0f) continue;
            if (g_tempC[i] >= ((float)TEMP_WARN_C - BAL_OT_CLEAR_HYST_C)) {
                all_cool = false;
                break;
            }
        }
        if (all_cool) g_balance_overtemp = false;
    }
}

void meas_cell_data() {
    bool volt_ok = ltc_read_voltages(g_balanceCells);
    bool temp_ok = ltc_read_temperatures(g_tempC, g_balanceCells);
 
    if (!volt_ok || !temp_ok) {
        fault_set(FAULT_SPI);
        Serial.printf("[meas] SPI read failed — volt_ok=%d  temp_ok=%d\n",
                      volt_ok, temp_ok);
        return;
    }

    extern float    _cellV[NUM_CELLS];
    extern uint16_t _cellRaw[NUM_CELLS];
    for (int i = 0; i < NUM_CELLS; i++) {
        g_cellV[i]   = _cellV[i];
        g_cellRaw[i] = _cellRaw[i];
    }
 
    fault_clear(FAULT_SPI);
}


float meas_min_cell_voltage() {
     float mn = g_cellV[0];
    for (int i = 1; i < NUM_CELLS; i++) {
        if (g_cellV[i] < mn) mn = g_cellV[i];
    }
    return mn;
}
 
float meas_max_cell_voltage() {
    float mx = g_cellV[0];
    for (int i = 1; i < NUM_CELLS; i++) {
        if (g_cellV[i] > mx) mx = g_cellV[i];
    }
    return mx;
}
