// ============================================================================
// bms_balance.cpp — Cell balancing logic
//
// All functions take measurement_data_t* — no direct global access.
// g_balance_overtemp is still read here as it is a hardware-state flag
// set by meas_check_balance_overtemp, not a measurement value.
// ============================================================================

#include <Arduino.h>
#include <SPI.h>
#include "bms_config.h"
#include "bms_types.h"
#include "bms_fault.h"
#include "ltc_spi.h"
#include "bms_balance.h"

// ── Balance overtemp flag (set by meas_check_balance_overtemp) ───────────────
extern bool g_balance_overtemp;

// ── Internal helpers ──────────────────────────────────────────────────────────
//
// Fix #8: build_safe_config previously hardcoded gpio_pulldown[g] = false for
// all 5 GPIOs. On IC1, GPIO1–5 are the thermistor mux lines — if any of those
// pull-downs are needed by ltc_read_temperatures, every balance_apply() and
// balance_stop() call was silently clobbering the temperature measurement config.
//
// The fix passes a caller-supplied gpio_pulldown array. Callers that have a
// live config (from ltc_read_config) pass those bits through unchanged.
// balance_stop() has no live config to hand, so it passes all-false, which is
// safe — it only writes DCC=0 to stop balancing; temperature reads are not
// active while balance_stop() is called (FSM is leaving BALANCE or entering
// FAULT/SLEEP, where the LTC is about to be put to sleep anyway).
//
// If your schematic uses the LTC internal pull-downs for any NTC divider,
// replace the balance_stop() call below with the actual pull-down config.
static void build_safe_config(LtcConfig cfg[2],
                               uint16_t dcc0, uint16_t dcc1,
                               const bool gpio_pd[5])
{
    for (int ic = 0; ic < TOTAL_IC; ic++) {
        cfg[ic].refon  = true;
        cfg[ic].adcopt = false;
        cfg[ic].dcto   = 0x00;
        for (int g = 0; g < 5; g++) cfg[ic].gpio_pulldown[g] = gpio_pd[g];
        cfg[ic].vuv = (uint16_t)((CELL_UV_RAW / 16u) - 1u);
        cfg[ic].vov = (uint16_t) (CELL_OV_RAW / 16u);
    }
    cfg[0].dcc = dcc0;
    cfg[1].dcc = dcc1;
}

// ============================================================================
// balance_compute_mask
// Marks cells in meas->balance_cells that are above min + threshold.
// ============================================================================
void balance_compute_mask(measurement_data_t *meas) {
    // Find global minimum raw count
    uint16_t min_raw = meas->cell_raw[0][0];
    for (int ic = 0; ic < TOTAL_IC; ic++)
        for (int c = 0; c < CELLS_PER_IC; c++)
            if (meas->cell_raw[ic][c] < min_raw)
                min_raw = meas->cell_raw[ic][c];

    uint16_t threshold = min_raw + BAL_THRESHOLD_UV;

    for (int ic = 0; ic < TOTAL_IC; ic++) {
        for (int c = 0; c < CELLS_PER_IC; c++) {
            bool bal = (meas->cell_raw[ic][c] > threshold);
            meas->balance_cells[ic][c] = bal;
            if (bal) {
                Serial.printf("[bal] IC%d cell%d flagged: %u counts > min %u + %u\n",
                              ic + 1, c + 1,
                              meas->cell_raw[ic][c], min_raw, BAL_THRESHOLD_UV);
            }
        }
    }
}

// ============================================================================
// balance_satisfied
// Returns true if max-min delta is within threshold.
// ============================================================================
bool balance_satisfied(const measurement_data_t *meas) {
    uint16_t min_raw = meas->cell_raw[0][0];
    uint16_t max_raw = meas->cell_raw[0][0];

    for (int ic = 0; ic < TOTAL_IC; ic++) {
        for (int c = 0; c < CELLS_PER_IC; c++) {
            uint16_t r = meas->cell_raw[ic][c];
            if (r < min_raw) min_raw = r;
            if (r > max_raw) max_raw = r;
        }
    }

    uint16_t delta = max_raw - min_raw;
    Serial.printf("[bal] Delta: %u counts (%.1f mV) — %s\n",
                  delta, delta * 0.1f,
                  delta < BAL_THRESHOLD_UV ? "SATISFIED" : "balancing");
    return delta < BAL_THRESHOLD_UV;
}

// ============================================================================
// balance_apply
// Pushes balance_cells mask from meas into LTC hardware config.
// Skipped if g_balance_overtemp is active.
// ============================================================================
ltc_status_t balance_apply(const measurement_data_t *meas) {
    if (g_balance_overtemp) {
        Serial.println("[bal] balance_apply() skipped — overtemp pause active");
        return LTC_OK;
    }

    uint16_t dcc[TOTAL_IC] = {};
    for (int ic = 0; ic < TOTAL_IC; ic++) {
        for (int c = 0; c < CELLS_PER_IC; c++) {
            if (meas->balance_cells[ic][c]) {
                dcc[ic] |= (uint16_t)(1u << c);
            }
        }
        Serial.printf("[bal] IC%d DCC mask: 0x%03X\n", ic + 1, dcc[ic]);
    }

    // Fix #8: read the current LTC config so we can preserve the GPIO
    // pull-down bits that ltc_read_temperatures relies on. Writing
    // gpio_pulldown=false unconditionally clobbered the thermistor mux
    // configuration on every balance write.
    LtcConfig live[TOTAL_IC];
    bool gpio_pd[5] = {false, false, false, false, false};
    if (ltc_read_config(live)) {
        // Use IC1's GPIO state as the authoritative source — IC1 owns the
        // thermistor GPIOs. If read fails we fall back to all-false, which
        // is safer than using stale data.
        for (int g = 0; g < 5; g++) gpio_pd[g] = live[0].gpio_pulldown[g];
    } else {
        Serial.println("[bal] balance_apply: ltc_read_config failed — GPIO pull-downs set to false");
    }

    LtcConfig cfg[TOTAL_IC];
    build_safe_config(cfg, dcc[0], dcc[1], gpio_pd);
    ltc_write_config(cfg);
    return LTC_OK;
}

// ============================================================================
// balance_stop
// Clears balance_cells in meas and pushes zeroed DCC to hardware.
// ============================================================================
void balance_stop(measurement_data_t *meas) {
    if (meas) {
        for (int ic = 0; ic < TOTAL_IC; ic++)
            for (int c = 0; c < CELLS_PER_IC; c++)
                meas->balance_cells[ic][c] = false;
    }

    // Fix #8: balance_stop is called when leaving BALANCE, entering FAULT,
    // or entering SLEEP. In all three cases the LTC is either going to sleep
    // or the FSM is faulted — temperature reads are not active, so passing
    // all-false for GPIO pull-downs here is safe. If your schematic uses the
    // LTC internal pull-downs for NTC dividers, substitute the real config.
    static const bool gpio_pd_off[5] = {false, false, false, false, false};
    LtcConfig cfg[TOTAL_IC];
    build_safe_config(cfg, 0x0000, 0x0000, gpio_pd_off);
    ltc_write_config(cfg);

    Serial.println("[bal] balance_stop() — all DCC bits cleared");
}