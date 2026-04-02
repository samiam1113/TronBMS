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

// ── LTC driver (defined in ltc_spi.cpp) ──────────────────────────────────────
struct LtcConfig {
    bool     gpio_pulldown[5];
    bool     refon;
    bool     adcopt;
    uint16_t vuv;
    uint16_t vov;
    uint16_t dcc;
    uint8_t  dcto;
};

extern ltc_status_t ltc_write_config(const LtcConfig cfg[2]);

// ── Balance overtemp flag (set by meas_check_balance_overtemp) ───────────────
extern bool g_balance_overtemp;

// ── Internal helpers ──────────────────────────────────────────────────────────
static void build_safe_config(LtcConfig cfg[2], uint16_t dcc0, uint16_t dcc1) {
    for (int ic = 0; ic < TOTAL_IC; ic++) {
        cfg[ic].refon  = true;
        cfg[ic].adcopt = false;
        cfg[ic].dcto   = 0x00;
        for (int g = 0; g < 5; g++) cfg[ic].gpio_pulldown[g] = false;
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

    LtcConfig cfg[TOTAL_IC];
    build_safe_config(cfg, dcc[0], dcc[1]);
    return ltc_write_config(cfg);
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

    LtcConfig cfg[TOTAL_IC];
    build_safe_config(cfg, 0x0000, 0x0000);
    ltc_write_config(cfg);

    Serial.println("[bal] balance_stop() — all DCC bits cleared");
}