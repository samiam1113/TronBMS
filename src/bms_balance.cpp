#include <Arduino.h>
#include <SPI.h>
#include "bms_config.h"

// ── External globals ──────────────────────────────────────────────────────────
extern uint16_t g_cellRaw[NUM_CELLS];       // from bms_fault.cpp
extern bool     g_balanceCells[NUM_CELLS];  // from bms_fault.cpp
extern bool     g_balance_overtemp;         // from bms_measurements.cpp
 
// ── LTC driver types and function (from ltc_spi.cpp) ─────────────────────────
struct LtcConfig {
    bool     gpio_pulldown[5];
    bool     refon;
    bool     adcopt;
    uint16_t vuv;
    uint16_t vov;
    uint16_t dcc;
    uint8_t  dcto;
};
 
extern void ltc_write_config(const LtcConfig cfg[2]);

void balance_compute_mask() {
    // Find minimum raw count across all cells
    uint16_t min_raw = g_cellRaw[0];
    for (int i = 1; i < NUM_CELLS; i++) {
        if (g_cellRaw[i] < min_raw) min_raw = g_cellRaw[i];
    }
 
    // Mark cells that need bypassing
    uint16_t threshold = min_raw + BAL_THRESHOLD_UV;
    for (int i = 0; i < NUM_CELLS; i++) {
        g_balanceCells[i] = (g_cellRaw[i] > threshold);
    }
 
    // Debug — print any cells being balanced this tick
    for (int i = 0; i < NUM_CELLS; i++) {
        if (g_balanceCells[i]) {
            Serial.printf("[bal] Cell %2d flagged: %u counts > min %u + %u\n",
                          i + 1, g_cellRaw[i], min_raw, BAL_THRESHOLD_UV);
        }
    }
}

bool balance_satisfied() {
    uint16_t min_raw = g_cellRaw[0];
    uint16_t max_raw = g_cellRaw[0];
    for (int i = 1; i < NUM_CELLS; i++) {
        if (g_cellRaw[i] < min_raw) min_raw = g_cellRaw[i];
        if (g_cellRaw[i] > max_raw) max_raw = g_cellRaw[i];
    }
    uint16_t delta = max_raw - min_raw;
    Serial.printf("[bal] Delta: %u counts (%.1f mV) — %s\n",
                  delta, delta * 0.1f,
                  delta < BAL_THRESHOLD_UV ? "SATISFIED" : "balancing");
    return delta < BAL_THRESHOLD_UV;
}

void balance_apply() {
    if (g_balance_overtemp) {
        Serial.println("[bal] balance_apply() skipped — overtemp pause active");
        return;
    }
 
    LtcConfig cfg[2];
 
    for (int ic = 0; ic < 2; ic++) {
        cfg[ic].refon          = true;
        cfg[ic].adcopt         = false;
        cfg[ic].dcto           = 0x00;   // discharge timer disabled
        cfg[ic].dcc            = 0x0000;
        for (int g = 0; g < 5; g++) cfg[ic].gpio_pulldown[g] = false;
 
        // LTC UV/OV threshold registers use a different scale:
        cfg[ic].vuv = (uint16_t)((CELL_UV_RAW / 16u) - 1u);
        cfg[ic].vov = (uint16_t) (CELL_OV_RAW / 16u);
 
        // Build DCC bitmask for this IC's 10 cells
        int base = ic * CELLS_PER_IC;   // 0 for IC1, 10 for IC2
        uint16_t dcc = 0;
        for (int c = 0; c < CELLS_PER_IC; c++) {
            if (g_balanceCells[base + c]) {
                dcc |= (uint16_t)(1u << c);
            }
        }
        cfg[ic].dcc = dcc;
 
        Serial.printf("[bal] IC%d DCC mask: 0x%03X\n", ic + 1, dcc);
    }
 
    ltc_write_config(cfg);
}

void balance_stop() {
    // Clear software mask
    for (int i = 0; i < NUM_CELLS; i++) {
        g_balanceCells[i] = false;
    }
 
    // Push zeroed config to hardware — REFON stays on, DCC all clear
    LtcConfig cfg[2];
    for (int ic = 0; ic < 2; ic++) {
        cfg[ic].refon          = true;
        cfg[ic].adcopt         = false;
        cfg[ic].dcto           = 0x00;
        cfg[ic].dcc            = 0x0000;
        for (int g = 0; g < 5; g++) cfg[ic].gpio_pulldown[g] = false;
        cfg[ic].vuv = (uint16_t)((CELL_UV_RAW / 16u) - 1u);
        cfg[ic].vov = (uint16_t) (CELL_OV_RAW / 16u);
    }
 
    ltc_write_config(cfg);
    Serial.println("[bal] balance_stop() — all DCC bits cleared");
}
