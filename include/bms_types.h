#ifndef BMS_TYPES_H
#define BMS_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include "bms_config.h"

// ============================================================
// ENUMS
// ============================================================

typedef enum {
    BMS_OK,
    BMS_ERR_GENERIC
} bms_err_t;

typedef enum {
    LTC_OK,
    LTC_ERR_PEC,
    LTC_ERR_SPI,
    LTC_ERR_TIMEOUT
} ltc_status_t;

typedef enum {
    ADS_OK,
    ADS_ERR_ID,
    ADS_ERR_SPI
} ads_status_t;

// ============================================================
// STRUCTS
// ============================================================

// Single source of truth for all pack measurements.
// Replaces the scattered globals g_cellV, g_cellRaw, g_tempC,
// g_amps, and g_balanceCells.
typedef struct {
    float    cell_v[TOTAL_IC][CELLS_PER_IC];      // [ic][cell] volts
    uint16_t cell_raw[TOTAL_IC][CELLS_PER_IC];    // [ic][cell] LTC raw counts (100µV/count)
    bool     balance_cells[TOTAL_IC][CELLS_PER_IC]; // [ic][cell] balance enable flags
    float    temps[5];                            // NTC readings in °C 
    float    current_a;                            // Pack current in amps
} measurement_data_t;

typedef struct {
    uint8_t  refon;
    uint8_t  adcopt;
    uint8_t  dten;
    uint16_t vuv;
    uint16_t vov;
    uint16_t dcc;
} ltc_config_t;

typedef struct {
    uint16_t sc;
    uint16_t itmp;
    uint16_t va;
    uint16_t vd;
    uint8_t  flags;
} ltc_status_reg_t;

typedef struct {
    uint16_t dcc_mask[TOTAL_IC];  // Discharge cell bitmask per IC
    uint16_t target_v;            // Target balance voltage in LTC counts (100µV/count)
} balance_state_t;

#endif // BMS_TYPES_H