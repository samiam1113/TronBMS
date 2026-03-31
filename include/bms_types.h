#ifndef BMS_TYPES_H
#define BMS_TYPES_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================
// ENUMS
// ============================================================

typedef enum {
    BMS_OK,
    BMS_ERR_GENERIC
} bms_err_t;

typedef enum {
    STATE_INIT,
    STATE_STARTUP,
    STATE_NORMAL,
    STATE_BALANCE,
    STATE_SLEEP,
    STATE_FAULT
} bms_state_t;

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

typedef enum {
    FAULT_NONE,
    FAULT_OVERVOLTAGE,
    FAULT_UNDERVOLTAGE,
    FAULT_OVERCURRENT,
    FAULT_OVERTEMP,
    FAULT_BAL_OVERTEMP,
    FAULT_SPI_LTC,
    FAULT_SPI_ADS,
    FAULT_STARTUP
} fault_code_t;

// ============================================================
// STRUCTS
// ============================================================

typedef struct {
    float cell_v[2][10];      // Cell voltages in volts indexed [ic][cell]
    float temps[4];           // Temperature readings in celsius all on IC1
    float current_a;          // Pack current in amps from ADS131M02
} measurement_data_t;

typedef struct {
    uint8_t refon;            // REFON bit
    uint8_t adcopt;           // ADCOPT bit
    uint8_t dten;             // Discharge timer enable
    uint16_t vuv;             // Undervoltage comparison voltage
    uint16_t vov;             // Overvoltage comparison voltage
    uint16_t dcc;             // Discharge cell bitmask
} ltc_config_t;

typedef struct {
    uint16_t sc;              // Sum of cells
    uint16_t itmp;            // Internal die temperature
    uint16_t va;              // Analog supply voltage
    uint16_t vd;              // Digital supply voltage
    uint8_t flags;            // Status flags (OV/UV per cell)
} ltc_status_reg_t;

typedef struct {
    fault_code_t code;
    bms_state_t state_at_fault;
    measurement_data_t meas;  // Full measurement snapshot at time of fault
} fault_snapshot_t;

typedef struct {
    uint16_t dcc_mask[2];     // Discharge cell bitmask per IC
    uint16_t target_v;        // Target balance voltage in LTC6811 register units (100µV per LSB)
} balance_state_t;

#endif // BMS_TYPES_H