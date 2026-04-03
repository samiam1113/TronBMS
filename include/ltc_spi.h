#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "bms_config.h"
#include "bms_types.h"

// ============================================================================
// ltc_spi.h — LTC6811-1 isoSPI driver declarations
// ============================================================================

// ----------------------------------------------------------------------------
// LtcConfig — mirrors CFGRA register layout
// ----------------------------------------------------------------------------
struct LtcConfig {
    bool     refon;
    bool     adcopt;
    uint8_t  dcto;           // Discharge timer: 0x00 = disabled
    uint16_t vuv;            // Under-voltage comparison: (VUV / 16) - 1
    uint16_t vov;            // Over-voltage comparison:   VOV / 16
    uint16_t dcc;            // Discharge cell bitmask, bits [11:0] = C12..C1
    bool     gpio_pulldown[5]; // GPIO[5:1] pull-down enable
};

// ----------------------------------------------------------------------------
// LtcStatus — mirrors STATA + STATB register layout
// ----------------------------------------------------------------------------
struct LtcStatus {
    float   sc_v;       // Sum of cells voltage (V)
    float   itmp_c;     // Internal temperature (°C)
    float   va_v;       // Analog supply (V)
    float   vd_v;       // Digital supply (V)
    bool    c_uv[12];   // Cell under-voltage flags
    bool    c_ov[12];   // Cell over-voltage flags
    bool    muxfail;
    bool    thsd;       // Thermal shutdown
    bool    rev[4];     // Revision code bits
};

// ----------------------------------------------------------------------------
// CRC-15 helpers
// ----------------------------------------------------------------------------
uint16_t ltc_pec15_calc(uint8_t *data, uint8_t len);
bool     ltc_pec15_verify(uint8_t *data, uint8_t len, uint16_t received_pec);

// ----------------------------------------------------------------------------
// Wakeup / idle
// ----------------------------------------------------------------------------
void ltc_wakeup_sleep(void);   // Wake from SLEEP state (long CS pulse + delay)
void ltc_wakeup_idle(void);    // Wake from IDLE state (short dummy transfer)

// ----------------------------------------------------------------------------
// Config register access
// ----------------------------------------------------------------------------
void ltc_write_config(const LtcConfig cfg[2]);   // cfg[0]=IC1, cfg[1]=IC2
bool ltc_read_config(LtcConfig cfg_out[2]);

// ----------------------------------------------------------------------------
// ADC conversion
// ----------------------------------------------------------------------------
bool ltc_start_adc_conversion(bool cells, bool gpio);

// ----------------------------------------------------------------------------
// Measurement reads — populate measurement_data_t fields
// ----------------------------------------------------------------------------
bool ltc_read_voltages(measurement_data_t *meas);
bool ltc_read_temperatures(measurement_data_t *meas);

// ----------------------------------------------------------------------------
// Status register read
// ----------------------------------------------------------------------------
bool ltc_read_status(LtcStatus status[2]);

// ----------------------------------------------------------------------------
// Communications self-test
// ----------------------------------------------------------------------------
bool ltc_comms_test(void);
void ltc_scope_loop(void);   

// ----------------------------------------------------------------------------
// Balance cell control (low-level — prefer balance_apply() in bms_balance)
// ----------------------------------------------------------------------------
void ltc_set_balance(bool cells[20]);
void ltc_clear_balance(void);