#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "bms_config.h"

// ============================================================================
// bms_hardware.h — ADS131M02 current sensor and gate driver declarations
// ============================================================================

// ── ADS131M02 ────────────────────────────────────────────────────────────────

/**
 * ads_reset — assert then deassert ADS131 /RESET pin.
 * Wait at least 1ms before any SPI traffic after calling this.
 */
void ads_reset(void);

/**
 * ads_configure — configure ADS131M02 (formerly configureADS131M02).
 * Writes CLOCK and GAIN1 registers, reads back and verifies.
 * Returns true if both register readbacks match expected values.
 */
bool ads_configure(void);

/**
 * ads_checkid — read the ADS131M02 ID register and compare to ADS_EXPECTED_ID.
 * Returns true on match.
 */
bool ads_checkid(void);

/**
 * ads_read_current — convenience wrapper: read raw count and convert to amps.
 * Returns pack current in amps. Positive = discharge, negative = charge.
 */
float ads_read_current(void);

/**
 * ads_read_raw — read CH1 raw 24-bit signed count, return as int16_t (top 16 bits).
 */
int32_t ads_read_raw(void);

/**
 * ads_counts_to_amps — scale a raw int16_t count to amps using ADS_FULL_SCALE_A.
 */
float ads_counts_to_amps(int32_t raw);

// ── Gate driver ──────────────────────────────────────────────────────────────

/**
 * gate_driver_selftest — pulse CHG and DSG pins high/low in sequence.
 * Returns true (no feedback pin on this design — extend if feedback added).
 */
bool gate_driver_selftest(void);

/**
 * gate_driver_enable — assert both CHG and DSG gate drive pins HIGH.
 */
void gate_driver_enable(void);

/**
 * gate_driver_disable — de-assert both CHG and DSG gate drive pins LOW.
 */
void gate_driver_disable(void);

// ── Peripheral init ──────────────────────────────────────────────────────────

/**
 * bms_gpio_init — configure all GPIO pins used by the BMS.
 * Call before bms_spi_init().
 */
void bms_gpio_init(void);

/**
 * bms_spi_init — initialise VSPI (ADS131) and HSPI (LTC6811) peripherals.
 * Call once from setup() before any SPI driver calls.
 */
void bms_spi_init(void);