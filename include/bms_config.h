#pragma once

// ============================================================================
// bms_config.h — Hardware pin assignments, protection thresholds, and
//                compile-time constants for TronBMS.
//
// bms_thresholds.h is superseded by this file — do not include both.
// ============================================================================

// ── Pack topology ─────────────────────────────────────────────────────────────
#define TOTAL_IC        2
#define CELLS_PER_IC    10
#define NUM_CELLS       20      // TOTAL_IC * CELLS_PER_IC

// ── Temperature thresholds ────────────────────────────────────────────────────
#define TEMP_WARN_C         60      // Balance pause / warning threshold (°C)
#define TEMP_CUTOFF_C       70      // Hard cutoff — any single sensor (°C)
#define NUM_TEMP_SENSORS    5

// ── Cell voltage (LTC6811 raw counts, 1 count = 100 µV) ──────────────────────
#define CELL_UV_RAW         27500u  // 2.75 V under-voltage cutoff
#define CELL_OV_RAW         41000u  // 4.10 V over-voltage cutoff
#define CELL_UV_V           (CELL_UV_RAW * 0.0001f)
#define CELL_OV_V           (CELL_OV_RAW * 0.0001f)

// ── Pack voltage ──────────────────────────────────────────────────────────────
#define PACK_UV_V           55.0f   // 55 V pack under-voltage
#define PACK_OV_V           82.0f   // 82 V pack over-voltage

// ── Current — discharge ───────────────────────────────────────────────────────
#define CURR_DSCHG_PEAK_A   600.0f  // 600 A max, < CURR_PEAK_MS
#define CURR_DSCHG_CONT_A   300.0f  // 300 A continuous

// ── Current — charge ──────────────────────────────────────────────────────────
#define CURR_CHG_PEAK_A     40.0f   // 40 A max charge peak
#define CURR_CHG_CONT_A     15.0f   // 15 A continuous charge
#define CURR_PEAK_MS        1000    // Peak current allowed duration (ms)

// ── Cell balancing ────────────────────────────────────────────────────────────
#define BAL_THRESHOLD_UV    200u    // Balance if max-min delta >= 2 mV (200 counts)
#define CELL_IMBALANCE_THRESHOLD_V 0.050f;  // 50 mV imbalance threshold


// ── ADS131M02-Q1 current sensing ──────────────────────────────────────────────
// Shunt: 0.4 mΩ  |  PGA gain: 4  |  Vref: 1.2 V (internal)
// Full-scale input = Vref / Gain = 1.2 / 4 = 0.300 V
// Full-scale current = 0.300 V / SHUNT_OHMS
// LSB (24-bit signed, 2^23 counts full-scale) = ADS_FULL_SCALE_A / 2^23
#define ADS_GAIN            4
#define ADS_VREF            1.2f
#define SHUNT_OHMS          0.0004f
#define ADS_FULL_SCALE_A    (ADS_VREF / ((float)ADS_GAIN * SHUNT_OHMS))  // 750 A
#define ADS_COUNTS_TO_AMPS  (ADS_FULL_SCALE_A / 8388608.0f)              // per count

#define ADS_EXPECTED_ID     0x2282  // ADS131M02-Q1 ID register
#define ADS_GAIN1_VAL       0x0040  // CH1 gain=4 (bits[8:6]=010), CH0 gain=1
#define ADS_CLOCK_VAL       0x030C  // OSR=1024, CH0+CH1 enabled, PWR=high-res

// ── NTC thermistor (Steinhart-Hart) ───────────────────────────────────────────
#define NTC_R25             10000.0f
#define NTC_BETA            3950.0f
#define NTC_RBIAS           10000.0f

// ── Pin assignments ───────────────────────────────────────────────────────────
// VSPI — ADS131M02 (CS tied to GND, managed in firmware)
#define VSPI_MISO           19
#define VSPI_MOSI           23
#define VSPI_SCLK           18
#define VSPI_SS             -1      // CS hardwired LOW — pass -1 to SPIClass::begin()

// HSPI — LTC6811-1 daisy chain
#define HSPI_MISO           12
#define HSPI_MOSI           13
#define HSPI_SCLK           14
#define HSPI_SS             4       // LTC6811 chain CS — managed manually

// Gate drivers
#define GATE_DSCHG_PIN      25
#define GATE_CHG_PIN        26

// ADS131M02 hardware reset
#define ADS_RESET_PIN       21
#define ADS_CLKIN_PIN       22
#define ADS_DRDY_PIN        32

// ── FreeRTOS event group bit definitions ──────────────────────────────────────
// Kept here so bms_config.h is the single include for all compile-time consts.
// Mirror definitions in bms_tasks.h are authoritative for the task API;
// these are provided so modules that only include bms_config.h still compile.
#define EVT_MEASURE_START       (1u << 0)   // FSM → task_measure: start cycle
#define EVT_FAULT_LTC           (1u << 1)   // SPI error on LTC6811
#define EVT_FAULT_ADS           (1u << 2)   // SPI error on ADS131
#define EVT_FAULT_OV            (1u << 3)   // Overvoltage detected
#define EVT_FAULT_UV            (1u << 4)   // Undervoltage detected
#define EVT_FAULT_OC            (1u << 5)   // Overcurrent detected
#define EVT_FAULT_OT            (1u << 6)   // Overtemperature detected
#define EVT_FAULT_BAL_OT        (1u << 7)   // Balance overtemperature pause
#define EVT_FAULT_TASK_STALL    (1u << 8)   // Task missed watchdog checkin
#define EVT_BALANCE_DONE        (1u << 9)   // task_balance signals completion
#define EVT_FAULT_ANY           (0x1FEu)    // OR of all fault event bits (1–8)