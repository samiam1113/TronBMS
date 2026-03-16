#pragma once

//  BMS PROTECTION THRESHOLDS

// Temperature
#define TEMP_WARN_C         60      // Warn if average of all sensors >= 60°C
#define TEMP_CUTOFF_C       70      // Hard cutoff if ANY single sensor >= 70°C

// Cell Voltage (LTC6811 raw counts, 1 count = 100µV) 
#define CELL_UV_RAW         27500u  // 2.75V under-voltage cutoff
#define CELL_OV_RAW         41000u  // 4.1V  over-voltage cutoff

// Pack Voltage
#define PACK_UV_V           55.0f   // 55V pack under-voltage
#define PACK_OV_V           82.0f   // 82V pack over-voltage

// Current - Discharge
#define CURR_DSCHG_PEAK_A   600.0f  // 600A max, < 1 second
#define CURR_DSCHG_CONT_A   300.0f  // 300A continuous

// Current - Charge
#define CURR_CHG_PEAK_A     40.0f   // 40A  max charge
#define CURR_CHG_CONT_A     15.0f   // 15A  continuous charge

// Cell Balancing
#define BAL_THRESHOLD_UV    200u    // Balance if cell delta >= 2mV (200 counts)

// ADS131M02 Current Sensing 
#define ADS_GAIN            4
#define ADS_SAMPLE_US       1       // 1µs between current samples

// Shunt resistor value 
#define SHUNT_OHMS          0.0004f  

// Convert ADS131 raw count to amps:
//   Full scale = ±VREF/GAIN across shunt
//   VREF = 1.2V (ADS131M02 internal ref), GAIN = 4
//   Full scale input = 1.2V / 4 = 0.3V
//   Full scale current = 0.3V / SHUNT_OHMS
//   Count to amps = FULL_SCALE_A / 2^23
#define ADS_VREF            1.2f
#define ADS_FULL_SCALE_V    (ADS_VREF / ADS_GAIN)               // 0.3V
#define ADS_FULL_SCALE_A    (ADS_FULL_SCALE_V / SHUNT_OHMS)     // update per shunt
#define ADS_COUNTS_TO_AMPS  (ADS_FULL_SCALE_A / 8388608.0f)     // per count (2^23)
