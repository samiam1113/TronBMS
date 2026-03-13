#pragma once
#include <Arduino.h>

// ── LTC6811-1  (HSPI) ───────────────────
#define LTC_SCLK_PIN    14
#define LTC_MISO_PIN    12
#define LTC_MOSI_PIN    13
#define LTC_CS_PIN       4
#define TOTAL_IC         2

// ── ADS131M02  (VSPI) ───────────────────
#define ADS_SCLK_PIN    18
#define ADS_MISO_PIN    19
#define ADS_MOSI_PIN    23
#define ADS_RST_PIN     21
#define ADS_CS_PIN       5   
#define ADS_EXPECTED_ID  0x2282

// ── UCC37322DR  (GPIO) ──────────────────
#define GATE_DSCHG_PIN  25
#define GATE_CHG_PIN    26
