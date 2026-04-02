#include <Arduino.h>
#include <SPI.h>
#include "bms_config.h"

SPIClass *vspi = NULL;
SPIClass *hspi = NULL;

static const int ADS_SPI_CLK = 500000; 

// ── ADS131M02-Q1 commands and registers ──────────────────────────────────────
#define ADS_WAKEUP    0x0033
#define ADS_STANDBY   0x0022
#define ADS_NULL      0x0000
#define ADS_RESET     0x0011
#define ADS_REG_CLOCK 0x03
#define ADS_REG_GAIN1 0x04

static uint32_t adsXfer24(uint8_t b0, uint8_t b1, uint8_t b2) {
  uint8_t tx[3] = {b0, b1, b2};
  uint8_t rx[3] = {0, 0, 0};
  vspi->transferBytes(tx, rx, 3);
  return ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | rx[2];
}


void wakeupADS131M02() {
  vspi->beginTransaction(SPISettings(ADS_SPI_CLK, MSBFIRST, SPI_MODE1));
  adsXfer24(0x00, 0x33, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  vspi->endTransaction();
}

void sleepADS131M02() {
  vspi->beginTransaction(SPISettings(ADS_SPI_CLK, MSBFIRST, SPI_MODE1));
  adsXfer24(0x00, 0x22, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  vspi->endTransaction();
}

void ads_reset(){
    vspi->beginTransaction(SPISettings(ADS_SPI_CLK, MSBFIRST, SPI_MODE1));
    adsXfer24(0x00, 0x11, 0x00);
    adsXfer24(0x00, 0x00, 0x00);
    adsXfer24(0x00, 0x00, 0x00);
    adsXfer24(0x00, 0x00, 0x00);
    vspi->endTransaction();
}

bool configureADS131M02() {
  digitalWrite(ADS_RESET_PIN, LOW);
  delay(2);
  digitalWrite(ADS_RESET_PIN, HIGH);
  delay(1);

  vspi->beginTransaction(SPISettings(ADS_SPI_CLK, MSBFIRST, SPI_MODE1));

  // STANDBY
  adsXfer24(0x00, 0x22, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);

  // NULL — let STANDBY complete
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);

  // WAKEUP — response in w1
  uint32_t wakeStatus = adsXfer24(0x00, 0x33, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  Serial.printf("  WAKEUP STATUS=0x%04X\n", (wakeStatus >> 8));

  // NULL — let WAKEUP complete
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);

  // WREG CLOCK
  adsXfer24(0x61, 0x80, 0x00);
  adsXfer24(0x03, 0x0C, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);

  // WREG GAIN1
  adsXfer24(0x62, 0x00, 0x00);
  adsXfer24(0x00, 0x40, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);

  // RREG CLOCK — response comes back in w1 of the following frame
  adsXfer24(0xA1, 0x80, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);

  // Clock out response frame — CLOCK register in w1
  uint32_t clockRead = adsXfer24(0x00, 0x00, 0x00);  // w1: CLOCK register
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);

  // RREG GAIN1
  adsXfer24(0xA2, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);

  // Clock out response frame — GAIN1 register in w1
  uint32_t gainRead = adsXfer24(0x00, 0x00, 0x00);  // w1: GAIN1 register
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);

  vspi->endTransaction();

  // Responses are MSB-aligned in 24 bits — shift down to get 16-bit value
  uint16_t clockVal = (uint16_t)(clockRead >> 8);
  uint16_t gainVal  = (uint16_t)(gainRead  >> 8);

  Serial.printf("  ADS CLOCK reg: wrote 0x%04X, read 0x%04X %s\n",
    ADS_CLOCK_VAL, clockVal, clockVal == ADS_CLOCK_VAL ? "(OK)" : "(MISMATCH)");
  Serial.printf("  ADS GAIN1 reg: wrote 0x%04X, read 0x%04X %s\n",
    ADS_GAIN1_VAL, gainVal, gainVal == ADS_GAIN1_VAL ? "(OK)" : "(MISMATCH)");

  return (clockVal == ADS_CLOCK_VAL && gainVal == ADS_GAIN1_VAL);
}
// ════════════════════════════════════════════════════════════════════════════
//  ADS131M02-Q1 — CH1 current read
// ════════════════════════════════════════════════════════════════════════════

// Returns signed 16-bit raw count — convert with adsCounts_to_amps()
int16_t ads_read_raw() {
  vspi->beginTransaction(SPISettings(ADS_SPI_CLK, MSBFIRST, SPI_MODE1));
 
  adsXfer24(0x00, 0x33, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
 
  adsXfer24(0x00, 0x00, 0x00);                    // w1: STATUS
  adsXfer24(0x00, 0x00, 0x00);                    // w2: CH0 discard
  uint32_t ch1raw = adsXfer24(0x00, 0x00, 0x00);  // w3: CH1
  adsXfer24(0x00, 0x00, 0x00);                    // w4: CRC
 
  vspi->endTransaction();
 
  int32_t val = (int32_t)(ch1raw << 8) >> 8;  // sign extend 24→32
  return (int16_t)(val >> 8);                  // top 16 bits
}
 
float ads_counts_to_amps(int16_t raw) {
  return raw * ADS_FULL_SCALE_A;
}

float ads_read_current() {
    int16_t raw = ads_read_raw();
    return ads_counts_to_amps(raw);
}

bool ads_checkid() {
  vspi->beginTransaction(SPISettings(ADS_SPI_CLK, MSBFIRST, SPI_MODE1));
  uint32_t id = adsXfer24(0x20, 0x00, 0x00) >> 8;  // RREG ID
  vspi->endTransaction();
  Serial.printf("ADS131M02 ID: read 0x%04X, expected 0x%04X %s\n",
    id, ADS_EXPECTED_ID, id == ADS_EXPECTED_ID ? "(OK)" : "(MISMATCH)");
  return id == ADS_EXPECTED_ID;

}

bool gate_driver_selftest() {
    digitalWrite(GATE_CHG_PIN, LOW);
    digitalWrite(GATE_DSCHG_PIN, LOW);
    if(digitalRead(GATE_CHG_PIN) != LOW || digitalRead(GATE_DSCHG_PIN) != LOW) {
        Serial.println("Gate driver self-test failed: unable to set LOW");
        return false;
    }
    return true;

}

void gate_driver_enable() {
    digitalWrite(GATE_CHG_PIN, HIGH);
    digitalWrite(GATE_DSCHG_PIN, HIGH);
}

void gate_driver_disable() {
    digitalWrite(GATE_CHG_PIN, LOW);
    digitalWrite(GATE_DSCHG_PIN, LOW);
}


void contactorInit() {
  pinMode(GATE_DSCHG_PIN, OUTPUT);
  pinMode(GATE_CHG_PIN,   OUTPUT);
  digitalWrite(GATE_DSCHG_PIN, LOW);
  digitalWrite(GATE_CHG_PIN,   LOW);
}

void bms_gpio_init() {
  pinMode(GATE_DSCHG_PIN, OUTPUT);
  pinMode(GATE_CHG_PIN,   OUTPUT);
  pinMode(ADS_RESET_PIN, OUTPUT);
  pinMode(HSPI_SS, OUTPUT);
}

void bms_spi_init() {
  vspi = new SPIClass(VSPI);
  hspi = new SPIClass(HSPI);

  vspi->begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI);
  hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS);
}



