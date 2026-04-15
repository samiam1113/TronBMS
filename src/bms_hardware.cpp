#include <Arduino.h>
#include <SPI.h>
#include "bms_config.h"
#include "bms_types.h"
#include "bms_hardware.h"

SPIClass *vspi = NULL;
SPIClass *hspi = NULL;

static const int ADS_SPI_CLK = 4000000;

// ============================================================================
// SPI_MODE0 is the only mode that keeps DRDY alive on this hardware.
// The device samples DIN on the falling edge (CPHA=1), but MODE0 samples on
// the rising edge — one half-cycle early. This shifts all received/transmitted
// bytes by 1 bit. To compensate:
//   TX: pre-shift command words right by 1 bit before sending
//   RX: post-shift received words right by 1 bit after receiving
// NULL frames (all zeros) are unaffected.
// ============================================================================

static uint32_t adsXfer24(uint8_t b0, uint8_t b1, uint8_t b2) {
  uint8_t tx[3] = {b0, b1, b2};
  uint8_t rx[3] = {0, 0, 0};
  vspi->transferBytes(tx, rx, 3);
  return ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | rx[2];
}

// Send a 16-bit command + 16-bit data as one 4-word 24-bit frame
// Both cmd and data are pre-shifted right by 1 bit to compensate for MODE0
static void adsSendCmd(uint16_t cmd, uint16_t data) {
  // cmd: 16-bit >> 1 = 15 bits, MSB-aligned in 24-bit word
  adsXfer24((cmd >> 9) & 0xFF, (cmd >> 1) & 0xFF, (cmd << 7) & 0x80);
  // data: 16-bit >> 1, MSB-aligned in 24-bit word
  adsXfer24((data >> 9) & 0xFF, (data >> 1) & 0xFF, (data << 7) & 0x80);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
}

// Correct a received 24-bit word for the MODE0 1-bit shift
// Device output is shifted left 1 — shift right 1 to correct
// Returns the 16-bit register value (MSB-aligned in 24-bit word)
static uint16_t adsCorrectRx(uint32_t raw) {
  uint32_t corrected = (raw >> 1) & 0x7FFFFF;
  return (uint16_t)(corrected >> 8);
}

// ============================================================================
// ads_reset / wakeup / sleep
// ============================================================================
void ads_reset() {
  vspi->beginTransaction(SPISettings(ADS_SPI_CLK, MSBFIRST, SPI_MODE0));
  adsSendCmd(0x0011, 0x0000);
  vspi->endTransaction();
}

void wakeupADS131M02() {
  vspi->beginTransaction(SPISettings(ADS_SPI_CLK, MSBFIRST, SPI_MODE0));
  adsSendCmd(0x0033, 0x0000);
  vspi->endTransaction();
}

void sleepADS131M02() {
  vspi->beginTransaction(SPISettings(ADS_SPI_CLK, MSBFIRST, SPI_MODE0));
  adsSendCmd(0x0022, 0x0000);
  vspi->endTransaction();
}

// ============================================================================
// ads_configure
// ============================================================================
bool ads_configure() {
  digitalWrite(ADS_RESET_PIN, LOW);
  delay(10);
  digitalWrite(ADS_RESET_PIN, HIGH);
  uint32_t t0 = millis();
  while (digitalRead(ADS_DRDY_PIN) == LOW) {
    if (millis() - t0 > 100) { Serial.println("  [ERR] DRDY timeout"); return false; }
  }
  Serial.println("  [ADS] DRDY high — device ready");
  return true;
}
// ============================================================================
// ads_read_raw
// NULL frames are unaffected by the MODE0 shift so no correction needed
// ============================================================================
int32_t ads_read_raw() {
  uint32_t t0 = millis();
  while (digitalRead(ADS_DRDY_PIN) == HIGH) {
    if (millis() - t0 > 10) {
      Serial.println("[ads] DRDY timeout");
      return 0;
    }
  }

  vspi->beginTransaction(SPISettings(ADS_SPI_CLK, MSBFIRST, SPI_MODE0));
  uint32_t status = adsXfer24(0x00, 0x00, 0x00);
  uint32_t ch0raw = adsXfer24(0x00, 0x00, 0x00);
  uint32_t ch1raw = adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  vspi->endTransaction();

  int32_t raw24 = (int32_t)((ch1raw >> 8) & 0xFFFFFF);
  if (raw24 & 0x800000) raw24 |= 0xFF000000;
  return raw24;
}

// ============================================================================
// ads_checkid
// RREG ID (0xA000) pre-shifted right 1 = 0x5000
// ============================================================================
bool ads_checkid() {
  Serial.println("ADS131M02 ID: skipped (running on defaults)");
  return true;
}

// ============================================================================
// Scaling
// ============================================================================
float ads_counts_to_amps(int32_t raw) {
  return (float)raw * ADS_COUNTS_TO_AMPS;
}

float ads_read_current() {
  //return ads_counts_to_amps(ads_read_raw());
  return 0.0f; //ADS131M02 Disabled, hardware issue
}

// ============================================================================
// Gate driver
// ============================================================================
bool gate_driver_selftest() {
  digitalWrite(GATE_CHG_PIN,   LOW);
  digitalWrite(GATE_DSCHG_PIN, LOW);
  Serial.println("[gate] Self-test: outputs set LOW (no feedback pin — extend when available)");
  return true;
}

void gate_driver_enable() {
  digitalWrite(GATE_CHG_PIN,   HIGH);
  digitalWrite(GATE_DSCHG_PIN, HIGH);
}

void gate_driver_disable() {
  digitalWrite(GATE_CHG_PIN,   LOW);
  digitalWrite(GATE_DSCHG_PIN, LOW);
}

// ============================================================================
// Peripheral init
// ============================================================================
void contactorInit() {
  pinMode(GATE_DSCHG_PIN, OUTPUT);
  pinMode(GATE_CHG_PIN,   OUTPUT);
  digitalWrite(GATE_DSCHG_PIN, LOW);
  digitalWrite(GATE_CHG_PIN,   LOW);
}

void bms_gpio_init() {
  pinMode(GATE_DSCHG_PIN, OUTPUT);
  pinMode(GATE_CHG_PIN,   OUTPUT);
  pinMode(ADS_RESET_PIN,  OUTPUT);
  pinMode(HSPI_SS,        OUTPUT);
  pinMode(ADS_DRDY_PIN,   INPUT);

  digitalWrite(ADS_RESET_PIN,  HIGH);
  digitalWrite(GATE_DSCHG_PIN, LOW);
  digitalWrite(GATE_CHG_PIN,   LOW);
  digitalWrite(HSPI_SS,        HIGH);

  // Generate 1.024 MHz clock for ADS131M02 CLKIN on GPIO22
  ledcSetup(0, 1024000, 1);
  ledcAttachPin(22, 0);
  ledcWrite(0, 1);
}

void bms_spi_init() {
  vspi = new SPIClass(VSPI);
  hspi = new SPIClass(HSPI);
  vspi->begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI);
  hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS);
}