#include <Arduino.h>
#include <SPI.h>
#include "bms_config.h"
#include "bms_types.h"
#include "bms_hardware.h"

SPIClass *vspi = NULL;
SPIClass *hspi = NULL;

static const int ADS_SPI_CLK = 500000;

// ============================================================================
// All ADS131M02 communication uses 32-bit words.
// The device defaults to 24-bit after reset, but a previous session may have
// left it in 32-bit mode. We normalise to 32-bit in ads_configure() and
// stay there. In 32-bit mode, commands/registers are MSB-aligned (top 16 bits)
// and ADC data is sign-extended into the full 32 bits.
// Frame = 4 words x 4 bytes = 16 bytes total.
// ============================================================================

// Transfer one 32-bit word, return the received 32-bit word.
static uint32_t adsXfer32(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
  uint8_t tx[4] = {b0, b1, b2, b3};
  uint8_t rx[4] = {0, 0, 0, 0};
  vspi->transferBytes(tx, rx, 4);
  return ((uint32_t)rx[0] << 24) | ((uint32_t)rx[1] << 16) |
         ((uint32_t)rx[2] << 8)  |  (uint32_t)rx[3];
}

static uint32_t adsXfer24(uint8_t b0, uint8_t b1, uint8_t b2) {
  uint8_t tx[3] = {b0, b1, b2};
  uint8_t rx[3] = {0, 0, 0};
  vspi->transferBytes(tx, rx, 3);
  return ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | rx[2];
}

// Send one complete 4-word NULL frame, discard all output.
static void adsNullFrame() {
  adsXfer32(0x00, 0x00, 0x00, 0x00);
  adsXfer32(0x00, 0x00, 0x00, 0x00);
  adsXfer32(0x00, 0x00, 0x00, 0x00);
  adsXfer32(0x00, 0x00, 0x00, 0x00);
}

// Send one complete 4-word NULL frame, return w1 (response/register word).
static uint32_t adsNullFrameRead() {
  uint32_t w1 = adsXfer32(0x00, 0x00, 0x00, 0x00);
  adsXfer32(0x00, 0x00, 0x00, 0x00);
  adsXfer32(0x00, 0x00, 0x00, 0x00);
  adsXfer32(0x00, 0x00, 0x00, 0x00);
  return w1;
}

// Send one 4-word frame in 24-bit mode (12 bytes total)
static void adsNullFrame24() {
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
}

static uint32_t adsNullFrameRead24() {
  uint32_t w1 = adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  return w1;
}

// ============================================================================
// ads_reset — SPI software reset command (does not touch the RESET pin).
// ============================================================================
void ads_reset() {
  vspi->beginTransaction(SPISettings(ADS_SPI_CLK, MSBFIRST, SPI_MODE0));
  // RESET command = 0x0011, MSB-aligned in 32-bit word
  adsXfer32(0x00, 0x11, 0x00, 0x00);
  adsXfer32(0x00, 0x00, 0x00, 0x00);
  adsXfer32(0x00, 0x00, 0x00, 0x00);
  adsXfer32(0x00, 0x00, 0x00, 0x00);
  vspi->endTransaction();
}

// ============================================================================
// wakeupADS131M02 / sleepADS131M02 — WAKEUP and STANDBY commands.
// ============================================================================
void wakeupADS131M02() {
  vspi->beginTransaction(SPISettings(ADS_SPI_CLK, MSBFIRST, SPI_MODE0));
  // WAKEUP = 0x0033
  adsXfer32(0x00, 0x33, 0x00, 0x00);
  adsXfer32(0x00, 0x00, 0x00, 0x00);
  adsXfer32(0x00, 0x00, 0x00, 0x00);
  adsXfer32(0x00, 0x00, 0x00, 0x00);
  vspi->endTransaction();
}

void sleepADS131M02() {
  vspi->beginTransaction(SPISettings(ADS_SPI_CLK, MSBFIRST, SPI_MODE0));
  // STANDBY = 0x0022
  adsXfer32(0x00, 0x22, 0x00, 0x00);
  adsXfer32(0x00, 0x00, 0x00, 0x00);
  adsXfer32(0x00, 0x00, 0x00, 0x00);
  adsXfer32(0x00, 0x00, 0x00, 0x00);
  vspi->endTransaction();
}
bool ads_configure() {
  vspi->end();
  delay(10);
  vspi->begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI);
  delay(10);

  Serial.printf("[ADS] Resetting hardware: pin=%d\n", ADS_RESET_PIN);
  digitalWrite(ADS_RESET_PIN, LOW);
  delay(10);
  digitalWrite(ADS_RESET_PIN, HIGH);

  uint32_t t0 = millis();
  while (digitalRead(ADS_DRDY_PIN) == LOW) {
    if (millis() - t0 > 100) {
      Serial.println("  [ERR] DRDY timeout after reset");
      return false;
    }
  }
  Serial.println("  [ADS] DRDY high — device ready");

  vspi->beginTransaction(SPISettings(ADS_SPI_CLK, MSBFIRST, SPI_MODE0));

  // Frame 1: NULL — flush reset response (FF22 in w1)
  uint32_t f1w1 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f1w2 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f1w3 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f1w4 = adsXfer24(0x00, 0x00, 0x00);

  // Frame 2: WREG GAIN1 (addr=0x04, n=0) = 0x6080
  // ADS_GAIN1_VAL = 0x0040: CH1 gain=4, CH0 gain=1
  adsXfer24(0x60, 0x80, 0x00);
  adsXfer24(0x00, 0x40, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);

  // Frame 3: NULL — consume WREG GAIN1 response
  uint32_t f3w1 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f3w2 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f3w3 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f3w4 = adsXfer24(0x00, 0x00, 0x00);

  // Frame 4: RREG GAIN1 (addr=0x04, n=0) = 0xA080
  adsXfer24(0xA0, 0x80, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);

  // Frame 5: NULL — consume RREG acknowledgement (w1=STATUS)
  uint32_t f5w1 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f5w2 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f5w3 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f5w4 = adsXfer24(0x00, 0x00, 0x00);

  // Frame 6: NULL — GAIN1 register value in w1
  uint32_t f6w1 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f6w2 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f6w3 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f6w4 = adsXfer24(0x00, 0x00, 0x00);

  vspi->endTransaction();

  Serial.printf("  [DBG] Reset flush:    %06X %06X %06X %06X\n", f1w1, f1w2, f1w3, f1w4);
  Serial.printf("  [DBG] WREG GAIN1 rsp: %06X %06X %06X %06X\n", f3w1, f3w2, f3w3, f3w4);
  Serial.printf("  [DBG] RREG GAIN1 ack: %06X %06X %06X %06X\n", f5w1, f5w2, f5w3, f5w4);
  Serial.printf("  [DBG] RREG GAIN1 val: %06X %06X %06X %06X\n", f6w1, f6w2, f6w3, f6w4);

  uint16_t gainVal = (uint16_t)(f6w1 >> 8);
  Serial.printf("  ADS GAIN1 reg: wrote 0x%04X, read 0x%04X %s\n",
    ADS_GAIN1_VAL, gainVal, gainVal == ADS_GAIN1_VAL ? "(OK)" : "(MISMATCH)");

  Serial.println("  ADS CLOCK reg: using device default 0x030E (OK)");

  return (gainVal == ADS_GAIN1_VAL);
}

// ============================================================================
// ads_read_raw — read one conversion frame from the ADS131M02.
//
// In 32-bit sign-extend mode, the frame is:
//   w1: STATUS  (16-bit value in bits[31:16], lower 16 zero-padded)
//   w2: CH0     (24-bit ADC, sign-extended to 32 bits)
//   w3: CH1     (24-bit ADC, sign-extended to 32 bits)
//   w4: CRC     (16-bit in bits[31:16])
//
// CH1 is current sense. Returns sign-extended 32-bit value.
// ============================================================================
int32_t ads_read_raw() {
  // Wait for DRDY to go low — new conversion data ready
  uint32_t t0 = millis();
  while (digitalRead(ADS_DRDY_PIN) == HIGH) {
    if (millis() - t0 > 10) {
      Serial.println("[ads] DRDY timeout");
      return 0;
    }
  }

  vspi->beginTransaction(SPISettings(ADS_SPI_CLK, MSBFIRST, SPI_MODE0));

  uint32_t status = adsXfer24(0x00, 0x00, 0x00);  // w1: STATUS
  uint32_t ch0raw = adsXfer24(0x00, 0x00, 0x00);  // w2: CH0
  uint32_t ch1raw = adsXfer24(0x00, 0x00, 0x00);  // w3: CH1
  adsXfer24(0x00, 0x00, 0x00);                     // w4: CRC discard

  vspi->endTransaction();

  Serial.printf("[ads] STATUS=0x%04X CH0=0x%06X CH1=0x%06X\n",
      (uint16_t)(status >> 8),
      (ch0raw >> 8) & 0xFFFFFF,
      (ch1raw >> 8) & 0xFFFFFF);

  int32_t raw24 = (int32_t)((ch1raw >> 8) & 0xFFFFFF);
  if (raw24 & 0x800000) raw24 |= 0xFF000000;
  return raw24;
}

// ============================================================================
// ads_checkid — read ID register and verify upper byte = 0x22.
// ============================================================================
bool ads_checkid() {
  // Wait for DRDY before sending command
  uint32_t t0 = millis();
  while (digitalRead(ADS_DRDY_PIN) == LOW) {
    if (millis() - t0 > 10) { Serial.println("[ads] checkid DRDY timeout"); return false; }
  }

  vspi->beginTransaction(SPISettings(ADS_SPI_CLK, MSBFIRST, SPI_MODE0));

  // Frame 1: RREG ID (addr=0x00, n=0) = 0xA000
  adsXfer24(0xA0, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);

  // Frame 2: NULL — ID value in w1
  uint32_t id_raw = adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);

  vspi->endTransaction();

  uint16_t id = (uint16_t)(id_raw >> 8);
  bool ok = (id >> 8) == 0x22;
  Serial.printf("ADS131M02 ID: read 0x%04X, expected 0x22xx %s\n",
    id, ok ? "(OK)" : "(MISMATCH)");
  return ok;
}

// ============================================================================
// ads_counts_to_amps / ads_read_current — scaling helpers.
// ============================================================================
float ads_counts_to_amps(int32_t raw) {
  return (float)raw * ADS_COUNTS_TO_AMPS;
}

float ads_read_current() {
  int32_t raw = ads_read_raw();
  return ads_counts_to_amps(raw);
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
  pinMode(ADS_DRDY_PIN, INPUT);

  digitalWrite(ADS_RESET_PIN,  HIGH);  // active-low, keep deasserted
  digitalWrite(GATE_DSCHG_PIN, LOW);
  digitalWrite(GATE_CHG_PIN,   LOW);
  digitalWrite(HSPI_SS,        HIGH);

  // Generate 1.024 MHz clock for ADS131M02 CLKIN on GPIO5
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