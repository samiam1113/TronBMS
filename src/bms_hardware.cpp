#include <Arduino.h>
#include <SPI.h>
#include "bms_config.h"
#include "bms_types.h"
#include "bms_hardware.h"

SPIClass *vspi = NULL;
SPIClass *hspi = NULL;

static const int ADS_SPI_CLK = 4000000;

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
  vspi->beginTransaction(SPISettings(ADS_SPI_CLK, MSBFIRST, SPI_MODE1));
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
  vspi->beginTransaction(SPISettings(ADS_SPI_CLK, MSBFIRST, SPI_MODE1));
  // WAKEUP = 0x0033
  adsXfer32(0x00, 0x33, 0x00, 0x00);
  adsXfer32(0x00, 0x00, 0x00, 0x00);
  adsXfer32(0x00, 0x00, 0x00, 0x00);
  adsXfer32(0x00, 0x00, 0x00, 0x00);
  vspi->endTransaction();
}

void sleepADS131M02() {
  vspi->beginTransaction(SPISettings(ADS_SPI_CLK, MSBFIRST, SPI_MODE1));
  // STANDBY = 0x0022
  adsXfer32(0x00, 0x22, 0x00, 0x00);
  adsXfer32(0x00, 0x00, 0x00, 0x00);
  adsXfer32(0x00, 0x00, 0x00, 0x00);
  adsXfer32(0x00, 0x00, 0x00, 0x00);
  vspi->endTransaction();
}

// ============================================================================
// ads_configure — full device initialisation.
//
// Strategy:
//   1. Hardware reset via RESET pin → device goes to default state.
//   2. Flush reset response frame using 32-bit words (safe regardless of
//      whatever word length a previous session left the device in, because
//      32-bit is a superset of 24-bit framing — extra zero bytes are ignored).
//   3. Write MODE register first: set WLENGTH=10 (32-bit, sign-extend),
//      TIMEOUT=0 (disabled — required when CS is tied low), RESET bit clear.
//   4. Write CLOCK and GAIN1.
//   5. Read back CLOCK and GAIN1 to verify.
//
// MODE value 0x0600:
//   Bits 15:14 = 00  (reserved)
//   Bit  13    =  0  (REG_CRC_EN disabled)
//   Bit  12    =  0  (RX_CRC_EN disabled)
//   Bit  11    =  0  (CRC_TYPE = CCITT)
//   Bit  10    =  1  (RESET = 1, write 0 to clear — write 0 here)
//   Bits  9:8  = 11  (WLENGTH = 32-bit sign-extend)
//   Bits  7:5  = 000 (reserved)
//   Bit   4    =  0  (TIMEOUT disabled)
//   Bits  3:2  = 00  (DRDY_SEL = most lagging)
//   Bit   1    =  0  (DRDY_HiZ = push-pull)
//   Bit   0    =  0  (DRDY_FMT = level)
// → 0x0300 with RESET bit cleared and WLENGTH=11: 0x0300
//   WLENGTH=11 = bits[9:8]=11 → 0x0300
//   Plus TIMEOUT=0 → stays 0x0300
//
// ADS_CLOCK_VAL = 0x030C (from bms_config.h):
//   CH1_EN=1, CH0_EN=1, OSR=1024, PWR=high-res
//
// ADS_GAIN1_VAL = 0x0040 (from bms_config.h):
//   CH1 gain=4 (PGAGAIN1[2:0]=010), CH0 gain=1
// ============================================================================
bool ads_configure() {
  Serial.printf("[ADS] Resetting hardware: pin=%d\n", ADS_RESET_PIN);
  digitalWrite(ADS_RESET_PIN, LOW);
  delay(10);
  digitalWrite(ADS_RESET_PIN, HIGH);
  delay(1);

  vspi->beginTransaction(SPISettings(ADS_SPI_CLK, MSBFIRST, SPI_MODE1));

  // Frame 1: NULL — flush reset response
  uint32_t f1w1 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f1w2 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f1w3 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f1w4 = adsXfer24(0x00, 0x00, 0x00);

  // Frame 2: WREG MODE (0x6040), value 0x0500
  adsXfer24(0x60, 0x40, 0x00);
  adsXfer24(0x05, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);

  // Frame 3: NULL — consume WREG MODE response
  uint32_t f3w1 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f3w2 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f3w3 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f3w4 = adsXfer24(0x00, 0x00, 0x00);

  // Frame 4: RREG MODE (0xA040)
  adsXfer24(0xA0, 0x40, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);

  // Frame 5: NULL — MODE value in w1
  uint32_t f5w1 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f5w2 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f5w3 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f5w4 = adsXfer24(0x00, 0x00, 0x00);

  // Frame 6: WREG CLOCK (0x6060)
  adsXfer24(0x60, 0x60, 0x00);
  adsXfer24(0x03, 0x0C, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);

  // Frame 7: NULL — consume WREG CLOCK response
  uint32_t f7w1 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f7w2 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f7w3 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f7w4 = adsXfer24(0x00, 0x00, 0x00);

  // Frame 8: WREG GAIN1 (0x6080)
  adsXfer24(0x60, 0x80, 0x00);
  adsXfer24(0x00, 0x40, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);

  // Frame 9: NULL — consume WREG GAIN1 response
  uint32_t f9w1 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f9w2 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f9w3 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f9w4 = adsXfer24(0x00, 0x00, 0x00);

  // Frame 10: RREG CLOCK (0xA060)
  adsXfer24(0xA0, 0x60, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);

  // Frame 11: NULL — CLOCK value in w1
  uint32_t f11w1 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f11w2 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f11w3 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f11w4 = adsXfer24(0x00, 0x00, 0x00);

  // Frame 12: RREG GAIN1 (0xA080)
  adsXfer24(0xA0, 0x80, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);
  adsXfer24(0x00, 0x00, 0x00);

  // Frame 13: NULL — GAIN1 value in w1
  uint32_t f13w1 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f13w2 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f13w3 = adsXfer24(0x00, 0x00, 0x00);
  uint32_t f13w4 = adsXfer24(0x00, 0x00, 0x00);

  vspi->endTransaction();

  // All prints AFTER endTransaction — never print inside the transaction
  Serial.printf("  [DBG] Reset flush:        %06X %06X %06X %06X\n", f1w1, f1w2, f1w3, f1w4);
  Serial.printf("  [DBG] WREG MODE rsp:      %06X %06X %06X %06X\n", f3w1, f3w2, f3w3, f3w4);
  Serial.printf("  [DBG] RREG MODE rsp:      %06X %06X %06X %06X\n", f5w1, f5w2, f5w3, f5w4);
  Serial.printf("  [DBG] WREG CLOCK rsp:     %06X %06X %06X %06X\n", f7w1, f7w2, f7w3, f7w4);
  Serial.printf("  [DBG] WREG GAIN1 rsp:     %06X %06X %06X %06X\n", f9w1, f9w2, f9w3, f9w4);
  Serial.printf("  [DBG] RREG CLOCK rsp:     %06X %06X %06X %06X\n", f11w1, f11w2, f11w3, f11w4);
  Serial.printf("  [DBG] RREG GAIN1 rsp:     %06X %06X %06X %06X\n", f13w1, f13w2, f13w3, f13w4);

  uint16_t modeVal  = (uint16_t)(f5w1  >> 8);
  uint16_t clockVal = (uint16_t)(f11w1 >> 8);
  uint16_t gainVal  = (uint16_t)(f13w1 >> 8);

  Serial.printf("  ADS MODE  reg: wrote 0x0500, read 0x%04X %s\n",
    modeVal, modeVal == 0x0500 ? "(OK)" : "(MISMATCH)");
  Serial.printf("  ADS CLOCK reg: wrote 0x%04X, read 0x%04X %s\n",
    ADS_CLOCK_VAL, clockVal, clockVal == ADS_CLOCK_VAL ? "(OK)" : "(MISMATCH)");
  Serial.printf("  ADS GAIN1 reg: wrote 0x%04X, read 0x%04X %s\n",
    ADS_GAIN1_VAL, gainVal, gainVal == ADS_GAIN1_VAL ? "(OK)" : "(MISMATCH)");

  return (clockVal == ADS_CLOCK_VAL && gainVal == ADS_GAIN1_VAL);
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
  vspi->beginTransaction(SPISettings(ADS_SPI_CLK, MSBFIRST, SPI_MODE1));

  uint32_t status = adsXfer24(0x00, 0x00, 0x00);  // w1: STATUS
  uint32_t ch0raw = adsXfer24(0x00, 0x00, 0x00);  // w2: CH0
  uint32_t ch1raw = adsXfer24(0x00, 0x00, 0x00);  // w3: CH1
  adsXfer24(0x00, 0x00, 0x00);                     // w4: CRC discard

  vspi->endTransaction();

  Serial.printf("[ads] STATUS=0x%04X CH0=0x%06X CH1=0x%06X\n",
      (uint16_t)(status >> 8),
      (ch0raw >> 8) & 0xFFFFFF,
      (ch1raw >> 8) & 0xFFFFFF);

  // Sign-extend 24-bit two's complement
  int32_t raw24 = (int32_t)((ch1raw >> 8) & 0xFFFFFF);
  if (raw24 & 0x800000) raw24 |= 0xFF000000;
  return raw24;
}

// ============================================================================
// ads_checkid — read ID register and verify upper byte = 0x22.
// ============================================================================
bool ads_checkid() {
  vspi->beginTransaction(SPISettings(ADS_SPI_CLK, MSBFIRST, SPI_MODE1));

  // Flush any in-flight conversion frame
  adsNullFrame();

  // RREG ID (addr=0x00, n=0) = 0xA000, MSB-aligned → 0xA000_0000
  adsXfer32(0xA0, 0x00, 0x00, 0x00);  // w1: RREG ID command
  adsXfer32(0x00, 0x00, 0x00, 0x00);  // w2
  adsXfer32(0x00, 0x00, 0x00, 0x00);  // w3
  adsXfer32(0x00, 0x00, 0x00, 0x00);  // w4

  // NULL frame — w1 contains ID register value (MSB-aligned in bits[31:16])
  uint32_t id_raw = adsNullFrameRead();

  vspi->endTransaction();

  uint16_t id = (uint16_t)(id_raw >> 16);
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