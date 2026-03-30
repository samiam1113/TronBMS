/* ESP32 Dual SPI — 2x LTC6811-1 (direct SPI daisy chain) + ADS131M02-Q1
 * VSPI: MISO=19, MOSI=23, CLK=18, SS=GND  → ADS131M02-Q1 (CH1 current)
 * HSPI: MISO=12, MOSI=13, CLK=14, SS=4    → LTC6811-1 x2 daisy chain (20 cells)
 *
 * LTC6811-1 daisy chain note (direct SPI mode):
 *   Commands broadcast to ALL ICs simultaneously via MOSI.
 *   Responses shift back through chain — IC2 data arrives before IC1 data.
 *   IC1 monitors cells 1–12, IC2 monitors cells 13–20 (C9–C12 on IC2 unused).
 *   Each IC has 4 cell voltage register groups (A/B/C/D), 3 cells each:
 *     Group A: C1–C3   (RDCVA 0x0004)
 *     Group B: C4–C6   (RDCVB 0x0006)
 *     Group C: C7–C9   (RDCVC 0x0008)
 *     Group D: C10–C12 (RDCVD 0x000A)
 *
 * ADS131M02-Q1 note:
 *   CH1 only, PGA gain = 4, shunt = 0.4mΩ.
 *   Full scale = 750A, resolution ≈ 22.9mA per count.
 *
 * Pack config: 20s7p
 *   IC1: cells 1–12  (all 4 groups used)
 *   IC2: cells 13–20 (groups A+B used fully, group C C7+C8 used, C9 discarded)
 *                    (group D not connected — skip entirely)
 *
 * Fixes vs original:
 *   1. WREG/RREG address field: << 7 → << 5 (bits[10:5] per ADS131 Table 13)
 *   2. adsWriteReg echo read on frame 3, not frame 4
 *   3. ltcWaitConversion: busy-poll → delay(3)
 *   4. Removed spurious second wakeupLTC6811() before register reads
 *   5. NUM_CELLS corrected to 20 (LTC6811-1 is 12 cells/IC, not 6)
 *   6. Added RDCVC/RDCVD commands, full 4-group read for IC1, 3-group for IC2
 */
#include <Arduino.h>
#include <SPI.h>
#include <Math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "bms_thresholds.h"

#define ADS_RESET_PIN 21  // change to whatever GPIO your /RESET is wired to

// ── Pin definitions ──────────────────────────────────────────────────────────
#define VSPI_MISO 19
#define VSPI_MOSI 23
#define VSPI_SCLK 18
#define VSPI_SS   -1    // ADS131M02-Q1: single device, CS tied to GND

#define HSPI_MISO 12
#define HSPI_MOSI 13
#define HSPI_SCLK 14
#define HSPI_SS   4     // LTC6811-1 chain CS

#define GATE_DSCHG_PIN 25
#define GATE_CHG_PIN   26

// ── LTC6811-1 aux commands ───────────────────────────────────────────────────
#define LTC_ADAX   0x0560  // Start all GPIO + 2nd ref ADC, 7kHz mode
#define LTC_RDAUXA 0x000C  // Read aux group A: GPIO1, GPIO2, GPIO3
#define LTC_RDAUXB 0x000E  // Read aux group B: GPIO4, GPIO5, VREF2

#if !defined(CONFIG_IDF_TARGET_ESP32)
  #define VSPI FSPI
#endif

// ── Protection thresholds ─────────────────────────────────────────────────
#define OV_THRESHOLD     4.20f   // V — overvoltage per cell
#define UV_THRESHOLD     2.80f   // V — undervoltage per cell
#define OT_THRESHOLD     60.0f   // C — overtemperature
#define OC_CHG_THRESHOLD 100.0f  // A — overcurrent charge (positive)
#define OC_DSG_THRESHOLD 200.0f  // A — overcurrent discharge (negative)

// ── SPI clocks ───────────────────────────────────────────────────────────────
static const int LTC_SPI_CLK = 1000000;  // LTC6811-1 max 1 MHz
static const int ADS_SPI_CLK = 500000;  // ADS131M02-Q1 max 25 MHz, 1 MHz for safety

// ── System config ────────────────────────────────────────────────────────────
static const int NUM_ICS   = 2;   // LTC6811-1 ICs in daisy chain
static const int NUM_CELLS = 20;  // 20s pack: IC1 cells 1–12, IC2 cells 13–20

// ── ADS131M02-Q1 scaling ─────────────────────────────────────────────────────
static const float VREF   = 1.2f;
static const float GAIN   = 4.0f;
static const float RSHUNT = 0.0004f;
// Full scale: 1.2 / (4 * 0.0004) = 750A
// Resolution: 750 / 32768 ≈ 22.9mA per count

SPIClass *vspi = NULL;
SPIClass *hspi = NULL;

// ── LTC6811-1 commands (datasheet Table 38) ──────────────────────────────────
#define LTC_ADCV  0x0370  // Start all-cell ADC, 7kHz mode
#define LTC_RDCVA 0x0004  // Read cell voltage group A (C1–C3)
#define LTC_RDCVB 0x0006  // Read cell voltage group B (C4–C6)
#define LTC_RDCVC 0x0008  // Read cell voltage group C (C7–C9)
#define LTC_RDCVD 0x000A  // Read cell voltage group D (C10–C12)

// ── NTC thermistor ───────────────────────────────────────────────────────────
#define NTC_R25    10000.0f
#define NTC_BETA    3950.0f
#define NTC_RBIAS  10000.0f

// ── ADS131M02-Q1 commands and registers ──────────────────────────────────────
#define ADS_WAKEUP    0x0033
#define ADS_STANDBY   0x0022
#define ADS_NULL      0x0000
#define ADS_RESET     0x0011
#define ADS_REG_CLOCK 0x03
#define ADS_REG_GAIN1 0x04

#define ADS_GAIN             4
#define SHUNT_OHMS           0.0004f
#define ADS_VREF             1.2f
#define ADS_FULLSCALE_A      (ADS_VREF / (ADS_GAIN * SHUNT_OHMS))  // 750A
#define ADS_LSB_A            (ADS_FULLSCALE_A / 32768.0f)           // ~22.9mA/count

#define ADS_GAIN1_VAL 0x0040  // CH1 gain=4 (bits[8:6]=010), CH0 gain=1
#define ADS_CLOCK_VAL 0x030C  // OSR=4096, CH0+CH1 enabled

SemaphoreHandle_t hspiBus  = NULL;  // HSPI bus — one task at a time
SemaphoreHandle_t dataMux  = NULL;  // cellV, cellRaw, tempC, amps
SemaphoreHandle_t faultMux = NULL;  // faultRegister, chgOpen, dsgOpen
SemaphoreHandle_t balMux   = NULL;  // balanceCells

float    g_cellV[NUM_CELLS]  = {};
uint16_t g_cellRaw[NUM_CELLS]= {};
float    g_tempC[10]         = {};
float    g_amps              = 0;
bool     g_balanceCells[20]  = {};
uint16_t g_faultRegister     = 0;
bool     g_chgOpen           = true;
bool     g_dsgOpen           = true;

// Peak overcurrent timers — only written/read by taskProtection
static unsigned long ocChgPeakStart = 0;
static unsigned long ocDsgPeakStart = 0;
// ════════════════════════════════════════════════════════════════════════════
//  Fault register
// ════════════════════════════════════════════════════════════════════════════
 
enum FaultBit : uint16_t {
  FAULT_CELL_OV  = (1 << 0),
  FAULT_CELL_UV  = (1 << 1),
  FAULT_PACK_OV  = (1 << 2),
  FAULT_PACK_UV  = (1 << 3),
  FAULT_OT       = (1 << 4),
  FAULT_OC_CHG   = (1 << 5),
  FAULT_OC_DSG   = (1 << 6),
  FAULT_SPI      = (1 << 7),
};

static const uint16_t FAULT_BLOCKS_CHG =
  FAULT_CELL_OV | FAULT_PACK_OV | FAULT_OT | FAULT_OC_CHG | FAULT_SPI;
static const uint16_t FAULT_BLOCKS_DSG =
  FAULT_CELL_UV | FAULT_PACK_UV | FAULT_OT | FAULT_OC_DSG | FAULT_SPI;

SPIClass *vspi = NULL;
SPIClass *hspi = NULL;

// ════════════════════════════════════════════════════════════════════════════
//  LTC6811-1 — CRC-15
// ════════════════════════════════════════════════════════════════════════════
static uint16_t pec15Table[256];
static bool pec15TableInit = false;

static void initPEC15Table() {
  uint16_t remainder;
  for (int i = 0; i < 256; i++) {
    remainder = i << 7;
    for (int bit = 8; bit > 0; bit--) {
      if (remainder & 0x4000)
        remainder = ((remainder << 1)) ^ 0x4599;
      else
        remainder = (remainder << 1);
    }
    pec15Table[i] = remainder & 0xFFFF;
  }
}

uint16_t ltcCRC15(uint8_t *data, uint8_t len) {
  if (!pec15TableInit) { initPEC15Table(); pec15TableInit = true; }
  uint16_t remainder = 16;
  for (uint8_t i = 0; i < len; i++) {
    uint16_t address = ((remainder >> 7) ^ data[i]) & 0xFF;
    remainder = (remainder << 8) ^ pec15Table[address];
  }
  return (remainder * 2) & 0xFFFF;
}

// ════════════════════════════════════════════════════════════════════════════
//  Contactor control
//  Call setFault/clearFault — do not call open/close directly from tasks
// ════════════════════════════════════════════════════════════════════════════
 
void openCharge() {
  digitalWrite(GATE_CHG_PIN, LOW);
  if (xSemaphoreTake(faultMux, pdMS_TO_TICKS(10))) {
    g_chgOpen = true;
    xSemaphoreGive(faultMux);
  }
}
 
void openDischarge() {
  digitalWrite(GATE_DSCHG_PIN, LOW);
  if (xSemaphoreTake(faultMux, pdMS_TO_TICKS(10))) {
    g_dsgOpen = true;
    xSemaphoreGive(faultMux);
  }
}
 
void closeCharge() {
  uint16_t fr = 0;
  if (xSemaphoreTake(faultMux, pdMS_TO_TICKS(10))) {
    fr = g_faultRegister;
    xSemaphoreGive(faultMux);
  }
  if (fr & FAULT_BLOCKS_CHG) return;
  digitalWrite(GATE_CHG_PIN, HIGH);
  if (xSemaphoreTake(faultMux, pdMS_TO_TICKS(10))) {
    g_chgOpen = false;
    xSemaphoreGive(faultMux);
  }
}
 
void closeDischarge() {
  uint16_t fr = 0;
  if (xSemaphoreTake(faultMux, pdMS_TO_TICKS(10))) {
    fr = g_faultRegister;
    xSemaphoreGive(faultMux);
  }
  if (fr & FAULT_BLOCKS_DSG) return;
  digitalWrite(GATE_DSCHG_PIN, HIGH);
  if (xSemaphoreTake(faultMux, pdMS_TO_TICKS(10))) {
    g_dsgOpen = false;
    xSemaphoreGive(faultMux);
  }
}
 
void setFault(uint16_t bit) {
  if (xSemaphoreTake(faultMux, pdMS_TO_TICKS(10))) {
    g_faultRegister |= bit;
    xSemaphoreGive(faultMux);
  }
  // GPIO writes are safe from any task — no mutex needed
  uint16_t fr = 0;
  if (xSemaphoreTake(faultMux, pdMS_TO_TICKS(10))) {
    fr = g_faultRegister;
    xSemaphoreGive(faultMux);
  }
  if (fr & FAULT_BLOCKS_CHG) openCharge();
  if (fr & FAULT_BLOCKS_DSG) openDischarge();
}
 
void clearFault(uint16_t bit) {
  if (xSemaphoreTake(faultMux, pdMS_TO_TICKS(10))) {
    g_faultRegister &= ~bit;
    xSemaphoreGive(faultMux);
  }
  uint16_t fr = 0;
  if (xSemaphoreTake(faultMux, pdMS_TO_TICKS(10))) {
    fr = g_faultRegister;
    xSemaphoreGive(faultMux);
  }
  if (!(fr & FAULT_BLOCKS_CHG)) closeCharge();
  if (!(fr & FAULT_BLOCKS_DSG)) closeDischarge();
}
 
void contactorInit() {
  pinMode(GATE_DSCHG_PIN, OUTPUT);
  pinMode(GATE_CHG_PIN,   OUTPUT);
  digitalWrite(GATE_DSCHG_PIN, LOW);
  digitalWrite(GATE_CHG_PIN,   LOW);
}
 
void printFaultState() {
  uint16_t fr = 0;
  bool chg = true, dsg = true;
  if (xSemaphoreTake(faultMux, pdMS_TO_TICKS(10))) {
    fr  = g_faultRegister;
    chg = g_chgOpen;
    dsg = g_dsgOpen;
    xSemaphoreGive(faultMux);
  }
  if (fr == 0) {
    Serial.printf("  Faults: none  CHG=%s DSG=%s\n",
      chg ? "OPEN" : "CLOSED", dsg ? "OPEN" : "CLOSED");
    return;
  }
  Serial.printf("  Faults:[%s%s%s%s%s%s%s%s] CHG=%s DSG=%s\n",
    fr & FAULT_CELL_OV ? "CELL_OV " : "",
    fr & FAULT_CELL_UV ? "CELL_UV " : "",
    fr & FAULT_PACK_OV ? "PACK_OV " : "",
    fr & FAULT_PACK_UV ? "PACK_UV " : "",
    fr & FAULT_OT      ? "OT "      : "",
    fr & FAULT_OC_CHG  ? "OC_CHG "  : "",
    fr & FAULT_OC_DSG  ? "OC_DSG "  : "",
    fr & FAULT_SPI     ? "SPI "     : "",
    chg ? "OPEN" : "CLOSED", dsg ? "OPEN" : "CLOSED");
}

// ════════════════════════════════════════════════════════════════════════════
//  LTC6811-1 — Wakeup
// ════════════════════════════════════════════════════════════════════════════

void wakeupLTC6811() {
  // Stage 1: CS low pulse — wakes IC1 from SLEEP to STANDBY
  digitalWrite(HSPI_SS, LOW);
  delayMicroseconds(400);  // tWAKE max 400us
  digitalWrite(HSPI_SS, HIGH);
  delay(5);  // tREFUP max 4.4ms + margin — IC1 reference stabilizes

  // Stage 2: Dummy RDCFGA — causes IC1 to generate isoSPI wakeup
  // pulse on Port B, which wakes IC2 (tREADY < 10us)
  uint8_t cmdBuf[4];
  uint16_t cmd = 0x0002;  // RDCFGA
  cmdBuf[0] = (cmd >> 8) & 0xFF;
  cmdBuf[1] =  cmd       & 0xFF;
  uint16_t crc = ltcCRC15(cmdBuf, 2);
  cmdBuf[2] = (crc >> 8) & 0xFF;
  cmdBuf[3] = crc & 0xFF;

  uint8_t rx[6] = {0};
  hspi->beginTransaction(SPISettings(LTC_SPI_CLK, MSBFIRST, SPI_MODE3));
  digitalWrite(HSPI_SS, LOW);
  hspi->transfer(cmdBuf, 4);
  hspi->transfer(rx, 6);  // clock out IC1 response — discard
  digitalWrite(HSPI_SS, HIGH);
  hspi->endTransaction();

  delayMicroseconds(500);  // IC2 tREADY < 10us — 500us is safe margin
                           // stay well under tIDLE min (4.3ms)
}

// ════════════════════════════════════════════════════════════════════════════
//  LTC6811-1 — Command and read helpers
// ════════════════════════════════════════════════════════════════════════════

void ltcSendCommand(uint16_t cmd) {
  uint8_t buf[4];
  buf[0] = (cmd >> 8) & 0xFF;
  buf[1] =  cmd       & 0xFF;
  uint16_t crc = ltcCRC15(buf, 2);
  buf[2] = (crc >> 8) & 0xFF;
  buf[3] = crc & 0xFF;

  hspi->beginTransaction(SPISettings(LTC_SPI_CLK, MSBFIRST, SPI_MODE3));
  digitalWrite(HSPI_SS, LOW);
  hspi->transfer(buf, 4);
  digitalWrite(HSPI_SS, HIGH);
  hspi->endTransaction();
}

// Fixed delay — 7kHz mode worst case ~2ms, 3ms gives margin.
// Previous busy-poll was sending 0xFF bytes to the LTC with CS low,
// which it interpreted as commands and corrupted SPI state.
void ltcWaitConversion() {

  delay(10);
}

// Send WRCFGA broadcast — REFON=1, per-IC DCC bits
// IC2 data sent first (last in chain), IC1 data sent second
void ltcWriteCFGR(uint8_t dcc4, uint8_t dcc5) {
  uint8_t cfgr[6] = {0xFC, 0x00, 0x00, 0x00, dcc4, dcc5};
  uint16_t dataCrc = ltcCRC15(cfgr, 6);
 
  uint8_t buf[12];
  buf[0] = 0x00; buf[1] = 0x01;
  uint16_t cmdCrc = ltcCRC15(buf, 2);
  buf[2] = (cmdCrc >> 8) & 0xFF;
  buf[3] = cmdCrc & 0xFF;
  memcpy(buf + 4, cfgr, 6);
  buf[10] = (dataCrc >> 8) & 0xFF;
  buf[11] = dataCrc & 0xFF;
 
  hspi->beginTransaction(SPISettings(LTC_SPI_CLK, MSBFIRST, SPI_MODE3));
  digitalWrite(HSPI_SS, LOW);
  hspi->transfer(buf, 12);
  digitalWrite(HSPI_SS, HIGH);
  hspi->endTransaction();
}
 
// Read one cell voltage register group from both ICs.
// rxBuf[0..7]  = IC2 (last in chain, shifts out first)
// rxBuf[8..15] = IC1
// ic0 = IC1 data, ic1 = IC2 data.
// Returns false if either IC fails CRC.
bool ltcReadCVGroup(uint16_t cmd, uint8_t ic0[6], uint8_t ic1[6]) {
  uint8_t cmdBuf[4];
  cmdBuf[0] = (cmd >> 8) & 0xFF;
  cmdBuf[1] =  cmd       & 0xFF;
  uint16_t crc = ltcCRC15(cmdBuf, 2);
  cmdBuf[2] = (crc >> 8) & 0xFF;
  cmdBuf[3] = crc & 0xFF;

  uint8_t rxBuf[16] = {0};

  hspi->beginTransaction(SPISettings(LTC_SPI_CLK, MSBFIRST, SPI_MODE3));
  digitalWrite(HSPI_SS, LOW);
  hspi->transfer(cmdBuf, 4);
  hspi->transfer(rxBuf, 16);
  digitalWrite(HSPI_SS, HIGH);
  hspi->endTransaction();

  // Raw dump — print before CRC check
  Serial.print("  IC2 raw: ");
  for (int i = 0; i < 8; i++) Serial.printf("%02X ", rxBuf[i]);
  Serial.println();
  Serial.print("  IC1 raw: ");
  for (int i = 8; i < 16; i++) Serial.printf("%02X ", rxBuf[i]);
  Serial.println();

  uint8_t *ic2rx = rxBuf;
  uint8_t *ic1rx = rxBuf + 8;

  uint16_t crc2calc = ltcCRC15(ic2rx, 6);
  uint16_t crc2recv = ((uint16_t)ic2rx[6] << 8) | ic2rx[7];
  uint16_t crc1calc = ltcCRC15(ic1rx, 6);
  uint16_t crc1recv = ((uint16_t)ic1rx[6] << 8) | ic1rx[7];;

  Serial.printf("  IC2 CRC calc=0x%04X recv=0x%04X %s\n",
    crc2calc, crc2recv, crc2calc == crc2recv ? "OK" : "FAIL");
  Serial.printf("  IC1 CRC calc=0x%04X recv=0x%04X %s\n",
    crc1calc, crc1recv, crc1calc == crc1recv ? "OK" : "FAIL");

  if (crc2calc != crc2recv || crc1calc != crc1recv) return false;

  memcpy(ic0, ic1rx, 6);
  memcpy(ic1, ic2rx, 6);
  return true;
}

bool ltcReadAuxGroup(uint16_t cmd, uint8_t ic0[6], uint8_t ic1[6]) {
  uint8_t cmdBuf[4];
  cmdBuf[0] = (cmd >> 8) & 0xFF;
  cmdBuf[1] =  cmd       & 0xFF;
  uint16_t crc = ltcCRC15(cmdBuf, 2);
  cmdBuf[2] = (crc >> 8) & 0xFF;
  cmdBuf[3] = crc & 0xFF;
 
  uint8_t rxBuf[16] = {0};
 
  hspi->beginTransaction(SPISettings(LTC_SPI_CLK, MSBFIRST, SPI_MODE3));
  digitalWrite(HSPI_SS, LOW);
  hspi->transfer(cmdBuf, 4);
  hspi->transfer(rxBuf, 16);
  digitalWrite(HSPI_SS, HIGH);
  hspi->endTransaction();
 
  uint8_t *ic2rx = rxBuf;
  uint8_t *ic1rx = rxBuf + 8;
 
  uint16_t crc2calc = ltcCRC15(ic2rx, 6);
  uint16_t crc2recv = ((uint16_t)ic2rx[6] << 8) | ic2rx[7];
  uint16_t crc1calc = ltcCRC15(ic1rx, 6);
  uint16_t crc1recv = ((uint16_t)ic1rx[6] << 8) | ic1rx[7];
 
  if (crc2calc != crc2recv || crc1calc != crc1recv) return false;
 
  memcpy(ic0, ic1rx, 6);
  memcpy(ic1, ic2rx, 6);
  return true;
}

// Parse 3 cells from a 6-byte group — saves both float V and raw counts
void parseGroup(uint8_t *raw, float *outV, uint16_t *outRaw) {
  for (int i = 0; i < 3; i++) {
    uint16_t val = (uint16_t)raw[i*2] | ((uint16_t)raw[i*2+1] << 8);
    outV[i]   = val * 0.0001f;
    outRaw[i] = val;
  }
}



// ════════════════════════════════════════════════════════════════════════════
//  LTC6811-1 — Cell Balancing
// ════════════════════════════════════════════════════════════════════════════


void ltcWriteCFGR(uint8_t dcc4, uint8_t dcc5) {
  // cfgr[0]: REFON=1, GPIO pull-downs off = 0xFC
  // cfgr[4]: DCC8..DCC1  (cells 1-8,  bit0=DCC1 .. bit7=DCC8)
  // cfgr[5]: DCTO[3:0] + DCC12..DCC9 (cells 9-12, bit0=DCC9 .. bit3=DCC12)
  //          DCTO=0 means watchdog disabled — DCC bits reset after ~2s of no command
  uint8_t cfgr[6] = {0xFC, 0x00, 0x00, 0x00, dcc4, dcc5};
  uint16_t dataCrc = ltcCRC15(cfgr, 6);

  uint8_t buf[12];
  buf[0] = 0x00; buf[1] = 0x01;
  uint16_t cmdCrc = ltcCRC15(buf, 2);
  buf[2] = (cmdCrc >> 8) & 0xFF;
  buf[3] = cmdCrc & 0xFF;
  memcpy(buf + 4, cfgr, 6);
  buf[10] = (dataCrc >> 8) & 0xFF;
  buf[11] = dataCrc & 0xFF;

  hspi->beginTransaction(SPISettings(LTC_SPI_CLK, MSBFIRST, SPI_MODE3));
  digitalWrite(HSPI_SS, LOW);
  hspi->transfer(buf, 12);
  digitalWrite(HSPI_SS, HIGH);
  hspi->endTransaction();
}

// ── Build DCC bytes from a 20-element bool array and send WRCFGA ─────────────
// cells[0] = cell 1 ... cells[19] = cell 20
// IC1 handles cells 1-12, IC2 handles cells 13-20.
// WRCFGA is broadcast — both ICs receive the same cfgr[4]/cfgr[5].
// IC2 only uses cells 13-20, which map to its own DCC1-DCC8 pins,
// so the same byte layout applies per-IC relative to its own cell numbering.
//
// cfgr[4] bit map (per IC, relative cell numbers):
//   bit0=DCC1  bit1=DCC2  bit2=DCC3  bit3=DCC4
//   bit4=DCC5  bit5=DCC6  bit6=DCC7  bit7=DCC8
// cfgr[5] bit map:
//   bit0=DCC9  bit1=DCC10 bit2=DCC11 bit3=DCC12
//   bit4-7=DCTO (0 = timer disabled)
void setBalancing(bool cells[20]) {
  // IC1: pack cells 1-12 → IC1 DCC1-DCC12
  uint8_t ic1_dcc4 = 0, ic1_dcc5 = 0;
  for (int i = 0; i < 8;  i++) if (cells[i])   ic1_dcc4 |= (1 << i);
  for (int i = 8; i < 12; i++) if (cells[i])   ic1_dcc5 |= (1 << (i - 8));

  // IC2: pack cells 13-20 → IC2 DCC1-DCC8
  uint8_t ic2_dcc4 = 0;
  for (int i = 12; i < 20; i++) if (cells[i])  ic2_dcc4 |= (1 << (i - 12));

  // WRCFGA is broadcast — send IC1's bytes first (IC1 is primary, data sent first)
  // then IC2's bytes. With 2 ICs in chain, buf must carry both sets.
  // Each IC latches the 6 bytes addressed to it in sequence.
  uint8_t cfgr_ic2[6] = {0xFC, 0x00, 0x00, 0x00, ic2_dcc4, 0x00};
  uint8_t cfgr_ic1[6] = {0xFC, 0x00, 0x00, 0x00, ic1_dcc4, ic1_dcc5};

  uint16_t crc_ic2 = ltcCRC15(cfgr_ic2, 6);
  uint16_t crc_ic1 = ltcCRC15(cfgr_ic1, 6);

  uint8_t cmd[4];
  cmd[0] = 0x00; cmd[1] = 0x01;
  uint16_t cmdCrc = ltcCRC15(cmd, 2);
  cmd[2] = (cmdCrc >> 8) & 0xFF;
  cmd[3] = cmdCrc & 0xFF;

  // Full frame: 4 cmd bytes + 8 bytes IC2 data + 8 bytes IC1 data = 20 bytes
  uint8_t buf[20];
  memcpy(buf, cmd, 4);
  memcpy(buf + 4,  cfgr_ic2, 6);
  buf[10] = (crc_ic2 >> 8) & 0xFF;
  buf[11] = crc_ic2 & 0xFF;
  memcpy(buf + 12, cfgr_ic1, 6);
  buf[18] = (crc_ic1 >> 8) & 0xFF;
  buf[19] = crc_ic1 & 0xFF;

  hspi->beginTransaction(SPISettings(LTC_SPI_CLK, MSBFIRST, SPI_MODE3));
  digitalWrite(HSPI_SS, LOW);
  hspi->transfer(buf, 20);
  digitalWrite(HSPI_SS, HIGH);
  hspi->endTransaction();
}

void stopBalancing() {
  bool none[20] = {};
  setBalancing(none);
}


// ════════════════════════════════════════════════════════════════════════════
//  LTC6811-1 — Main voltage read (20 cells)
// ════════════════════════════════════════════════════════════════════════════

// Reads all 20 active cell voltages across 2x LTC6811-1.
//
// cellV index map:
//   [0..2]   IC1 group A → pack cells 1–3
//   [3..5]   IC1 group B → pack cells 4–6
//   [6..8]   IC1 group C → pack cells 7–9
//   [9..11]  IC1 group D → pack cells 10–12
//   [12..14] IC2 group A → pack cells 13–15
//   [15..17] IC2 group B → pack cells 16–18
//   [18]     IC2 group C C7 → pack cell 19
//   [19]     IC2 group C C8 → pack cell 20
//            IC2 group C C9 — not connected, discarded
//            IC2 group D    — not connected, skipped entirely
//
// Local working arrays — only written by taskMeasure under hspiBus
static float    _cellV[NUM_CELLS];
static uint16_t _cellRaw[NUM_CELLS];
 
bool readAllCellVoltages(bool bal[20]) {
  wakeupLTC6811();
  setBalancing(bal);   // WRCFGA with actual DCC bits — keeps reference on
  delay(5);
  ltcSendCommand(LTC_ADCV);
  ltcWaitConversion();
 
  uint8_t ic0[6], ic1[6];
  float    grpV[3];
  uint16_t grpR[3];
 
  // ── IC1 Group A → cells 1–3 ──────────────────────────────────────────────
  if (!ltcReadCVGroup(LTC_RDCVA, ic0, ic1)) return false;
  parseGroup(ic0, grpV, grpR);
  _cellV[0]=grpV[0]; _cellV[1]=grpV[1]; _cellV[2]=grpV[2];
  _cellRaw[0]=grpR[0]; _cellRaw[1]=grpR[1]; _cellRaw[2]=grpR[2];
 
  // ── IC1 Group B → cells 4–6 ──────────────────────────────────────────────
  if (!ltcReadCVGroup(LTC_RDCVB, ic0, ic1)) return false;
  parseGroup(ic0, grpV, grpR);
  _cellV[3]=grpV[0]; _cellV[4]=grpV[1]; _cellV[5]=grpV[2];
  _cellRaw[3]=grpR[0]; _cellRaw[4]=grpR[1]; _cellRaw[5]=grpR[2];
 
  // ── IC1 Group C → cells 7–9 ──────────────────────────────────────────────
  if (!ltcReadCVGroup(LTC_RDCVC, ic0, ic1)) return false;
  parseGroup(ic0, grpV, grpR);
  _cellV[6]=grpV[0]; _cellV[7]=grpV[1]; _cellV[8]=grpV[2];
  _cellRaw[6]=grpR[0]; _cellRaw[7]=grpR[1]; _cellRaw[8]=grpR[2];
 
  // ── IC1 Group D → cells 10–12 ────────────────────────────────────────────
  if (!ltcReadCVGroup(LTC_RDCVD, ic0, ic1)) return false;
  parseGroup(ic0, grpV, grpR);
  _cellV[9]=grpV[0]; _cellV[10]=grpV[1]; _cellV[11]=grpV[2];
  _cellRaw[9]=grpR[0]; _cellRaw[10]=grpR[1]; _cellRaw[11]=grpR[2];
 
  // ── IC2 Group A → cells 13–15 ────────────────────────────────────────────
  if (!ltcReadCVGroup(LTC_RDCVA, ic0, ic1)) return false;
  parseGroup(ic1, grpV, grpR);
  _cellV[12]=grpV[0]; _cellV[13]=grpV[1]; _cellV[14]=grpV[2];
  _cellRaw[12]=grpR[0]; _cellRaw[13]=grpR[1]; _cellRaw[14]=grpR[2];
 
  // ── IC2 Group B → cells 16–18 ────────────────────────────────────────────
  if (!ltcReadCVGroup(LTC_RDCVB, ic0, ic1)) return false;
  parseGroup(ic1, grpV, grpR);
  _cellV[15]=grpV[0]; _cellV[16]=grpV[1]; _cellV[17]=grpV[2];
  _cellRaw[15]=grpR[0]; _cellRaw[16]=grpR[1]; _cellRaw[17]=grpR[2];
 
  // ── IC2 Group C → cells 19–20 only (C9 not connected) ───────────────────
  if (!ltcReadCVGroup(LTC_RDCVC, ic0, ic1)) return false;
  parseGroup(ic1, grpV, grpR);
  _cellV[18]=grpV[0];   _cellV[19]=grpV[1];
  _cellRaw[18]=grpR[0]; _cellRaw[19]=grpR[1];
  // grpV[2]/grpR[2] = IC2 C9 not connected — discard
  // IC2 Group D not connected — skip entirely
 
  return true;
}


// ════════════════════════════════════════════════════════════════════════════
//  LTC6811-1 — NTC temperature read
//
//  Assumes 10k NTC + 10k bias resistor from VREF2 per IC (datasheet Fig 57).
//  VREF2 is read from RDAUXB bytes [4..5] of each IC.
//  GPIO pin assignment is yours to define — adjust the mapping below to match
//  your schematic. This example uses GPIO1-GPIO5 on each IC = 10 sensors total.
//
//  tempC index map (example — adjust to your wiring):
//    [0..4]  IC1 GPIO1–GPIO5
//    [5..9]  IC2 GPIO1–GPIO5
//
//  Returns false on any CRC failure.
// ════════════════════════════════════════════════════════════════════════════

static float ntcToTemp(float vgpio, float vref2) {
  if (vref2 < 0.01f) return -99.0f;  // guard divide-by-zero
  float rntc = NTC_RBIAS * vgpio / (vref2 - vgpio);
  if (rntc <= 0.0f) return -99.0f;
  float tK = 1.0f / (logf(rntc / NTC_R25) / NTC_BETA + 1.0f / 298.15f);
  return tK - 273.15f;
}

bool readAllTemperatures(float tempC[10], bool bal[20]) {
  wakeupLTC6811();
  setBalancing(bal);
  delay(5);
 
  ltcSendCommand(LTC_ADAX);
  delay(10);
 
  uint8_t ic0[6], ic1[6];
 
  if (!ltcReadAuxGroup(LTC_RDAUXA, ic0, ic1)) return false;
 
  float ic1_g1 = ((uint16_t)ic0[0] | ((uint16_t)ic0[1] << 8)) * 0.0001f;
  float ic1_g2 = ((uint16_t)ic0[2] | ((uint16_t)ic0[3] << 8)) * 0.0001f;
  float ic1_g3 = ((uint16_t)ic0[4] | ((uint16_t)ic0[5] << 8)) * 0.0001f;
  float ic2_g1 = ((uint16_t)ic1[0] | ((uint16_t)ic1[1] << 8)) * 0.0001f;
  float ic2_g2 = ((uint16_t)ic1[2] | ((uint16_t)ic1[3] << 8)) * 0.0001f;
  float ic2_g3 = ((uint16_t)ic1[4] | ((uint16_t)ic1[5] << 8)) * 0.0001f;
 
  if (!ltcReadAuxGroup(LTC_RDAUXB, ic0, ic1)) return false;
 
  float ic1_g4   = ((uint16_t)ic0[0] | ((uint16_t)ic0[1] << 8)) * 0.0001f;
  float ic1_g5   = ((uint16_t)ic0[2] | ((uint16_t)ic0[3] << 8)) * 0.0001f;
  float ic1_vref = ((uint16_t)ic0[4] | ((uint16_t)ic0[5] << 8)) * 0.0001f;
  float ic2_g4   = ((uint16_t)ic1[0] | ((uint16_t)ic1[1] << 8)) * 0.0001f;
  float ic2_g5   = ((uint16_t)ic1[2] | ((uint16_t)ic1[3] << 8)) * 0.0001f;
  float ic2_vref = ((uint16_t)ic1[4] | ((uint16_t)ic1[5] << 8)) * 0.0001f;
 
  tempC[0] = ntcToTemp(ic1_g1, ic1_vref);
  tempC[1] = ntcToTemp(ic1_g2, ic1_vref);
  tempC[2] = ntcToTemp(ic1_g3, ic1_vref);
  tempC[3] = ntcToTemp(ic1_g4, ic1_vref);
  tempC[4] = ntcToTemp(ic1_g5, ic1_vref);
  tempC[5] = ntcToTemp(ic2_g1, ic2_vref);
  tempC[6] = ntcToTemp(ic2_g2, ic2_vref);
  tempC[7] = ntcToTemp(ic2_g3, ic2_vref);
  tempC[8] = ntcToTemp(ic2_g4, ic2_vref);
  tempC[9] = ntcToTemp(ic2_g5, ic2_vref);
 
  return true;
}

// ════════════════════════════════════════════════════════════════════════════
//  ADS131M02-Q1 — Low level SPI
// ════════════════════════════════════════════════════════════════════════════

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

// int16_t readADSCurrent() {
//   vspi->beginTransaction(SPISettings(ADS_SPI_CLK, MSBFIRST, SPI_MODE1));

//   // WAKEUP frame
//   adsXfer24(0x00, 0x33, 0x00);
//   adsXfer24(0x00, 0x00, 0x00);
//   adsXfer24(0x00, 0x00, 0x00);
//   adsXfer24(0x00, 0x00, 0x00);

//   // NULL frame — read conversion data
//   adsXfer24(0x00, 0x00, 0x00);                        // w1: STATUS
//   uint32_t ch0raw = adsXfer24(0x00, 0x00, 0x00);      // w2: CH0 — discard
//   uint32_t ch1raw = adsXfer24(0x00, 0x00, 0x00);      // w3: CH1
//   adsXfer24(0x00, 0x00, 0x00);                        // w4: CRC

//   vspi->endTransaction();

//   int32_t val = (int32_t)(ch1raw << 8) >> 8;  // sign extend 24→32
//   return (int16_t)(val >> 8);                  // top 16 bits
// }
// ════════════════════════════════════════════════════════════════════════════
//  ADS131M02-Q1 — Boot configuration
// ════════════════════════════════════════════════════════════════════════════


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
// VSPI is only used by taskMeasure — no mutex needed for vspi itself
int16_t readADSCurrent() {
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
 
float adsCounts_to_amps(int16_t raw) {
  return raw * ADS_LSB_A;
}

// ════════════════════════════════════════════════════════════════════════════
//  Auto-balancing — uses raw counts to avoid float threshold comparison
// ════════════════════════════════════════════════════════════════════════════
 
void updateBalancing(uint16_t rawCounts[NUM_CELLS]) {
  uint16_t minRaw = rawCounts[0];
  for (int i = 1; i < NUM_CELLS; i++)
    if (rawCounts[i] < minRaw) minRaw = rawCounts[i];
 
  bool newBal[20];
  for (int i = 0; i < NUM_CELLS; i++)
    newBal[i] = (rawCounts[i] - minRaw) >= BAL_THRESHOLD_UV;
 
  if (xSemaphoreTake(balMux, pdMS_TO_TICKS(10))) {
    memcpy(g_balanceCells, newBal, sizeof(g_balanceCells));
    xSemaphoreGive(balMux);
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  Protection logic — runs on local copies, no mutex held during processing
// ════════════════════════════════════════════════════════════════════════════
 
void runProtection(float cellV[NUM_CELLS], float tempC[10], float amps) {
 
  // ── Cell overvoltage ──────────────────────────────────────────────────────
  bool ovFault = false;
  for (int i = 0; i < NUM_CELLS; i++)
    if (cellV[i] > CELL_OV_V) { ovFault = true; break; }
  ovFault ? setFault(FAULT_CELL_OV) : clearFault(FAULT_CELL_OV);
 
  // ── Cell undervoltage ─────────────────────────────────────────────────────
  bool uvFault = false;
  for (int i = 0; i < NUM_CELLS; i++)
    if (cellV[i] < CELL_UV_V) { uvFault = true; break; }
  uvFault ? setFault(FAULT_CELL_UV) : clearFault(FAULT_CELL_UV);
 
  // ── Pack voltage ──────────────────────────────────────────────────────────
  float packV = 0;
  for (int i = 0; i < NUM_CELLS; i++) packV += cellV[i];
  packV > PACK_OV_V ? setFault(FAULT_PACK_OV) : clearFault(FAULT_PACK_OV);
  packV < PACK_UV_V ? setFault(FAULT_PACK_UV) : clearFault(FAULT_PACK_UV);
 
  // ── Temperature ───────────────────────────────────────────────────────────
  bool otFault = false;
  float tempSum = 0;
  for (int i = 0; i < 10; i++) {
    if (tempC[i] > -90.0f) tempSum += tempC[i];  // skip -99 sentinel
    if (tempC[i] >= TEMP_CUTOFF_C) otFault = true;
  }
  float tempAvg = tempSum / 10.0f;
  if (tempAvg >= TEMP_WARN_C)
    Serial.printf("  WARN: avg temp %.1fC\n", tempAvg);
  otFault ? setFault(FAULT_OT) : clearFault(FAULT_OT);
 
  // ── Charge overcurrent — peak vs continuous ───────────────────────────────
  if (amps > CURR_CHG_PEAK_A) {
    setFault(FAULT_OC_CHG);
    Serial.printf("  FAULT: charge peak overcurrent %.1fA\n", amps);
    ocChgPeakStart = 0;
  } else if (amps > CURR_CHG_CONT_A) {
    if (ocChgPeakStart == 0) ocChgPeakStart = millis();
    if (millis() - ocChgPeakStart > CURR_PEAK_MS) {
      setFault(FAULT_OC_CHG);
      Serial.printf("  FAULT: charge cont overcurrent %.1fA\n", amps);
    }
  } else {
    ocChgPeakStart = 0;
    clearFault(FAULT_OC_CHG);
  }
 
  // ── Discharge overcurrent — peak vs continuous ────────────────────────────
  if (amps < 0) {
    float absA = fabsf(amps);
    if (absA > CURR_DSCHG_PEAK_A) {
      setFault(FAULT_OC_DSG);
      Serial.printf("  FAULT: discharge peak overcurrent %.1fA\n", absA);
      ocDsgPeakStart = 0;
    } else if (absA > CURR_DSCHG_CONT_A) {
      if (ocDsgPeakStart == 0) ocDsgPeakStart = millis();
      if (millis() - ocDsgPeakStart > CURR_PEAK_MS) {
        setFault(FAULT_OC_DSG);
        Serial.printf("  FAULT: discharge cont overcurrent %.1fA\n", absA);
      }
    } else {
      ocDsgPeakStart = 0;
      clearFault(FAULT_OC_DSG);
    }
  } else {
    ocDsgPeakStart = 0;
    clearFault(FAULT_OC_DSG);
  }
}

// ── Task 1: watchdog — highest priority ───────────────────────────────────
// Fires every 500ms, sends WRCFGA with current balance state.
// Must never miss — DCC bits reset after 2s.
void taskWatchdog(void *pv) {
  for (;;) {
    bool localBal[20];
    if (xSemaphoreTake(balMux, pdMS_TO_TICKS(10))) {
      memcpy(localBal, g_balanceCells, sizeof(g_balanceCells));
      xSemaphoreGive(balMux);
    }
    if (xSemaphoreTake(hspiBus, pdMS_TO_TICKS(100))) {
      wakeupLTC6811();
      setBalancing(localBal);
      xSemaphoreGive(hspiBus);
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// ── Task 3: measure — priority 1 (lowest) ───────────────────────────────────
// Reads voltages every loop, current every loop, temperatures every 5s.
void taskMeasure(void *pv) {
  TickType_t lastTemp = xTaskGetTickCount();
 
  for (;;) {
    bool localBal[20];
    if (xSemaphoreTake(balMux, pdMS_TO_TICKS(10))) {
      memcpy(localBal, g_balanceCells, sizeof(g_balanceCells));
      xSemaphoreGive(balMux);
    }
 
    // ── Cell voltages ────────────────────────────────────────────────────────
    if (xSemaphoreTake(hspiBus, pdMS_TO_TICKS(200))) {
      bool ok = readAllCellVoltages(localBal);
      xSemaphoreGive(hspiBus);
 
      if (ok) {
        if (xSemaphoreTake(dataMux, pdMS_TO_TICKS(50))) {
          memcpy(g_cellV,   _cellV,   sizeof(_cellV));
          memcpy(g_cellRaw, _cellRaw, sizeof(_cellRaw));
          xSemaphoreGive(dataMux);
        }
        for (int i = 0; i < NUM_CELLS; i++)
          Serial.printf("  Cell %2d: %.4fV\n", i + 1, _cellV[i]);
      } else {
        Serial.println("  FAULT: SPI/CRC failure on voltage read");
        setFault(FAULT_SPI);
      }
    }
 
    // ── Current ──────────────────────────────────────────────────────────────
    int16_t raw = readADSCurrent();
    float localAmps = adsCounts_to_amps(raw);
    if (xSemaphoreTake(dataMux, pdMS_TO_TICKS(50))) {
      g_amps = localAmps;
      xSemaphoreGive(dataMux);
    }
    Serial.printf("  Current: %.2fA\n", localAmps);
 
    // ── Temperatures every 5s ────────────────────────────────────────────────
    if (xTaskGetTickCount() - lastTemp >= pdMS_TO_TICKS(5000)) {
      float localTempC[10];
      if (xSemaphoreTake(hspiBus, pdMS_TO_TICKS(200))) {
        bool ok = readAllTemperatures(localTempC, localBal);
        xSemaphoreGive(hspiBus);
        if (ok) {
          if (xSemaphoreTake(dataMux, pdMS_TO_TICKS(50))) {
            memcpy(g_tempC, localTempC, sizeof(g_tempC));
            xSemaphoreGive(dataMux);
          }
          for (int i = 0; i < 10; i++)
            Serial.printf("  Temp %d: %.1fC\n", i + 1, localTempC[i]);
        } else {
          Serial.println("  WARNING: temperature read CRC failure");
        }
      }
      lastTemp = xTaskGetTickCount();
    }
 
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ── Task 2: protection — priority 2 ─────────────────────────────────────────
// Snapshots latest data, runs protection, updates balance state.
// Never holds dataMux during protection processing.
void taskProtection(void *pv) {
  for (;;) {
    float    localCellV[NUM_CELLS];
    uint16_t localCellRaw[NUM_CELLS];
    float    localTempC[10];
    float    localAmps;
 
    if (xSemaphoreTake(dataMux, pdMS_TO_TICKS(50))) {
      memcpy(localCellV,   g_cellV,   sizeof(g_cellV));
      memcpy(localCellRaw, g_cellRaw, sizeof(g_cellRaw));
      memcpy(localTempC,   g_tempC,   sizeof(g_tempC));
      localAmps = g_amps;
      xSemaphoreGive(dataMux);
    }
 
    runProtection(localCellV, localTempC, localAmps);
    updateBalancing(localCellRaw);  // writes to g_balanceCells under balMux
    printFaultState();
 
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  Setup
// ════════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
  while (!Serial) delay(10);
 
  // ── Contactors — open before anything else ────────────────────────────────
  contactorInit();
 
  // ── SPI buses ─────────────────────────────────────────────────────────────
  vspi = new SPIClass(VSPI);
  hspi = new SPIClass(HSPI);
  vspi->begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI);
  hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS);
 
  pinMode(HSPI_SS, OUTPUT);
  digitalWrite(HSPI_SS, HIGH);
 
  // ── ADS131M02-Q1 CLKIN — must be stable before reset sequence ────────────
  pinMode(ADS_RESET_PIN, OUTPUT);
  digitalWrite(ADS_RESET_PIN, HIGH);
  ledcSetup(0, 4096000, 1);
  ledcAttachPin(22, 0);
  ledcWrite(0, 1);
  delay(10);
 
  Serial.println("========================================");
  Serial.println(" BMS — LTC6811-1 x2 + ADS131M02-Q1");
  Serial.println(" 20s7p | 750A full scale | FreeRTOS");
  Serial.println("========================================");
 
  // ── ADS131M02-Q1 config ───────────────────────────────────────────────────
  if (configureADS131M02()) {
    Serial.println("  ADS131M02-Q1 OK");
    Serial.printf("  Full scale: %.0fA  LSB: %.4fA\n",
      ADS_FULLSCALE_A, ADS_LSB_A);
  } else {
    Serial.println("  ERROR: ADS131M02-Q1 config failed");
  }
 
  // ── LTC6811-1 CRC table check ─────────────────────────────────────────────
  uint8_t test[2] = {0x00, 0x01};
  uint16_t testCrc = ltcCRC15(test, 2);
  Serial.printf("  LTC CRC check: 0x%04X (expect 0x3D6E) %s\n",
    testCrc, testCrc == 0x3D6E ? "OK" : "FAIL");
 
  // ── Create mutexes ────────────────────────────────────────────────────────
  hspiBus  = xSemaphoreCreateMutex();
  dataMux  = xSemaphoreCreateMutex();
  faultMux = xSemaphoreCreateMutex();
  balMux   = xSemaphoreCreateMutex();
 
  // ── Start tasks — all pinned to core 1 ───────────────────────────────────
  // Core 0 left free for WiFi/BT if added later
  xTaskCreatePinnedToCore(taskWatchdog,   "watchdog",   2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskProtection, "protection", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskMeasure,    "measure",    8192, NULL, 1, NULL, 1);
 
  Serial.println("  Tasks started");
 
  // ── Attempt to close contactors — will stay open if any fault is set ─────
  closeCharge();
  closeDischarge();
}

// ════════════════════════════════════════════════════════════════════════════
//  Main loop
// ════════════════════════════════════════════════════════════════════════════

void loop() {
  
}