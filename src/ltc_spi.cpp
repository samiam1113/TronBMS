#include <Arduino.h>
#include <SPI.h>
#include "bms_config.h"

// ── LTC6811-1 commands ───────────────────────────────────────────────────────
#define LTC_WRCFGA 0x0001
#define LTC_RDCFGA 0x0002
#define LTC_ADCV   0x0370  // Start all-cell ADC, 7kHz mode
#define LTC_RDCVA  0x0004
#define LTC_RDCVB  0x0006
#define LTC_RDCVC  0x0008
#define LTC_RDCVD  0x000A
#define LTC_ADAX   0x0560  // Start all GPIO + 2nd ref ADC, 7kHz mode
#define LTC_RDAUXA 0x000C
#define LTC_RDAUXB 0x000E
#define LTC_RDSTATA 0x0010 // Status group A: SC, ITMP, VA
#define LTC_RDSTATB 0x0012 // Status group B: VD, flags
#define LTC_ADSTAT  0x0468 // Start status group ADC
#define LTC_WRCOMM  0x0721
#define LTC_RDCOMM  0x0722
#define LTC_PLADC   0x0714 // Poll ADC — SDO=0xFF when conversion done

static const int LTC_SPI_CLK = 1000000;  // LTC6811-1 max 1 MHz
//static const int NUM_CELLS = 20;  

SPIClass *hspi = NULL;

static uint16_t pec15Table[256];
static bool pec15TableInit = false;

// Internal Cell storage
static float _cellV[20];
static uint16_t _cellRaw[20];

struct LtcConfig {
    // CFGR0
    bool gpio_pulldown[5] = {false,false,false,false,false}; // false=pull-down OFF (GPIO input mode)
    bool refon            = true;   // Keep reference on between conversions
    bool adcopt           = false;  // false = use 7kHz/26Hz/422Hz/1kHz modes
    // CFGR1–CFGR3: under/over voltage thresholds (raw 12-bit values)
    // V_threshold = raw * 16 * 0.0001f
    uint16_t vuv = 0x0000; // Under-voltage comparison voltage
    uint16_t vov = 0x0000; // Over-voltage comparison voltage
    // CFGR4–CFGR5: discharge cell bitmask and timer
    uint16_t dcc  = 0x0000; // Bit0=DCC1(cell1)...Bit9=DCC10(cell10)
    uint8_t  dcto = 0x00;   // Discharge timer: 0=disabled
};

struct LtcStatus {
    float   sc_v;       // Sum of all cells voltage (V)
    float   itmp_c;     // Internal die temperature (°C)
    float   va_v;       // Analog supply voltage (V)
    float   vd_v;       // Digital supply voltage (V)
    bool    c_uv[12];   // Cell under-voltage flags
    bool    c_ov[12];   // Cell over-voltage flags
    bool    thsd;       // Thermal shutdown flag
    bool    muxfail;    // MUX self-test failure flag
    bool    rev[4];     // Revision code
};

// ════════════════════════════════════════════════════════════════════════════
//  LTC6811-1 — CRC-15
// ════════════════════════════════════════════════════════════════════════════

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

uint16_t ltc_pec15_calc(uint8_t *data, uint8_t len) {
  if (!pec15TableInit) { initPEC15Table(); pec15TableInit = true; }
  uint16_t remainder = 16; // PEC seed
  for (uint8_t i = 0; i < len; i++) {
    uint16_t address = ((remainder >> 7) ^ data[i]) & 0xFF;
    remainder = (remainder << 8) ^ pec15Table[address];
  }
  return (remainder * 2) & 0xFFFF;
}

bool ltc_pec15_verify(uint8_t *data, uint8_t len, uint16_t received_pec) {
    return ltc_pec15_calc(data, len) == received_pec;
}

static void buildCmd(uint16_t cmd, uint8_t out[4]) {
    out[0] = (cmd >> 8) & 0xFF;
    out[1] =  cmd       & 0xFF;
    uint16_t crc = ltc_pec15_calc(out, 2);
    out[2] = (crc >> 8) & 0xFF;
    out[3] =  crc       & 0xFF;
}

// Broadcast a 4-byte command to all ICs — no data phase
static void sendCmd(uint16_t cmd) {
    uint8_t buf[4];
    buildCmd(cmd, buf);
    hspi->beginTransaction(SPISettings(LTC_SPI_CLK, MSBFIRST, SPI_MODE0));
    digitalWrite(HSPI_SS, LOW);
    hspi->transfer(buf, 4);
    digitalWrite(HSPI_SS, HIGH);
    hspi->endTransaction();
}

// Read one register group from both ICs — returns ic0=IC1 data, ic1=IC2 data
static bool readGroup(uint16_t cmd, uint8_t ic0[6], uint8_t ic1[6]) {
    uint8_t cmdBuf[4];
    buildCmd(cmd, cmdBuf);
    uint8_t rxBuf[16] = {0};

    hspi->beginTransaction(SPISettings(LTC_SPI_CLK, MSBFIRST, SPI_MODE0));
    digitalWrite(HSPI_SS, LOW);
    hspi->transfer(cmdBuf, 4);
    hspi->transfer(rxBuf, 16);
    digitalWrite(HSPI_SS, HIGH);
    hspi->endTransaction();

    // Daisy-chain: first 8 bytes = IC2, last 8 bytes = IC1
    uint8_t *ic2rx = rxBuf;
    uint8_t *ic1rx = rxBuf + 8;

    uint16_t crc2recv = ((uint16_t)ic2rx[6] << 8) | ic2rx[7];
    uint16_t crc1recv = ((uint16_t)ic1rx[6] << 8) | ic1rx[7];

    if (!ltc_pec15_verify(ic2rx, 6, crc2recv)) return false;
    if (!ltc_pec15_verify(ic1rx, 6, crc1recv)) return false;

    memcpy(ic0, ic1rx, 6); // ic0 = IC1 (closer to ESP32)
    memcpy(ic1, ic2rx, 6); // ic1 = IC2 (far end of chain)
    return true;
}

// Write one register group to both ICs simultaneously
// ic0_data = IC1, ic1_data = IC2
static void writeGroup(uint16_t cmd,
                        const uint8_t ic0_data[6],
                        const uint8_t ic1_data[6]) {
    uint8_t cmdBuf[4];
    buildCmd(cmd, cmdBuf);

    uint16_t crc_ic2 = ltc_pec15_calc((uint8_t*)ic1_data, 6);
    uint16_t crc_ic1 = ltc_pec15_calc((uint8_t*)ic0_data, 6);

    // Full frame: 4 cmd + 8 IC2 + 8 IC1 = 20 bytes
    // Daisy-chain write order: IC2 first, IC1 second
    uint8_t buf[20];
    memcpy(buf, cmdBuf, 4);
    memcpy(buf + 4,  ic1_data, 6);
    buf[10] = (crc_ic2 >> 8) & 0xFF;
    buf[11] =  crc_ic2       & 0xFF;
    memcpy(buf + 12, ic0_data, 6);
    buf[18] = (crc_ic1 >> 8) & 0xFF;
    buf[19] =  crc_ic1       & 0xFF;

    hspi->beginTransaction(SPISettings(LTC_SPI_CLK, MSBFIRST, SPI_MODE0));
    digitalWrite(HSPI_SS, LOW);
    hspi->transfer(buf, 20);
    digitalWrite(HSPI_SS, HIGH);
    hspi->endTransaction();
}

// ════════════════════════════════════════════════════════════════════════════
//  Internal helpers
// ════════════════════════════════════════════════════════════════════════════

// Read one register group from both ICs — returns ic0=IC1 data, ic1=IC2 data
static bool readGroup(uint16_t cmd, uint8_t ic0[6], uint8_t ic1[6]) {
    uint8_t cmdBuf[4];
    buildCmd(cmd, cmdBuf);
    uint8_t rxBuf[16] = {0};

    hspi->beginTransaction(SPISettings(LTC_SPI_CLK, MSBFIRST, SPI_MODE0));
    digitalWrite(HSPI_SS, LOW);
    hspi->transfer(cmdBuf, 4);
    hspi->transfer(rxBuf, 16);
    digitalWrite(HSPI_SS, HIGH);
    hspi->endTransaction();

    // Daisy-chain: first 8 bytes = IC2, last 8 bytes = IC1
    uint8_t *ic2rx = rxBuf;
    uint8_t *ic1rx = rxBuf + 8;

    uint16_t crc2recv = ((uint16_t)ic2rx[6] << 8) | ic2rx[7];
    uint16_t crc1recv = ((uint16_t)ic1rx[6] << 8) | ic1rx[7];

    if (!ltc_pec15_verify(ic2rx, 6, crc2recv)) return false;
    if (!ltc_pec15_verify(ic1rx, 6, crc1recv)) return false;

    memcpy(ic0, ic1rx, 6); // ic0 = IC1 (closer to ESP32)
    memcpy(ic1, ic2rx, 6); // ic1 = IC2 (far end of chain)
    return true;
}

// Write one register group to both ICs simultaneously
// ic0_data = IC1, ic1_data = IC2
static void writeGroup(uint16_t cmd,
                        const uint8_t ic0_data[6],
                        const uint8_t ic1_data[6]) {
    uint8_t cmdBuf[4];
    buildCmd(cmd, cmdBuf);

    uint16_t crc_ic2 = ltc_pec15_calc((uint8_t*)ic1_data, 6);
    uint16_t crc_ic1 = ltc_pec15_calc((uint8_t*)ic0_data, 6);

    // Full frame: 4 cmd + 8 IC2 + 8 IC1 = 20 bytes
    // Daisy-chain write order: IC2 first, IC1 second
    uint8_t buf[20];
    memcpy(buf, cmdBuf, 4);
    memcpy(buf + 4,  ic1_data, 6);
    buf[10] = (crc_ic2 >> 8) & 0xFF;
    buf[11] =  crc_ic2       & 0xFF;
    memcpy(buf + 12, ic0_data, 6);
    buf[18] = (crc_ic1 >> 8) & 0xFF;
    buf[19] =  crc_ic1       & 0xFF;

    hspi->beginTransaction(SPISettings(LTC_SPI_CLK, MSBFIRST, SPI_MODE0));
    digitalWrite(HSPI_SS, LOW);
    hspi->transfer(buf, 20);
    digitalWrite(HSPI_SS, HIGH);
    hspi->endTransaction();
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


// ════════════════════════════════════════════════════════════════════════════
//  LTC6811-1 — Wakeup
// ════════════════════════════════════════════════════════════════════════════

void ltc_wakeup_sleep() {
    // CS low pulse wakes IC1 from SLEEP → STANDBY
    digitalWrite(HSPI_SS, LOW);
    delayMicroseconds(400);  // tWAKE max 400µs
    digitalWrite(HSPI_SS, HIGH);
    delay(5);  // tREFUP max 4.4ms — reference stabilises

    // Dummy RDCFGA causes IC1 to generate an isoSPI wake pulse to IC2
    uint8_t cmdBuf[4];
    buildCmd(LTC_RDCFGA, cmdBuf);
    uint8_t rx[16] = {0}; // 2 ICs × 8 bytes — discard

    hspi->beginTransaction(SPISettings(LTC_SPI_CLK, MSBFIRST, SPI_MODE0));
    digitalWrite(HSPI_SS, LOW);
    hspi->transfer(cmdBuf, 4);
    hspi->transfer(rx, 16);
    digitalWrite(HSPI_SS, HIGH);
    hspi->endTransaction();

    delayMicroseconds(500); // IC2 tREADY < 10µs — 500µs is a safe margin
                            // must stay under tIDLE min (4.3ms)
}

// ════════════════════════════════════════════════════════════════════════════
//  ltc_wakeup_idle — wake both ICs from IDLE (isoSPI timed out after ~5ms)
//  Any SPI activity is enough — no need for a long CS hold
// ════════════════════════════════════════════════════════════════════════════

void ltc_wakeup_idle() {
    hspi->beginTransaction(SPISettings(LTC_SPI_CLK, MSBFIRST, SPI_MODE0));
    digitalWrite(HSPI_SS, LOW);
    uint8_t dummy = 0xFF;
    hspi->transfer(dummy);
    digitalWrite(HSPI_SS, HIGH);
    hspi->endTransaction();
    delayMicroseconds(10); // tREADY
}

// ════════════════════════════════════════════════════════════════════════════
//  ltc_write_config — write LtcConfig to both ICs
// ════════════════════════════════════════════════════════════════════════════

// Encode an LtcConfig into the 6 raw CFGR bytes
static void encodeConfig(const LtcConfig &cfg, uint8_t d[6]) {
    memset(d, 0, 6);
    // CFGR0: GPIO[4:0] pull-down enables | REFON | ADCOPT
    d[0] = cfg.adcopt ? 0x01 : 0x00;
    d[0] |= cfg.refon  ? 0x04 : 0x00;
    for (int i = 0; i < 5; i++) {
        if (cfg.gpio_pulldown[i]) d[0] |= (1 << (i + 3));
    }
    // CFGR1: VUV[7:0]
    d[1] = cfg.vuv & 0xFF;
    // CFGR2: VUV[11:8] in low nibble, VOV[3:0] in high nibble
    d[2] = ((cfg.vuv >> 8) & 0x0F) | ((cfg.vov & 0x0F) << 4);
    // CFGR3: VOV[11:4]
    d[3] = (cfg.vov >> 4) & 0xFF;
    // CFGR4: DCC[7:0]
    d[4] = cfg.dcc & 0xFF;
    // CFGR5: DCC[11:8] low nibble, DCTO high nibble
    d[5] = ((cfg.dcc >> 8) & 0x0F) | ((cfg.dcto & 0x0F) << 4);
}

// cfg[0] = IC1 config, cfg[1] = IC2 config
void ltc_write_config(const LtcConfig cfg[2]) {
    uint8_t d0[6], d1[6];
    encodeConfig(cfg[0], d0); // IC1
    encodeConfig(cfg[1], d1); // IC2
    writeGroup(LTC_WRCFGA, d0, d1);
}

// ════════════════════════════════════════════════════════════════════════════
//  ltc_read_config — read CFGR back from both ICs and decode
// ════════════════════════════════════════════════════════════════════════════

static void decodeConfig(const uint8_t d[6], LtcConfig &cfg) {
    cfg.adcopt = d[0] & 0x01;
    cfg.refon  = (d[0] >> 2) & 0x01;
    for (int i = 0; i < 5; i++) cfg.gpio_pulldown[i] = (d[0] >> (i + 3)) & 0x01;
    cfg.vuv  = (uint16_t)d[1] | (((uint16_t)d[2] & 0x0F) << 8);
    cfg.vov  = ((uint16_t)(d[2] >> 4)) | ((uint16_t)d[3] << 4);
    cfg.dcc  = (uint16_t)d[4] | (((uint16_t)d[5] & 0x0F) << 8);
    cfg.dcto = (d[5] >> 4) & 0x0F;
}

// cfg_out[0] = IC1, cfg_out[1] = IC2
bool ltc_read_config(LtcConfig cfg_out[2]) {
    uint8_t ic0[6], ic1[6];
    if (!readGroup(LTC_RDCFGA, ic0, ic1)) return false;
    decodeConfig(ic0, cfg_out[0]);
    decodeConfig(ic1, cfg_out[1]);
    return true;
}

// ════════════════════════════════════════════════════════════════════════════
//  ltc_start_adc_conversion — broadcast ADCV and/or ADAX, poll until done
// ════════════════════════════════════════════════════════════════════════════

bool ltc_start_adc_conversion(bool cells, bool gpio) {
    if (cells) sendCmd(LTC_ADCV);
    if (gpio)  sendCmd(LTC_ADAX);

    // Poll PLADC — SDO held low while any IC is still converting
    // Normal mode max: ~2.5ms for 12 cells
    uint8_t cmdBuf[4];
    buildCmd(LTC_PLADC, cmdBuf);

    uint32_t start = micros();
    while (micros() - start < 4000) {
        delayMicroseconds(100);
        uint8_t rx = 0x00;
        hspi->beginTransaction(SPISettings(LTC_SPI_CLK, MSBFIRST, SPI_MODE0));
        digitalWrite(HSPI_SS, LOW);
        hspi->transfer(cmdBuf, 4);
        rx = hspi->transfer(0xFF);
        digitalWrite(HSPI_SS, HIGH);
        hspi->endTransaction();
        if (rx == 0xFF) return true;
    }
    return false; // Timeout
}

bool ltc_read_voltages(bool bal[20]) {
  ltc_wakeup_sleep();
  ltc_set_balance(bal);
  delay(5);
  ltcSendCommand(LTC_ADCV);
  delay(5);  

  uint8_t ic0[6], ic1[6];
  float    grpV[3];
  uint16_t grpR[3];

  // ── Group A: IC1 cells 1–3, IC2 cells 11–13 ─────────────────────────────
  if (!readGroup(LTC_RDCVA, ic0, ic1)) return false;
  parseGroup(ic0, grpV, grpR);
  _cellV[0]=grpV[0]; _cellV[1]=grpV[1]; _cellV[2]=grpV[2];
  _cellRaw[0]=grpR[0]; _cellRaw[1]=grpR[1]; _cellRaw[2]=grpR[2];
  parseGroup(ic1, grpV, grpR);
  _cellV[10]=grpV[0]; _cellV[11]=grpV[1]; _cellV[12]=grpV[2];
  _cellRaw[10]=grpR[0]; _cellRaw[11]=grpR[1]; _cellRaw[12]=grpR[2];

  // ── Group B: IC1 cells 4–6, IC2 cells 14–16 ─────────────────────────────
  if (!readGroup(LTC_RDCVB, ic0, ic1)) return false;
  parseGroup(ic0, grpV, grpR);
  _cellV[3]=grpV[0]; _cellV[4]=grpV[1]; _cellV[5]=grpV[2];
  _cellRaw[3]=grpR[0]; _cellRaw[4]=grpR[1]; _cellRaw[5]=grpR[2];
  parseGroup(ic1, grpV, grpR);
  _cellV[13]=grpV[0]; _cellV[14]=grpV[1]; _cellV[15]=grpV[2];
  _cellRaw[13]=grpR[0]; _cellRaw[14]=grpR[1]; _cellRaw[15]=grpR[2];

  // ── Group C: IC1 cells 7–9, IC2 cells 17–19 ─────────────────────────────
  if (!readGroup(LTC_RDCVC, ic0, ic1)) return false;
  parseGroup(ic0, grpV, grpR);
  _cellV[6]=grpV[0]; _cellV[7]=grpV[1]; _cellV[8]=grpV[2];
  _cellRaw[6]=grpR[0]; _cellRaw[7]=grpR[1]; _cellRaw[8]=grpR[2];
  parseGroup(ic1, grpV, grpR);
  _cellV[16]=grpV[0]; _cellV[17]=grpV[1]; _cellV[18]=grpV[2];
  _cellRaw[16]=grpR[0]; _cellRaw[17]=grpR[1]; _cellRaw[18]=grpR[2];

  // ── Group D: IC1 cell 10 only, IC2 cell 20 only (C11/C12 unused on both) ─
  if (!readGroup(LTC_RDCVD, ic0, ic1)) return false;
  parseGroup(ic0, grpV, grpR);
  _cellV[9]=grpV[0];    // C10 only — C11/C12 not connected
  _cellRaw[9]=grpR[0];
  parseGroup(ic1, grpV, grpR);
  _cellV[19]=grpV[0];   // C10 only — C11/C12 not connected
  _cellRaw[19]=grpR[0];

  return true;
}

void ltc_set_balance(bool cells[20]) {
  // Both ICs have identical layout: cells 1–10 → DCC1–DCC10
  // DCC1–DCC8  live in cfgr[4] (ic_dcc4), bits 0–7
  // DCC9–DCC10 live in cfgr[5] (ic_dcc5), bits 0–1

  uint8_t ic1_dcc4 = 0, ic1_dcc5 = 0;
  uint8_t ic2_dcc4 = 0, ic2_dcc5 = 0;

  // IC1 — pack cells 1–10
  for (int i = 0; i < 8;  i++) if (cells[i])      ic1_dcc4 |= (1 << i);
  for (int i = 8; i < 10; i++) if (cells[i])      ic1_dcc5 |= (1 << (i - 8));

  // IC2 — pack cells 11–20
  for (int i = 0; i < 8;  i++) if (cells[i + 10]) ic2_dcc4 |= (1 << i);
  for (int i = 8; i < 10; i++) if (cells[i + 10]) ic2_dcc5 |= (1 << (i - 8));

  uint8_t cfgr_ic2[6] = {0xFC, 0x00, 0x00, 0x00, ic2_dcc4, ic2_dcc5};
  uint8_t cfgr_ic1[6] = {0xFC, 0x00, 0x00, 0x00, ic1_dcc4, ic1_dcc5};

  uint16_t crc_ic2 = ltcCRC15(cfgr_ic2, 6);
  uint16_t crc_ic1 = ltcCRC15(cfgr_ic1, 6);

  uint8_t cmd[4];
  cmd[0] = 0x00; cmd[1] = 0x01;
  uint16_t cmdCrc = ltcCRC15(cmd, 2);
  cmd[2] = (cmdCrc >> 8) & 0xFF;
  cmd[3] = cmdCrc & 0xFF;

  // Daisy-chain write: IC2 data first, IC1 data second
  uint8_t buf[20];
  memcpy(buf, cmd, 4);
  memcpy(buf + 4,  cfgr_ic2, 6);
  buf[10] = (crc_ic2 >> 8) & 0xFF;
  buf[11] = crc_ic2 & 0xFF;
  memcpy(buf + 12, cfgr_ic1, 6);
  buf[18] = (crc_ic1 >> 8) & 0xFF;
  buf[19] = crc_ic1 & 0xFF;

  hspi->beginTransaction(SPISettings(LTC_SPI_CLK, MSBFIRST, SPI_MODE0));
  digitalWrite(HSPI_SS, LOW);
  hspi->transfer(buf, 20);
  digitalWrite(HSPI_SS, HIGH);
  hspi->endTransaction();
}

void ltc_clear_balance() {
  bool none[20] = {};
  ltc_set_balance(none);
}

// Parse 3 cells from a 6-byte group — saves both float V and raw counts
void parseGroup(uint8_t *raw, float *outV, uint16_t *outRaw) {
  for (int i = 0; i < 3; i++) {
    uint16_t val = (uint16_t)raw[i*2] | ((uint16_t)raw[i*2+1] << 8);
    outV[i]   = val * 0.0001f;
    outRaw[i] = val;
  }
}


static float ntcToTemp(float vgpio, float vref2) {
  if (vref2 < 0.01f) return -99.0f;  // guard divide-by-zero
  float rntc = NTC_RBIAS * vgpio / (vref2 - vgpio);
  if (rntc <= 0.0f) return -99.0f;
  float tK = 1.0f / (logf(rntc / NTC_R25) / NTC_BETA + 1.0f / 298.15f);
  return tK - 273.15f;
}

bool ltc_read_temperatures(float tempC[10], bool bal[20]) {
  ltc_wakeup_sleep();
  ltc_set_balance(bal);
  delay(5);
 
  ltcSendCommand(LTC_ADAX);
  delay(10);
 
  uint8_t ic0[6], ic1[6];
 
  if (!readGroup(LTC_RDAUXA, ic0, ic1)) return false;
 
  float ic1_g1 = ((uint16_t)ic0[0] | ((uint16_t)ic0[1] << 8)) * 0.0001f;
  float ic1_g2 = ((uint16_t)ic0[2] | ((uint16_t)ic0[3] << 8)) * 0.0001f;
  float ic1_g3 = ((uint16_t)ic0[4] | ((uint16_t)ic0[5] << 8)) * 0.0001f;
  float ic2_g1 = ((uint16_t)ic1[0] | ((uint16_t)ic1[1] << 8)) * 0.0001f;
  float ic2_g2 = ((uint16_t)ic1[2] | ((uint16_t)ic1[3] << 8)) * 0.0001f;
  float ic2_g3 = ((uint16_t)ic1[4] | ((uint16_t)ic1[5] << 8)) * 0.0001f;
 
  if (!readGroup(LTC_RDAUXB, ic0, ic1)) return false;
 
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
//  ltc_read_status — read STATA + STATB from both ICs
// ════════════════════════════════════════════════════════════════════════════

// status[0] = IC1, status[1] = IC2
bool ltc_read_status(LtcStatus status[2]) {
    // Must run ADSTAT first to populate status registers
    sendCmd(LTC_ADSTAT);
    delay(3); // ~2.5ms conversion time

    uint8_t ic0[6], ic1[6];

    // Status group A: SC[15:0], ITMP[15:0], VA[15:0]
    if (!readGroup(LTC_RDSTATA, ic0, ic1)) return false;

    for (int ic = 0; ic < 2; ic++) {
        uint8_t *d = (ic == 0) ? ic0 : ic1;
        LtcStatus &s = status[ic];

        uint16_t sc_raw   = (uint16_t)d[0] | ((uint16_t)d[1] << 8);
        uint16_t itmp_raw = (uint16_t)d[2] | ((uint16_t)d[3] << 8);
        uint16_t va_raw   = (uint16_t)d[4] | ((uint16_t)d[5] << 8);

        // SC = sum of cells: raw * 0.0001V * 20 (internal scaling factor)
        s.sc_v   = sc_raw * 0.0001f * 20.0f;
        // ITMP: (raw * 0.0001V / 7.5mV per °C) — 273°C offset per datasheet
        s.itmp_c = (itmp_raw * 0.0001f / 0.0075f) - 273.0f;
        s.va_v   = va_raw * 0.0001f;
    }

    // Status group B: VD[15:0], C_UV/C_OV flags, MUXFAIL, THSD, REV
    if (!readGroup(LTC_RDSTATB, ic0, ic1)) return false;

    for (int ic = 0; ic < 2; ic++) {
        uint8_t *d = (ic == 0) ? ic0 : ic1;
        LtcStatus &s = status[ic];

        uint16_t vd_raw = (uint16_t)d[0] | ((uint16_t)d[1] << 8);
        s.vd_v = vd_raw * 0.0001f;

        // d[2]: C4UV|C3UV|C2UV|C1UV flags (bits 7:4:2:0 pairs)
        // d[3]: C8UV|C7UV|C6UV|C5UV
        // d[4]: C12UV|C11UV|C10UV|C9UV
        // Each cell has 2 bits: bit0=UV, bit1=OV
        for (int c = 0; c < 12; c++) {
            uint8_t byte_idx = 2 + (c / 4);
            uint8_t bit_pos  = (c % 4) * 2;
            s.c_uv[c] = (d[byte_idx] >> bit_pos) & 0x01;
            s.c_ov[c] = (d[byte_idx] >> (bit_pos + 1)) & 0x01;
        }

        s.muxfail = (d[5] >> 1) & 0x01;
        s.thsd    = (d[5] >> 0) & 0x01;
        for (int i = 0; i < 4; i++) s.rev[i] = (d[5] >> (4 + i)) & 0x01;
    }

    return true;
}

// ════════════════════════════════════════════════════════════════════════════
//  ltc_comms_test — write known pattern to COMM register, read back, verify
// ════════════════════════════════════════════════════════════════════════════

bool ltc_comms_test() {
    const uint8_t pattern[6] = {0x78, 0x9A, 0xBC, 0xDE, 0xF0, 0x12};

    // Write same pattern to both ICs
    writeGroup(LTC_WRCOMM, pattern, pattern);

    // Read back
    uint8_t ic0[6], ic1[6];
    if (!readGroup(LTC_RDCOMM, ic0, ic1)) {
        Serial.println("[ltc_comms_test] PEC failed on readback");
        return false;
    }

    bool ic1_ok = memcmp(pattern, ic0, 6) == 0;
    bool ic2_ok = memcmp(pattern, ic1, 6) == 0;

    Serial.printf("[ltc_comms_test] IC1: %s  IC2: %s\n",
                  ic1_ok ? "PASS" : "FAIL",
                  ic2_ok ? "PASS" : "FAIL");

    return ic1_ok && ic2_ok;
}