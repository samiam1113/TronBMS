#include <Arduino.h>
#include <SPI.h>
#include <esp_system.h>
#include "bms_config.h"
#include "bms_thresholds.h"
#include <LTC681x.h>
#include <LTC6811.h>
 
static cell_asic BMS_IC[TOTAL_IC];
static SPIClass  ads_spi(SPI);
 
 
void printResult(const char* name, bool pass, const char* detail = "") {
    Serial.print(pass ? F("  [PASS] ") : F("  [FAIL] "));
    Serial.print(name);
    if (strlen(detail)) { Serial.print(F(" -- ")); Serial.print(detail); }
    Serial.println();
}
 
static uint32_t ads_xfer(uint32_t tx) {
    uint32_t rx = 0;
    rx |= (uint32_t)ads_spi.transfer((tx >> 16) & 0xFF) << 16;
    rx |= (uint32_t)ads_spi.transfer((tx >>  8) & 0xFF) << 8;
    rx |= (uint32_t)ads_spi.transfer( tx        & 0xFF);
    ads_spi.transfer(0x00);
    return rx & 0x00FFFFFF;
}
 
static uint16_t ads_read_reg(uint8_t addr) {
    uint32_t cmd = ((uint32_t)0x20 << 16) | ((uint32_t)(addr & 0x3F) << 10);
    digitalWrite(ADS_CS_PIN, LOW);
    ads_xfer(cmd); ads_xfer(0); ads_xfer(0);
    digitalWrite(ADS_CS_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(ADS_CS_PIN, LOW);
    uint32_t resp = ads_xfer(0); ads_xfer(0); ads_xfer(0);
    digitalWrite(ADS_CS_PIN, HIGH);
    return (uint16_t)((resp >> 8) & 0xFFFF);
}
 
static void ads_read_channels(int32_t* ch0, int32_t* ch1) {
    digitalWrite(ADS_CS_PIN, LOW);
    ads_xfer(0x000000);
    uint32_t r0 = ads_xfer(0);
    uint32_t r1 = ads_xfer(0);
    digitalWrite(ADS_CS_PIN, HIGH);
    *ch0 = (r0 & 0x800000) ? (int32_t)(r0 | 0xFF000000) : (int32_t)r0;
    *ch1 = (r1 & 0x800000) ? (int32_t)(r1 | 0xFF000000) : (int32_t)r1;
}
 
 
void test_ltc6811() {
    Serial.println(F("\n[ LTC6811-1 ]"));
 
    SPI.begin(LTC_SCLK_PIN, LTC_MISO_PIN, LTC_MOSI_PIN, LTC_CS_PIN);
    LTC6811_init_reg_limits(TOTAL_IC, BMS_IC);
    wakeup_sleep(TOTAL_IC);
 
    // Set UV/OV thresholds from bms_thresholds.h
    for (uint8_t i = 0; i < TOTAL_IC; i++) {
        BMS_IC[i].config.tx_data[1] =  (CELL_UV_RAW & 0xFF);
        BMS_IC[i].config.tx_data[2] = ((CELL_OV_RAW & 0x00F) << 4) |
                                       ((CELL_UV_RAW >> 8) & 0x0F);
        BMS_IC[i].config.tx_data[3] =  (CELL_OV_RAW >> 4) & 0xFF;
    }
 
    // SPI comms check
    LTC6811_wrcfg(TOTAL_IC, BMS_IC);
    int8_t pec = LTC6811_rdcfg(TOTAL_IC, BMS_IC);
    printResult("SPI comms + PEC", pec == 0,
        pec == 0 ? "PEC OK" : "PEC error -- check IO14/12/13/4");
    if (pec != 0) return;
 
    // Cell voltages
    wakeup_sleep(TOTAL_IC);
    LTC6811_adcv(MD_7KHZ_3KHZ, DCP_DISABLED, CELL_CH_ALL);
    LTC6811_pollAdc();
    LTC6811_rdcv(REG_1, TOTAL_IC, BMS_IC);
 
    bool any_uv = false, any_ov = false;
    float pack_v = 0;
    for (uint8_t ic = 0; ic < TOTAL_IC; ic++) {
        for (uint8_t c = 0; c < 12; c++) {
            uint16_t raw = BMS_IC[ic].cells.c_codes[c];
            float v = raw * 0.0001f;
            pack_v += v;
            if (raw < CELL_UV_RAW) any_uv = true;
            if (raw > CELL_OV_RAW) any_ov = true;
        }
    }
 
    char dv[48];
    float ic1c1 = BMS_IC[0].cells.c_codes[0] * 0.0001f;
    snprintf(dv, sizeof(dv), "IC1 C1=%.3fV  pack~%.1fV", ic1c1, pack_v);
    printResult("Cell voltages in range (2.75-4.1V)", !any_uv && !any_ov, dv);
 
    if (any_uv) Serial.println(F("  [WARN] Cell(s) UNDER voltage threshold (2.75V)"));
    if (any_ov) Serial.println(F("  [WARN] Cell(s) OVER voltage threshold (4.1V)"));
 
    // Pack voltage sanity check
    bool pack_ok = (pack_v >= PACK_UV_V && pack_v <= PACK_OV_V);
    char dp[32]; snprintf(dp, sizeof(dp), "%.1fV (%.0f-%.0fV)", pack_v, PACK_UV_V, PACK_OV_V);
    printResult("Pack voltage in range (55-82V)", pack_ok, dp);
}
 
void test_ads131() {
    Serial.println(F("\n[ ADS131M02 ]"));
 
    pinMode(ADS_RST_PIN, OUTPUT);
    pinMode(ADS_CS_PIN,  OUTPUT);
    digitalWrite(ADS_CS_PIN,  HIGH);
    digitalWrite(ADS_RST_PIN, LOW);  delayMicroseconds(20);
    digitalWrite(ADS_RST_PIN, HIGH); delay(5);
 
    ads_spi.begin(ADS_SCLK_PIN, ADS_MISO_PIN, ADS_MOSI_PIN, ADS_CS_PIN);
    ads_spi.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE1));
 
    // Device ID
    uint16_t id = ads_read_reg(0x00);
    char d[48]; snprintf(d, sizeof(d), "ID=0x%04X", id);
    printResult("Device ID (expect 0x2282)", id == ADS_EXPECTED_ID, d);
    if (id != ADS_EXPECTED_ID) { ads_spi.endTransaction(); return; }
 
    // Set gain = 4 on both channels
    // CLOCK reg (0x03): CH0 and CH1 enabled, OSR=4096 for 1µs effective rate
    // CH0_CFG (0x09) and CH1_CFG (0x0E): gain field bits[4:2] = 0b010 for gain 4
    // Write gain reg: WREG cmd = 0x40 << 16 | addr << 10
    auto wreg = [](SPIClass& spi, uint8_t cs, uint8_t addr, uint16_t val) {
        uint32_t cmd = ((uint32_t)0x40 << 16) | ((uint32_t)(addr & 0x3F) << 10);
        uint32_t data_word = (uint32_t)val << 8;
        digitalWrite(cs, LOW);
        // Frame 1: command
        spi.transfer((cmd >> 16) & 0xFF);
        spi.transfer((cmd >>  8) & 0xFF);
        spi.transfer( cmd        & 0xFF);
        spi.transfer(0x00);
        // Frame 1 word 2: register value
        spi.transfer((data_word >> 16) & 0xFF);
        spi.transfer((data_word >>  8) & 0xFF);
        spi.transfer( data_word        & 0xFF);
        spi.transfer(0x00);
        // Frame 1 word 3: dummy
        spi.transfer(0x00); spi.transfer(0x00);
        spi.transfer(0x00); spi.transfer(0x00);
        digitalWrite(cs, HIGH);
        delayMicroseconds(5);
    };
 
    // Gain of 4 = 0b010 in bits[4:2] of CH_CFG register
    const uint16_t GAIN4 = (0b010 << 2);
    wreg(ads_spi, ADS_CS_PIN, 0x09, GAIN4);  // CH0_CFG
    wreg(ads_spi, ADS_CS_PIN, 0x0E, GAIN4);  // CH1_CFG
 
    // Verify gain was written by reading back CH0_CFG
    uint16_t ch0cfg = ads_read_reg(0x09);
    bool gain_ok = ((ch0cfg >> 2) & 0x07) == 0b010;
    snprintf(d, sizeof(d), "CH0_CFG=0x%04X", ch0cfg);
    printResult("Gain=4 set on CH0", gain_ok, d);
 
    // Zero-current baseline (no load)
    Serial.println(F("  [INFO] Ensure no current through shunt for baseline check"));
    delay(100);
    int32_t ch0, ch1;
    ads_read_channels(&ch0, &ch1);
    float amps = ch0 * ADS_COUNTS_TO_AMPS;
    snprintf(d, sizeof(d), "CH0=%ld counts (%.2fA)", ch0, amps);
    // At zero current expect near 0 counts — accept within 5% full scale
    printResult("Zero-current baseline", abs(ch0) < 419430, d);
 
    ads_spi.endTransaction();
 
    // Print current limits for reference
    Serial.println(F("  [INFO] Current limits loaded:"));
    Serial.print(F("         Discharge: ")); Serial.print(CURR_DSCHG_CONT_A);
    Serial.print(F("A cont / ")); Serial.print(CURR_DSCHG_PEAK_A); Serial.println(F("A peak (<1s)"));
    Serial.print(F("         Charge:    ")); Serial.print(CURR_CHG_CONT_A);
    Serial.print(F("A cont / ")); Serial.print(CURR_CHG_PEAK_A);  Serial.println(F("A peak"));
    Serial.print(F("         Sample rate: every ")); Serial.print(ADS_SAMPLE_US); Serial.println(F("us"));
}
 
void test_ucc37322() {
    Serial.println(F("\n[ UCC37322DR ]"));
 
    pinMode(GATE_DSCHG_PIN, OUTPUT);
    pinMode(GATE_CHG_PIN,   OUTPUT);
    digitalWrite(GATE_DSCHG_PIN, LOW);
    digitalWrite(GATE_CHG_PIN,   LOW);
 
    Serial.println(F("  Pulsing DSCHG (IO25) and CHG (IO26) 3x each..."));
    for (uint8_t i = 0; i < 3; i++) {
        digitalWrite(GATE_DSCHG_PIN, HIGH);
        digitalWrite(GATE_CHG_PIN,   HIGH);
        delay(300);
        digitalWrite(GATE_DSCHG_PIN, LOW);
        digitalWrite(GATE_CHG_PIN,   LOW);
        delay(300);
    }
 
    Serial.println(F("  Did both gate pins toggle? (y/n)"));
    uint32_t t0 = millis();
    while (millis() - t0 < 10000) {
        if (Serial.available()) {
            char c = Serial.read();
            printResult("Gate pins toggle", c == 'y' || c == 'Y',
                c == 'y' || c == 'Y' ? "confirmed" : "not observed");
            return;
        }
    }
    printResult("Gate pins toggle", false, "timed out");
}
 
// ── Entry points ──────────────────────────────────────────────
 
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println(F("  BMS BASIC FUNCTIONALITY TEST"));
    Serial.println(F("  Send any key to begin..."));
    Serial.println(F("  Thresholds:"));
    Serial.print  (F("    Cell UV=2.75V  OV=4.1V  |  Pack UV=55V  OV=82V\n"));
    Serial.print  (F("    Temp warn=60C avg  cutoff=70C any sensor\n"));
    Serial.print  (F("    Balance delta=2mV\n"));
}
 
void loop() {
    if (!Serial.available()) return;
    while (Serial.available()) Serial.read();
 
    Serial.println(F("\n Running tests"));
    test_ltc6811();
    test_ads131();
    test_ucc37322();
    Serial.println(F("\n Tests complete"));
}
