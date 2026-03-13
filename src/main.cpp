#include <Arduino.h>
#include <SPI.h>
#include <esp_system.h>
#include "bms_config.h"
#include <LTC681x.h>
#include <LTC6811.h>

//  BMS TEST
//  Send any key via serial monitor to run all tests.

static cell_asic BMS_IC[TOTAL_IC];

static SPIClass ads_spi(SPI);


void printResult(const char* name, bool pass, const char* detail = "") {
    Serial.print(pass ? F("  [PASS] ") : F("  [FAIL] "));
    Serial.print(name);
    if (strlen(detail)) { Serial.print(F(" — ")); Serial.print(detail); }
    Serial.println();
}

// ADS131: transfer one 24-bit word + padding byte
static uint32_t ads_xfer(uint32_t tx) {
    uint32_t rx = 0;
    rx |= (uint32_t)ads_spi.transfer((tx >> 16) & 0xFF) << 16;
    rx |= (uint32_t)ads_spi.transfer((tx >>  8) & 0xFF) << 8;
    rx |= (uint32_t)ads_spi.transfer( tx        & 0xFF);
    ads_spi.transfer(0x00);
    return rx & 0x00FFFFFF;
}

// ADS131: read one register 
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

// Tests for each programmable component
void test_esp32() {
    Serial.println(F("\n[ ESP32 ]"));

    uint32_t heap = esp_get_free_heap_size();
    char d[32];
    snprintf(d, sizeof(d), "%lu bytes", heap);
    printResult("Free heap", heap > 100000, d);

    uint32_t flash = ESP.getFlashChipSize();
    snprintf(d, sizeof(d), "%luMB", flash / (1024*1024));
    printResult("Flash size", flash >= 4*1024*1024, d);
}

void test_ltc6811() {
    Serial.println(F("\n[ LTC6811-1 ]"));

    SPI.begin(LTC_SCLK_PIN, LTC_MISO_PIN, LTC_MOSI_PIN, LTC_CS_PIN);
    LTC6811_init_reg_limits(TOTAL_IC, BMS_IC);
    wakeup_sleep(TOTAL_IC);

    // Write/read config — PEC mismatch = SPI comms failure
    LTC6811_wrcfg(TOTAL_IC, BMS_IC);
    int8_t pec = LTC6811_rdcfg(TOTAL_IC, BMS_IC);
    printResult("SPI comms + PEC", pec == 0,
        pec == 0 ? "PEC OK" : "PEC error — check IO14/12/13/4 wiring");

    if (pec != 0) return;

    // Trigger a cell voltage conversion and read back
    wakeup_sleep(TOTAL_IC);
    LTC6811_adcv(MD_7KHZ_3KHZ, DCP_DISABLED, CELL_CH_ALL);
    LTC6811_pollAdc();
    int8_t cv_pec = LTC6811_rdcv(REG_1, TOTAL_IC, BMS_IC);

    float v = BMS_IC[0].cells.c_codes[0] * 0.0001f;
    bool ok = (cv_pec == 0 && v > 2.0f && v < 4.5f);
    char dv[32];
    snprintf(dv, sizeof(dv), "IC1 C1=%.3fV", v);
    printResult("Cell voltage read", ok, dv);
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

    uint16_t id = ads_read_reg(0x00);
    char d[32];
    snprintf(d, sizeof(d), "ID=0x%04X", id);
    printResult("Device ID (expect 0x2282)", id == ADS_EXPECTED_ID, d);

    ads_spi.endTransaction();
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


void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println(F("  BMS TEST Press any key to continue"));
}

void loop() {
    if (!Serial.available()) return;
    while (Serial.available()) Serial.read();

    Serial.println(F("\n--- Running tests ---"));
    test_esp32();
    test_ltc6811();
    test_ads131();
    test_ucc37322();
    Serial.println(F("\n Tests complete. Press any key to run again."));
}
