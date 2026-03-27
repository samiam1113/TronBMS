#include <Arduino.h>
#include <SPI.h>
#include "bms_config.h"
#include "LT_SPI.h"
#include "LTC681x.h"
#include "LTC6811.h"

// ── Config ───────────────────────────────────────────────────
#define TOTAL_IC        1       // Set to 1 to isolate IC1 first, then 2

#define CELL_UV_V       2.75f
#define CELL_OV_V       4.20f
#define CELL_UV_RAW     ((uint16_t)(CELL_UV_V / 0.0001f))
#define CELL_OV_RAW     ((uint16_t)(CELL_OV_V / 0.0001f))

static cell_asic BMS_IC[TOTAL_IC];

// ── Helpers ──────────────────────────────────────────────────

static void println_sep(const char* label) {
    Serial.println();
    Serial.print(F("--- ")); Serial.println(label);
}

static void printResult(const char* name, bool pass, const char* detail = "") {
    Serial.print(pass ? F("  [PASS] ") : F("  [FAIL] "));
    Serial.print(name);
    if (strlen(detail)) { Serial.print(F(" -- ")); Serial.print(detail); }
    Serial.println();
}

// ── Stage 1: SPI comms + PEC ─────────────────────────────────

bool test_ltc_comms() {
    println_sep("STAGE 1: SPI Comms + PEC");

    // Init SPI via LT_SPI layer — sets up HSPI on correct pins
    quikeval_SPI_init();
    LTC6811_init_reg_limits(TOTAL_IC, BMS_IC);

    // Manual wake pulse — CS low 500us, well above 10us minimum
    Serial.println(F("  Manual wake pulse..."));
    digitalWrite(LTC_CS_PIN, LOW);
    delayMicroseconds(500);
    digitalWrite(LTC_CS_PIN, HIGH);
    delay(10);

    // Software wake — two pulses ensures isoSPI chain is awake
    Serial.println(F("  Software wake..."));
    wakeup_sleep(TOTAL_IC);
    delay(10);
    wakeup_sleep(TOTAL_IC);
    delay(10);

    // Write UV/OV thresholds
    for (uint8_t i = 0; i < TOTAL_IC; i++) {
        BMS_IC[i].config.tx_data[1] =  (CELL_UV_RAW & 0xFF);
        BMS_IC[i].config.tx_data[2] = ((CELL_OV_RAW & 0x0F) << 4) |
                                       ((CELL_UV_RAW >> 8) & 0x0F);
        BMS_IC[i].config.tx_data[3] =  (CELL_OV_RAW >> 4) & 0xFF;
    }

    LTC6811_wrcfg(TOTAL_IC, BMS_IC);
    delay(1);
    int8_t pec = LTC6811_rdcfg(TOTAL_IC, BMS_IC);

    char buf[48];
    snprintf(buf, sizeof(buf), "PEC=%d (%s)", pec, pec == 0 ? "OK" : "error");
    printResult("SPI comms + PEC", pec == 0, buf);

    if (pec != 0) {
        Serial.println(F("  [INFO] PEC failed. Check:"));
        Serial.println(F("         - CS toggling at LTC6811 CSB pin"));
        Serial.println(F("         - SCLK=IO14 MOSI=IO13 MISO=IO12 CS=IO4"));
        Serial.println(F("         - VREG stable at 5V"));
        Serial.println(F("         - isoSPI termination if TOTAL_IC > 1"));
        return false;
    }

    // Print config registers
    for (uint8_t ic = 0; ic < TOTAL_IC; ic++) {
        char cfg[64];
        snprintf(cfg, sizeof(cfg),
            "  IC%d config: %02X %02X %02X %02X %02X %02X", ic + 1,
            BMS_IC[ic].config.rx_data[0], BMS_IC[ic].config.rx_data[1],
            BMS_IC[ic].config.rx_data[2], BMS_IC[ic].config.rx_data[3],
            BMS_IC[ic].config.rx_data[4], BMS_IC[ic].config.rx_data[5]);
        Serial.println(cfg);
    }

    return true;
}

// ── Stage 2: Cell voltage read ───────────────────────────────

void test_ltc_voltages() {
    println_sep("STAGE 2: Cell Voltage Read");

    wakeup_sleep(TOTAL_IC);
    delay(10);

    LTC6811_adcv(MD_7KHZ_3KHZ, DCP_DISABLED, CELL_CH_ALL);
    LTC6811_pollAdc();
    delay(5);

    int8_t pec = LTC6811_rdcv(REG_ALL, TOTAL_IC, BMS_IC);

    char buf[48];
    snprintf(buf, sizeof(buf), "PEC=%d", pec);
    printResult("Cell voltage read PEC", pec == 0, buf);

    if (pec != 0)
        Serial.println(F("  [WARN] PEC error — values below may be invalid"));

    bool any_uv = false, any_ov = false;
    float pack_v = 0.0f;

    for (uint8_t ic = 0; ic < TOTAL_IC; ic++) {
        Serial.print(F("\n  IC")); Serial.print(ic + 1); Serial.println(F(":"));
        for (uint8_t c = 0; c < 12; c++) {
            uint16_t raw = BMS_IC[ic].cells.c_codes[c];
            float v = raw * 0.0001f;
            pack_v += v;

            bool uv = raw < CELL_UV_RAW;
            bool ov = raw > CELL_OV_RAW;
            if (uv) any_uv = true;
            if (ov) any_ov = true;

            char cv[48];
            snprintf(cv, sizeof(cv), "    C%02d: %5u raw  %.4fV  %s",
                c + 1, raw, v, uv ? "<UV" : ov ? ">OV" : "OK");
            Serial.println(cv);
        }
    }

    Serial.println();
    char ps[48];
    snprintf(ps, sizeof(ps), "%.3fV across %d cells", pack_v, TOTAL_IC * 12);
    printResult("Pack voltage", !any_uv && !any_ov, ps);

    if (any_uv) Serial.println(F("  [WARN] Cell(s) under voltage threshold (2.75V)"));
    if (any_ov) Serial.println(F("  [WARN] Cell(s) over voltage threshold (4.20V)"));

    // Sanity checks
    bool all_zero = true, all_same = true;
    uint16_t first = BMS_IC[0].cells.c_codes[0];
    for (uint8_t ic = 0; ic < TOTAL_IC; ic++) {
        for (uint8_t c = 0; c < 12; c++) {
            if (BMS_IC[ic].cells.c_codes[c] != 0)      all_zero = false;
            if (BMS_IC[ic].cells.c_codes[c] != first)  all_same = false;
        }
    }
    if (all_zero)              Serial.println(F("  [WARN] All cells 0V — ADC may not have converted"));
    if (all_same && !all_zero) Serial.println(F("  [WARN] All cells identical — possible SPI read issue"));
}

// ── Entry points ─────────────────────────────────────────────

void setup() {
    // Deassert CS immediately on boot before any library code runs
    pinMode(LTC_CS_PIN, OUTPUT);
    digitalWrite(LTC_CS_PIN, HIGH);

    Serial.begin(115200);
    delay(500);
    Serial.println(F("LTC6811 COMMS + VOLTAGE TEST"));
    Serial.println(F("============================"));
    Serial.print  (F("TOTAL_IC = ")); Serial.println(TOTAL_IC);
    Serial.print  (F("Pins: SCLK=")); Serial.print(LTC_SCLK_PIN);
    Serial.print  (F(" MISO="));      Serial.print(LTC_MISO_PIN);
    Serial.print  (F(" MOSI="));      Serial.print(LTC_MOSI_PIN);
    Serial.print  (F(" CS="));        Serial.println(LTC_CS_PIN);
    Serial.println(F("\nSend any key to run..."));
}

void loop() {
    if (!Serial.available()) return;
    while (Serial.available()) Serial.read();

    bool comms_ok = test_ltc_comms();
    if (comms_ok) {
        test_ltc_voltages();
    } else {
        Serial.println(F("\n  Skipping voltage read — fix comms first."));
    }

    spi_disable();
    Serial.println(F("\n=== Test complete. Send any key to repeat. ==="));
}