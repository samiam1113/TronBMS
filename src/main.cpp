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
#define ALTERNATE_PINS

#define VSPI_MISO 19
#define VSPI_MOSI 23
#define VSPI_SCLK 18
#define VSPI_SS   -1   // ADS131M02-Q1: single device, CS tied to GND

#define HSPI_MISO 12
#define HSPI_MOSI 13
#define HSPI_SCLK 14
#define HSPI_SS   4    // LTC6811-1 chip select

#if !defined(CONFIG_IDF_TARGET_ESP32)
  #define VSPI FSPI
#endif

// LTC6811-1: max SPI clock is 1 MHz
// ADS131M02-Q1: max SPI clock is 25 MHz, using 1 MHz for safety
static const int spiClk = 1000000;  // 1 MHz

SPIClass *vspi = NULL;
SPIClass *hspi = NULL;

// ── LTC6811-1 wakeup ────────────────────────────────────────────────────────
// Step 1: Toggle CS low for >10us to exit SLEEP state (datasheet p.24)
// Step 2: Send 0xFF dummy byte to exit IDLE state and clock the isoSPI core
void wakeupLTC6811() {

  // Wake from SLEEP — CS pulse must be held low for at least 10us
  digitalWrite(HSPI_SS, LOW);
  delayMicroseconds(300);   // 300us >> 10us minimum, covers worst-case tWAKE
  digitalWrite(HSPI_SS, HIGH);
  delayMicroseconds(10);    // tREADY: wait for LTC to be ready (~10us typical)

  // Wake from IDLE — send a dummy byte to clock the isoSPI interface
  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3)); // LTC6811 uses SPI Mode 3
  digitalWrite(HSPI_SS, LOW);
  hspi->transfer(0xFF);     // Dummy byte — no meaningful command, just clocks isoSPI
  digitalWrite(HSPI_SS, HIGH);
  hspi->endTransaction();

  delayMicroseconds(10);    // Allow isoSPI to stabilise before next command
}

// ── ADS131M02-Q1 wakeup ─────────────────────────────────────────────────────
// Send the WAKEUP command word 0x0033 as a 16-bit transfer (datasheet p.65)
// ADS131M02-Q1 uses SPI Mode 1 (CPOL=0, CPHA=1)
// Device exits standby and resumes conversions after receiving this command
// void wakeupADS131M02() {
//   vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1)); // ADS131M02 uses SPI Mode 1
//   vspi->transfer16(0x0033); // WAKEUP command word per datasheet Table 13
//   vspi->endTransaction();

//   delayMicroseconds(5);     // Wait for device to resume — tSETTLE not specified, 5us is safe
// }



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


    LTC6811_init_reg_limits(TOTAL_IC, BMS_IC);

    wakeupLTC6811();


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

    wakeupLTC6811();
    delay(10);

    LTC6811_adcv(MD_7KHZ_3KHZ, DCP_DISABLED, CELL_CH_ALL);
    LTC6811_pollAdc();
    delay(5);

    int8_t pec = LTC6811_rdcv(0, TOTAL_IC, BMS_IC);

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
    // Deassert CS immediately — before any library code
    pinMode(LTC_CS_PIN, OUTPUT);
    digitalWrite(LTC_CS_PIN, HIGH);

    Serial.begin(115200);
    delay(500);

    // Init both SPI buses
    vspi = new SPIClass(VSPI);
    hspi = new SPIClass(HSPI);

    vspi->begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS);
    hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS);

    pinMode(HSPI_SS, OUTPUT);
    digitalWrite(HSPI_SS, HIGH);

    Serial.println(F("LTC6811 COMMS + VOLTAGE TEST"));
    Serial.println(F("============================"));
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

    // spi_disable() is fine now — it just calls hspi->end()
    spi_disable();
    Serial.println(F("\n=== Test complete. Send any key to repeat. ==="));
}