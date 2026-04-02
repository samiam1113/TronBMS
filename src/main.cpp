// ============================================================================
// main.cpp — TronBMS hardware validation build
//
// PURPOSE: One-shot sequential test of all subsystems except balancing.
//          Runs from a single FreeRTOS task (task_test) so the FSM and its
//          fault machinery are fully active — faults trip contactors and log
//          to NVS exactly as they would in production.
//
// TEST SEQUENCE:
//   1. NVS init + previous fault report
//   2. GPIO / SPI init
//   3. LTC6811 wakeup + comms test (WRCOMM/RDCOMM loopback)
//   4. ADS131M02 configure + ID verify
//   5. Gate driver selftest
//   6. LTC6811 WRCFGA — write and read back config (REFON=1, DCC=0)
//   7. Cell voltage read — prints all 20 cells, flags any OV/UV
//   8. Temperature read — prints all 5 sensors, flags any OT
//   9. ADS131M02 current read — prints raw count + amps
//  10. Protection check pass-through — runs all meas_check_* functions
//      against the sampled data and trips FSM fault state if any trigger
//
// OMITTED (not under test):
//   - task_balance / balance_compute_mask / balance_apply
//   - task_daq / CAN telemetry
//   - Sleep state
//
// After the test sequence completes the task prints a PASS/FAIL summary and
// suspends. The FSM continues running in task_fsm — if any fault was tripped
// during the test it will remain in FAULT state with contactors open.
// ============================================================================

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "bms_config.h"
#include "bms_types.h"
#include "bms_fsm.h"
#include "bms_fault.h"
#include "bms_hardware.h"
#include "bms_measurements.h"
#include "bms_tasks.h"
#include "ltc_spi.h"

// ── Global FSM context — extern'd by bms_tasks.cpp and bms_fsm.cpp ───────────
BmsFsm g_fsm;

// ── Test result tracking ──────────────────────────────────────────────────────
static int s_tests_run    = 0;
static int s_tests_passed = 0;

static void test_result(const char *name, bool passed) {
    s_tests_run++;
    if (passed) s_tests_passed++;
    Serial.printf("  [%s] %s\n", passed ? "PASS" : "FAIL", name);
}

// ── Separator helpers ─────────────────────────────────────────────────────────
static void print_section(const char *title) {
    Serial.printf("\n── %s ", title);
    for (int i = strlen(title); i < 40; i++) Serial.print('-');
    Serial.println();
}

// ============================================================================
// task_test — runs once, suspends on completion
// ============================================================================
static void task_test(void *pvParameters) {
    // Small settle delay — let Serial flush and SPI peripherals stabilise
    vTaskDelay(pdMS_TO_TICKS(200));

    Serial.println("\n╔══════════════════════════════════════════╗");
    Serial.println("║     TronBMS Hardware Validation Test     ║");
    Serial.println("╚══════════════════════════════════════════╝");

    measurement_data_t meas = {};

    // ── Wake LTC chain before any register access ─────────────────────────────
    ltc_wakeup_sleep();
    ltc_wakeup_idle();

    // =========================================================================
    // TEST 1 — LTC6811 comms loopback (WRCOMM / RDCOMM)
    // =========================================================================
    print_section("TEST 1: LTC6811 Comms");
    {
        bool ok = ltc_comms_test();
        test_result("LTC6811 WRCOMM/RDCOMM loopback (both ICs)", ok);
        if (!ok) {
            fault_set(static_cast<uint16_t>(Fault::SPI));
            fsm_set_state(g_fsm, BmsState::FAULT);
            Serial.println("  !! LTC comms failed — remaining LTC tests skipped");
            goto print_summary;  // no point continuing LTC tests
        }
    }

    // =========================================================================
    // TEST 2 — LTC6811 config write/readback (REFON=1, DCC=0)
    // =========================================================================
    print_section("TEST 2: LTC6811 Config Write/Readback");
    {
        LtcConfig cfg_write[TOTAL_IC] = {};
        for (int ic = 0; ic < TOTAL_IC; ic++) {
            cfg_write[ic].refon    = true;
            cfg_write[ic].adcopt  = false;
            cfg_write[ic].dcto    = 0x00;
            cfg_write[ic].dcc     = 0x0000;
            cfg_write[ic].vuv     = (uint16_t)((CELL_UV_RAW / 16u) - 1u);
            cfg_write[ic].vov     = (uint16_t) (CELL_OV_RAW / 16u);
            for (int g = 0; g < 5; g++) cfg_write[ic].gpio_pulldown[g] = false;
        }
        ltc_write_config(cfg_write);
        vTaskDelay(pdMS_TO_TICKS(2));  // settle

        LtcConfig cfg_read[TOTAL_IC] = {};
        bool readback_ok = ltc_read_config(cfg_read);
        test_result("RDCFGA PEC valid", readback_ok);

        if (readback_ok) {
            for (int ic = 0; ic < TOTAL_IC; ic++) {
                bool refon_ok = (cfg_read[ic].refon  == cfg_write[ic].refon);
                bool vuv_ok   = (cfg_read[ic].vuv    == cfg_write[ic].vuv);
                bool vov_ok   = (cfg_read[ic].vov    == cfg_write[ic].vov);
                bool dcc_ok   = (cfg_read[ic].dcc    == 0x0000);
                char label[48];
                snprintf(label, sizeof(label), "IC%d REFON/VUV/VOV/DCC match", ic + 1);
                test_result(label, refon_ok && vuv_ok && vov_ok && dcc_ok);
                Serial.printf("    IC%d: REFON=%d VUV=0x%03X(exp 0x%03X) "
                              "VOV=0x%03X(exp 0x%03X) DCC=0x%03X\n",
                              ic + 1,
                              cfg_read[ic].refon,
                              cfg_read[ic].vuv,  cfg_write[ic].vuv,
                              cfg_read[ic].vov,  cfg_write[ic].vov,
                              cfg_read[ic].dcc);
            }
        }
    }

    // =========================================================================
    // TEST 3 — Cell voltage read
    // =========================================================================
    print_section("TEST 3: Cell Voltages");
    {
        // ltc_read_voltages now issues ADCV internally
        bool ok = ltc_read_voltages(&meas);
        test_result("ltc_read_voltages() PEC valid", ok);

        if (ok) {
            bool any_ov = false, any_uv = false, any_zero = false;
            for (int ic = 0; ic < TOTAL_IC; ic++) {
                for (int c = 0; c < CELLS_PER_IC; c++) {
                    float v = meas.cell_v[ic][c];
                    bool  ov = (v >= CELL_OV_V);
                    bool  uv = (v <= CELL_UV_V);
                    bool  zero = (v < 0.01f);
                    if (ov)   any_ov   = true;
                    if (uv)   any_uv   = true;
                    if (zero) any_zero = true;
                    Serial.printf("    IC%d C%02d: %.4f V%s%s%s\n",
                                  ic + 1, c + 1, v,
                                  ov   ? " [OV]"   : "",
                                  uv   ? " [UV]"   : "",
                                  zero ? " [ZERO — check cell/wiring]" : "");
                }
            }
            test_result("No cells at 0V (open circuit check)", !any_zero);
            test_result("No cell OV",  !any_ov);
            test_result("No cell UV",  !any_uv);

            // Trip FSM if protection checks fail
            if (any_ov) { fault_set(static_cast<uint16_t>(Fault::CELL_OV)); fsm_set_state(g_fsm, BmsState::FAULT); }
            if (any_uv) { fault_set(static_cast<uint16_t>(Fault::CELL_UV)); fsm_set_state(g_fsm, BmsState::FAULT); }
        } else {
            fault_set(static_cast<uint16_t>(Fault::SPI));
            fsm_set_state(g_fsm, BmsState::FAULT);
        }
    }

    // =========================================================================
    // TEST 4 — Temperature read
    // =========================================================================
    print_section("TEST 4: Temperatures");
    {
        bool ok = ltc_read_temperatures(&meas);
        test_result("ltc_read_temperatures() PEC valid", ok);

        if (ok) {
            bool any_ot   = false;
            bool any_open = false;  // ntcToTemp returns -99 on open/short
            for (int i = 0; i < NUM_TEMP_SENSORS; i++) {
                float t = meas.temps[i];
                bool  ot   = (t >= (float)TEMP_CUTOFF_C);
                bool  open = (t < -50.0f);
                if (ot)   any_ot   = true;
                if (open) any_open = true;
                Serial.printf("    Sensor %d: %.1f °C%s%s\n",
                              i,
                              t,
                              ot   ? " [OT]"                     : "",
                              open ? " [OPEN/SHORT — check NTC]" : "");
            }
            test_result("No sensors open/shorted (-99 °C)", !any_open);
            test_result("No sensors over TEMP_CUTOFF_C",     !any_ot);

            if (any_ot) { fault_set(static_cast<uint16_t>(Fault::OT)); fsm_set_state(g_fsm, BmsState::FAULT); }
        } else {
            fault_set(static_cast<uint16_t>(Fault::SPI));
            fsm_set_state(g_fsm, BmsState::FAULT);
        }
    }

    // =========================================================================
    // TEST 5 — ADS131M02 configure + ID verify
    // =========================================================================
    print_section("TEST 5: ADS131M02 Configure + ID");
    {
        bool cfg_ok = ads_configure();
        test_result("ADS131M02 CLOCK + GAIN1 register readback match", cfg_ok);

        bool id_ok  = ads_checkid();
        test_result("ADS131M02 ID register matches 0x2282", id_ok);

        if (!cfg_ok || !id_ok) {
            fault_set(static_cast<uint16_t>(Fault::ADS_ID));
            fsm_set_state(g_fsm, BmsState::FAULT);
        }
    }

    // =========================================================================
    // TEST 6 — ADS131M02 current read
    // =========================================================================
    print_section("TEST 6: ADS131M02 Current");
    {
        // Take 8 samples and average to reduce noise
        const int   N = 8;
        int32_t     sum = 0;
        for (int i = 0; i < N; i++) {
            sum += ads_read_raw();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        int32_t avg_raw = sum / N;
        float   amps    = ads_counts_to_amps(avg_raw);
        meas.current_a  = amps;

        Serial.printf("    Raw (avg %d samples): %ld counts\n", N, (long)avg_raw);
        Serial.printf("    Current: %.3f A (expect ~0 A at rest)\n", amps);

        // At rest with no load the current should be within ±2 A of zero.
        // If it's way off, the shunt wiring or gain config is wrong.
        bool idle_ok = (amps > -2.0f && amps < 2.0f);
        test_result("Idle current within ±2 A of zero", idle_ok);

        // Also run the overcurrent protection check against sampled data
        bool oc = meas_check_overcurrent(&meas);
        test_result("meas_check_overcurrent() — no fault at rest", !oc);
        if (oc) { fault_set(static_cast<uint16_t>(Fault::OC_DSG)); fsm_set_state(g_fsm, BmsState::FAULT); }
    }

    // =========================================================================
    // TEST 7 — Gate driver selftest
    // =========================================================================
    print_section("TEST 7: Gate Driver");
    {
        bool ok = gate_driver_selftest();
        test_result("gate_driver_selftest() returned true", ok);
        // Note: selftest currently sets outputs LOW only (no feedback pin).
        // Contactors should remain open — verify with a multimeter.
        Serial.println("    >> Verify with multimeter: CHG and DSG outputs = LOW");
        if (!ok) { fault_set(static_cast<uint16_t>(Fault::GATE)); fsm_set_state(g_fsm, BmsState::FAULT); }
    }

    // =========================================================================
    // SUMMARY
    // =========================================================================
    print_summary:
    Serial.println();
    Serial.println("══════════════════════════════════════════");
    Serial.printf( "  RESULT: %d / %d tests passed\n", s_tests_passed, s_tests_run);
    if (s_tests_passed == s_tests_run) {
        Serial.println("  STATUS: ALL PASS ✓");
    } else {
        Serial.printf( "  STATUS: %d FAILED\n", s_tests_run - s_tests_passed);
    }
    Serial.printf(  "  FSM state after test: %d\n",
                    static_cast<uint8_t>(fsm_get_state(g_fsm)));
    Serial.printf(  "  Fault register: 0x%04X\n", fault_get());
    Serial.println("══════════════════════════════════════════");
    Serial.println("  task_test suspended. FSM still running.");
    Serial.println("  Reset ESP32 to run again.");

    vTaskSuspend(nullptr);
}

// ============================================================================
// setup / loop
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("[main] TronBMS test build starting...");

    // NVS init
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Hardware init
    bms_gpio_init();
    bms_spi_init();

    // RTOS primitives
    g_event_group    = xEventGroupCreate();
    g_meas_mutex     = xSemaphoreCreateMutex();
    g_snapshot_mutex = xSemaphoreCreateMutex();  // Fix #4: guards fault snapshot struct + flag
    g_meas_queue     = xQueueCreate(2, sizeof(measurement_data_t));

    // FSM init — starts in INIT state, contactors open
    fsm_init(g_fsm);

    // Report any fault from the previous run
    fault_snapshot_t prev_snap = {};
    if (fault_log_read(&prev_snap)) {
        Serial.printf("[main] Previous fault: code=%d state=%d\n",
                      static_cast<uint8_t>(prev_snap.code),
                      static_cast<uint8_t>(prev_snap.state_at_fault));
    }

    // Spawn only the tasks needed for this test build.
    // task_balance and task_daq are intentionally omitted.
    xTaskCreatePinnedToCore(task_fsm,      "fsm",      8192, nullptr, 4, nullptr,  1);
    xTaskCreatePinnedToCore(task_watchdog, "wdt",      2048, nullptr, 5, nullptr,  1);
    xTaskCreatePinnedToCore(task_test,     "test",     8192, nullptr, 3, nullptr,  1);

    Serial.println("[main] Tasks running — waiting for test output.");
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}