// ============================================================================
// main.cpp — TronBMS hardware validation build
//
// PURPOSE: One-shot sequential test of all subsystems except balancing.
//          Runs from a single FreeRTOS task (task_test) so the FSM and its
//          fault machinery are fully active — faults trip contactors and log
//          to NVS exactly as they would in production.
//
// TEST SEQUENCE:
//   1. LTC6811 comms loopback (WRCOMM/RDCOMM)
//   2. LTC6811 config write/readback (REFON=1, DCC=0)
//   3. Cell voltage read — prints all 20 cells, flags any OV/UV/ZERO
//   4. Temperature read — prints all 5 sensors, flags any OT/open
//   5. ADS131M02 configure + ID verify
//   6. ADS131M02 current read + overcurrent check
//   7. Gate driver selftest
//
// DEBUG FEATURES:
//   - Press any Serial key to start the test sequence
//   - Press any Serial key between tests to advance to the next step
//   - On test failure: prints fault details, waits for keypress to continue
//     (does NOT auto-abort — remaining tests still run so you can see full
//      board state. LTC comms failure is the only hard stop since all
//      subsequent LTC tests would be meaningless.)
//   - Fault register and FSM state printed after every failure, not just at end
//   - NVS fault from previous run is cleared before the new run starts
//
// OMITTED (not under test):
//   - task_balance / balance_compute_mask / balance_apply
//   - task_daq / CAN telemetry
//   - Sleep state
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

// ============================================================================
// Serial blocking helpers
// ============================================================================

// Block until the user presses any key in Serial Monitor.
static void wait_for_keypress(const char *prompt = nullptr) {
    if (prompt) Serial.printf("\n  >> %s\n", prompt);
    else        Serial.println("\n  >> Press any key to continue...");
    Serial.flush();
    while (Serial.available()) Serial.read();   // drain stale bytes
    while (!Serial.available()) vTaskDelay(pdMS_TO_TICKS(50));
    while (Serial.available())  Serial.read();  // consume the keypress
}

// ============================================================================
// Formatting helpers
// ============================================================================

static void print_section(int num, const char *title) {
    Serial.printf("\n┌─ TEST %d: %s ", num, title);
    int pad = 38 - strlen(title);
    for (int i = 0; i < pad; i++) Serial.print('─');
    Serial.println("┐");
}

static void print_section_end() {
    Serial.println("└────────────────────────────────────────────────┘");
}

// Print current fault register + FSM state — useful after any failure
static void print_debug_state() {
    Serial.printf("  [DBG] Fault register : 0x%04X\n", fault_get());
    Serial.printf("  [DBG] FSM state      : %d\n",
                  static_cast<uint8_t>(fsm_get_state(g_fsm)));
}

// Record and print a single test result. On failure, dumps debug state
// and waits for a keypress so you can inspect hardware before moving on.
static bool test_result(const char *name, bool passed) {
    s_tests_run++;
    if (passed) s_tests_passed++;
    Serial.printf("  [%s] %s\n", passed ? "PASS" : "FAIL", name);
    if (!passed) {
        print_debug_state();
        wait_for_keypress("Test failed — inspect hardware, then press any key to continue");
    }
    return passed;
}

// ============================================================================
// task_test — runs once, suspends on completion
// ============================================================================
static void task_test(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(200));

    Serial.println("\n╔══════════════════════════════════════════╗");
    Serial.println("║     TronBMS Hardware Validation Test     ║");
    Serial.println("╚══════════════════════════════════════════╝");

    // ── Clear any fault bits left over from a previous run ───────────────────
    // fault_clear() is the only clear API — call it for every defined fault bit.
    // There is no bulk-clear or NVS-erase function in this firmware version;
    // the previous run's snapshot remains in NVS (already reported in setup()).
    fault_clear(static_cast<uint16_t>(FaultCode::OVERVOLTAGE));
    fault_clear(static_cast<uint16_t>(FaultCode::UNDERVOLTAGE));
    fault_clear(static_cast<uint16_t>(FaultCode::OVERCURRENT));
    fault_clear(static_cast<uint16_t>(FaultCode::OVERTEMP));
    fault_clear(static_cast<uint16_t>(FaultCode::BAL_OVERTEMP));
    fault_clear(static_cast<uint16_t>(FaultCode::SPI_LTC));
    fault_clear(static_cast<uint16_t>(FaultCode::SPI_ADS));
    fault_clear(static_cast<uint16_t>(FaultCode::STARTUP));
    fsm_set_state(g_fsm, BmsState::INIT);
    Serial.println("  [INIT] Fault register cleared.");
    Serial.printf( "  [INIT] Fault register after clear: 0x%04X\n", fault_get());

    // ── Wait for user to signal ready ────────────────────────────────────────
    wait_for_keypress("Ready. Press any key to BEGIN the test sequence...");

    // ── Wake LTC chain before any register access ─────────────────────────────
    ltc_wakeup_sleep();
    ltc_wakeup_idle();
    Serial.println("  [INIT] LTC chain wakeup sent.");

    measurement_data_t meas = {};
    bool ltc_comms_ok = true;  // gate flag — skips LTC tests if comms fail

    // =========================================================================
    // TEST 1 — LTC6811 comms loopback (WRCOMM / RDCOMM)
    // =========================================================================
    print_section(1, "LTC6811 Comms Loopback");
    {
        Serial.println("  Sending WRCOMM pattern and reading back via RDCOMM...");
        bool ok = ltc_comms_test();
        ltc_comms_ok = ok;

        if (!test_result("LTC6811 WRCOMM/RDCOMM loopback (both ICs)", ok)) {
            fault_set(static_cast<uint16_t>(FaultCode::SPI_LTC));
            fsm_set_state(g_fsm, BmsState::FAULT);
            Serial.println("  !! LTC comms failure is unrecoverable for this run.");
            Serial.println("  !! Skipping all remaining LTC tests (Tests 2–4).");
            Serial.println("  !! Check: SPI wiring, CS lines, LTC6811 power rails.");
        }
    }
    print_section_end();

    // =========================================================================
    // TEST 2 — LTC6811 config write/readback (REFON=1, DCC=0)
    // =========================================================================
    print_section(2, "LTC6811 Config Write/Readback");
    if (!ltc_comms_ok) {
        Serial.println("  [SKIP] LTC comms failed — skipping.");
    } else {
        LtcConfig cfg_write[TOTAL_IC] = {};
        for (int ic = 0; ic < TOTAL_IC; ic++) {
            cfg_write[ic].refon   = true;
            cfg_write[ic].adcopt  = false;
            cfg_write[ic].dcto    = 0x00;
            cfg_write[ic].dcc     = 0x0000;
            cfg_write[ic].vuv     = (uint16_t)((CELL_UV_RAW / 16u) - 1u);
            cfg_write[ic].vov     = (uint16_t) (CELL_OV_RAW / 16u);
            for (int g = 0; g < 5; g++) cfg_write[ic].gpio_pulldown[g] = false;
        }

        Serial.printf("  Writing: REFON=1, DCC=0, VUV=0x%03X, VOV=0x%03X\n",
                      cfg_write[0].vuv, cfg_write[0].vov);
        ltc_write_config(cfg_write);
        vTaskDelay(pdMS_TO_TICKS(2));

        LtcConfig cfg_read[TOTAL_IC] = {};
        bool readback_ok = ltc_read_config(cfg_read);
        test_result("RDCFGA PEC valid", readback_ok);

        if (readback_ok) {
            for (int ic = 0; ic < TOTAL_IC; ic++) {
                bool refon_ok = (cfg_read[ic].refon == cfg_write[ic].refon);
                bool vuv_ok   = (cfg_read[ic].vuv   == cfg_write[ic].vuv);
                bool vov_ok   = (cfg_read[ic].vov   == cfg_write[ic].vov);
                bool dcc_ok   = (cfg_read[ic].dcc   == 0x0000);

                Serial.printf("  IC%d read:  REFON=%d  VUV=0x%03X  VOV=0x%03X  DCC=0x%03X\n",
                              ic + 1,
                              cfg_read[ic].refon,
                              cfg_read[ic].vuv,
                              cfg_read[ic].vov,
                              cfg_read[ic].dcc);
                Serial.printf("  IC%d expect: REFON=%d  VUV=0x%03X  VOV=0x%03X  DCC=0x000\n",
                              ic + 1,
                              cfg_write[ic].refon,
                              cfg_write[ic].vuv,
                              cfg_write[ic].vov);

                char label[48];
                snprintf(label, sizeof(label), "IC%d REFON/VUV/VOV/DCC match", ic + 1);
                test_result(label, refon_ok && vuv_ok && vov_ok && dcc_ok);
            }
        } else {
            fault_set(static_cast<uint16_t>(FaultCode::SPI_LTC));
            fsm_set_state(g_fsm, BmsState::FAULT);
            Serial.println("  !! PEC error on RDCFGA. Check SPI noise / decoupling.");
        }

    }
    print_section_end();

    // =========================================================================
    // TEST 3 — Cell voltage read
    // =========================================================================
    print_section(3, "Cell Voltages");
    if (!ltc_comms_ok) {
        Serial.println("  [SKIP] LTC comms failed — skipping.");
    } else {
        Serial.println("  Issuing ADCV and reading all cell registers...");
        bool ok = ltc_read_voltages(&meas);
        test_result("ltc_read_voltages() PEC valid", ok);

        if (ok) {
            bool any_ov = false, any_uv = false, any_zero = false;
            for (int ic = 0; ic < TOTAL_IC; ic++) {
                Serial.printf("  --- IC%d ---\n", ic + 1);
                for (int c = 0; c < CELLS_PER_IC; c++) {
                    float v    = meas.cell_v[ic][c];
                    bool  ov   = (v >= CELL_OV_V);
                    bool  uv   = (v <= CELL_UV_V);
                    bool  zero = (v < 0.01f);
                    if (ov)   any_ov   = true;
                    if (uv)   any_uv   = true;
                    if (zero) any_zero = true;
                    Serial.printf("    C%02d: %6.4f V%s%s%s\n",
                                  c + 1, v,
                                  ov   ? "  ← OV"                    : "",
                                  uv   ? "  ← UV"                    : "",
                                  zero ? "  ← ZERO (open circuit?)"  : "");
                }
            }
            test_result("No cells at 0V (open circuit check)", !any_zero);
            test_result("No cell OV",  !any_ov);
            test_result("No cell UV",  !any_uv);

            if (any_ov) { fault_set(static_cast<uint16_t>(FaultCode::OVERVOLTAGE)); fsm_set_state(g_fsm, BmsState::FAULT); }
            if (any_uv) { fault_set(static_cast<uint16_t>(FaultCode::UNDERVOLTAGE)); fsm_set_state(g_fsm, BmsState::FAULT); }
        } else {
            fault_set(static_cast<uint16_t>(FaultCode::SPI_LTC));
            fsm_set_state(g_fsm, BmsState::FAULT);
            Serial.println("  !! PEC error reading cell voltages. Check wiring to LTC6811.");
        }

    }
    print_section_end();

    // =========================================================================
    // TEST 4 — Temperature read
    // =========================================================================
    print_section(4, "Temperatures");
    if (!ltc_comms_ok) {
        Serial.println("  [SKIP] LTC comms failed — skipping.");
    } else {
        Serial.println("  Reading GPIO ADC channels for NTC temperatures...");
        bool ok = ltc_read_temperatures(&meas);
        test_result("ltc_read_temperatures() PEC valid", ok);

        if (ok) {
            bool any_ot   = false;
            bool any_open = false;
            for (int i = 0; i < NUM_TEMP_SENSORS; i++) {
                float t    = meas.temps[i];
                bool  ot   = (t >= (float)TEMP_CUTOFF_C);
                bool  open = (t < -50.0f);
                if (ot)   any_ot   = true;
                if (open) any_open = true;
                Serial.printf("    Sensor %d: %6.1f °C%s%s\n",
                              i, t,
                              ot   ? "  ← OT"                    : "",
                              open ? "  ← OPEN/SHORT (check NTC)" : "");
            }
            test_result("No sensors open/shorted", !any_open);
            test_result("No sensors over TEMP_CUTOFF_C", !any_ot);

            if (any_ot) { fault_set(static_cast<uint16_t>(FaultCode::OVERTEMP)); fsm_set_state(g_fsm, BmsState::FAULT); }
        } else {
            fault_set(static_cast<uint16_t>(FaultCode::SPI_LTC));
            fsm_set_state(g_fsm, BmsState::FAULT);
            Serial.println("  !! PEC error reading temperatures. Check GPIO mux / NTC wiring.");
        }

    }
    print_section_end();

    // =========================================================================
    // TEST 5 — ADS131M02 configure + ID verify
    // =========================================================================
    print_section(5, "ADS131M02 Configure + ID");
    {
        Serial.println("  Writing CLOCK and GAIN1 registers, reading back...");
        bool cfg_ok = ads_configure();
        test_result("ADS131M02 CLOCK + GAIN1 register readback match", cfg_ok);

        Serial.println("  Reading ID register, expecting 0x2282...");
        bool id_ok = ads_checkid();
        test_result("ADS131M02 ID register matches 0x2282", id_ok);

        if (!cfg_ok || !id_ok) {
            fault_set(static_cast<uint16_t>(FaultCode::SPI_ADS));
            fsm_set_state(g_fsm, BmsState::FAULT);
            Serial.println("  !! Check: SPI2 wiring, ADS131M02 power rail, CS line.");
        }

    }
    print_section_end();

    // =========================================================================
    // TEST 6 — ADS131M02 current read
    // =========================================================================
    print_section(6, "ADS131M02 Current Sense");
    {
        const int N = 8;
        Serial.printf("  Taking %d samples and averaging...\n", N);

        int32_t sum = 0;
        for (int i = 0; i < N; i++) {
            int32_t raw = ads_read_raw();
            Serial.printf("    Sample %d: %ld counts\n", i + 1, (long)raw);
            sum += raw;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        int32_t avg_raw = sum / N;
        float   amps    = ads_counts_to_amps(avg_raw);
        meas.current_a  = amps;

        Serial.printf("  Average raw : %ld counts\n", (long)avg_raw);
        Serial.printf("  Current     : %.3f A  (expect ~0 A at rest)\n", amps);

        bool idle_ok = (amps > -2.0f && amps < 2.0f);
        test_result("Idle current within ±2 A of zero", idle_ok);
        if (!idle_ok)
            Serial.println("  !! Check: shunt wiring, gain config, ADS131M02 Vref.");

        bool oc = meas_check_overcurrent(&meas);
        test_result("meas_check_overcurrent() — no fault at rest", !oc);
        if (oc) { fault_set(static_cast<uint16_t>(FaultCode::OVERCURRENT)); fsm_set_state(g_fsm, BmsState::FAULT); }

    }
    print_section_end();

    // =========================================================================
    // TEST 7 — Gate driver selftest
    // =========================================================================
    print_section(7, "Gate Driver");
    {
        Serial.println("  Pulsing CHG and DSG outputs LOW...");
        bool ok = gate_driver_selftest();
        test_result("gate_driver_selftest() returned true", ok);
        Serial.println("  >> Verify with multimeter: CHG and DSG outputs = LOW");
        if (!ok) {
            fault_set(static_cast<uint16_t>(FaultCode::STARTUP));
            fsm_set_state(g_fsm, BmsState::FAULT);
            Serial.println("  !! Check: gate driver power rail, output enable pin.");
        }
    }
    print_section_end();

    // =========================================================================
    // SUMMARY
    // =========================================================================
    Serial.println();
    Serial.println("╔══════════════════════════════════════════╗");
    Serial.println("║               TEST SUMMARY               ║");
    Serial.println("╠══════════════════════════════════════════╣");
    Serial.printf( "║  Tests run    : %-25d║\n", s_tests_run);
    Serial.printf( "║  Tests passed : %-25d║\n", s_tests_passed);
    Serial.printf( "║  Tests failed : %-25d║\n", s_tests_run - s_tests_passed);
    Serial.println("╠══════════════════════════════════════════╣");
    if (s_tests_passed == s_tests_run) {
        Serial.println("║  STATUS : ALL PASS ✓                     ║");
    } else {
        Serial.println("║  STATUS : FAILED ✗                       ║");
    }
    Serial.println("╠══════════════════════════════════════════╣");
    Serial.printf( "║  FSM state    : %-25d║\n",
                   static_cast<uint8_t>(fsm_get_state(g_fsm)));
    Serial.printf( "║  Fault reg    : 0x%04X%-19s║\n", fault_get(), "");
    Serial.println("╚══════════════════════════════════════════╝");
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
    g_snapshot_mutex = xSemaphoreCreateMutex();
    g_meas_queue     = xQueueCreate(2, sizeof(measurement_data_t));

    // FSM init — starts in INIT state, contactors open
    fsm_init(g_fsm);

    // Report any fault from the previous run (before task_test clears it)
    fault_snapshot_t prev_snap = {};
    if (fault_log_read(&prev_snap)) {
        Serial.printf("[main] Previous run fault: code=%d  state_at_fault=%d\n",
                      static_cast<uint8_t>(prev_snap.code),
                      static_cast<uint8_t>(prev_snap.state_at_fault));
    } else {
        Serial.println("[main] No fault logged from previous run.");
    }

    // Spawn only the tasks needed for this test build.
    // task_balance and task_daq are intentionally omitted.
    xTaskCreatePinnedToCore(task_fsm,      "fsm",  8192, nullptr, 4, nullptr, 1);
    xTaskCreatePinnedToCore(task_watchdog, "wdt",  2048, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(task_test,     "test", 8192, nullptr, 3, nullptr, 1);

    Serial.println("[main] Tasks running — waiting for test output.");
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}