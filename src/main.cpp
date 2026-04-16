// ============================================================================
// main.cpp — TronBMS Production Build
//
// Task map (higher number = higher priority on ESP32 FreeRTOS):
//   task_watchdog       : 5  — feeds hardware WDT, detects task stalls
//   task_fsm            : 4  — drives state machine
//   task_measure        : 3  — ADC reads (voltage, temp, current)
//   task_balance        : 2  — DCC refresh during balancing
//   task_daq            : 1  — CAN telemetry transmit + WiFi push
//   task_wifi_broadcast : 1  — WiFi WebSocket broadcast (pinned to core 0)
//
// Charging balance logic (implemented in bms_fsm.cpp state_charging):
//   - Balance WHILE charging continues (charge gate stays closed normally)
//   - Only the specific high cell is balanced (selective per-cell DCC mask)
//   - Balance triggered when: cell >= 3.8V AND cell > (lowest + 25mV)
//   - Normal resume: drain high cell to within 3mV of lowest, continue
//   - Hard gate-open: any cell hits 4.1V AND others are >25mV behind
//     → open charge gate, drain 4.1V cells to within 3mV of lowest,
//       close charge gate, continue
//   - Emergency: any spread >= 75mV → open charge gate immediately,
//     drain all high cells to within 3mV of lowest, then resume charging
//   - Charge done: all cells within 3mV of 4.1V → open charge gate
//   - SoC% = (sum of all cell voltages / PACK_OV_V) * 100
// ============================================================================

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"

#include "bms_config.h"
#include "bms_types.h"
#include "bms_state.h"
#include "bms_fsm.h"
#include "bms_fault.h"
#include "bms_hardware.h"
#include "bms_measurements.h"
#include "bms_balance.h"
#include "bms_tasks.h"
#include "bms_telemetry.h"
#include "bms_wifi.h"
#include "ltc_spi.h"

// ── Global FSM context — extern'd by bms_tasks.cpp and bms_fsm.cpp ───────────
BmsFsm g_fsm;

// ============================================================================
// setup
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("[main] TronBMS production build starting...");

    // ── NVS init ──────────────────────────────────────────────────────────────
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    esp_task_wdt_init(10, true);  // 10 second timeout, panic on trigger

    // ── Report any fault snapshot from the previous run ───────────────────────
    fault_snapshot_t prev_snap = {};
    if (fault_log_read(&prev_snap)) {
        Serial.printf("[main] Previous run fault: code=%d  state_at_fault=%d\n",
                      static_cast<uint8_t>(prev_snap.code),
                      static_cast<uint8_t>(prev_snap.state_at_fault));
    } else {
        Serial.println("[main] No fault logged from previous run.");
    }

    // ── Hardware init ─────────────────────────────────────────────────────────
    bms_gpio_init();
    bms_spi_init();

    // Wake LTC chain immediately — before FSM or any SPI access
    ltc_wakeup_sleep();
    ltc_wakeup_idle();
    Serial.println("[main] LTC chain wakeup sent.");
    
    // ── Pre-warm temperature sensors ──────────────────────────────────────────
    // Thermistors require several LTC ADC cycles to stabilize on first power-up.
    // Poll them repeatedly here so they read correctly from the first FSM tick.
    Serial.println("[main] Pre-warming temperature sensors...");
    for (int i = 0; i < 20; i++) {
        ltc_wakeup_idle();
        measurement_data_t dummy = {};
        meas_cell_data(&dummy);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    Serial.println("[main] Temperature sensors ready.");

    // ── RTOS primitives ───────────────────────────────────────────────────────
    g_event_group    = xEventGroupCreate();
    g_meas_mutex     = xSemaphoreCreateMutex();
    g_snapshot_mutex = xSemaphoreCreateMutex();
    g_meas_queue     = xQueueCreate(2, sizeof(measurement_data_t));

    // ── FSM init — starts in INIT state, contactors open ─────────────────────
    fsm_init(g_fsm);

    // ── WiFi AP + HTTP/WebSocket server ───────────────────────────────────────
    wifi_server_init();

    // ── Spawn FreeRTOS tasks ──────────────────────────────────────────────────
    // Core 1: all time-critical BMS tasks
    xTaskCreatePinnedToCore(task_watchdog, "wdt",  4096, nullptr, 5,
                            nullptr, 1);
    xTaskCreatePinnedToCore(task_fsm,      "fsm",  8192, nullptr, 4,
                            nullptr, 1);
    xTaskCreatePinnedToCore(task_measure,  "meas", 8192, nullptr, 3,
                            nullptr, 1);
    xTaskCreatePinnedToCore(task_balance,  "bal",  4096, nullptr, 2,
                            &g_balance_task_handle, 1);
    xTaskCreatePinnedToCore(task_daq,      "daq",    4096, nullptr, 1,
                            nullptr, 1);
    xTaskCreatePinnedToCore(task_serial,   "serial", 4096, nullptr, 1,
                            nullptr, 1);

    // Core 0: WiFi broadcast — isolated from BMS timing
    xTaskCreatePinnedToCore(task_wifi_broadcast, "wifi_bc", 4096, nullptr, 1,
                            nullptr, 0);

    Serial.println("[main] All tasks running.");
}

// ============================================================================
// loop — idle, all work is done in FreeRTOS tasks
// ============================================================================
void loop() {
    vTaskDelay(portMAX_DELAY);
}