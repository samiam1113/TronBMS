#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bms_config.h"
#include "bms_types.h"
#include "bms_fsm.h"
#include "bms_fault.h"
#include "bms_tasks.cpp"    // pulls in all other .cpp files transitively

// ── Global FSM context — extern'd by bms_tasks.cpp ───────────────────────────
BmsFsm g_fsm;

// ── Global measurement data — extern'd by bms_fault.h ────────────────────────
measurement_data_t g_meas = {};

// ── Global fault register — extern'd by bms_fault.h ─────────────────────────
volatile uint16_t g_faultRegister = 0;

// ── Task function forward declarations (implemented in bms_tasks.cpp) ─────────
void task_fsm(void *pvParameters);
void task_measure(void *pvParameters);
void task_daq(void *pvParameters);
void task_watchdog(void *pvParameters);

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("[main] TronBMS starting...");

    // GPIO: gate drivers and ADS reset pin
    bms_gpio_init();

    // SPI: HSPI for LTC6811, VSPI for ADS131M02
    bms_spi_init();

    // FSM: zero-initialise and enter INIT state
    fsm_init(g_fsm);

    Serial.println("[main] Spawning tasks...");

    // task_fsm owns RTOS primitive creation (event group, mutex, queue)
    // and spawns task_balance internally, so it must start first.
    xTaskCreatePinnedToCore(
        task_fsm, "fsm",
        8192, nullptr, 4,
        nullptr, 1
    );

    xTaskCreatePinnedToCore(
        task_measure, "meas",
        8192, nullptr, 3,
        nullptr, 1
    );

    xTaskCreatePinnedToCore(
        task_daq, "daq",
        4096, nullptr, 1,
        nullptr, 0          // run on core 0 to keep core 1 for real-time tasks
    );

    xTaskCreatePinnedToCore(
        task_watchdog, "wdt",
        2048, nullptr, 5,
        nullptr, 1
    );

    Serial.println("[main] Tasks running.");
}

// ============================================================================
// loop — kept empty; all work is done inside FreeRTOS tasks
// ============================================================================
void loop() {
    vTaskDelay(portMAX_DELAY);
}