// ============================================================================
// bms_fault.cpp — Fault register management, NVS logging, context capture
// ============================================================================

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "bms_config.h"
#include "bms_types.h"
#include "bms_fault.h"

// ── Global state ─────────────────────────────────────────────────────────────
volatile uint16_t g_faultRegister = 0;

// Measurement globals — defined here, extern'd in bms_measurements.cpp etc.
float    g_cellV[NUM_CELLS]     = {};
uint16_t g_cellRaw[NUM_CELLS]   = {};
float    g_tempC[10]            = {};
float    g_amps                 = 0.0f;
bool     g_balanceCells[NUM_CELLS] = {};

// ── NVS key ──────────────────────────────────────────────────────────────────
static const char *NVS_NAMESPACE = "bms_fault";
static const char *NVS_KEY_SNAP  = "last_snap";

// ── Fault register API ───────────────────────────────────────────────────────

void fault_set(uint16_t bit) {
    portENTER_CRITICAL_SAFE(nullptr);
    g_faultRegister |= bit;
    portEXIT_CRITICAL_SAFE(nullptr);
}

void fault_clear(uint16_t bit) {
    portENTER_CRITICAL_SAFE(nullptr);
    g_faultRegister &= (uint16_t)(~bit);
    portEXIT_CRITICAL_SAFE(nullptr);
}

uint16_t fault_get(void) {
    return g_faultRegister;
}

// ── Context capture ──────────────────────────────────────────────────────────

void fault_capture_context(fault_snapshot_t *snap, bms_state_t state_at_fault) {
    if (!snap) return;

    snap->state_at_fault = state_at_fault;

    // Map fault register to fault_code_t for the snapshot
    uint16_t fr = g_faultRegister;
    if      (fr & FAULT_CELL_OV)  snap->code = FAULT_OVERVOLTAGE;
    else if (fr & FAULT_PACK_OV)  snap->code = FAULT_OVERVOLTAGE;
    else if (fr & FAULT_CELL_UV)  snap->code = FAULT_UNDERVOLTAGE;
    else if (fr & FAULT_PACK_UV)  snap->code = FAULT_UNDERVOLTAGE;
    else if (fr & FAULT_OC_CHG)   snap->code = FAULT_OVERCURRENT;
    else if (fr & FAULT_OC_DSG)   snap->code = FAULT_OVERCURRENT;
    else if (fr & FAULT_OT)       snap->code = FAULT_OVERTEMP;
    else if (fr & FAULT_SPI)      snap->code = FAULT_SPI_LTC;
    else if (fr & FAULT_ADS_ID)   snap->code = FAULT_SPI_ADS;
    else if (fr & FAULT_GATE)     snap->code = FAULT_STARTUP;
    else                           snap->code = FAULT_NONE;

    // Capture measurement snapshot from globals
    snap->meas.current_a = g_amps;

    for (int ic = 0; ic < TOTAL_IC; ic++) {
        for (int c = 0; c < CELLS_PER_IC; c++) {
            snap->meas.cell_v[ic][c] = g_cellV[ic * CELLS_PER_IC + c];
        }
    }

    for (int i = 0; i < 4; i++) {
        snap->meas.temps[i] = g_tempC[i];
    }

    Serial.printf("[fault] Context captured: code=%d state=%d fault_reg=0x%04X\n",
                  snap->code, snap->state_at_fault, g_faultRegister);
}

// ── NVS logging ──────────────────────────────────────────────────────────────

void fault_log_write(const fault_snapshot_t *snap) {
    if (!snap) return;

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        Serial.printf("[fault] NVS open failed: %s\n", esp_err_to_name(err));
        return;
    }

    err = nvs_set_blob(handle, NVS_KEY_SNAP, snap, sizeof(fault_snapshot_t));
    if (err != ESP_OK) {
        Serial.printf("[fault] NVS write failed: %s\n", esp_err_to_name(err));
    } else {
        nvs_commit(handle);
        Serial.printf("[fault] Fault snapshot written to NVS (code=%d)\n", snap->code);
    }

    nvs_close(handle);
}

bool fault_log_read(fault_snapshot_t *snap_out) {
    if (!snap_out) return false;

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        Serial.printf("[fault] NVS open (read) failed: %s\n", esp_err_to_name(err));
        return false;
    }

    size_t required = sizeof(fault_snapshot_t);
    err = nvs_get_blob(handle, NVS_KEY_SNAP, snap_out, &required);
    nvs_close(handle);

    if (err != ESP_OK || required != sizeof(fault_snapshot_t)) {
        Serial.printf("[fault] NVS read failed or size mismatch: %s\n",
                      esp_err_to_name(err));
        return false;
    }

    Serial.printf("[fault] Fault snapshot read from NVS (code=%d state=%d)\n",
                  snap_out->code, snap_out->state_at_fault);
    return true;
}