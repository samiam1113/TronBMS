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
#include "bms_fsm.h"

// ── Single global measurement struct — replaces all scattered globals ─────────
// Written by task_measure under g_meas_mutex.
// Read by fault_capture_context (also under mutex in task_fsm).
measurement_data_t g_meas = {};

// ── Fault register ────────────────────────────────────────────────────────────
volatile uint16_t g_faultRegister = 0;

// ── NVS keys ──────────────────────────────────────────────────────────────────
static const char *NVS_NAMESPACE = "bms_fault";
static const char *NVS_KEY_SNAP  = "last_snap";

// ── Fault register API ────────────────────────────────────────────────────────

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

// ── Context capture ───────────────────────────────────────────────────────────

void fault_capture_context(fault_snapshot_t *snap, BmsState state_at_fault) {
    if (!snap) return;

    snap->state_at_fault = state_at_fault;

    uint16_t fr = g_faultRegister;
    if      (fr & Fault::CELL_OV) snap->code = FaultCode::OVERVOLTAGE;
    else if (fr & Fault::PACK_OV) snap->code = FaultCode::OVERVOLTAGE;
    else if (fr & Fault::CELL_UV) snap->code = FaultCode::UNDERVOLTAGE;
    else if (fr & Fault::PACK_UV) snap->code = FaultCode::UNDERVOLTAGE;
    else if (fr & Fault::OT)      snap->code = FaultCode::OVERTEMP;
    else if (fr & Fault::OC_CHG)  snap->code = FaultCode::OVERCURRENT;
    else if (fr & Fault::OC_DSG)  snap->code = FaultCode::OVERCURRENT;
    else if (fr & Fault::SPI)     snap->code = FaultCode::SPI_LTC;
    else if (fr & Fault::ADS_ID)  snap->code = FaultCode::SPI_ADS;
    else if (fr & Fault::GATE)    snap->code = FaultCode::STARTUP;
    else                          snap->code = FaultCode::NONE;

    // Copy current pack state into snapshot — g_meas is the single source
    snap->meas = g_meas;

    Serial.printf("[fault] Context captured: code=%d state=%d fault_reg=0x%04X\n",
                  static_cast<uint8_t>(snap->code),
                  static_cast<uint8_t>(snap->state_at_fault),
                  g_faultRegister);
}

// ── NVS logging ───────────────────────────────────────────────────────────────

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
        Serial.printf("[fault] Snapshot written to NVS (code=%d)\n",
                      static_cast<uint8_t>(snap->code));
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

    Serial.printf("[fault] Snapshot read from NVS (code=%d state=%d)\n",
                  static_cast<uint8_t>(snap_out->code),
                  static_cast<uint8_t>(snap_out->state_at_fault));
    return true;
}