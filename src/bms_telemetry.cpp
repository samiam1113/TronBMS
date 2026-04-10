// ============================================================================
// bms_telemetry.cpp — CAN telemetry via ESP32 TWAI peripheral
//
// Frame map:
//   0x100 + n  (n = 0..9)  — cell voltages, 2 per frame (4-byte float each)
//   0x110 + n  (n = 0..1)  — temperatures, 2 per frame
//   0x120                  — pack current (float) + BMS state (uint32_t)
//   0x130                  — fault code (uint32_t) + state at fault (uint32_t)
// ============================================================================

#include <Arduino.h>
#include "driver/twai.h"
#include "bms_config.h"
#include "bms_types.h"
#include "bms_telemetry.h"

// ── CAN frame IDs ─────────────────────────────────────────────────────────────
#define CAN_ID_CELLS            0x100
#define CAN_ID_TEMPS            0x110
#define CAN_ID_CURRENT_STATE    0x120
#define CAN_ID_FAULT            0x130

#define CAN_TX_GPIO             16
#define CAN_RX_GPIO             17
#define CAN_BUS_SPEED           TWAI_TIMING_CONFIG_500KBITS()

// ── Helpers ──────────────────────────────────────────────────────────────────

static void pack_float(uint8_t *buf, int offset, float val) {
    uint32_t raw;
    memcpy(&raw, &val, sizeof(raw));
    buf[offset + 0] = (raw >> 24) & 0xFF;
    buf[offset + 1] = (raw >> 16) & 0xFF;
    buf[offset + 2] = (raw >>  8) & 0xFF;
    buf[offset + 3] = (raw      ) & 0xFF;
}

static void pack_u32(uint8_t *buf, int offset, uint32_t val) {
    buf[offset + 0] = (val >> 24) & 0xFF;
    buf[offset + 1] = (val >> 16) & 0xFF;
    buf[offset + 2] = (val >>  8) & 0xFF;
    buf[offset + 3] = (val      ) & 0xFF;
}

static bool twai_transmit_frame(uint32_t id, const uint8_t *data, uint8_t dlc) {
    twai_message_t msg = {};
    msg.identifier   = id;
    msg.data_length_code = dlc;
    msg.extd         = 0;
    msg.rtr          = 0;
    memcpy(msg.data, data, dlc);

    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(5));
    if (err != ESP_OK) {
        static uint32_t s_last_can_err_ms = 0;
        if ((millis() - s_last_can_err_ms) >= 5000) {
        s_last_can_err_ms = millis();
        Serial.printf("[telem] TX failed id=0x%03X err=%s (suppressed repeats)\n",
                  id, esp_err_to_name(err));
}
        return false;
    }
    return true;
}

// ── Public API ───────────────────────────────────────────────────────────────

void telemetry_init(void) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_GPIO,
        (gpio_num_t)CAN_RX_GPIO,
        TWAI_MODE_NORMAL
    );
    twai_timing_config_t  t_config = CAN_BUS_SPEED;
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        Serial.printf("[telem] TWAI install failed: %s\n", esp_err_to_name(err));
        return;
    }

    err = twai_start();
    if (err != ESP_OK) {
        Serial.printf("[telem] TWAI start failed: %s\n", esp_err_to_name(err));
        return;
    }

    Serial.println("[telem] TWAI started (TX=GPIO16, RX=GPIO17, 500kbps)");
}

void telemetry_send(measurement_data_t *meas, BmsState state) {
    if (!meas) return;

    uint8_t buf[8];

    // ── Cell voltages: 2 cells per frame, 10 frames per IC ───────────────────
    // Frame 0x100: IC1 cells 0,1
    // Frame 0x104: IC2 cells 0,1
    // Frame 0x109: IC2 cells 8,9
    for (int ic = 0; ic < TOTAL_IC; ic++) {
        for (int pair = 0; pair < CELLS_PER_IC / 2; pair++) {
            uint32_t frame_id = CAN_ID_CELLS + (ic * (CELLS_PER_IC / 2)) + pair;
            pack_float(buf, 0, meas->cell_v[ic][pair * 2]);
            pack_float(buf, 4, meas->cell_v[ic][pair * 2 + 1]);
            twai_transmit_frame(frame_id, buf, 8);
        }
    }

    // ── Temperatures: 2 channels per frame ───────────────────────────────────
    // Frame 0x110: temps[0,1]
    // Frame 0x111: temps[2,3]
    // Frame 0x112: temps[4] + 0.0f padding
    for (int pair = 0; pair < NUM_TEMP_SENSORS / 2; pair++) {
        pack_float(buf, 0, meas->temps[pair * 2]);
        pack_float(buf, 4, meas->temps[pair * 2 + 1]);
        twai_transmit_frame(CAN_ID_TEMPS + pair, buf, 8);
    }
    if (NUM_TEMP_SENSORS % 2 != 0) {
        pack_float(buf, 0, meas->temps[NUM_TEMP_SENSORS - 1]);
        pack_float(buf, 4, 0.0f);
        twai_transmit_frame(CAN_ID_TEMPS + (NUM_TEMP_SENSORS / 2), buf, 8);
    }

    // ── Current + state: frame 0x120 ─────────────────────────────────────────
    pack_float(buf, 0, meas->current_a);
    pack_u32  (buf, 4, (uint32_t)state);
    twai_transmit_frame(CAN_ID_CURRENT_STATE, buf, 8);
}

void telemetry_send_fault(fault_snapshot_t *snapshot) {
    if (!snapshot) return;

    uint8_t buf[8];
    pack_u32(buf, 0, (uint32_t)snapshot->code);
    pack_u32(buf, 4, (uint32_t)snapshot->state_at_fault);
    twai_transmit_frame(CAN_ID_FAULT, buf, 8);

    Serial.printf("[telem] Fault frame sent: code=%d state=%d\n",
                  snapshot->code, snapshot->state_at_fault);
}