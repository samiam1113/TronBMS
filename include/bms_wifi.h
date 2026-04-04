// ============================================================================
// bms_wifi.h — WiFi Access Point + WebSocket telemetry server interface
// ============================================================================
#pragma once

#include "bms_types.h"
#include "bms_state.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the WiFi AP and HTTP/WebSocket server.
 *        Call once in setup() after bms_spi_init().
 */
void wifi_server_init(void);

/**
 * @brief Push a new measurement snapshot to the WiFi broadcast buffer.
 *        Thread-safe — call from task_measure or task_daq.
 *
 * @param meas      Pointer to the completed measurement struct.
 * @param state     Current FSM state.
 * @param fault_reg Current fault register bitmask (from fault_get()).
 */
void wifi_push_telemetry(const measurement_data_t *meas,
                         BmsState state,
                         uint16_t fault_reg);

void task_wifi_broadcast(void *pv);

#ifdef __cplusplus
}
#endif