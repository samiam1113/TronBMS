#ifndef BMS_TASKS_H
#define BMS_TASKS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "bms_types.h"
#include "bms_config.h"
#include "bms_fault.h"

// Shared RTOS primitives
// Defined in bms_tasks.c, declared here for access by other modules
extern EventGroupHandle_t g_event_group;
extern SemaphoreHandle_t g_meas_mutex;
extern SemaphoreHandle_t g_snapshot_mutex;   // Fix #4: guards g_fault_snapshot + pending flag
extern QueueHandle_t g_meas_queue;
extern TaskHandle_t g_balance_task_handle;

// Shared fault snapshot
// Written by task_fsm on fault transition
// Read by task_daq for telemetry_send_fault
extern fault_snapshot_t g_fault_snapshot;

// Fault snapshot pending flag
// Set by task_fsm when a fault snapshot is captured
// Cleared by task_daq after telemetry_send_fault is called
extern volatile bool g_fault_snapshot_pending;
// Task functions
// All follow FreeRTOS task signature void f(void *pvParameters)

// task_fsm — owns state machine, triggers measurement cycles,
// evaluates fault and balance events each tick
void task_fsm(void *pvParameters);

// task_measure — waits for EVT_MEASURE_START, runs full
// measurement sequence, posts results to queue and shared struct
void task_measure(void *pvParameters);

// task_balance — suspended at startup, resumed by FSM when
// entering STATE_BALANCE, computes and applies DCC masks
void task_balance(void *pvParameters);

// task_daq — consumes measurement queue, transmits telemetry
// and fault data over CAN via bms_telemetry
void task_daq(void *pvParameters);

// task_watchdog — monitors task checkins, feeds hardware WDT
// only when FSM is not in STATE_FAULT
void task_watchdog(void *pvParameters);

void task_serial(void *pvParameters);
#endif