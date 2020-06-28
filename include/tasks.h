#ifndef BLE_MESH_TASKS
#define BLE_MESH_TASKS

#include <stdio.h>
#include "esp_log.h"
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "types.h"
#include "ble_mesh.h"

#define TASK_MAX_REGISTERED_FUNCS 8
#define TASK_EXECUTOR_MAX_ITEMS 4

#define TASK_TAG "tasks"

#define TIMER_DIVIDER         512
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)
#define TIMER_SCALE_MS        (TIMER_SCALE / 1000)
#define TIMER_SCALE_MS_FP     ((double)TIMER_BASE_CLK / 512000.0 )

#define LOGIC_TIME_RATE_MULTIPLIER 1000
#define LOGIC_TIME_THRESHOLD 1000 * 1000

#define TIME_TRANSMISSION_DELAY 15*3 // ms
#define TIME_BEACON_DELAY_BASE 5000 // ms
#define TIME_BEACON_DELAY_THERESHOLD 500 // ms
#define TIME_BEACON_MAX_ITERATION_DIFFERENCE 5

#define LOGIC_TIME_TO_HARDWARE(logic_time) ((uint64_t)(LOGIC_TIME_RATE_MULTIPLIER * logic_time / logic_time_rate) - logic_time_offset)
#define HARDWARE_TIME_TO_LOGIC(hardware_time) ((uint64_t)((logic_time_rate * hardware_time) / LOGIC_TIME_RATE_MULTIPLIER) + logic_time_offset)


task_item_t *tasks_queue;
task_item_t *tasks_finished;


void enqueue_task(task_item_t *task);
void finalize_task(task_item_t *task);

uint8_t register_task(uint8_t task_id, task_func_t func);

void initialize_task_timer(void);
void initialize_task_executor(void);

void update_neighbour_time_beacon(uint16_t sender, timesync_beacon_t *beacon);
void update_neighbour_drift_beacon(uint16_t sender, timesync_drift_beacon_t *beacon);

#endif