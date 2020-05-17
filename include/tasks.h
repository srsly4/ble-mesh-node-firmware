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
#include "ble_mesh.h"

#define TASK_MAX_REGISTERED_FUNCS 8
#define TASK_EXECUTOR_MAX_ITEMS 4

#define TASK_TAG "tasks"

#define TIMER_DIVIDER         512  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_SCALE_MS        (TIMER_SCALE / 1000)

typedef struct task_registered_ret_t {
    uint8_t ret_len;
    void* data;
} task_registered_ret_t;

typedef task_registered_ret_t (*task_func_t)(void*);
typedef struct task_registered_t {
    uint8_t task_id;
    task_func_t func;
} task_registered_t;


typedef struct task_item_t task_item_t;

struct task_item_t {
    uint64_t time;
    uint16_t func_code;
    void* arg_data;
    task_registered_ret_t *res_data;
    struct task_item_t *next;
};



task_item_t *tasks_queue;
task_item_t *tasks_finished;


void enqueue_task(task_item_t *task);
void finalize_task(task_item_t *task);

uint8_t register_task(uint8_t task_id, task_func_t func);

void initialize_task_timer(void);
void initialize_task_executor(void);

#endif