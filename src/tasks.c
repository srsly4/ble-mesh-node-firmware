#include "tasks.h"
#include "driver/timer.h"
#include <string.h>

static xTaskHandle executor_task_handle;
QueueHandle_t executor_queue;

int task_registered_funcs_len = 0;
static task_registered_t task_registered_funcs[TASK_MAX_REGISTERED_FUNCS] = {0};

static uint64_t logic_time_rate = 1 * LOGIC_TIME_RATE_MULTIPLIER;
static uint64_t logic_time_offset = 0;

static bool logic_time_in_sync = false;

// hardware time - local timer in ms (JS-compatible)
// logic time - absolute time in ms

uint64_t get_logic_time() {
    uint64_t hardware_time;
    if (timer_get_counter_value(TIMER_GROUP_0, 0, &hardware_time) != ESP_OK) {
        ESP_LOGE(TASK_TAG, "Could not get timer value");
        return 0;
    }

    hardware_time /= TIMER_SCALE_MS;

    return HARDWARE_TIME_TO_LOGIC(hardware_time);
}

void enqueue_task(task_item_t *task) {
    uint64_t curr_time = get_logic_time();

    if (task->time <= curr_time) {
        ESP_LOGI(TASK_TAG, "Task time before current, running immediately");
        if (xQueueSend(executor_queue, task, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TASK_TAG, "Could not push task to queue");
        }
        return;
    }

    uint64_t target_time = LOGIC_TIME_TO_HARDWARE(task->time);

    ESP_LOGI(TASK_TAG, "Adding task to task queue and setting an alarm");
    // add to task queue
    if (tasks_queue == NULL) {
        tasks_queue = task;
    } else {
        task_item_t* curr = tasks_queue;
        while (curr->next != NULL && curr->next->time < target_time) {
            curr = curr->next;
        }

        task->next= curr->next;
        curr->next = task;
    }

    ESP_LOGI(TASK_TAG, "Setting an alarm");
    // set timer for this task
    timer_set_alarm_value(TIMER_GROUP_0, 0, target_time);
    timer_set_alarm(TIMER_GROUP_0, 0, TIMER_ALARM_EN);
    ESP_LOGI(TASK_TAG, "Task enqueued, alert set for %llu", target_time);
}

void finalize_task(task_item_t *task) {
    task->next = tasks_finished;
    tasks_finished = task;

    ESP_LOGI(TASK_TAG, "Task finished");
}

uint8_t register_task(uint8_t task_id, task_func_t func) {
    if (task_registered_funcs_len + 1 == TASK_MAX_REGISTERED_FUNCS) {
        ESP_LOGE(TASK_TAG, "Max registered tasks limit excedeed");
        return 1;
    }

    for (int i = 0; i < task_registered_funcs_len; i++) {
        if (task_registered_funcs[i].task_id == task_id) {
            ESP_LOGE(TASK_TAG, "Task with id %02x have been already registered", task_id);
            return 1;
        }
    }

    task_registered_funcs[task_registered_funcs_len].task_id = task_id;
    task_registered_funcs[task_registered_funcs_len].func = func;


    task_registered_funcs_len++;

    return 0;
}

void IRAM_ATTR timer_group0_isr(void *params) {
    /* Retrieve the interrupt status and the counter value
       from the timer that reported the interrupt */
    ets_printf("timer triggered!\n");
    uint32_t intr_status = TIMERG0.int_st_timers.val;
    TIMERG0.hw_timer[0].update = 1;
    uint64_t timer_counter_value = 
        ((uint64_t) TIMERG0.hw_timer[0].cnt_high) << 32
        | TIMERG0.hw_timer[0].cnt_low;


    TIMERG0.int_clr_timers.t0 = 1;

    uint64_t logic_time = HARDWARE_TIME_TO_LOGIC(timer_counter_value / TIMER_SCALE_MS);

    task_item_t* next;
    while (tasks_queue != NULL && tasks_queue->time <= logic_time) {
        next = tasks_queue->next;
        tasks_queue->next = NULL;
        ets_printf("pushing to queue task_id %02x!\n", tasks_queue->func_code);
        xQueueSendFromISR(executor_queue, tasks_queue, NULL);
        tasks_queue = next;
    }
}

void initialize_task_timer(void) {
    const uint8_t timer_idx = TIMER_0;
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_DIS;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = 0;
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    //timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr,
                       (void *)0, ESP_INTR_FLAG_IRAM, NULL);

    timer_start(TIMER_GROUP_0, timer_idx);
}

static void executor_task(void* args) {

    task_item_t *task = (task_item_t*)args;

    task_registered_t *task_reg = NULL;
    for (int i = 0; i < task_registered_funcs_len; i++) {
        task_registered_t task_cmp = task_registered_funcs[i];
        if (task_cmp.task_id == task->func_code) {
            task_reg = &task_cmp;
            break;
        }
    }

    if (task_reg == NULL) {
        ESP_LOGE(TASK_TAG, "Could not find task function with code %02x", task->func_code);
    }
    if (task_reg->func == NULL) {
        ESP_LOGE(TASK_TAG, "Queued task function callback NULL");
        return;
    }

    ESP_LOGI(TASK_TAG, "Executing inner-function of task with func id %02x ptr: %p arg_data ptr: %p", task->func_code, task_reg->func, task->arg_data);
    task_registered_ret_t ret = task_reg->func(task->arg_data);

    ESP_LOGI(TASK_TAG, "Task %02x returned with data length %u", task->func_code, (unsigned int)ret.ret_len);

    task->res_data = malloc(sizeof(task_registered_ret_t));
    *(task->res_data) = ret;
    finalize_task(task);
    vTaskDelete(NULL);
}

static void executor_loop(void* args) {
    executor_queue = xQueueCreate(TASK_EXECUTOR_MAX_ITEMS, sizeof(task_item_t));
    
    task_item_t current_task;

    while (1) {
        if (xQueueReceive(executor_queue, &current_task, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TASK_TAG, "Could not receive from queue");
            return;
        }

        task_item_t *passed_task = malloc(sizeof(task_item_t));
        if (passed_task == NULL) {
            ESP_LOGE(TASK_TAG, "Couldn't malloc for task");
            return;
        }

        memcpy(passed_task, &current_task, sizeof(task_item_t));

        ESP_LOGI(TASK_TAG, "Executing task with func id %02x, ptr to task: %p", passed_task->func_code, passed_task);

        TaskHandle_t handle = NULL;
        if (xTaskCreate(executor_task, "executor_task", 16*1024, (void *)passed_task, 3, &handle) != pdTRUE) {
            ESP_LOGE(TASK_TAG, "Could not create execution task");
        }
    }
}

static void show_curr_time_task(void* args) {
    uint64_t curr_time;
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if (timer_get_counter_value(TIMER_GROUP_0, 0, &curr_time) != ESP_OK) {
            ESP_LOGE(TASK_TAG, "Could not get timer value");
            continue;
        }

        uint64_t hw_time = curr_time / TIMER_SCALE_MS;

        ESP_LOGI(TASK_TAG, "Current local time: %llu ms, %llu ticks, timer scale ms: %u", hw_time, curr_time, TIMER_SCALE_MS);

        if (logic_time_in_sync) {
            publish_timesync_data(logic_time_rate, HARDWARE_TIME_TO_LOGIC(hw_time));
        }
    }
}

void initialize_task_executor(void) {
    task_registered_funcs_len = 0;
    xTaskCreate(executor_loop, "executor_loop", 8096, NULL, 3, &executor_task_handle);
    xTaskCreate(show_curr_time_task, "curr_time", 8096, NULL, 3, (void*)(&show_curr_time_task));
}

void update_logic_time(uint64_t logic_rate, uint64_t logic_time) {
    uint64_t current_logic_time = get_logic_time();

    ESP_LOGI(TASK_TAG, "update_logic_time, rate: %llu, time: %llu, local_logic_time: %llu", logic_rate, logic_time, current_logic_time);


    if (logic_time > current_logic_time && logic_time > LOGIC_TIME_THRESHOLD && logic_time - current_logic_time > LOGIC_TIME_THRESHOLD) {
        ESP_LOGI(TASK_TAG, "logic time threshold excedded, replacing local offset");
        logic_time_offset += logic_time - current_logic_time;
        goto set_alarm;
    }

    int64_t offset = ((int64_t)logic_time - (int64_t)current_logic_time)/2;

    //todo: handle more neighbours (with map and timeouts)
    ESP_LOGI(TASK_TAG, "updating time with offset %lld", offset);

    if (offset < 0) {
        logic_time_offset -= abs(offset);
    } else {
        logic_time_offset += offset;
    }
    logic_time_in_sync = true;

    set_alarm:
    if (tasks_queue != NULL) {
        uint64_t target_time = LOGIC_TIME_TO_HARDWARE(tasks_queue->time);
        ESP_LOGI(TASK_TAG, "reset alarm to HW clock %llu", target_time);
        timer_set_alarm_value(TIMER_GROUP_0, 0, target_time);
        timer_set_alarm(TIMER_GROUP_0, 0, TIMER_ALARM_EN);
    }

}