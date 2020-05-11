#include "tasks.h"
#include "driver/timer.h"

static xTaskHandle executor_task_handle;
QueueHandle_t executor_queue;

static int task_registered_funcs_len = 0;
static task_registered_t task_registered_funcs[TASK_MAX_REGISTERED_FUNCS];


void enqueue_task(task_item_t *task) {
    uint64_t curr_time;

    ESP_LOGI(TASK_TAG, "Getting current timer value");
    if (timer_get_counter_value(TIMER_GROUP_0, 0, &curr_time) != ESP_OK) {
        ESP_LOGE(TASK_TAG, "Could not get timer value");
        return;
    }

    uint64_t target_time = task->time*TIMER_SCALE_MS;

    if (target_time <= curr_time) {
        ESP_LOGI(TASK_TAG, "Task time before current, running immediately");
        if (xQueueSend(executor_queue, task, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TASK_TAG, "Could not push task to queue");
        }
        return;
    }

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
    tasks_queue = task->next;

    task->next = tasks_finished;
    tasks_finished = task;

    //todo: ble mesh task completed publish
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

    task_registered_t registered_task;

    registered_task.task_id = task_id;
    registered_task.func = func;

    task_registered_funcs[task_registered_funcs_len] = registered_task;

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

    task_item_t* next;
    while (tasks_queue != NULL && tasks_queue->time <= timer_counter_value) {
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

    ESP_LOGI(TASK_TAG, "timer_init end");

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);
    ESP_LOGI(TASK_TAG, "timer_set_counter_value end");

    /* Configure the alarm value and the interrupt on alarm. */
    //timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    ESP_LOGI(TASK_TAG, "timer_enable_intr end");
    timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr,
                       (void *)0, ESP_INTR_FLAG_IRAM, NULL);
    ESP_LOGI(TASK_TAG, "timer_isr_register end");

    timer_start(TIMER_GROUP_0, timer_idx);
    ESP_LOGI(TASK_TAG, "timer_start end");
}

static void executor_task(void* args) {
    ESP_LOGI(TASK_TAG, "In-executor of task");

    task_item_t *task = (task_item_t*)args;

    task_registered_t *task_reg = NULL;
    for (int i = 0; i < task_registered_funcs_len; i++) {
    ESP_LOGI(TASK_TAG, "In-executor of task, iter %d", i);
        if (task_registered_funcs[i].task_id == task->func_code) { // tu jest segmentation fault
            task_reg = &(task_registered_funcs[i]);
            ESP_LOGI(TASK_TAG, "In-executor of task, before break");
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

    ESP_LOGI(TASK_TAG, "Executing inner-function of task with func id %02x", task->func_code);
    task_registered_ret_t ret = task_reg->func(task->arg_data);

    ESP_LOGI(TASK_TAG, "Task %02x returned with data length %u", task->func_code, (unsigned int)ret.ret_len);
    finalize_task(task);
}

static void executor_loop(void* args) {
    executor_queue = xQueueCreate(TASK_EXECUTOR_MAX_ITEMS, sizeof(task_item_t));
    
    task_item_t current_task;
    ESP_LOGI(TASK_TAG, "Executor loop started");
    while (1) {
        if (xQueueReceive(executor_queue, &current_task, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TASK_TAG, "Could not receive from queue");
            return;
        }
        ESP_LOGI(TASK_TAG, "Executing task with func id %02x", current_task.func_code);

        xTaskCreate(executor_task, "executor_task", 16384, NULL, 3, &current_task);
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

        ESP_LOGI(TASK_TAG, "Current local time: %llu ms, %llu ticks, timer scale ms: %u", curr_time / TIMER_SCALE_MS, curr_time, TIMER_SCALE_MS);
    }
}

void initialize_task_executor(void) {
    task_registered_funcs_len = 0;
    xTaskCreate(executor_loop, "executor_loop", 8096, NULL, 3, &executor_task_handle);
    xTaskCreate(show_curr_time_task, "curr_time", 8096, NULL, 3, &show_curr_time_task);
}