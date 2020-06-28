#include "tasks.h"
#include <math.h>
#include "driver/timer.h"
#include <string.h>

static xTaskHandle executor_task_handle;
static xTaskHandle time_beacon_task_handle;
QueueHandle_t executor_queue;

int task_registered_funcs_len = 0;
static task_registered_t task_registered_funcs[TASK_MAX_REGISTERED_FUNCS] = {0};

// hardware time - ticks (with divider) in uint64_t
// logic time - ms doubkle

static double logic_time_base = 0.0;
static double logic_time_rate = 0.0;
static double logic_time_offset = 0.0;

static uint64_t last_time_request = 0; // in timer ticks

static uint32_t gtsp_current_iteration = 0;
static bool logic_time_in_sync = false;

static gtsp_neighbour_t *neighbours = NULL;

uint64_t get_hardware_time() {
    uint64_t timer_time;
    if (timer_get_counter_value(TIMER_GROUP_0, 0, &timer_time) != ESP_OK) {
        ESP_LOGE(TASK_TAG, "Could not get timer value");
        return 0;
    }

    return timer_time;
}

time_model_t last_time_model;
// calculates logic time based on local hw time difference
time_model_t get_logic_time() {
    uint64_t timer_time = get_hardware_time();
    uint64_t previous_time_request = last_time_request;
    last_time_request = timer_time;

    // hw_time_d already includes hardware clock drift
    double hw_time_d = (double)(timer_time - previous_time_request);

    logic_time_base += ((hw_time_d * logic_time_rate) + hw_time_d) / TIMER_SCALE_MS_FP;

    time_model_t time_model = {
        .logic_time = logic_time_base + logic_time_offset,
        .hardware_time = timer_time
    };

    // ESP_LOGI(TASK_TAG, "GET_LOGIC_TIME (hw_diff, logic_diff): %llu, %.12f", time_model.hardware_time - last_time_model.hardware_time, time_model.logic_time - last_time_model.logic_time);
    last_time_model = time_model;


    return time_model;
}

// double update_logic_time_drift_rate(double drift_rate) {
//     get_logic_time();
//     logic_time_rate = drift_rate;
// }

// double update_logic_time_offset(double offset) {
//     logic_time_offset = offset;
// }

hardware_predicted_time_t get_hardware_predicted_time_for_logic(double given_logic_time) {
    time_model_t time_model = get_logic_time();
    double diff = (given_logic_time - time_model.logic_time) * TIMER_SCALE_MS_FP; // positive if in the future

    hardware_predicted_time_t predicted;

    predicted.base = time_model;
    predicted.predicted = time_model.hardware_time + (uint64_t)(diff/(1.0 + logic_time_rate));

    return predicted;
}

void enqueue_task(task_item_t *task) {
    task->predicted_time = get_hardware_predicted_time_for_logic(task->time);
    task->next = NULL;

    if (task->predicted_time.predicted <= task->predicted_time.base.hardware_time) {
        ESP_LOGI(TASK_TAG, "Task time before current, running immediately");
        if (xQueueSend(executor_queue, task, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TASK_TAG, "Could not push task to queue");
        }
        return;
    }

    uint64_t target_time = task->predicted_time.predicted;

    ESP_LOGI(TASK_TAG, "Adding task to task queue and setting an alarm");
    // add to task queue
    if (tasks_queue == NULL) {
        tasks_queue = task;
    } else {
        task_item_t* curr = tasks_queue;
        while (curr->next != NULL && curr->next->predicted_time.predicted < target_time) {
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
    ets_printf("Timer triggered!\n");
    TIMERG0.hw_timer[0].update = 1;
    uint64_t timer_counter_value = 
        ((uint64_t) TIMERG0.hw_timer[0].cnt_high) << 32
        | TIMERG0.hw_timer[0].cnt_low;


    TIMERG0.int_clr_timers.t0 = 1;

    task_item_t *next, *current;
    while (tasks_queue != NULL && tasks_queue->predicted_time.predicted <= timer_counter_value) {
        next = tasks_queue->next;
        tasks_queue->next = NULL;

        current = tasks_queue;
        tasks_queue = next;
        ets_printf("Pushing to queue task_id %02x!\n", current->func_code);
        xQueueSendFromISR(executor_queue, current, NULL);
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


static void time_beacon_task(void* args) {
    while (1) {
        // todo: add random additional delay
        vTaskDelay((TIME_BEACON_DELAY_BASE - 500) / portTICK_PERIOD_MS);

        gtsp_current_iteration++;
        ESP_LOGI(TASK_TAG, "Time beacon task iteration %u", gtsp_current_iteration);

        if (!logic_time_in_sync) {
            continue;
        }

        time_model_t time = get_logic_time();

        double avgRate = logic_time_rate;
        double avgOffset = 0;
        uint8_t neighbour_count = 0;
        gtsp_neighbour_t *curr = neighbours, *prev = NULL;

        while (curr != NULL) {
            if (gtsp_current_iteration - curr->iteration > TIME_BEACON_MAX_ITERATION_DIFFERENCE) {
                ESP_LOGI(TASK_TAG, "Deleting neighbour %04x", curr->sender);
                // delete old neighbour
                if (prev != NULL) {
                    prev->next = curr->next;
                } else {
                    neighbours = curr->next;
                }
                curr = curr->next;
                continue;
            }
            if (!curr->read && curr->is_drift_recv) {
                curr->read = true;
                avgRate += curr->relative_logic_rate;
                avgOffset += (double)BEACON_LOGIC_TIME(curr->beacon) - curr->time_recv.logic_time;
                neighbour_count++;
            } else if (curr->time_authority) {
                double hw_time_diff = (double)(time.hardware_time - curr->time_recv.hardware_time) / TIMER_SCALE_MS_FP;
                double predicted_logic_time = BEACON_LOGIC_TIME(curr->beacon) + hw_time_diff;

                avgOffset += predicted_logic_time - time.logic_time;
                neighbour_count++;   
            }

            prev = curr;
            curr = curr->next;
        }

        avgRate /= neighbour_count + 1;
        avgOffset /= neighbour_count + 1;

        logic_time_offset += avgOffset;
        logic_time_rate = 0;

        ESP_LOGI(TASK_TAG, "Current local logic time: %f ms, hardware time: %llu, offset diff/rate: %f/%f", time.logic_time, time.hardware_time/TIMER_SCALE_MS, avgOffset, logic_time_rate);

        if (logic_time_in_sync) {
            time = get_logic_time();
            timesync_beacon_t beacon;
            beacon.hardware_time = (double)(time.hardware_time / TIMER_SCALE_MS_FP);

            uint64_t logic_time_val = (uint64_t)time.logic_time;

            beacon.logic_time_low = (uint32_t)(logic_time_val); // truncate
            beacon.logic_time_high = (uint32_t)(logic_time_val >> 32);

            publish_timesync_data(beacon);
            vTaskDelay((TIME_BEACON_DELAY_BASE - 500) / portTICK_PERIOD_MS);

            time = get_logic_time();
            timesync_drift_beacon_t drift_beacon;
            drift_beacon.hardware_time = (double)(time.hardware_time / TIMER_SCALE_MS_FP);
            drift_beacon.rate = (int32_t)(logic_time_rate * 2147483647.0);

            publish_timedrift_data(drift_beacon);
        }


        // update all predicted task times
        task_item_t *task = tasks_queue;
        while (task != NULL) {
            task->predicted_time = get_hardware_predicted_time_for_logic(task->time);
            task = task->next;
        }
        
        // update alarm for the first task
        if (tasks_queue != NULL) {
            timer_set_alarm_value(TIMER_GROUP_0, 0, tasks_queue->predicted_time.predicted);
            timer_set_alarm(TIMER_GROUP_0, 0, TIMER_ALARM_EN);
        }

    }
}

void initialize_task_executor(void) {
    task_registered_funcs_len = 0;
    xTaskCreate(executor_loop, "executor_loop", 8096, NULL, 3, &executor_task_handle);
    xTaskCreate(time_beacon_task, "time_beacon", 8096, NULL, 3, (void*)(&time_beacon_task_handle));
}

void update_neighbour_time_beacon(uint16_t sender, timesync_beacon_t *beacon) {
    // beacon->hardware_time += TIME_TRANSMISSION_DELAY;
    // beacon->logic_time += TIME_TRANSMISSION_DELAY;
    time_model_t current_time = get_logic_time();

    gtsp_neighbour_t *curr = neighbours, *prev = NULL;

    timesync_beacon_t beacon_val = *beacon;
    //todo: verify somehow if offset is not significantly larger than previously received (due to retransmissions) – idea: storing 5 last offsets
    double offset = (double)BEACON_LOGIC_TIME(beacon_val) - current_time.logic_time;

    if (offset > LOGIC_TIME_THRESHOLD) {
        ESP_LOGI(TASK_TAG, "Logic time threshold excedded, jumping into the future of %llums", BEACON_LOGIC_TIME(beacon_val));
        logic_time_offset = offset;
        logic_time_in_sync = true;
        return;
    }

    while (curr != NULL) {
        // node found
        if (curr->sender == sender) {
            break;
        }

        prev = curr;
        curr = curr->next;
    }

    if (curr == NULL) {
        ESP_LOGI(TASK_TAG, "New neighbour detected: 0x%04x", sender);
        curr = malloc(sizeof(gtsp_neighbour_t));
        curr->sender = sender;
        curr->next = NULL; // append new neighbour to the end
        curr->relative_logic_rate = 1.0;
        curr->time_authority = sender == 1;
        curr->is_drift_recv = false;

        if (prev) {
            prev->next = curr;
        } else {
            // first node
            neighbours = curr;
        }

    } else {
        // double delta_local = (double)(current_time.hardware_time - curr->time_recv.hardware_time) / TIMER_SCALE_MS; // hi
        // uint16_t delta_neighbour = beacon->hardware_time - curr->beacon.hardware_time; // hj - ile faktycznie minęło na innym nodzie

        // if (delta_neighbour == 0) { // it's a retransmission
        //     ESP_LOGI(TASK_TAG, "Already received beacon with this hardware_time, ignoring");
        //     return;
        // }

        // double current_rate = ((double)delta_neighbour - delta_local) / delta_local; // hj / hi

        // curr->relative_logic_rate = current_rate; //
        // ESP_LOGI(TASK_TAG, "delta local/neighbour: %f/%u", delta_local, delta_neighbour);
    }

    curr->iteration = gtsp_current_iteration;
    curr->beacon = *beacon;
    curr->time_recv = current_time;
    curr->read = false;

    ESP_LOGI(TASK_TAG, "Beacon from 0x%04x: relative_logic_rate %f, offset %f", sender, curr->relative_logic_rate, offset);
    ESP_LOGI(TASK_TAG, "logic time: %llu (high/low: %u %u) hardware_time: %u", BEACON_LOGIC_TIME(beacon_val), beacon_val.logic_time_high, beacon_val.logic_time_low, beacon->hardware_time);
}

void update_neighbour_drift_beacon(uint16_t sender, timesync_drift_beacon_t *beacon) {
    time_model_t current_time = get_logic_time();

    gtsp_neighbour_t *curr = neighbours, *prev = NULL;

    while (curr != NULL) {
        // node found
        if (curr->sender == sender) {
            break;
        }

        prev = curr;
        curr = curr->next;
    }

    if (curr == NULL) {
        ESP_LOGI(TASK_TAG, "Rate beacon from unknown neighbour: 0x%04x", sender);
        return;
    }

    if (curr->is_drift_recv == false) {
        curr->is_drift_recv = true;
        curr->drift_beacon = *beacon;
        curr->drift_recv = current_time;
        ESP_LOGI(TASK_TAG, "First rate beacon from neighbour: 0x%04x", sender);
        return;
    }

    double delta_local = (double)(current_time.hardware_time - curr->drift_recv.hardware_time) / TIMER_SCALE_MS_FP; // hi
    uint16_t delta_neighbour = beacon->hardware_time - curr->beacon.hardware_time; // hj - ile faktycznie minęło na innym nodzie

    double current_rate = ((double)delta_neighbour) / delta_local; // hj / hi

    double beacon_logic_rate = ((double)beacon->rate) / 2147483647.0;
    curr->relative_logic_rate = current_rate * beacon_logic_rate;

    if (curr->relative_logic_rate > 0.1) {
        curr->relative_logic_rate = 0.1;
    }
    if (curr->relative_logic_rate < -0.1) {
        curr->relative_logic_rate = -0.1;
    }

    ESP_LOGI(TASK_TAG, "Drift beacon, delta local/neighbour: %f/%u", delta_local, delta_neighbour);

}