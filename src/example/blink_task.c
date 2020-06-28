#include "example/blink_task.h"

#include "esp_log.h"

#define BLINK_TASK_TAG "blink"


task_registered_ret_t do_blink_task(void* args) {
    ESP_LOGI(BLINK_TASK_TAG, "Task started");


    uint8_t count = args != NULL ? *((uint8_t*) args) : 5;

    for (int i = 0; i < count; i++) {
        /* Blink on (output high) */
        gpio_set_level(CONFIG_BLINK_GPIO, 1);
        gpio_set_level(26, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        /* Blink off (output low) */
        gpio_set_level(CONFIG_BLINK_GPIO, 0);
        gpio_set_level(26, 0);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    uint8_t* ret_data = malloc(sizeof(uint8_t));

    *ret_data = 123;

    task_registered_ret_t ret = {
        .ret_len = sizeof(uint8_t),
        .data = ret_data
    };
    return ret;
}


void register_all_tasks() {
    register_task(0xBB, &do_blink_task);
}