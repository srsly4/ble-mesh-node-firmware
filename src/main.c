
/* Blink Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "ble_mesh.h"
#include "tasks.h"
#include "example/blink_task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "wifi.h"

#define BLINK_GPIO CONFIG_BLINK_GPIO

static uint8_t dev_uuid[16] = { 0xdd, 0xdd };

void app_main()
{

    gpio_pad_select_gpio(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    esp_err_t err;


    ESP_LOGI(TAG, "Initializing timers...");
    initialize_task_timer();
    ESP_LOGI(TAG, "Initializing executor...");
    initialize_task_executor();
    ESP_LOGI(TAG, "Registering available tasks...");
    register_all_tasks();

    ESP_LOGI(TAG, "Initializing BT & BLE MESH...");

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = bluetooth_init();
    if (err) {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    ble_mesh_get_dev_uuid(dev_uuid);

    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    example_connect();
}