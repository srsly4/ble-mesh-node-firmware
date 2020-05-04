
/* Blink Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "ble_mesh.h"

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE


#define BLINK_GPIO CONFIG_BLINK_GPIO


#define LED_ON  1
#define LED_OFF 0

struct _led_state {
    uint8_t current;
    uint8_t previous;
    uint8_t pin;
    char *name;
};

extern struct _led_state led_state[3];


void blink_task(void *params)
{
    while(1) {
        /* Blink off (output low) */
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}


static void esp_ble_mesh_custom_model_cb(esp_ble_mesh_model_cb_event_t event,
                                             esp_ble_mesh_model_cb_param_t *param)
{
    ESP_LOGI(TAG, "CUSTOM MODEL EVENT");
    switch (event) {
        case ESP_BLE_MESH_MODEL_OPERATION_EVT:
            if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_SEND) {
                uint16_t tid = *(uint16_t *)param->model_operation.msg;
                ESP_LOGI(TAG, "Recv 0x%06x, tid 0x%04x", param->model_operation.opcode, tid);
                esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
                        param->model_operation.ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS,
                        sizeof(tid), (uint8_t *)&tid);
                if (err) {
                    ESP_LOGE(TAG, "Faild to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
                }
            }
            break;
        case ESP_BLE_MESH_MODEL_SEND_COMP_EVT:
            if (param->model_send_comp.err_code) {
                ESP_LOGE(TAG, "Failed to send message 0x%06x", param->model_send_comp.opcode);
                break;
            }
            ESP_LOGI(TAG, "Send 0x%06x", param->model_send_comp.opcode);
            break;
        default:
            break;
    }
}



void app_main()
{
    /*esp_log_level_set("BLE_MESH", ESP_LOG_VERBOSE);
    esp_log_level_set("BT_LOG", ESP_LOG_VERBOSE);
    esp_log_level_set("bt", ESP_LOG_VERBOSE);
    esp_log_level_set("btc", ESP_LOG_VERBOSE);
    esp_log_level_set("ble", ESP_LOG_VERBOSE);*/

    gpio_pad_select_gpio(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    esp_err_t err;

    ESP_LOGI(TAG, "Initializing...");

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


    TaskHandle_t xHandle = NULL;
    xTaskCreate(blink_task, "blink_task", 8096, NULL, tskIDLE_PRIORITY, &xHandle);
    configASSERT(xHandle);
}