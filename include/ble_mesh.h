#ifndef BLE_MESH_CONF
#define BLE_MESH_CONF

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_health_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"

#include "types.h"
#include "tasks.h"

#define TAG "blemesh"

#define CID_ESP 0x02E5

#define TASK_VND_MODEL_ID_CLIENT    0x00A1
#define TASK_VND_MODEL_ID_SERVER    0x00A2

#define TASK_VND_MODEL_OP_GET           ESP_BLE_MESH_MODEL_OP_3(0x00, CID_ESP)
#define TASK_VND_MODEL_OP_STATUS        ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)
#define TASK_VND_MODEL_OP_ENQUEUE       ESP_BLE_MESH_MODEL_OP_3(0x02, CID_ESP)

#define TIMESYNC_VND_MODEL_ID_CLIENT    0x00A3
#define TIMESYNC_VND_MODEL_ID_SERVER    0x00A4

#define TIMESYNC_VND_MODEL_OP_BEACON    ESP_BLE_MESH_MODEL_OP_3(0x05, CID_ESP)
#define TIMESYNC_VND_MODEL_OP_DRIFT_BEACON    ESP_BLE_MESH_MODEL_OP_3(0x06, CID_ESP)

#define TIMESYNC_VND_MODEL_ADDRESS_PUBLISH 0xFFFF


typedef struct __attribute__ ((__packed__)) task_enqueue_t {
    uint16_t tid;
    uint8_t func_code;
    uint64_t time;
    void* args;
} __attribute__ ((__packed__)) task_enqueue_t;

void ble_mesh_get_dev_uuid(uint8_t *dev_uuid);
esp_err_t bluetooth_init(void);
esp_err_t ble_mesh_init(void);

void publish_timesync_data(timesync_beacon_t beacon_data);
void publish_timedrift_data(timesync_drift_beacon_t beacon_data);

#endif
