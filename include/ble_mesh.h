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


#define TAG "demo"

static uint8_t dev_uuid[16] = { 0xdd, 0xdd };

#define CID_ESP 0x02E5

#define TASK_VND_MODEL_ID_CLIENT    0x00A1
#define TASK_VND_MODEL_ID_SERVER    0x00A2

#define TASK_VND_MODEL_OP_GET           ESP_BLE_MESH_MODEL_OP_3(0x00, CID_ESP)
#define TASK_VND_MODEL_OP_STATUS        ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)
#define TASK_VND_MODEL_OP_ENQUEUE       ESP_BLE_MESH_MODEL_OP_3(0x02, CID_ESP)



static esp_ble_mesh_cfg_srv_t config_server;

static esp_ble_mesh_model_pub_t onoff_pub_0;
static esp_ble_mesh_gen_onoff_srv_t onoff_server_0;

static esp_ble_mesh_model_op_t vnd_op[];
static esp_ble_mesh_model_t vnd_models[];

static esp_ble_mesh_model_t root_models[];
static esp_ble_mesh_elem_t elements[];

static esp_ble_mesh_comp_t composition;
static esp_ble_mesh_prov_t provision;

void ble_mesh_get_dev_uuid(uint8_t *dev_uuid);
esp_err_t bluetooth_init(void);
esp_err_t ble_mesh_init(void);

#endif
