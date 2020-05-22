#ifndef BLE_MESH_TYPES
#define BLE_MESH_TYPES

#include <stdio.h>

// TIME STRUCTS
typedef struct timesync_beacon_t {
    double rate;
    double logic_time;
    double hardware_time; // miliseconds instead of ticks!
} timesync_beacon_t;

typedef struct time_model_t {
    double logic_time;
    uint64_t hardware_time;
} time_model_t;

typedef struct hardware_predicted_time_t {
    time_model_t base;
    uint64_t predicted;
} hardware_predicted_time_t;

typedef struct gtsp_neighbour_t gtsp_neighbour_t;

typedef struct gtsp_neighbour_t {
    timesync_beacon_t beacon;
    double relative_logic_rate;
    time_model_t time_recv;
    uint32_t iteration;
    uint16_t sender;
    struct gtsp_neighbour_t *next;
} gtsp_neighbour_t;


// TASKS STRUCTS
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
    uint16_t tid;
    uint64_t time;
    hardware_predicted_time_t predicted_time;
    uint16_t func_code;
    void* arg_data;
    task_registered_ret_t *res_data;
    struct task_item_t *next;
};

#endif
