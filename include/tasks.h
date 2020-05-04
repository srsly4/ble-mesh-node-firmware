#include <stdio.h>

typedef struct task_item_t task_item_t;

struct task_item_t {
    uint64_t time;
    uint16_t func_code;
    void* arg_data;
    void* res_data;
    struct task_item_t *next;
};

static task_item_t *tasks_queue;
static task_item_t *tasks_finished;

void enqueue_task(task_item_t *task) {
    uint64_t target_time = task->time;

    if (tasks_queue == NULL) {
        tasks_queue = task;
        return;
    }

    task_item_t* curr = tasks_queue;
    while (curr->next != NULL && curr->next->time < target_time) {
        curr = curr->next;
    }

    task->next= curr->next;
    curr->next = task;
}

void finalize_task(void) {
    task_item_t *task = tasks_queue;

    tasks_queue = task->next;


    task->next = tasks_finished;
    tasks_finished = task;
}

