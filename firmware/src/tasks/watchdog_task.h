#pragma once

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

namespace task {

class WatchdogTask {
public:
    bool run();

private:
    static void entry(void* argument);
    void loop();

    osThreadId_t task_handle_{nullptr};
    StaticTask_t task_control_block_{};
    StackType_t task_stack_[512U / sizeof(StackType_t)]{};
    const osThreadAttr_t task_attributes_{
        "Watchdog",
        0U,
        &task_control_block_,
        sizeof(task_control_block_),
        task_stack_,
        sizeof(task_stack_),
        osPriorityLow,
        0U,
        0U,
    };
};

}
