#pragma once

#include "cmsis_os.h"
#include <Flash/flash.h>
#include <cstdint>

#ifndef FLASH_READOUT_CSV_CAPACITY
#define FLASH_READOUT_CSV_CAPACITY 4096U
#endif

extern "C" {
extern volatile uint32_t g_flash_csv_ready;
extern volatile uint32_t g_flash_csv_len;
extern volatile uint32_t g_flash_csv_error;
extern volatile uint32_t g_flash_csv_records;
extern volatile uint32_t g_flash_csv_truncated;
extern char g_flash_csv_dump[FLASH_READOUT_CSV_CAPACITY];
}

namespace task {

class FlashReadoutTask {
public:
    explicit FlashReadoutTask(Flash& storage) : storage_(storage) {}

    void run();

private:
    void start();
    static void entry(void* argument);

    Flash& storage_;
    osThreadId_t taskHandle_{nullptr};

    const osThreadAttr_t task_attributes_{
        "FlashReadout",
        0,
        nullptr,
        0,
        nullptr,
        1024,
        osPriorityLow,
        0,
        0
    };
};

} // namespace task
