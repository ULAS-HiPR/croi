#pragma once
#include <cstdint>
#if F4
#include "stm32f4xx_hal.h"
#include "platform/stm_f4.h"
#endif
#include "cmsis_os.h"
#include <cstdio>

#include <data.h>
#include <Flash/flash.h>

#define LOGGER_DELAY_MS 100

namespace task{
class Logger {
    public:
        Logger(Flash* storage, osMessageQueueId_t logger_queue, osMessageQueueId_t reciver_queue) :
                storage_(storage),
                logger_queue_(logger_queue),
                reciver_queue_(reciver_queue),
                taskHandle_(nullptr)
        {};

        void run();

        struct LogMessage {
            uint32_t timestamp;
            imu_data imu;
            //add in for croi
        };

    private:
        void StartLogger();
        static void StartLoggerEntry(void *argument);
        uint32_t endlog_time{0};
        bool logging_stop_timer{true};
        bool is_logger_active(State state, uint32_t time);
       

        Flash* storage_;
        osMessageQueueId_t logger_queue_;
        osMessageQueueId_t reciver_queue_;
        osThreadId_t taskHandle_;

        const osThreadAttr_t task_attributes {
            "Logger",
            0,
            nullptr,
            0,
            nullptr,
            512,        // 1 KB stack
            osPriorityLow,
            0,
            0
        };

    };

}

