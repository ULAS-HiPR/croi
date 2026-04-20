#pragma once
#if F4
#include "stm32f4xx_hal.h"
#include "platform/stm_f4.h"
#endif
#include "cmsis_os.h"
#include <cstdio>
#include <data.h>
//#include <CAN/CanBus.h>


namespace task{
class CAN {
    public:
        CAN(osMessageQueueId_t can_r_queue, osMessageQueueId_t can_s_queue) : can_r_queue_(can_r_queue), can_s_queue_(can_s_queue), taskHandle_(nullptr) {};
        void run();

    private:
        void StartCAN();
        static void StartCANEntry(void *argument);
        char parse_message(char msg);
        uint* construct_can_message(flight_data data);

        //CAN& can_bus_;
        osMessageQueueId_t can_r_queue_;
        osMessageQueueId_t can_s_queue_;

        osThreadId_t taskHandle_;

        const osThreadAttr_t task_attributes {
            "CAN",
            0,
            nullptr,
            0,
            nullptr,
            512 * 4,        // 2 KB stack
            osPriorityNormal,
            0,
            0
        };
    };

}

