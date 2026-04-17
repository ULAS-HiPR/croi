#pragma once
#if F4
#include "stm32f4xx_hal.h"
#include "platform/stm_f4.h"
#endif
#include "cmsis_os.h"
#include <cstdio>
//#include <CAN/CanBus.h>


namespace task{
class CAN {
    public:
        CAN(
        // Radio radio_in, 
            osMessageQueueId_t can_queue_) : 
            //radio_(radio_in),
             can_queue_(can_queue_),taskHandle_(nullptr){};
        void run();

    private:
        void StartCAN();
        static void StartCANEntry(void *argument);
        char parse_message(char msg);

        //CAN& can_bus_;
        osMessageQueueId_t can_queue_;

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

