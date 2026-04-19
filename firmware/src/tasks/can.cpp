#include "can.h"


namespace task {

void CAN::run() {
    taskHandle_ = osThreadNew(&CAN::StartCANEntry,
                              this,
                              &task_attributes);
}

void CAN::StartCANEntry(void *argument) {
    auto *self = static_cast<CAN*>(argument);
    if (self) {
        self->StartCAN();
    }
}

void CAN::StartCAN() {

    printf("CAN started\n");
    flight_data data_to_send;
    for (;;) {
        //char data = radio.read();
        //  get data from can bus
        // parse data & send to controller
        osStatus_t status;
        status = osMessageQueueGet(can_s_queue_, &data_to_send, NULL, 0U);   

        if (status == osOK) {
            printf("Parsed message: %d\n", data_to_send.state);
            //cans.send(data_to_send);
        }

        // if can message reciver: osMessageQueuePut(can_queue_, &data, 0, 0);
        
        osDelay(1000);  
    }
}

char CAN::parse_message(char msg){
    // placeholder for parsing logic
    return msg;
}

}
