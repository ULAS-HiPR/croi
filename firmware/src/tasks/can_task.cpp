#include "can_task.h"

namespace task {

void CAN_Task::run() {
    taskHandle_ = osThreadNew(&CAN_Task::StartCAN_TaskEntry,
                              this,
                              &task_attributes);
}

void CAN_Task::StartCAN_TaskEntry(void *argument) {
    auto *self = static_cast<CAN_Task*>(argument);
    if (self) {
        self->StartCAN_Task();
    }
}

void CAN_Task::StartCAN_Task() {

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
 
uint32_t* CAN_Task::construct_can_message(flight_data data){
    
}

char CAN_Task::parse_message(char msg){
    // placeholder for parsing logic
    return msg;
}

}
