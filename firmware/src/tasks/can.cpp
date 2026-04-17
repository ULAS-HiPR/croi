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

    for (;;) {
        //char data = radio.read();
        char data = 'r'; // placeholder for testing
        //  get data from can bus
        // parse data & send to controller
        printf("Parsed message: %d\n", data);
        osMessageQueuePut(can_queue_, &data, 0, 0);
        osDelay(1000);  
    }
}

char CAN::parse_message(char msg){
    // placeholder for parsing logic
    return msg;
}

}