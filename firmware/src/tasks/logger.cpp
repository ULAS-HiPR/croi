#include "logger.h"
#include <cstdio>

namespace task {

void Logger::run() {
    taskHandle_ = osThreadNew(&Logger::StartLoggerEntry,
                              this,
                              &task_attributes);
}

void Logger::StartLoggerEntry(void *argument) {
    printf("Logger starting1\n");
    auto *self = static_cast<Logger*>(argument);
    if (self) {
        self->StartLogger();
    }
}

void Logger::StartLogger() {
    printf("Logger started\n");
    flight_data data;
    for (;;) {
        osMessageQueueGet(&logger_queue_, &data, 0, 0);
        //storage_.write(&msg);
        printf("Logged message: %u\n", data.time);
        osDelay(200);  
    }
}
}

