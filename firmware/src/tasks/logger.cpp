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
    for (;;) {
        uint32_t msg;
        osMessageQueueGet(&logger_queue_, &msg, 0, 0);
        //storage_.write(&msg);
        printf("Logged message: %u\n", msg);
        osDelay(200);  
    }
}
}
