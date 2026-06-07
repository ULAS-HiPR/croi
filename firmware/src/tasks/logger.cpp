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
        if (osMessageQueueGet(logger_queue_,&data, 0, 0U) == osOK) {
            if (is_logger_active(static_cast<State>(data.state), data.time)) {
                // Log the data
                storage_->write(data.time, reinterpret_cast<uint8_t*>(&data), sizeof(flight_data));
                 printf("Logged message: %u\n", data.time);
            }
        }   
    }
    osDelay(200);  
}


bool Logger::is_logger_active(State state, uint32_t time) {
    if (logging_stop_timer && (state == State::LANDED)) {
        logging_stop_timer = false;
        endlog_time = time;

        return true;
    }
    if (!logging_stop_timer && (time - endlog_time) > 60000) { // 1 minute after landing
        return false; // Stop logging after 1 minute on the ground
    }else{
        return true; // Continue logging
    }
}
}
