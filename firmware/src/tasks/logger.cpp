#include "logger.h"
#include "platform/hal_time.h"

namespace task {

namespace {
constexpr uint32_t max_payload_size() {
    return sizeof(flight_data) > sizeof(secondary_flight_data)
               ? sizeof(flight_data)
               : sizeof(secondary_flight_data);
}
}

void Logger::run() {
    taskHandle_ = osThreadNew(&Logger::StartLoggerEntry,
                              this,
                              &task_attributes);

    if (taskHandle_ == nullptr)
    {
        latch_fault(LoggerFault::Unknown, FlashLogStatus::NotInitialized);
        printf("LOGGER TASK CREATION FAILED!\n");
    }
    else
    {
        printf("Logger task created: %p\n", taskHandle_);
    }
}

void Logger::StartLoggerEntry(void *argument) {
    auto *self = static_cast<Logger*>(argument);
    if (self) {
        self->StartLogger();
    }
}

void Logger::StartLogger() {
    printf("Logger started\n");
    if (health_ != nullptr) {
        health_->initialized = false;
        health_->preflight_ok = false;
        health_->fault_latched = false;
        health_->logging_stopped = false;
        health_->fault = LoggerFault::None;
        health_->flash_status = FlashLogStatus::NotInitialized;
        health_->run_id = 0U;
        health_->records_written = 0U;
    }

    if (storage_ == nullptr) {
        latch_fault(LoggerFault::NoStorage, FlashLogStatus::NotInitialized);
        flight_data ignored{};
        for (;;) {
            (void)osMessageQueueGet(logger_queue_, &ignored, 0, 0U);
            (void)osMessageQueueGet(reciver_queue_, &ignored, 0, 0U);
            osDelay(LOGGER_DELAY_MS);
        }
    }

    FlashLogger flash_logger(*storage_);
    if (storage_ != nullptr && configure_logger(flash_logger)) {
        update_health_ok(flash_logger);
    }

    flight_data data;
    for (;;) {
        if (osMessageQueueGet(logger_queue_,&data, 0, 0U) == osOK) {
            if (is_logger_active(static_cast<State>(data.state), data.time)) {
                (void)log_flight_data(flash_logger, data);
            }
        }   
        if (osMessageQueueGet(reciver_queue_, &data, 0, 0U) == osOK) {
            (void)log_flight_data(flash_logger, data);
        }
        osDelay(LOGGER_DELAY_MS); // Check for new messages every 100 ms
    }  
}

bool Logger::configure_logger(FlashLogger& flash_logger) {
    if (!storage_->init()) {
        latch_fault(LoggerFault::FlashInitFailed, FlashLogStatus::FlashError);
        return false;
    }

    FlashLogConfig config = FlashLogger::default_mx25_config(
        LOGGER_FLASH_START_ADDRESS,
        LOGGER_FLASH_LENGTH);
    config.max_payload_size = max_payload_size();
    config.verify_writes = true;

    if (!flash_logger.begin(config)) {
        latch_fault(map_flash_status(flash_logger.status()), flash_logger.status());
        return false;
    }

    return true;
}

bool Logger::log_flight_data(FlashLogger& flash_logger, const flight_data& data) {
    if (health_ != nullptr && health_->logging_stopped) {
        return false;
    }

    const bool ok = flash_logger.append_record_typed(
        data,
        HAL_GetTick(),
        FlashLogPayloadType::FlightData);

    if (!ok) {
        latch_fault(map_flash_status(flash_logger.status()), flash_logger.status());
        if (preflight_state(static_cast<State>(data.state)) && health_ != nullptr) {
            health_->preflight_ok = false;
        }
        return false;
    }

    if (health_ != nullptr) {
        const FlashLogInfo info = flash_logger.info();
        health_->flash_status = FlashLogStatus::Ok;
        health_->records_written = info.record_count;
        health_->run_id = info.run_id;
    }

    return true;
}

bool Logger::log_secondary_data(FlashLogger& flash_logger,
                                const secondary_flight_data& data) {
    if (health_ != nullptr && health_->logging_stopped) {
        return false;
    }

    const bool ok = flash_logger.append_record_typed(
        data,
        HAL_GetTick(),
        FlashLogPayloadType::SecondaryFlightData);

    if (!ok) {
        latch_fault(map_flash_status(flash_logger.status()), flash_logger.status());
        return false;
    }

    if (health_ != nullptr) {
        const FlashLogInfo info = flash_logger.info();
        health_->flash_status = FlashLogStatus::Ok;
        health_->records_written = info.record_count;
        health_->run_id = info.run_id;
    }

    return true;
}

void Logger::latch_fault(LoggerFault fault, FlashLogStatus status) {
    if (health_ == nullptr) {
        return;
    }

    health_->fault = fault;
    health_->flash_status = status;
    health_->fault_latched = true;
    health_->logging_stopped = true;
}

void Logger::update_health_ok(const FlashLogger& flash_logger) {
    if (health_ == nullptr) {
        return;
    }

    const FlashLogInfo info = flash_logger.info();
    health_->initialized = true;
    health_->preflight_ok = true;
    health_->fault_latched = false;
    health_->logging_stopped = false;
    health_->fault = LoggerFault::None;
    health_->flash_status = FlashLogStatus::Ok;
    health_->run_id = info.run_id;
    health_->records_written = info.record_count;
}

bool Logger::preflight_state(State state) {
    return state == State::CALIBRATING || state == State::READY;
}

LoggerFault Logger::map_flash_status(FlashLogStatus status) {
    switch (status) {
        case FlashLogStatus::Ok:
            return LoggerFault::None;
        case FlashLogStatus::NotInitialized:
            return LoggerFault::NotInitialized;
        case FlashLogStatus::BadConfig:
        case FlashLogStatus::RunIdExhausted:
            return LoggerFault::LogBeginFailed;
        case FlashLogStatus::Full:
            return LoggerFault::Full;
        case FlashLogStatus::PayloadTooLarge:
            return LoggerFault::PayloadTooLarge;
        case FlashLogStatus::Corrupt:
            return LoggerFault::Corrupt;
        case FlashLogStatus::VerifyFailed:
            return LoggerFault::VerifyFailed;
        case FlashLogStatus::FlashError:
            return LoggerFault::FlashIo;
        default:
            return LoggerFault::Unknown;
    }
}


bool Logger::is_logger_active(State state, uint32_t time) {
    if (logging_stop_timer && (state == State::LANDED)) {
        logging_stop_timer = false;
        endlog_time = time;

        return true;
    }
    if (!logging_stop_timer && (time - endlog_time) > LOGGER_POST_LANDING_MS) {
        return false; // Stop logging after 1 minute on the ground
    }else{
        return true; // Continue logging
    }
}
}
