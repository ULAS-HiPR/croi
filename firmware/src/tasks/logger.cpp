#include "logger.h"
#include "platform/hal_time.h"

namespace task {

namespace {
constexpr uint32_t max_payload_size() {
    return sizeof(flight_data) > sizeof(secondary_flight_data)
               ? sizeof(flight_data)
               : sizeof(secondary_flight_data);
}

#ifdef READING
// Reads one record at `cursor`, type-checks the payload length against the
// struct it claims to be, and copies into the matching out-param on success.
// Returns false on end-of-log, corruption, or an unrecognized/size-mismatched
// record. Caller should inspect flash_logger.status() to distinguish
// "done" (NotFound) from "skip and keep going" (anything else).
bool read_one_record(FlashLogger& flash_logger,
                     FlashLogCursor& cursor,
                     flight_data& flight_out,
                     secondary_flight_data& secondary_out,
                     FlashLogPayloadType& type_out) {
    FlashLogRecordHeader header{};
    uint8_t raw[max_payload_size()];
    size_t payload_len = 0U;

    type_out = FlashLogPayloadType::Unspecified;

    if (!flash_logger.read_next(cursor, header, raw, sizeof(raw), &payload_len)) {
        return false; // NotFound (end of log) or genuine corrupt/flash error
    }

    switch (static_cast<FlashLogPayloadType>(header.payload_type)) {
        case FlashLogPayloadType::FlightData:
            if (payload_len != sizeof(flight_data)) {
                printf("SKIP @0x%08lx: FlightData size mismatch (%u != %u)\n",
                       static_cast<unsigned long>(cursor.address),
                       static_cast<unsigned>(payload_len),
                       static_cast<unsigned>(sizeof(flight_data)));
                return false;
            }
            std::memcpy(&flight_out, raw, sizeof(flight_data));
            type_out = FlashLogPayloadType::FlightData;
            return true;

        case FlashLogPayloadType::SecondaryFlightData:
            if (payload_len != sizeof(secondary_flight_data)) {
                printf("SKIP @0x%08lx: SecondaryFlightData size mismatch (%u != %u)\n",
                       static_cast<unsigned long>(cursor.address),
                       static_cast<unsigned>(payload_len),
                       static_cast<unsigned>(sizeof(secondary_flight_data)));
                return false;
            }
            std::memcpy(&secondary_out, raw, sizeof(secondary_flight_data));
            type_out = FlashLogPayloadType::SecondaryFlightData;
            return true;

        default:
            printf("SKIP @0x%08lx: unknown payload type %u, len %u\n",
                   static_cast<unsigned long>(cursor.address),
                   static_cast<unsigned>(header.payload_type),
                   static_cast<unsigned>(payload_len));
            return false;
    }
}
#endif // READING
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
#ifdef READING
    printf("READ MODE ENABLED\n");

    if (storage_ == nullptr || !storage_->init()) {
        printf("FLASH INIT FAILED\n");
        for (;;) {
            osDelay(1000);
        }
    }

    FlashLogger flash_logger(*storage_);

    FlashLogConfig config = FlashLogger::default_mx25_config(
        LOGGER_FLASH_START_ADDRESS,
        LOGGER_FLASH_LENGTH);
    config.max_payload_size = max_payload_size();
    config.verify_writes = true;

    // begin() (not begin(config, run_id)) just scans and adopts the existing
    // log so we can read whatever's already on the chip without rolling the
    // run_id or erasing anything.
    if (!flash_logger.begin(config)) {
        latch_fault(map_flash_status(flash_logger.status()), flash_logger.status());

    }
    const FlashLogInfo info = flash_logger.info();
    printf("LOG INFO: records=%lu run_id=%lu highest_run_id=%lu used=%lu/%lu\n",
           static_cast<unsigned long>(info.record_count),
           static_cast<unsigned long>(info.run_id),
           static_cast<unsigned long>(info.highest_run_id),
           static_cast<unsigned long>(info.used_bytes),
           static_cast<unsigned long>(info.end_address - info.start_address));

    FlashLogCursor cursor = flash_logger.cursor();
    flight_data flight_record{};
    secondary_flight_data secondary_record{};
    FlashLogPayloadType type{};

    for (;;) {
        if (read_one_record(flash_logger, cursor, flight_record, secondary_record, type)) {
            switch (type) {
                case FlashLogPayloadType::FlightData:
                    printf("FLIGHT @0x%08lx state=%d time=%lu\n",
                           static_cast<unsigned long>(cursor.address),
                           static_cast<int>(flight_record.state),
                           static_cast<unsigned long>(flight_record.time));
                    break;
                case FlashLogPayloadType::SecondaryFlightData:
                    printf("SECONDARY @0x%08lx\n",
                           static_cast<unsigned long>(cursor.address));
                    break;
                default:
                    break;
            }
        } else if (flash_logger.status() == FlashLogStatus::NotFound) {
            // Reached the end of written records — wrap around and re-scan.
            printf("END OF LOG, restarting scan\n");
            cursor = flash_logger.cursor();
            osDelay(1000);
        }
        // Any other status (Corrupt/Incomplete/etc) was already logged by
        // read_one_record; read_next() has already advanced past it.

        osDelay(LOGGER_DELAY_MS);
    }

#else

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
        secondary_flight_data ignored_secondary{};
        for (;;) {
            (void)osMessageQueueGet(logger_queue_, &ignored, 0, 0U);
            (void)osMessageQueueGet(reciver_queue_, &ignored_secondary, 0, 0U);
            osDelay(LOGGER_DELAY_MS);
        }
    }

    FlashLogger flash_logger(*storage_);
    if (configure_logger(flash_logger)) {
        update_health_ok(flash_logger);
    } else {
        flight_data ignored{};
        secondary_flight_data ignored_secondary{};
        for (;;) {
            (void)osMessageQueueGet(logger_queue_, &ignored, 0, 0U);
            (void)osMessageQueueGet(reciver_queue_, &ignored_secondary, 0, 0U);
            osDelay(LOGGER_DELAY_MS);
        }
    }

    flight_data data;
    secondary_flight_data secondary_data;

    for (;;) {
        if (osMessageQueueGet(logger_queue_, &data, 0, 0U) == osOK) {
            if (is_logger_active(static_cast<State>(data.state), data.time)) {
                (void)log_flight_data(flash_logger, data);
            }
        }

        if (osMessageQueueGet(reciver_queue_, &secondary_data, 0, 0U) == osOK) {
            (void)log_secondary_data(flash_logger, secondary_data);
        }

        osDelay(LOGGER_DELAY_MS);
    }
#endif
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