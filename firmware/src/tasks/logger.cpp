#include "logger.h"
#include "croi_status.h"
#include "platform/hal_time.h"

extern "C" {

struct OgmaFlashReadMailbox {
    uint32_t magic;
    uint32_t version;
    uint32_t command;
    uint32_t state;
    uint32_t request_seq;
    uint32_t response_seq;
    uint32_t address;
    uint32_t length;
    uint32_t bytes_read;
    uint32_t result;
    uint32_t log_start;
    uint32_t log_length;
    uint8_t buffer[512];
};

volatile OgmaFlashReadMailbox ogma_flash_mailbox{};

struct OgmaDebugControl {
    uint32_t magic;
    uint32_t version;
    uint32_t request_seq;
    uint32_t unlock_key;
    uint32_t lease_ms;
    uint32_t unlocked_until_ms;
    uint32_t accepted_seq;
    uint32_t denied_count;
};

__attribute__((used)) volatile OgmaDebugControl ogma_debug_control{};

}

namespace task {

namespace {
constexpr uint32_t OGMA_FLASH_MAILBOX_MAGIC = 0x4F47464DU; // OGFM
constexpr uint32_t OGMA_FLASH_MAILBOX_VERSION = 1U;
constexpr uint32_t OGMA_FLASH_COMMAND_READ = 1U;
constexpr uint32_t OGMA_FLASH_STATE_IDLE = 0U;
constexpr uint32_t OGMA_FLASH_STATE_BUSY = 1U;
constexpr uint32_t OGMA_FLASH_STATE_DONE = 2U;
constexpr uint32_t OGMA_FLASH_STATE_ERROR = 3U;
constexpr uint32_t OGMA_FLASH_RESULT_OK = 0U;
constexpr uint32_t OGMA_FLASH_RESULT_BAD_COMMAND = 1U;
constexpr uint32_t OGMA_FLASH_RESULT_BAD_RANGE = 2U;
constexpr uint32_t OGMA_FLASH_RESULT_READ_FAILED = 3U;
constexpr uint32_t OGMA_FLASH_RESULT_LOCKED = 4U;
constexpr uint32_t OGMA_FLASH_RESULT_UNSAFE_STATE = 5U;
constexpr uint32_t OGMA_FLASH_CHUNK_BYTES = 512U;
constexpr uint32_t OGMA_DEBUG_CONTROL_MAGIC = 0x4F474442U; // OGDB
constexpr uint32_t OGMA_DEBUG_CONTROL_VERSION = 1U;
constexpr uint32_t OGMA_DEBUG_UNLOCK_KEY = 0x0BEE11E5U;
constexpr uint32_t OGMA_DEBUG_MIN_LEASE_MS = 1000U;
constexpr uint32_t OGMA_DEBUG_MAX_LEASE_MS = 60000U;

bool tick_after(uint32_t a, uint32_t b) {
    return static_cast<int32_t>(a - b) > 0;
}

bool flash_read_allowed() {
    const uint32_t state = croi_status.flight_state;
    return state < 2U || state >= 6U;
}

bool debug_lease_active(uint32_t now_ms) {
    if (ogma_debug_control.magic == OGMA_DEBUG_CONTROL_MAGIC &&
        ogma_debug_control.version == OGMA_DEBUG_CONTROL_VERSION &&
        ogma_debug_control.request_seq != 0U &&
        ogma_debug_control.request_seq != ogma_debug_control.accepted_seq &&
        ogma_debug_control.unlock_key == OGMA_DEBUG_UNLOCK_KEY) {
        uint32_t lease_ms = ogma_debug_control.lease_ms;
        if (lease_ms < OGMA_DEBUG_MIN_LEASE_MS) {
            lease_ms = OGMA_DEBUG_MIN_LEASE_MS;
        }
        if (lease_ms > OGMA_DEBUG_MAX_LEASE_MS) {
            lease_ms = OGMA_DEBUG_MAX_LEASE_MS;
        }
        ogma_debug_control.unlocked_until_ms = now_ms + lease_ms;
        ogma_debug_control.accepted_seq = ogma_debug_control.request_seq;
        ogma_debug_control.unlock_key = 0U;
    }

    return tick_after(ogma_debug_control.unlocked_until_ms, now_ms);
}

void deny_debug_command() {
    ogma_debug_control.denied_count++;
}

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

bool Logger::run() {
    taskHandle_ = osThreadNew(&Logger::StartLoggerEntry,
                              this,
                              &task_attributes);

    if (taskHandle_ == nullptr)
    {
        latch_fault(LoggerFault::Unknown, FlashLogStatus::NotInitialized);
        printf("LOGGER TASK CREATION FAILED!\n");
        return false;
    }
    else
    {
        printf("Logger task created: %p\n", taskHandle_);
    }
    return true;
}

void Logger::StartLoggerEntry(void *argument) {
    auto *self = static_cast<Logger*>(argument);
    if (self) {
        self->StartLogger();
    }
}

void Logger::StartLogger() {
    printf("Logger started\n");
    ogma_flash_mailbox.magic = OGMA_FLASH_MAILBOX_MAGIC;
    ogma_flash_mailbox.version = OGMA_FLASH_MAILBOX_VERSION;
    ogma_flash_mailbox.state = OGMA_FLASH_STATE_IDLE;
    ogma_flash_mailbox.result = OGMA_FLASH_RESULT_OK;
    ogma_flash_mailbox.log_start = LOGGER_FLASH_START_ADDRESS;
    ogma_flash_mailbox.log_length = health_ == nullptr ? 0U : health_->used_bytes;
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
    if (health_ != nullptr) {
        health_->initialized = true;
        health_->preflight_ok = true;
        health_->fault_latched = false;
        health_->logging_stopped = false;
        health_->fault = LoggerFault::None;
        health_->flash_status = FlashLogStatus::Ok;
        health_->run_id = info.run_id;
        health_->records_written = info.record_count;
        health_->used_bytes = info.used_bytes;
        mirror_health_to_status();
    }
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
        service_flash_mailbox();

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
        health_->used_bytes = 0U;
        mirror_health_to_status();
    }

    if (storage_ == nullptr) {
        latch_fault(LoggerFault::NoStorage, FlashLogStatus::NotInitialized);
        flight_data ignored{};
        secondary_flight_data ignored_secondary{};
        for (;;) {
            croi_status.logger_task_heartbeat_ms = HAL_GetTick();
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
            croi_status.logger_task_heartbeat_ms = HAL_GetTick();
            (void)osMessageQueueGet(logger_queue_, &ignored, 0, 0U);
            (void)osMessageQueueGet(reciver_queue_, &ignored_secondary, 0, 0U);
            osDelay(LOGGER_DELAY_MS);
        }
    }

    flight_data data;
    secondary_flight_data secondary_data;

    for (;;) {
        croi_status.logger_task_heartbeat_ms = HAL_GetTick();
        service_flash_mailbox();

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

void Logger::service_flash_mailbox() {
    croi_status.uptime_ms = HAL_GetTick();
    ogma_flash_mailbox.magic = OGMA_FLASH_MAILBOX_MAGIC;
    ogma_flash_mailbox.version = OGMA_FLASH_MAILBOX_VERSION;
    ogma_flash_mailbox.log_start = LOGGER_FLASH_START_ADDRESS;
    ogma_flash_mailbox.log_length = health_ == nullptr ? 0U : health_->used_bytes;
    mirror_health_to_status();

    if (storage_ == nullptr ||
        ogma_flash_mailbox.command == 0U ||
        ogma_flash_mailbox.request_seq == 0U ||
        ogma_flash_mailbox.request_seq == ogma_flash_mailbox.response_seq) {
        return;
    }

    const uint32_t request_seq = ogma_flash_mailbox.request_seq;
    const uint32_t command = ogma_flash_mailbox.command;
    const uint32_t address = ogma_flash_mailbox.address;
    uint32_t length = ogma_flash_mailbox.length;

    if (!debug_lease_active(HAL_GetTick())) {
        deny_debug_command();
        ogma_flash_mailbox.result = OGMA_FLASH_RESULT_LOCKED;
        ogma_flash_mailbox.state = OGMA_FLASH_STATE_ERROR;
        ogma_flash_mailbox.response_seq = request_seq;
        return;
    }

    if (!flash_read_allowed()) {
        deny_debug_command();
        ogma_flash_mailbox.result = OGMA_FLASH_RESULT_UNSAFE_STATE;
        ogma_flash_mailbox.state = OGMA_FLASH_STATE_ERROR;
        ogma_flash_mailbox.response_seq = request_seq;
        return;
    }

    ogma_flash_mailbox.state = OGMA_FLASH_STATE_BUSY;
    ogma_flash_mailbox.bytes_read = 0U;
    ogma_flash_mailbox.result = OGMA_FLASH_RESULT_OK;

    if (command != OGMA_FLASH_COMMAND_READ) {
        ogma_flash_mailbox.result = OGMA_FLASH_RESULT_BAD_COMMAND;
        ogma_flash_mailbox.state = OGMA_FLASH_STATE_ERROR;
        ogma_flash_mailbox.response_seq = request_seq;
        return;
    }

    if (length == 0U || length > OGMA_FLASH_CHUNK_BYTES ||
        address < LOGGER_FLASH_START_ADDRESS ||
        address > (LOGGER_FLASH_START_ADDRESS + LOGGER_FLASH_LENGTH) ||
        length > ((LOGGER_FLASH_START_ADDRESS + LOGGER_FLASH_LENGTH) - address)) {
        ogma_flash_mailbox.result = OGMA_FLASH_RESULT_BAD_RANGE;
        ogma_flash_mailbox.state = OGMA_FLASH_STATE_ERROR;
        ogma_flash_mailbox.response_seq = request_seq;
        return;
    }

    if (!storage_->read(address,
                        const_cast<uint8_t*>(ogma_flash_mailbox.buffer),
                        length)) {
        ogma_flash_mailbox.result = OGMA_FLASH_RESULT_READ_FAILED;
        ogma_flash_mailbox.state = OGMA_FLASH_STATE_ERROR;
        ogma_flash_mailbox.response_seq = request_seq;
        return;
    }

    ogma_flash_mailbox.bytes_read = length;
    ogma_flash_mailbox.result = OGMA_FLASH_RESULT_OK;
    ogma_flash_mailbox.state = OGMA_FLASH_STATE_DONE;
    ogma_flash_mailbox.response_seq = request_seq;
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

#if defined(CROI_WIPE_FLASH_ON_BOOT)
    constexpr uint32_t kWipeChunkBytes = 0x00010000U;
    croi_status.flash_wipe_state = 1U;
    croi_status.flash_wipe_progress_percent = 0U;
    croi_status.flash_wipe_address = LOGGER_FLASH_START_ADDRESS;

    for (uint32_t offset = 0U; offset < LOGGER_FLASH_LENGTH; offset += kWipeChunkBytes) {
        const uint32_t remaining = LOGGER_FLASH_LENGTH - offset;
        const uint32_t chunk = remaining < kWipeChunkBytes ? remaining : kWipeChunkBytes;
        const uint32_t address = LOGGER_FLASH_START_ADDRESS + offset;
        croi_status.flash_wipe_address = address;
        croi_status.flash_wipe_progress_percent = (offset * 100U) / LOGGER_FLASH_LENGTH;

        if (!storage_->erase(address, chunk)) {
            croi_status.flash_wipe_state = 3U;
            latch_fault(LoggerFault::FlashIo, FlashLogStatus::FlashError);
            return false;
        }

        croi_status.flash_wipe_progress_percent = ((offset + chunk) * 100U) / LOGGER_FLASH_LENGTH;
    }

    croi_status.flash_wipe_state = 2U;
    croi_status.flash_wipe_progress_percent = 100U;
    croi_status.flash_wipe_address = LOGGER_FLASH_START_ADDRESS + LOGGER_FLASH_LENGTH;

    if (!flash_logger.begin(config)) {
        latch_fault(map_flash_status(flash_logger.status()), flash_logger.status());
        return false;
    }
    return true;
#else
    if (!flash_logger.begin(config)) {
        latch_fault(map_flash_status(flash_logger.status()), flash_logger.status());
        return false;
    }

    return true;
#endif
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
        health_->used_bytes = info.used_bytes;
        mirror_health_to_status();
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
        FlashLogPayloadType::SecondaryFlightData,
        2U);

    if (!ok) {
        latch_fault(map_flash_status(flash_logger.status()), flash_logger.status());
        return false;
    }

    if (health_ != nullptr) {
        const FlashLogInfo info = flash_logger.info();
        health_->flash_status = FlashLogStatus::Ok;
        health_->records_written = info.record_count;
        health_->run_id = info.run_id;
        health_->used_bytes = info.used_bytes;
        mirror_health_to_status();
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
    mirror_health_to_status();
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
    health_->used_bytes = info.used_bytes;
    mirror_health_to_status();
}

void Logger::mirror_health_to_status() const {
    if (health_ == nullptr) {
        return;
    }

    croi_status.flash_init_ok = health_->initialized ? 1U : 0U;
    croi_status.logger_fault_latched = health_->fault_latched ? 1U : 0U;
    croi_status.logger_logging_stopped = health_->logging_stopped ? 1U : 0U;
    croi_status.logger_fault = static_cast<uint32_t>(health_->fault);
    croi_status.logger_flash_status = static_cast<uint32_t>(health_->flash_status);
    croi_status.logger_run_id = health_->run_id;
    croi_status.logger_records_written = health_->records_written;
    croi_status.logger_used_bytes = health_->used_bytes;
    croi_status.logger_stack_free_bytes =
        osThreadGetStackSpace(taskHandle_) * sizeof(StackType_t);
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
