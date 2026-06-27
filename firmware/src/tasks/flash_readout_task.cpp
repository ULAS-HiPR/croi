#include "flash_readout_task.h"

#ifdef READING

#include "logger.h"
#include <Flash/FlashLogger.h>
#include <cstdarg>
#include <cstdio>
#include <cstring>

extern "C" {
volatile uint32_t g_flash_csv_ready = 0U;
volatile uint32_t g_flash_csv_done = 0U;
volatile uint32_t g_flash_csv_len = 0U;
volatile uint32_t g_flash_csv_error = 0U;
volatile uint32_t g_flash_csv_records = 0U;
volatile uint32_t g_flash_csv_chunk_index = 0U;
volatile uint32_t g_flash_csv_chunk_records = 0U;
volatile uint32_t g_flash_csv_total_len = 0U;
volatile uint32_t g_flash_csv_truncated = 0U;
char g_flash_csv_dump[FLASH_READOUT_CSV_CAPACITY] = {};
}

namespace {

constexpr uint32_t CSV_ERROR_FLASH_INIT = 1U << 0;
constexpr uint32_t CSV_ERROR_LOG_BEGIN = 1U << 1;
constexpr uint32_t CSV_ERROR_READ = 1U << 2;
constexpr uint32_t CSV_ERROR_TRUNCATED = 1U << 3;
constexpr uint32_t CSV_ERROR_ROW_TOO_LARGE = 1U << 4;

constexpr const char* CSV_HEADER =
    "row,run_id,sequence,timestamp_ms,payload_type,payload_version,payload_length,status,"
    "flight_time_ms,state,pred_alt_mm,pred_vel_mms,pred_accel_mg,core_time_ms,"
    "pressure_pa,baro_temp_centi,baro_alt_mm,ax_mg,ay_mg,az_mg,gx,gy,gz,imu_temp,"
    "gps_time,gps_lat_e7,gps_lon_e7,gps_alt_mm,gps_vel_mms,gps_sats,"
    "sec_ax_mg,sec_ay_mg,sec_az_mg\r\n";

constexpr uint32_t max_payload_size() {
    return sizeof(flight_data) > sizeof(secondary_flight_data)
               ? sizeof(flight_data)
               : sizeof(secondary_flight_data);
}

class CsvBuffer {
public:
    CsvBuffer(char* data, uint32_t capacity) : data_(data), capacity_(capacity) {
        reset();
    }

    void reset() {
        size_ = 0U;
        truncated_ = false;
        if (capacity_ > 0U) {
            data_[0] = '\0';
        }
    }

    void append(const char* text) {
        if (text == nullptr) {
            return;
        }

        const uint32_t length = static_cast<uint32_t>(std::strlen(text));
        append_bytes(text, length);
    }

    void appendf(const char* fmt, ...) {
        if ((fmt == nullptr) || full()) {
            return;
        }

        char scratch[192];
        va_list args;
        va_start(args, fmt);
        const int written = std::vsnprintf(scratch, sizeof(scratch), fmt, args);
        va_end(args);

        if (written <= 0) {
            return;
        }

        const uint32_t count = static_cast<uint32_t>(written);
        if (count >= sizeof(scratch)) {
            truncated_ = true;
            append_bytes(scratch, sizeof(scratch) - 1U);
            return;
        }

        append_bytes(scratch, count);
    }

    uint32_t size() const { return size_; }
    bool truncated() const { return truncated_; }
    bool can_fit(uint32_t length) const { return length <= remaining(); }
    uint32_t remaining() const {
        if (full()) {
            return 0U;
        }
        return (capacity_ - 1U) - size_;
    }

    void append_bytes(const char* text, uint32_t length) {
        if (full()) {
            truncated_ = true;
            return;
        }

        const uint32_t available = remaining();
        const uint32_t copied = length < available ? length : available;
        std::memcpy(&data_[size_], text, copied);
        size_ += copied;
        data_[size_] = '\0';

        if (copied != length) {
            truncated_ = true;
        }
    }

private:
    bool full() const {
        return (capacity_ == 0U) || (size_ >= (capacity_ - 1U));
    }

    char* data_;
    uint32_t capacity_;
    uint32_t size_{0U};
    bool truncated_{false};
};

int32_t scaled_float(float value, float scale) {
    return static_cast<int32_t>(value * scale);
}

int32_t scaled_double(double value, double scale) {
    return static_cast<int32_t>(value * scale);
}

void append_empty_columns(CsvBuffer& csv, uint32_t count) {
    for (uint32_t i = 0; i < count; ++i) {
        csv.append(",");
    }
}

const char* payload_type_name(uint16_t payload_type) {
    switch (static_cast<FlashLogPayloadType>(payload_type)) {
        case FlashLogPayloadType::FlightData:
            return "flight";
        case FlashLogPayloadType::SecondaryFlightData:
            return "secondary";
        case FlashLogPayloadType::Unspecified:
            return "unspecified";
        default:
            return "unknown";
    }
}

void append_prefix(CsvBuffer& csv,
                   uint32_t row,
                   const FlashLogRecordHeader& header,
                   const char* status) {
    csv.appendf("%lu,%lu,%lu,%lu,%s,%u,%lu,%s",
                static_cast<unsigned long>(row),
                static_cast<unsigned long>(header.run_id),
                static_cast<unsigned long>(header.sequence),
                static_cast<unsigned long>(header.timestamp_ms),
                payload_type_name(header.payload_type),
                static_cast<unsigned>(header.payload_version),
                static_cast<unsigned long>(header.payload_length),
                status);
}

void append_flight_row(CsvBuffer& csv,
                       uint32_t row,
                       const FlashLogRecordHeader& header,
                       const uint8_t* payload,
                       size_t payload_length) {
    if (payload_length != sizeof(flight_data)) {
        append_prefix(csv, row, header, "bad_size");
        append_empty_columns(csv, 25U);
        csv.append("\r\n");
        return;
    }

    flight_data data{};
    std::memcpy(&data, payload, sizeof(data));

    append_prefix(csv, row, header, "ok");
    csv.appendf(",%lu,%d,%ld,%ld,%ld,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%d,%d,%d,%d",
                static_cast<unsigned long>(data.time),
                data.state,
                static_cast<long>(scaled_float(data.prediction.altitude, 1000.0f)),
                static_cast<long>(scaled_float(data.prediction.velocity, 1000.0f)),
                static_cast<long>(scaled_float(data.prediction.acceleration, 1000.0f)),
                static_cast<unsigned long>(data.core_data.time),
                static_cast<long>(data.core_data.barometer.pressure),
                static_cast<long>(scaled_float(data.core_data.barometer.temperature, 100.0f)),
                static_cast<long>(scaled_float(data.core_data.barometer.altitude, 1000.0f)),
                static_cast<long>(scaled_float(data.core_data.imu.acceleration.x, 1000.0f)),
                static_cast<long>(scaled_float(data.core_data.imu.acceleration.y, 1000.0f)),
                static_cast<long>(scaled_float(data.core_data.imu.acceleration.z, 1000.0f)),
                data.core_data.imu.gyro.x,
                data.core_data.imu.gyro.y,
                data.core_data.imu.gyro.z,
                data.core_data.imu.temperature);
    append_empty_columns(csv, 9U);
    csv.append("\r\n");
}

void append_secondary_row(CsvBuffer& csv,
                          uint32_t row,
                          const FlashLogRecordHeader& header,
                          const uint8_t* payload,
                          size_t payload_length) {
    if (payload_length != sizeof(secondary_flight_data)) {
        append_prefix(csv, row, header, "bad_size");
        append_empty_columns(csv, 25U);
        csv.append("\r\n");
        return;
    }

    secondary_flight_data data{};
    std::memcpy(&data, payload, sizeof(data));

    append_prefix(csv, row, header, "ok");
    append_empty_columns(csv, 16U);
    csv.appendf(",%u,%ld,%ld,%ld,%ld,%u,%ld,%ld,%ld\r\n",
                static_cast<unsigned>(data.time),
                static_cast<long>(scaled_double(data.gps.latitude, 10000000.0)),
                static_cast<long>(scaled_double(data.gps.longitude, 10000000.0)),
                static_cast<long>(scaled_float(data.gps.altitude, 1000.0f)),
                static_cast<long>(scaled_float(data.gps.velocity, 1000.0f)),
                static_cast<unsigned>(data.gps.satellites),
                static_cast<long>(scaled_float(data.acceleration.x, 1000.0f)),
                static_cast<long>(scaled_float(data.acceleration.y, 1000.0f)),
                static_cast<long>(scaled_float(data.acceleration.z, 1000.0f)));
}

void append_unknown_row(CsvBuffer& csv,
                        uint32_t row,
                        const FlashLogRecordHeader& header,
                        const char* status) {
    append_prefix(csv, row, header, status);
    append_empty_columns(csv, 25U);
    csv.append("\r\n");
}

bool final_status_is_error(FlashLogStatus status) {
    return (status != FlashLogStatus::Ok) &&
           (status != FlashLogStatus::NotFound) &&
           (status != FlashLogStatus::Full);
}

void debug_break() {
    __asm volatile ("bkpt #0");
}

uint32_t format_row(char* row_buffer,
                    uint32_t row_capacity,
                    uint32_t row,
                    const FlashLogRecordHeader& header,
                    const uint8_t* payload,
                    size_t payload_length,
                    bool& truncated) {
    CsvBuffer row_csv(row_buffer, row_capacity);

    if (header.payload_type == static_cast<uint16_t>(FlashLogPayloadType::FlightData)) {
        append_flight_row(row_csv, row, header, payload, payload_length);
    } else if (header.payload_type == static_cast<uint16_t>(FlashLogPayloadType::SecondaryFlightData)) {
        append_secondary_row(row_csv, row, header, payload, payload_length);
    } else {
        append_unknown_row(row_csv, row, header, "unknown_payload");
    }

    truncated = row_csv.truncated();
    return row_csv.size();
}

void publish_chunk(CsvBuffer& chunk, bool done) {
    g_flash_csv_len = chunk.size();
    g_flash_csv_done = done ? 1U : 0U;
    g_flash_csv_ready = 1U;
    g_flash_csv_total_len += g_flash_csv_len;
    debug_break();
    g_flash_csv_ready = 0U;
}

} // namespace

namespace task {

void FlashReadoutTask::run() {
    taskHandle_ = osThreadNew(&FlashReadoutTask::entry, this, &task_attributes_);
    if (taskHandle_ == nullptr) {
        g_flash_csv_error |= CSV_ERROR_READ;
        g_flash_csv_done = 1U;
        g_flash_csv_ready = 1U;
    }
}

void FlashReadoutTask::entry(void* argument) {
    auto* self = static_cast<FlashReadoutTask*>(argument);
    if (self != nullptr) {
        self->start();
    }
}

void FlashReadoutTask::start() {
    g_flash_csv_ready = 0U;
    g_flash_csv_done = 0U;
    g_flash_csv_len = 0U;
    g_flash_csv_error = 0U;
    g_flash_csv_records = 0U;
    g_flash_csv_chunk_index = 0U;
    g_flash_csv_chunk_records = 0U;
    g_flash_csv_total_len = 0U;
    g_flash_csv_truncated = 0U;
    std::memset(g_flash_csv_dump, 0, FLASH_READOUT_CSV_CAPACITY);

    CsvBuffer chunk(g_flash_csv_dump, FLASH_READOUT_CSV_CAPACITY);

    if (!storage_.init()) {
        chunk.append(CSV_HEADER);
        g_flash_csv_error |= CSV_ERROR_FLASH_INIT;
        publish_chunk(chunk, true);
        for (;;) {
            osDelay(1000U);
        }
    }

    FlashLogger logger(storage_);
    FlashLogConfig config = FlashLogger::default_mx25_config(
        LOGGER_FLASH_START_ADDRESS,
        LOGGER_FLASH_LENGTH);
    config.max_payload_size = max_payload_size();
    config.verify_writes = false;

    const bool begin_ok = logger.begin(config, 0U);
    const FlashLogStatus begin_status = logger.status();
    if (!begin_ok && final_status_is_error(begin_status)) {
        g_flash_csv_error |= CSV_ERROR_LOG_BEGIN;
    }

    FlashLogCursor cursor = logger.cursor();
    FlashLogRecordHeader header{};
    uint8_t payload[max_payload_size()] = {};
    size_t payload_length = 0U;
    uint32_t row = 0U;
    bool header_pending = true;
    bool done = false;

    while (!done) {
        chunk.reset();
        g_flash_csv_chunk_records = 0U;

        if (header_pending) {
            chunk.append(CSV_HEADER);
            header_pending = false;
        }

        while (chunk.remaining() > 0U) {
            const FlashLogCursor cursor_before = cursor;
            if (!logger.read_next(cursor, header, payload, sizeof(payload), &payload_length)) {
                const FlashLogStatus read_status = logger.status();
                if (read_status != FlashLogStatus::NotFound) {
                    g_flash_csv_error |= CSV_ERROR_READ;
                }
                done = true;
                break;
            }

            char row_buffer[FLASH_READOUT_ROW_CAPACITY] = {};
            bool row_truncated = false;
            const uint32_t next_row = row + 1U;
            const uint32_t row_len = format_row(row_buffer,
                                                sizeof(row_buffer),
                                                next_row,
                                                header,
                                                payload,
                                                payload_length,
                                                row_truncated);
            if (row_truncated) {
                g_flash_csv_error |= CSV_ERROR_ROW_TOO_LARGE;
                g_flash_csv_truncated = 1U;
            }

            if (!chunk.can_fit(row_len)) {
                if (g_flash_csv_chunk_records == 0U) {
                    chunk.append_bytes(row_buffer, row_len);
                    row = next_row;
                    ++g_flash_csv_chunk_records;
                    g_flash_csv_records = row;
                    if (chunk.truncated()) {
                        g_flash_csv_error |= CSV_ERROR_TRUNCATED;
                        g_flash_csv_truncated = 1U;
                    }
                } else {
                    cursor = cursor_before;
                }
                break;
            }

            chunk.append_bytes(row_buffer, row_len);
            row = next_row;
            ++g_flash_csv_chunk_records;
            g_flash_csv_records = row;
        }

        if (chunk.truncated()) {
            g_flash_csv_error |= CSV_ERROR_TRUNCATED;
            g_flash_csv_truncated = 1U;
        }

        publish_chunk(chunk, done);
        ++g_flash_csv_chunk_index;
    }

    g_flash_csv_done = 1U;
    g_flash_csv_ready = 1U;

    for (;;) {
        osDelay(1000U);
    }
}

} // namespace task

#else

namespace task {

void FlashReadoutTask::run() {}

} // namespace task

#endif
