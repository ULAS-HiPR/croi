#pragma once

#include <cstdint>

constexpr uint64_t logging_capacity_bytes(
    uint32_t sample_period_ms,
    uint32_t minimum_flight_ms,
    uint32_t post_landing_ms,
    uint32_t flight_record_bytes,
    uint32_t remote_record_bytes,
    bool include_remote) {
    if (sample_period_ms == 0U) {
        return UINT64_MAX;
    }
    const uint64_t duration_ms =
        static_cast<uint64_t>(minimum_flight_ms) + post_landing_ms;
    const uint64_t samples =
        (duration_ms + sample_period_ms - 1U) / sample_period_ms;
    const uint64_t bytes_per_sample =
        static_cast<uint64_t>(flight_record_bytes) +
        (include_remote ? remote_record_bytes : 0U);
    return samples * bytes_per_sample;
}
