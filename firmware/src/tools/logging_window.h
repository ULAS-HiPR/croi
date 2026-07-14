#pragma once

#include <cstdint>

#include <data.h>

class LoggingWindow {
public:
    explicit LoggingWindow(uint32_t post_landing_ms)
        : post_landing_ms_(post_landing_ms) {}

    bool active(State state, uint32_t now_ms) {
        switch (state) {
            case State::POWERED:
            case State::COASTING:
            case State::DROUGE:
            case State::MAIN:
                flight_started_ = true;
                return true;

            case State::LANDED:
                if (!flight_started_) {
                    return false;
                }
                if (!landing_seen_) {
                    landing_seen_ = true;
                    landed_at_ms_ = now_ms;
                }
                return static_cast<uint32_t>(now_ms - landed_at_ms_) <= post_landing_ms_;

            case State::CALIBRATING:
            case State::READY:
            default:
                return false;
        }
    }

private:
    uint32_t post_landing_ms_;
    uint32_t landed_at_ms_{0U};
    bool flight_started_{false};
    bool landing_seen_{false};
};
