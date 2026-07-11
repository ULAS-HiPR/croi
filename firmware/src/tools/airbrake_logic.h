#pragma once

#include <cstdint>

#include <data.h>

struct AirbrakeCommand {
    bool active;
    uint8_t output_index;
    uint8_t angle_deg;
};

class AirbrakeLogic {
public:
    AirbrakeLogic(bool enabled,
                  uint8_t output_index,
                  uint8_t retracted_angle_deg,
                  uint8_t maximum_angle_deg,
                  uint32_t start_delay_ms,
                  uint32_t stow_delay_ms)
        : enabled_(enabled),
          output_index_(output_index),
          retracted_angle_deg_(retracted_angle_deg),
          maximum_angle_deg_(maximum_angle_deg),
          start_delay_ms_(start_delay_ms),
          stow_delay_ms_(stow_delay_ms) {}

    AirbrakeCommand update(State state, uint32_t now_ms) {
        if (state == State::READY || state == State::CALIBRATING) {
            flight_started_ = false;
        } else if ((state == State::POWERED || state == State::COASTING) && !flight_started_) {
            flight_started_ = true;
            flight_start_ms_ = now_ms;
        }

        const bool controllable_state =
            state == State::READY || state == State::POWERED || state == State::COASTING;
        const bool deployment_state = state == State::POWERED || state == State::COASTING;
        const uint32_t elapsed_ms = now_ms - flight_start_ms_;
        const bool deploy = enabled_ && deployment_state && flight_started_ &&
            elapsed_ms >= start_delay_ms_ && elapsed_ms < stow_delay_ms_;

        return AirbrakeCommand{
            enabled_ && controllable_state,
            output_index_,
            deploy ? maximum_angle_deg_ : retracted_angle_deg_,
        };
    }

private:
    bool enabled_;
    uint8_t output_index_;
    uint8_t retracted_angle_deg_;
    uint8_t maximum_angle_deg_;
    uint32_t start_delay_ms_;
    uint32_t stow_delay_ms_;
    bool flight_started_{false};
    uint32_t flight_start_ms_{0U};
};
