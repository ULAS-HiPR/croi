#pragma once

#include <cmath>
#include <cstdint>

#include <data.h>

static_assert(State::CALIBRATING == 0 && State::READY == 1 &&
              State::POWERED == 2 && State::COASTING == 3 &&
              State::DROUGE == 4 && State::MAIN == 5 &&
              State::LANDED == 6,
              "Flight-state wire values changed");

class FlightPhaseLogic {
public:
    FlightPhaseLogic(float liftoff_threshold_m_s2,
                     float main_deploy_altitude_m,
                     uint32_t drogue_delay_ms)
        : liftoff_threshold_m_s2_(liftoff_threshold_m_s2),
          main_deploy_altitude_m_(main_deploy_altitude_m),
          drogue_delay_ms_(drogue_delay_ms) {}

    State update(State state,
                 float acceleration_m_s2,
                 float velocity_m_s,
                 float altitude_m,
                 uint32_t state_elapsed_ms) {
        bool condition = false;
        uint16_t required_samples = kTransitionConfirmSamples;

        switch (state) {
            case State::READY:
                condition = acceleration_m_s2 > liftoff_threshold_m_s2_;
                break;
            case State::POWERED:
                condition = acceleration_m_s2 < 0.0f;
                break;
            case State::COASTING:
                condition = velocity_m_s < 0.0f;
                break;
            case State::DROUGE:
                condition = state_elapsed_ms >= drogue_delay_ms_ &&
                            altitude_m < main_deploy_altitude_m_;
                break;
            case State::MAIN:
                condition = std::fabs(velocity_m_s) <= kLandedMaxSpeedMps &&
                            std::fabs(acceleration_m_s2) <= kLandedMaxAccelMps2;
                required_samples = kLandedConfirmSamples;
                break;
            default:
                reset();
                return state;
        }

        consecutive_samples_ = condition
            ? static_cast<uint16_t>(consecutive_samples_ + 1U)
            : 0U;
        if (consecutive_samples_ < required_samples) {
            return state;
        }

        reset();
        switch (state) {
            case State::READY: return State::POWERED;
            case State::POWERED: return State::COASTING;
            case State::COASTING: return State::DROUGE;
            case State::DROUGE: return State::MAIN;
            case State::MAIN: return State::LANDED;
            default: return state;
        }
    }

    void reset() { consecutive_samples_ = 0U; }

private:
    static constexpr uint16_t kTransitionConfirmSamples = 3U;
    static constexpr uint16_t kLandedConfirmSamples = 50U;
    static constexpr float kLandedMaxSpeedMps = 1.0f;
    static constexpr float kLandedMaxAccelMps2 = 2.5f;

    float liftoff_threshold_m_s2_;
    float main_deploy_altitude_m_;
    uint32_t drogue_delay_ms_;
    uint16_t consecutive_samples_{0U};
};
