#include <unity.h>
#include <tools/kalman_filter.h>
#include <tools/flight_phase_logic.h>
#include <tools/airbrake_logic.h>
#include <tools/logging_window.h>
#include <tools/logging_capacity.h>
#include <CAN/CAN_Frames.h>
void test_kalman_requires_valid_sample_count_for_calibration() {
    KalmanFilter filter;
    TEST_ASSERT_FALSE(filter.is_calibrated());

    for (int sample = 0; sample < 49; ++sample) {
        filter.update(0.0f, 9.80665f, 0.0f, 1.0f, 0.0f);
    }
    TEST_ASSERT_FALSE(filter.is_calibrated());

    filter.update(0.0f, 9.80665f, 0.0f, 1.0f, 0.0f);
    TEST_ASSERT_TRUE(filter.is_calibrated());
}

void test_kalman_publishes_finite_prediction() {
    KalmanFilter filter;
    prediction_data prediction{};

    for (int sample = 0; sample < 50; ++sample) {
        filter.predict(0.1f);
        filter.update(120.0f, 9.80665f, 0.0f, 1.0f, 0.0f);
    }
    filter.update_values(&prediction);

    TEST_ASSERT_TRUE(prediction.altitude > 0.0f);
    TEST_ASSERT_TRUE(prediction.altitude < 200.0f);
}

void test_kalman_rejects_moving_calibration_window() {
    KalmanFilter filter;
    for (int sample = 0; sample < 50; ++sample) {
        const float accel = (sample % 2 == 0) ? 0.2f : 1.8f;
        filter.update(0.0f, accel * 9.80665f, 0.0f, accel, 0.0f);
    }
    TEST_ASSERT_FALSE(filter.is_calibrated());
}

void test_phase_logic_requires_consecutive_samples() {
    FlightPhaseLogic logic(20.0f, 200.0f, 0U);
    TEST_ASSERT_EQUAL(State::READY, logic.update(State::READY, 25.0f, 0.0f, 0.0f, 0U));
    TEST_ASSERT_EQUAL(State::READY, logic.update(State::READY, 0.0f, 0.0f, 0.0f, 0U));
    TEST_ASSERT_EQUAL(State::READY, logic.update(State::READY, 25.0f, 0.0f, 0.0f, 0U));
    TEST_ASSERT_EQUAL(State::READY, logic.update(State::READY, 25.0f, 0.0f, 0.0f, 0U));
    TEST_ASSERT_EQUAL(State::POWERED, logic.update(State::READY, 25.0f, 0.0f, 0.0f, 0U));
}

void test_phase_logic_rejects_wrong_direction_liftoff_shock() {
    FlightPhaseLogic logic(20.0f, 200.0f, 0U);
    for (int sample = 0; sample < 10; ++sample) {
        TEST_ASSERT_EQUAL(
            State::READY,
            logic.update(State::READY, -25.0f, 0.0f, 0.0f, 0U));
    }
}

void test_phase_logic_honors_drogue_delay_and_main_altitude() {
    FlightPhaseLogic logic(20.0f, 200.0f, 1500U);
    TEST_ASSERT_EQUAL(State::DROUGE, logic.update(State::DROUGE, 0.0f, -5.0f, 150.0f, 1499U));
    TEST_ASSERT_EQUAL(State::DROUGE, logic.update(State::DROUGE, 0.0f, -5.0f, 150.0f, 1500U));
    TEST_ASSERT_EQUAL(State::DROUGE, logic.update(State::DROUGE, 0.0f, -5.0f, 150.0f, 1600U));
    TEST_ASSERT_EQUAL(State::MAIN, logic.update(State::DROUGE, 0.0f, -5.0f, 150.0f, 1700U));
}

void test_main_backup_requires_every_guard_consecutively() {
    FlightPhaseLogic logic(
        20.0f,
        200.0f,
        0U,
        MainRecoveryFallback{true, 5000U, 30.0f, 100.0f, 2000.0f, 5U});

    for (int sample = 0; sample < 8; ++sample) {
        TEST_ASSERT_EQUAL(
            State::DROUGE,
            logic.update(State::DROUGE, 0.0f, -40.0f, 1000.0f, 4999U));
    }
    for (int sample = 0; sample < 4; ++sample) {
        TEST_ASSERT_EQUAL(
            State::DROUGE,
            logic.update(State::DROUGE, 0.0f, -40.0f, 1000.0f, 5000U));
    }
    TEST_ASSERT_EQUAL(
        State::DROUGE,
        logic.update(State::DROUGE, 0.0f, -10.0f, 1000.0f, 5500U));
    for (int sample = 0; sample < 4; ++sample) {
        TEST_ASSERT_EQUAL(
            State::DROUGE,
            logic.update(State::DROUGE, 0.0f, -40.0f, 1000.0f, 5600U));
    }
    TEST_ASSERT_EQUAL(
        State::MAIN,
        logic.update(State::DROUGE, 0.0f, -40.0f, 1000.0f, 6000U));
    TEST_ASSERT_TRUE(logic.main_backup_triggered());
}

void test_main_backup_rejects_altitude_outside_window() {
    FlightPhaseLogic logic(
        20.0f,
        200.0f,
        0U,
        MainRecoveryFallback{true, 5000U, 30.0f, 100.0f, 2000.0f, 3U});

    for (int sample = 0; sample < 10; ++sample) {
        TEST_ASSERT_EQUAL(
            State::DROUGE,
            logic.update(State::DROUGE, 0.0f, -80.0f, 2500.0f, 6000U));
    }
    TEST_ASSERT_FALSE(logic.main_backup_triggered());
}

void test_phase_logic_requires_five_seconds_of_landed_samples() {
    FlightPhaseLogic logic(20.0f, 200.0f, 0U);
    for (int sample = 0; sample < 49; ++sample) {
        TEST_ASSERT_EQUAL(State::MAIN, logic.update(State::MAIN, 0.0f, 0.5f, 0.0f, 0U));
    }
    TEST_ASSERT_EQUAL(State::LANDED, logic.update(State::MAIN, 0.0f, 0.5f, 0.0f, 0U));
}

void test_airbrake_logic_is_fail_closed_when_disabled() {
    AirbrakeLogic logic(false, 2U, 5U, 80U, 0U, 1000U);
    const AirbrakeCommand command = logic.update(State::COASTING, 1000U);
    TEST_ASSERT_FALSE(command.active);
    TEST_ASSERT_EQUAL_UINT8(5U, command.angle_deg);
}

void test_airbrake_logic_renews_retracted_then_deploys() {
    AirbrakeLogic logic(true, 2U, 5U, 80U, 1500U, 2500U);
    AirbrakeCommand command = logic.update(State::READY, 900U);
    TEST_ASSERT_TRUE(command.active);
    TEST_ASSERT_EQUAL_UINT8(5U, command.angle_deg);

    command = logic.update(State::POWERED, 1000U);
    TEST_ASSERT_TRUE(command.active);
    TEST_ASSERT_EQUAL_UINT8(5U, command.angle_deg);
    command = logic.update(State::POWERED, 2499U);
    TEST_ASSERT_EQUAL_UINT8(5U, command.angle_deg);
    command = logic.update(State::COASTING, 2500U);
    TEST_ASSERT_EQUAL_UINT8(80U, command.angle_deg);
    command = logic.update(State::COASTING, 3499U);
    TEST_ASSERT_EQUAL_UINT8(80U, command.angle_deg);
    command = logic.update(State::COASTING, 3500U);
    TEST_ASSERT_TRUE(command.active);
    TEST_ASSERT_EQUAL_UINT8(5U, command.angle_deg);
    command = logic.update(State::DROUGE, 4000U);
    TEST_ASSERT_FALSE(command.active);
    TEST_ASSERT_EQUAL_UINT8(5U, command.angle_deg);
}

void test_logging_window_does_not_consume_flash_before_liftoff() {
    LoggingWindow window(60000U);
    TEST_ASSERT_FALSE(window.active(State::CALIBRATING, 0U));
    TEST_ASSERT_FALSE(window.active(State::READY, 3600000U));
    TEST_ASSERT_FALSE(window.active(State::LANDED, 3600100U));
}

void test_logging_window_covers_flight_and_post_landing_period() {
    LoggingWindow window(60000U);
    TEST_ASSERT_TRUE(window.active(State::POWERED, 1000U));
    TEST_ASSERT_TRUE(window.active(State::COASTING, 2000U));
    TEST_ASSERT_TRUE(window.active(State::DROUGE, 3000U));
    TEST_ASSERT_TRUE(window.active(State::MAIN, 4000U));
    TEST_ASSERT_TRUE(window.active(State::LANDED, 5000U));
    TEST_ASSERT_TRUE(window.active(State::LANDED, 65000U));
    TEST_ASSERT_FALSE(window.active(State::LANDED, 65001U));
}

void test_logging_window_post_landing_timer_handles_tick_wrap() {
    LoggingWindow window(100U);
    TEST_ASSERT_TRUE(window.active(State::POWERED, 0xFFFFFF00U));
    TEST_ASSERT_TRUE(window.active(State::LANDED, 0xFFFFFFF0U));
    TEST_ASSERT_TRUE(window.active(State::LANDED, 0x00000054U));
    TEST_ASSERT_FALSE(window.active(State::LANDED, 0x00000055U));
}

void test_logging_capacity_budgets_flight_and_remote_records() {
    TEST_ASSERT_EQUAL_UINT64(
        2570400U,
        logging_capacity_bytes(100U, 1200000U, 60000U, 100U, 104U, true));
    TEST_ASSERT_EQUAL_UINT64(
        1260000U,
        logging_capacity_bytes(100U, 1200000U, 60000U, 100U, 104U, false));
}

void test_pyro_command_tag_binds_every_field() {
    const uint16_t base = pyro_command_tag(PYRO_COMMAND_FIRE_DROGUE, 1U, 42U, 0x1234U);
    TEST_ASSERT_NOT_EQUAL(base, pyro_command_tag(PYRO_COMMAND_ARM, 1U, 42U, 0x1234U));
    TEST_ASSERT_NOT_EQUAL(base, pyro_command_tag(PYRO_COMMAND_FIRE_DROGUE, 2U, 42U, 0x1234U));
    TEST_ASSERT_NOT_EQUAL(base, pyro_command_tag(PYRO_COMMAND_FIRE_DROGUE, 1U, 43U, 0x1234U));
    TEST_ASSERT_NOT_EQUAL(base, pyro_command_tag(PYRO_COMMAND_FIRE_DROGUE, 1U, 42U, 0x1235U));
    TEST_ASSERT_NOT_EQUAL(base, pyro_command_tag(PYRO_COMMAND_FIRE_MAIN, 1U, 42U, 0x1234U));
}

void test_pyro_sequence_freshness_handles_wrap() {
    TEST_ASSERT_TRUE(pyro_sequence_newer(11U, 10U));
    TEST_ASSERT_FALSE(pyro_sequence_newer(10U, 10U));
    TEST_ASSERT_FALSE(pyro_sequence_newer(9U, 10U));
    TEST_ASSERT_TRUE(pyro_sequence_newer(1U, 0xFFFFU));
    TEST_ASSERT_FALSE(pyro_sequence_newer(0U, 0xFFFFU));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_kalman_requires_valid_sample_count_for_calibration);
    RUN_TEST(test_kalman_publishes_finite_prediction);
    RUN_TEST(test_kalman_rejects_moving_calibration_window);
    RUN_TEST(test_phase_logic_requires_consecutive_samples);
    RUN_TEST(test_phase_logic_rejects_wrong_direction_liftoff_shock);
    RUN_TEST(test_phase_logic_honors_drogue_delay_and_main_altitude);
    RUN_TEST(test_main_backup_requires_every_guard_consecutively);
    RUN_TEST(test_main_backup_rejects_altitude_outside_window);
    RUN_TEST(test_phase_logic_requires_five_seconds_of_landed_samples);
    RUN_TEST(test_airbrake_logic_is_fail_closed_when_disabled);
    RUN_TEST(test_airbrake_logic_renews_retracted_then_deploys);
    RUN_TEST(test_logging_window_does_not_consume_flash_before_liftoff);
    RUN_TEST(test_logging_window_covers_flight_and_post_landing_period);
    RUN_TEST(test_logging_window_post_landing_timer_handles_tick_wrap);
    RUN_TEST(test_logging_capacity_budgets_flight_and_remote_records);
    RUN_TEST(test_pyro_command_tag_binds_every_field);
    RUN_TEST(test_pyro_sequence_freshness_handles_wrap);
    return UNITY_END();
}
