#include "state_machine.h"
#include "croi_status.h"

namespace {

constexpr float M_S2_PER_G = 9.80665f;

int32_t scaled_float(float value, float scale) {
    return static_cast<int32_t>(value * scale);
}

void mirror_flight_status(const flight_data& data) {
    croi_status.flight_state = static_cast<uint32_t>(data.state);
    croi_status.baro_pressure_pa = static_cast<uint32_t>(data.core_data.barometer.pressure);
    croi_status.baro_temperature_c_x100 = scaled_float(data.core_data.barometer.temperature, 100.0f);
    croi_status.baro_altitude_m_x100 = scaled_float(data.core_data.barometer.altitude, 100.0f);
    croi_status.prediction_altitude_m_x100 = scaled_float(data.prediction.altitude, 100.0f);
    croi_status.prediction_velocity_m_s_x100 = scaled_float(data.prediction.velocity, 100.0f);
    croi_status.prediction_accel_m_s2_x100 = scaled_float(data.prediction.acceleration, 100.0f);
    croi_status.imu_accel_x_g_x1000 = scaled_float(data.core_data.imu.acceleration.x, 1000.0f);
    croi_status.imu_accel_y_g_x1000 = scaled_float(data.core_data.imu.acceleration.y, 1000.0f);
    croi_status.imu_accel_z_g_x1000 = scaled_float(data.core_data.imu.acceleration.z, 1000.0f);
    croi_status.imu_gyro_x_dps = static_cast<int32_t>(data.core_data.imu.gyro.x);
    croi_status.imu_gyro_y_dps = static_cast<int32_t>(data.core_data.imu.gyro.y);
    croi_status.imu_gyro_z_dps = static_cast<int32_t>(data.core_data.imu.gyro.z);
}

}

namespace task{
StateMachine::StateMachine(IMU *imu, Baro *baro, const flash_internal_data* settings, osMessageQueueId_t can_queue, osMessageQueueId_t logger_queue)
    : imu_(imu), baro_(baro), main_height(settings->main_height_m), drogue_delay_ms(settings->drogue_delay_ms),
      liftoff_threshold_m_s2(static_cast<float>(settings->liftoff_accel_m_s2_x100) / 100.0f),
      phase_logic_(liftoff_threshold_m_s2, static_cast<float>(main_height), drogue_delay_ms),
      can_queue_(can_queue), logger_queue_(logger_queue), taskHandle_(nullptr)
{
    // Initialize the state machine with function pointer array
    //to add 

    change_state(State::CALIBRATING);
   // current_state = State::CALIBRATING;
}

bool StateMachine::run() {
    taskHandle_ = osThreadNew(&StateMachine::StartStateMachineEntry,
                              this,
                              &task_attributes);
    return taskHandle_ != nullptr;
}

void StateMachine::StartStateMachineEntry(void *argument) {
    auto *self = static_cast<StateMachine*>(argument);
    printf("StateMachine starting1\n");
    if (self) {
        self->StartStateMachine();
    }
}

void StateMachine::StartStateMachine() {

    printf("StateMachine started\n");
    uint32_t time = 0U;
    flight_data raw_data{};
    raw_data.state = current_state; 
    imu_data imu_data{};

    for (;;)
    {
        time = HAL_GetTick();
        croi_status.fsm_task_heartbeat_ms = time;
        const bool imu_updated = imu_->update(&imu_data);
        const bool baro_updated = baro_->update(&raw_data.core_data.barometer);
        if (imu_updated) {
            raw_data.core_data.imu = imu_data;
            croi_status.imu_last_ok_ms = time;
        } else {
            ++croi_status.imu_read_failures;
        }
        if (baro_updated) {
            croi_status.baro_last_ok_ms = time;
        } else {
            ++croi_status.baro_read_failures;
        }

        raw_data.core_data.time = time;

        // Never advance filter or flight state from a stale/zero sensor sample.
        if (imu_updated && baro_updated) {
            const float time_diff = last_filter_update_ms_ == 0U
                ? static_cast<float>(FSM_DELAY_MS) / 1000.0f
                : static_cast<float>(time - last_filter_update_ms_) / 1000.0f;
            last_filter_update_ms_ = time;
            kalman_filter_.predict(time_diff);
            kalman_filter_.update(
                raw_data.core_data.barometer.altitude,
                vertical_acceleration_m_s2(raw_data.core_data.imu),
                raw_data.core_data.imu.acceleration.x,
                raw_data.core_data.imu.acceleration.y,
                raw_data.core_data.imu.acceleration.z);
            kalman_filter_.update_values(&raw_data.prediction);
            update_state(raw_data.prediction);
            croi_status.sensor_sample_valid = 1U;
        } else {
            croi_status.sensor_sample_valid = 0U;
            phase_logic_.reset();
        }

        //printf("data %d %f %f %f %d\n", time, raw_data.prediction.altitude, raw_data.prediction.velocity, raw_data.prediction.acceleration, current_state);
        if (raw_data.state != current_state) {
            //printf("State changed at time %d ms: %d -> %d \n", time, raw_data.state, current_state);
        }
        raw_data.state = current_state;
        mirror_flight_status(raw_data);
        croi_status.fsm_stack_free_bytes =
            osThreadGetStackSpace(taskHandle_) * sizeof(StackType_t);
        flight_data snapshot_can = raw_data;
        flight_data snapshot_logger = raw_data;
        if (osMessageQueuePut(can_queue_, &snapshot_can, 0, 0) != osOK) {
            ++croi_status.can_queue_drops;
        }
        if (croi_status.flash_init_ok == 0U) {
            ++croi_status.logger_startup_samples_skipped;
        } else if (osMessageQueuePut(logger_queue_, &snapshot_logger, 0, 0) != osOK) {
            ++croi_status.logger_queue_drops;
        }
        //printf("data %d %f %f %f %d\n", time, raw_data.prediction.altitude, raw_data.prediction.velocity, raw_data.prediction.acceleration, current_state);

        osDelay(FSM_DELAY_MS);
    }
}

void StateMachine::update_state(const prediction_data& prediction)
{
    // printf("State Machine\n");
    // printf("State: %d\n", current_state);
    // printf("Data: %d %d %f %f %f\n", data.time, data.barometer.pressure, data.acceleration.x, data.velocity, data.setting_pin);
    // printf("Velocity: %f\n", data.velocity);
    // printf("Altitude: %f\n", data.barometer.altitude);
    switch (current_state)
    {
    case State::CALIBRATING:
        check_calibrating_state_done();
        break;
    case State::READY:
    case State::POWERED:
    case State::COASTING:
    case State::DROUGE:
    case State::MAIN: {
        const State next = phase_logic_.update(
            current_state,
            prediction.acceleration,
            prediction.velocity,
            prediction.altitude,
            HAL_GetTick() - croi_status.state_entry_ms);
        change_state(next);
        break;
    }
    case State::LANDED:
        printf("landed\n");
        break;
    default:
        break;
    }
    return;
}


void StateMachine::check_calibrating_state_done()
{
    const bool preflight_ready = croi_status.init_ok != 0U &&
                                 croi_status.can_init_ok != 0U &&
                                 croi_status.flash_init_ok != 0U &&
                                 croi_status.logger_fault_latched == 0U &&
                                 croi_status.logger_logging_stopped == 0U;
    if (preflight_ready && kalman_filter_.is_calibrated()) {
        change_state(State::READY);
    }
}

void StateMachine::change_state(State new_state)
{
    if (new_state != current_state)
    {
        // add when need to confim states dont repear
        //if (!called_once[new_state])
        //{
        //    state_handlers[new_state]();
        //    called_once[new_state] = true;
        //}
        current_state = new_state;
        phase_logic_.reset();
        ++croi_status.state_transition_count;
        croi_status.state_entry_ms = HAL_GetTick();
        switch (new_state) {
            case State::READY: kalman_filter_.set_phase_ready(); break;
            case State::POWERED: kalman_filter_.set_phase_thrusting(); break;
            case State::COASTING: kalman_filter_.set_phase_coasting(); break;
            case State::DROUGE: kalman_filter_.set_phase_drogue(); break;
            case State::MAIN: kalman_filter_.set_phase_main(); break;
            default: break;
        }
    }
}

float StateMachine::vertical_acceleration_m_s2(const imu_data& imu)
{
    const float axes[] = {
        imu.acceleration.x,
        imu.acceleration.y,
        imu.acceleration.z,
    };
    return axes[CROI_MISSION_IMU_VERTICAL_AXIS] *
           static_cast<float>(CROI_MISSION_IMU_VERTICAL_SIGN) * M_S2_PER_G;
}

}
