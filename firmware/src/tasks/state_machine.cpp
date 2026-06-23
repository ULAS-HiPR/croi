#include "state_machine.h"

namespace task{
StateMachine::StateMachine(IMU *imu, Baro *baro, const flash_internal_data* settings, osMessageQueueId_t can_queue, osMessageQueueId_t logger_queue)
    : imu_(imu), baro_(baro), main_height(settings->main_height), drouge_delay(settings->drouge_delay),
      liftoff_threshold(settings->liftoff_thresh), can_queue_(can_queue), logger_queue_(logger_queue), taskHandle_(nullptr)
{
    // Initialize the state machine with function pointer array
    //to add 

    change_state(State::CALIBRATING);
   // current_state = State::CALIBRATING;
}

void StateMachine::run() {
    taskHandle_ = osThreadNew(&StateMachine::StartStateMachineEntry,
                              this,
                              &task_attributes);
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
    int time = 0;
    float time_diff = 0;
    flight_data raw_data{};
    flight_data old_data{};
    raw_data.state = current_state; 
    imu_data imu_data{};

    for (;;)
    {
        time = HAL_GetTick();
        time_diff = (time - old_data.time) / 1000.0f;
        if (imu_->update(&imu_data)){
           raw_data.core_data.imu.acceleration = imu_data.acceleration;
            printf("Got IMU\n");
        }
        if (baro_->update(&raw_data.core_data.barometer)){
            printf("Got Baro\n");
        }

        raw_data.core_data.time = time;

        kalman_filter_->predict(time_diff);
        if (raw_data.state > 4)
        {
            //acceleration not relivant after apogee
            raw_data.core_data.imu.acceleration.y = 0.0000f;
        }
        kalman_filter_->update(raw_data.core_data.barometer.altitude, raw_data.core_data.imu.acceleration.x, raw_data.core_data.imu.acceleration.y,raw_data.core_data.imu.acceleration.z);
        kalman_filter_->update_values(&raw_data.prediction);
        update_state(raw_data.core_data, raw_data.prediction);

        //printf("data %d %f %f %f %d\n", time, raw_data.prediction.altitude, raw_data.prediction.velocity, raw_data.prediction.acceleration, current_state);
        if (raw_data.state != current_state) {
            //printf("State changed at time %d ms: %d -> %d \n", time, raw_data.state, current_state);
        }
        raw_data.state = current_state;
        flight_data snapshot_can = raw_data;
        flight_data snapshot_logger = raw_data;
        osMessageQueuePut(can_queue_, &snapshot_can, 0, 0);
        osMessageQueuePut(logger_queue_, &snapshot_logger, 0, 0);
        old_data = raw_data;
        //printf("data %d %f %f %f %d\n", time, raw_data.prediction.altitude, raw_data.prediction.velocity, raw_data.prediction.acceleration, current_state);

        osDelay(FSM_DELAY_MS);
    }
}

void StateMachine::update_state(const core_flight_data raw_data, prediction_data prediction)
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
        check_ready_state_done(prediction.acceleration);
        break;
    case State::POWERED:
        check_powered_state_done(prediction.acceleration);
        break;
    case State::COASTING:
        check_coasting_state_done(prediction.velocity);
        break;
    case State::DROUGE:
        check_drouge_state_done(prediction.altitude);
        break;
    case State::MAIN:
        check_drouge_state_done(prediction.altitude);
        break;
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
    // printf("State Machine\n");
    change_state(State::READY);
    return;
}

void StateMachine::check_ready_state_done(float accel)
{
    //float accel = (accel_x * accel_x) + (accel_y * accel_y) + (accel_z * accel_z);

    //printf("State Machine: Ready, accel: %f, threshold: %f \n", fabsf(accel), (static_cast<float>(liftoff_threshold)/9.81));
    if (fabsf(accel) > (static_cast<float>(liftoff_threshold)/9.81f))
    {
        change_state(State::POWERED);
    }
}

void StateMachine::check_powered_state_done(float accel)
{
    //float accel = (accel_x * accel_x) + (accel_y * accel_y) + (accel_z * accel_z);
    if (accel < 0.0f)
    {
        printf("State Machine: Coasting, accel_x: %f\n", accel);
        change_state(State::COASTING);
    }
}

void StateMachine::check_coasting_state_done(float velocity)
{
    if (velocity < 0.0f)
    {
        printf("State Machine: Drouge, velocity: %f\n", velocity);
        change_state(State::DROUGE);
    }
}

void StateMachine::check_drouge_state_done(float height)
{
    if (height < static_cast<float>(main_height))
    { // add main var
        //printf("State Machine: Main, height: %f\n", height);
        change_state(State::MAIN);
    }
}

void StateMachine::check_main_state_done(float velocity)
{
    if (velocity < 0.0f)
    {
        printf("State Machine: Landed, velocity: %f\n", velocity);
        change_state(State::LANDED);
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
    }
}

}
