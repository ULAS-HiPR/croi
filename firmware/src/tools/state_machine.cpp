#include "state_machine.h"

StateMachine::StateMachine(const flash_internal_data settings)
    : main_height(settings.main_height),
      drouge_delay(settings.drouge_delay),
      liftoff_threshold(settings.liftoff_thresh)
{
    // Initialize the state machine with function pointer array
    //to add 

    change_state(State::CALIBRATING);
   // current_state = State::CALIBRATING;
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

    printf("State Machine: Ready, accel: %f, threshold: %f \n", fabsf(accel), (static_cast<float>(liftoff_threshold)/9.81));
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
        // printf("State Machine: Drouge, velocity: %f\n", velocity);
        change_state(State::DROUGE);
    }
}

void StateMachine::check_drouge_state_done(float height)
{
    if (height < static_cast<float>(main_height))
    { // add main var
        // printf("State Machine: Main, height: %f\n", height);
        change_state(State::MAIN);
    }
}

void StateMachine::check_main_state_done(float velocity)
{
    if (velocity < 0.0f)
    {
        // printf("State Machine: Landed, velocity: %f\n", velocity);
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

void StateMachine::run(void *pvParameters)
{
    // printf("State Machine Task\n");
    //AllQueuesArgs *args = static_cast<AllQueuesArgs *>(pvParameters);

    //QueueHandle_t coreDataQueue = args->coreDataQueue;
    //QueueHandle_t secDataQueue = args->secDataQueue;
    //QueueHandle_t flightDataQueue = args->flightDataQueue;

    flight_data raw_data;
    flight_data old_data;
    flight_data processed_data;
    printf("State Machine Intialised\n");
    int time = 0;
    float time_diff = 0;

    while (true)
    {
        time = to_ms_since_boot(get_absolute_time());
        time_diff = (time - old_data.core_data.time) / 1000.0f; // convert to seconds
        //printf("Time diff: %f\n", time_diff);
        //if (xQueueReceive(coreDataQueue, &raw_data.core_data, pdMS_TO_TICKS(100)) == pdTRUE)
        //{
            // process data in kalman filter
            // printf("Got raw\n Got alt %f", raw_data.core_data.barometer.altitude);
            //printf(" Got accel %f\n", raw_data.core_data.acceleration.x);
       // }
        if (xQueueReceive(secDataQueue, &raw_data.secondary_data, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            // adds to queue, prob better way to do this
            //printf("secondary data recived\n");
        }
        kalman_filter.predict(time_diff);
        if (raw_data.state > 4)
        {
            //acceleration not relivant after apogee
            raw_data.core_data.acceleration.y = 0.0000f;
        }
        kalman_filter.update(raw_data.core_data.barometer.altitude, (raw_data.core_data.acceleration.y)); // y axis for test data
        kalman_filter.update_values(&raw_data.prediction);
        update_state(raw_data.core_data, raw_data.prediction);
        //printf("State Machine: %d\n", current_state);
        //printf("Prediction: %f %f %f\n", raw_data.prediction.altitude, raw_data.prediction.velocity, raw_data.prediction.acceleration);

        printf("data %d %f %f %f %d\n", time, raw_data.prediction.altitude, raw_data.prediction.velocity, raw_data.prediction.acceleration, current_state);
        raw_data.state = current_state;
        //xQueueSend(flightDataQueue, &raw_data, pdMS_TO_TICKS(100));

        old_data = raw_data;
        vTaskDelay(pdMS_TO_TICKS(read_data_delay));
    }
}

// should take out prediction from core data