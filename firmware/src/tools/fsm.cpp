// src/fsm.cpp
#include "fsm.h"

// Fake HAL / FreeRTOS symbols for SIL will be linked externally
extern "C" uint32_t HAL_GetTick();
extern "C" void osDelay(uint32_t ms);

// Forward declaration of flash_internal_data if needed
// #include "data.h"


// Hardware-independent StartFSM
extern "C" void StartFSM(void* argument)
{
    auto* args = static_cast<FSM_TaskArgs*>(argument);

    IMU* imu = args->imu;
    Baro* baro = args->baro;
    KalmanFilter* kalman_filter = args->kalman;
    StateMachine* state_machine = args->state_machine;

    flight_data raw_data;
    flight_data old_data;
    flight_data processed_data;
    imu_data imu_data;

    uint32_t time = HAL_GetTick();
    float time_diff = 0.0f;

    for (;;)
    {
        time = HAL_GetTick();
        time_diff = (time - old_data.time) / 1000.0f;
        if (imu->update(&imu_data)){
            raw_data.core_data.acceleration = imu_data.acceleration;
            //printf("Got IMU\n");
        }
        if (baro->update(&raw_data.core_data.barometer)){
            //printf("Got Baro\n");
        }

        kalman_filter->predict(time_diff);
        if (raw_data.state > 4)
        {
            //acceleration not relivant after apogee
            raw_data.core_data.acceleration.y = 0.0000f;
        }
        kalman_filter->update(raw_data.core_data.barometer.altitude, (raw_data.core_data.acceleration.y)); // y axis for test data
        kalman_filter->update_values(&raw_data.prediction);
        state_machine->update_state(raw_data.core_data, raw_data.prediction);

        printf("data %u %f %f %f %d\n", time, raw_data.prediction.altitude, raw_data.prediction.velocity, raw_data.prediction.acceleration, state_machine->current_state);
        if (raw_data.state != state_machine->current_state) {
            printf("State changed at time %u ms: %d -> %d\n", time, raw_data.state, state_machine->current_state);
        }
        raw_data.state = state_machine->current_state;
        old_data = raw_data;


        // Delay (fake or real)
        osDelay(1000);
    }
}