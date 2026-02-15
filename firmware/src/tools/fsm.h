#pragma once

#include <IMU/IMU.h>
#include <Baro/baro.h>
#include "kalman_filter.h"
#include "state_machine.h"
#include <data.h>

struct FSM_TaskArgs {
    IMU* imu;
    Baro* baro;
    KalmanFilter* kalman;
    StateMachine* state_machine;
    flash_internal_data settings;
};

// Forward declaration for C linkage
extern "C" void StartFSM(void* argument);