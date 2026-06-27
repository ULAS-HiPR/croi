#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H
#include <data.h>
#include <IMU/IMU.h>
#include <Baro/baro.h>
#include "../tools/kalman_filter.h"
#include <stdio.h>
#include "cmsis_os.h"
#if F4
#include "stm32f4xx_hal.h"
#include "platform/stm_f4.h"
#endif
#if F0
#include "stm32f0xx_hal.h"
#include "platform/stm_f0.h"
#endif

#define FSM_DELAY_MS 100

struct LoggerHealth;

namespace task{
class StateMachine {
    public:
        StateMachine(IMU *imu, Baro *baro, const flash_internal_data* settings,
                     osMessageQueueId_t can_queue,
                     osMessageQueueId_t logger_queue,
                     const LoggerHealth* logger_health = nullptr);
              
        //should put this in data.h
        enum State {
            CALIBRATING,
            READY,
            POWERED,
            COASTING,
            DROUGE,
            MAIN,
            LANDED,
        };
        void run();
        State current_state{State::CALIBRATING};

    private:
        void StartStateMachine();
        static void StartStateMachineEntry(void *argument);
    
        void update_state(const core_flight_data raw_data, prediction_data prediction);
        void check_calibrating_state_done();
        void check_ready_state_done(float accel);
        void check_powered_state_done(float accel);
        void check_coasting_state_done(float velocity);
        void check_drouge_state_done(float height);
        void check_main_state_done(float height);
        void change_state(State new_state);
        bool logging_preflight_ok() const;
        
        IMU *imu_;
        Baro *baro_;

        //hard coded low limit values in case settings are not set
        int main_height{200};
        int drouge_delay{0};
        int liftoff_threshold{20};
        KalmanFilter *kalman_filter_ = new KalmanFilter();
        

        osMessageQueueId_t can_queue_;
        osMessageQueueId_t logger_queue_;
        const LoggerHealth* logger_health_;
        osThreadId_t taskHandle_;
        

        const osThreadAttr_t task_attributes {
            "FSM",
            0,
            nullptr,
            0,
            nullptr,
            2048 ,        // 2 KB stack
            osPriorityHigh,
            0,
            0
        };
};
}

#endif // STATE_MACHINE_H
