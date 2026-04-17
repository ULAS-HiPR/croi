#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H
#include <data.h>
#include "../tools/kalman_filter.h"
#include <stdio.h>

namespace task{
class StateMachine {
    public:
        StateMachine(flash_internal_data settings, osMessageQueueId_t can_queue, osMessageQueueId_t logger_queue)
            : can_queue_(can_queue), logger_queue_(logger_queue), taskHandle_(nullptr){};
              
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
        State current_state;

    private:
        void StartStateMachine();
        static void StartStateMachineEntry(void *argument);
    
        void check_calibrating_state_done();
        void check_ready_state_done(float accel);
        void check_powered_state_done(float accel);
        void check_coasting_state_done(float velocity);
        void check_drouge_state_done(float height);
        void check_main_state_done(float height);
        void change_state(State new_state);

        //hard coded low limit values in case settings are not set
        int main_height{200};
        int drouge_delay{0};
        int liftoff_threshold{20};
        KalmanFilter kalman_filter_ = KalmanFilter();
};
}

#endif // STATE_MACHINE_H