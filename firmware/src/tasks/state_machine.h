#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H
#include <data.h>
#include "croi_mission_config.h"
#include <IMU/IMU.h>
#include <Baro/baro.h>
#include "../tools/kalman_filter.h"
#include "../tools/flight_phase_logic.h"
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

#define FSM_DELAY_MS CROI_LOGGING_FLIGHT_SAMPLE_PERIOD_MS

namespace task{
class StateMachine {
    public:
        StateMachine(IMU *imu, Baro *baro, const flash_internal_data* settings, osMessageQueueId_t can_queue, osMessageQueueId_t logger_queue);
              
        bool run();
        State current_state{CALIBRATING};

    private:
        void StartStateMachine();
        static void StartStateMachineEntry(void *argument);
    
        void update_state(const prediction_data& prediction);
        void check_calibrating_state_done();
        void change_state(State new_state);
        static float vertical_acceleration_m_s2(const imu_data& imu);
        
        IMU *imu_;
        Baro *baro_;

        int main_height{200};
        uint32_t drogue_delay_ms{0U};
        float liftoff_threshold_m_s2{20.0f};
        KalmanFilter kalman_filter_{};
        FlightPhaseLogic phase_logic_;
        uint32_t last_filter_update_ms_{0U};
        

        osMessageQueueId_t can_queue_;
        osMessageQueueId_t logger_queue_;
        osThreadId_t taskHandle_;
        StaticTask_t task_control_block_{};
        StackType_t task_stack_[2048U / sizeof(StackType_t)]{};

        const osThreadAttr_t task_attributes {
            "FSM",
            0,
            &task_control_block_,
            sizeof(task_control_block_),
            task_stack_,
            sizeof(task_stack_),
            osPriorityHigh,
            0,
            0
        };
};
}

#endif // STATE_MACHINE_H
