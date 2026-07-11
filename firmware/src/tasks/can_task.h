#pragma once
#if F4
#include "stm32f4xx_hal.h"
#include "platform/stm_f4.h"
#endif
#if F0
#include "stm32f0xx_hal.h"
#include "platform/stm_f0.h"
#endif
#include "platform/hal_time.h"
#include "cmsis_os.h"
#include <cstdio>
#include <data.h>

#include <CAN/CAN_Handler.h>
#include <CAN/CAN_Frames.h>
#include <croi_mission_config.h>
#include "../tools/airbrake_logic.h"

#define CAN_DELAY_MS 10
#define CAN_HEARTBEAT_PERIOD_MS 1000U
#define CAN_FLIGHT_TX_MIN_PERIOD_MS 20U
#define CAN_BUS_RECOVERY_PERIOD_MS 250U
#define CAN_NODE_TIMEOUT_MS 5000U
#define CAN_AUTO_RECOVER_BUS_OFF 1
#define CAN_TX_RETRY_QUEUE_LEN 16U
#define CAN_TX_DRAIN_BUDGET_PER_LOOP 3U
#define CAN_MAX_TRACKED_NODES 8U
#define CAN_ACTUATOR_COMMAND_PERIOD_MS 100U
#define CAN_PYRO_ARM_PERIOD_MS 500U
#define CAN_PYRO_FIRE_SETTLE_MS 250U
#define CAN_PYRO_FIRE_RETRY_MS 250U
#define CAN_CRITICAL_TX_QUEUE_LEN 4U

namespace task{
class CAN_task {
    public:
        CAN_task(CAN_Handler& canbus, osMessageQueueId_t sender_queue,
                 osMessageQueueId_t reciver_queue, uint8_t node_id = NODE_CROI) :
            canbus_(canbus),
            sender_queue_(sender_queue),
            reciver_queue_(reciver_queue),
            taskHandle_(nullptr),
            node_id_(node_id) {};
        bool run();

    private:
        void StartCAN();
        static void StartCANEntry(void *argument);
        bool process_rx_frame(const CAN_Frame& frame, secondary_flight_data& shared_data);
        void send_flight_data(const flight_data& data);
        void service_airbrake_command(uint32_t now_ms, secondary_flight_data& shared_data);
        void service_pyro_commands(uint32_t now_ms, secondary_flight_data& shared_data);
        bool send_pyro_arm(uint8_t channel_mask, uint32_t now_ms,
                           secondary_flight_data& shared_data);
        bool send_pyro_fire(uint8_t channel, uint32_t now_ms,
                            secondary_flight_data& shared_data);
        void log_pyro_event(secondary_flight_data& shared_data,
                            uint32_t now_ms,
                            PyroEventAction action,
                            uint8_t channel,
                            uint16_t sequence,
                            uint8_t result,
                            uint8_t fault);
        void send_heartbeat(uint32_t now_ms);
        void service_bus_health(uint32_t now_ms);
        void expire_node_status(uint32_t now_ms);
        void flush_tx_queue();
        bool queue_tx_frame(const CAN_Frame& frame);
        bool queue_critical_frame(const CAN_Frame& frame);
        void record_heartbeat(const HEARTBEAT_Payload& heartbeat, uint32_t now_ms);
        bool send_frame(CAN_Frame& frame);
        bool send_critical_frame(CAN_Frame& frame);
        void update_status(uint32_t now_ms);

        CAN_Handler& canbus_;
        osMessageQueueId_t sender_queue_;
        osMessageQueueId_t reciver_queue_;

        osThreadId_t taskHandle_;
        StaticTask_t task_control_block_{};
        StackType_t task_stack_[1024U / sizeof(StackType_t)]{};
        uint8_t node_id_;
        uint32_t last_heartbeat_ms_{0U};
        uint32_t last_flight_tx_ms_{0U};
        uint32_t last_bus_recovery_ms_{0U};
        uint8_t flight_state_{0U};
        flight_data pending_outbound_data_{};
        bool has_pending_outbound_data_{false};
        AirbrakeLogic airbrake_logic_{
            CROI_MISSION_AIRBRAKE_ENABLED != 0U,
            CROI_MISSION_AIRBRAKE_CHANNEL,
            CROI_MISSION_AIRBRAKE_RETRACTED_ANGLE_DEG,
            CROI_MISSION_AIRBRAKE_MAX_ANGLE_DEG,
            CROI_MISSION_AIRBRAKE_START_DELAY_MS,
            CROI_MISSION_AIRBRAKE_STOW_DELAY_MS};
        uint32_t last_actuator_command_ms_{0U};
        uint32_t last_actuator_log_ms_{0U};
        uint16_t actuator_sequence_{0U};
        AirbrakeCommand previous_airbrake_command_{false, 0U, 0U};
        bool has_previous_airbrake_command_{false};

        uint32_t last_pyro_arm_ms_{0U};
        uint32_t pyro_state_entry_ms_{0U};
        uint16_t pyro_sequence_{0U};
        uint8_t previous_pyro_state_{0xFFU};
        uint8_t pyro_armed_mask_{0U};
        uint8_t pyro_continuity_mask_{0U};
        uint8_t pyro_fired_mask_{0U};
        uint32_t last_drogue_fire_attempt_ms_{0U};
        uint32_t last_main_fire_attempt_ms_{0U};

        CAN_Frame critical_tx_queue_[CAN_CRITICAL_TX_QUEUE_LEN]{};
        uint8_t critical_tx_head_{0U};
        uint8_t critical_tx_tail_{0U};
        uint8_t critical_tx_count_{0U};
        uint32_t critical_tx_drops_{0U};

        CAN_Frame tx_retry_queue_[CAN_TX_RETRY_QUEUE_LEN]{};
        uint8_t tx_retry_head_{0U};
        uint8_t tx_retry_tail_{0U};
        uint8_t tx_retry_count_{0U};
        uint32_t tx_retry_drops_{0U};
        uint32_t node_timeout_count_{0U};

        struct NodeStatus {
            uint8_t node_id{0U};
            uint8_t state{0U};
            uint8_t err{0U};
            uint8_t tx_error_count{0U};
            uint8_t rx_error_count{0U};
            uint8_t tx_queue_depth{0U};
            uint16_t uptime_s{0U};
            uint32_t last_seen_ms{0U};
            uint32_t rx_count{0U};
            bool active{false};
        };

        NodeStatus nodes_[CAN_MAX_TRACKED_NODES]{};

        const osThreadAttr_t task_attributes {
            "CAN",
            0,
            &task_control_block_,
            sizeof(task_control_block_),
            task_stack_,
            sizeof(task_stack_),
            osPriorityNormal,
            0,
            0
        };
    };

}
