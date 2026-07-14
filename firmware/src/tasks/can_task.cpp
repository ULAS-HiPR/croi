#include "can_task.h"
#include "croi_status.h"

namespace task {

namespace {

constexpr uint16_t kMissionTagRaw =
    static_cast<uint16_t>(CROI_MISSION_CONFIG_CRC32 & 0xFFFFU);
constexpr uint16_t kMissionTag = kMissionTagRaw == 0U ? 0xFFFFU : kMissionTagRaw;

constexpr uint8_t configured_pyro_mask() {
    uint8_t mask = 0U;
#if CROI_MISSION_PYRO_DROGUE_CHANNEL >= 0
    mask |= static_cast<uint8_t>(1U << CROI_MISSION_PYRO_DROGUE_CHANNEL);
#endif
#if CROI_MISSION_PYRO_MAIN_CHANNEL >= 0
    mask |= static_cast<uint8_t>(1U << CROI_MISSION_PYRO_MAIN_CHANNEL);
#endif
    return mask;
}

void put_drop_oldest(osMessageQueueId_t queue, const secondary_flight_data& data) {
    if (osMessageQueuePut(queue, &data, 0U, 0U) == osOK) {
        return;
    }

    secondary_flight_data discarded{};
    (void)osMessageQueueGet(queue, &discarded, nullptr, 0U);
    (void)osMessageQueuePut(queue, &data, 0U, 0U);
}

}

bool CAN_task::run() {
    taskHandle_ = osThreadNew(&CAN_task::StartCANEntry,
                              this,
                              &task_attributes);
    return taskHandle_ != nullptr;
}

void CAN_task::StartCANEntry(void *argument) {
    auto *self = static_cast<CAN_task*>(argument);

    if (self) {
        self->StartCAN();
    }
}

void CAN_task::StartCAN() {
    secondary_flight_data shared_data{};
    flight_data outbound_data{};
    last_heartbeat_ms_ = HAL_GetTick();
    last_flight_tx_ms_ = last_heartbeat_ms_;
    last_bus_recovery_ms_ = last_heartbeat_ms_;

    for (;;) {
        const uint32_t now_ms = HAL_GetTick();

        while (osMessageQueueGet(sender_queue_, &outbound_data, nullptr, 0U) == osOK) {
            pending_outbound_data_ = outbound_data;
            flight_state_ = static_cast<uint8_t>(outbound_data.state);
            has_pending_outbound_data_ = true;
        }

        service_bus_health(now_ms);
        expire_node_status(now_ms);
        flush_tx_queue();

        bool received_update = false;
        CAN_Frame rx_frame{};
        while (canbus_.receive(&rx_frame)) {
            received_update |= process_rx_frame(rx_frame, shared_data);
        }

        if (received_update) {
            put_drop_oldest(reciver_queue_, shared_data);
        }


        if (has_pending_outbound_data_ &&
            (now_ms - last_flight_tx_ms_) >= CAN_FLIGHT_TX_MIN_PERIOD_MS) {
            send_flight_data(pending_outbound_data_);
            has_pending_outbound_data_ = false;
            last_flight_tx_ms_ = now_ms;
        }

        service_airbrake_command(now_ms, shared_data);
        service_pyro_commands(now_ms, shared_data);

        if ((now_ms - last_heartbeat_ms_) >= CAN_HEARTBEAT_PERIOD_MS) {
            send_heartbeat(now_ms);
            last_heartbeat_ms_ = now_ms;
        }

        update_status(now_ms);
        osDelay(CAN_DELAY_MS);
    }
}

void CAN_task::log_pyro_event(secondary_flight_data& shared_data,
                              uint32_t now_ms,
                              PyroEventAction action,
                              uint8_t channel,
                              uint16_t sequence,
                              uint8_t result,
                              uint8_t fault) {
    shared_data.pyro.timestamp_ms = now_ms;
    shared_data.pyro.mission_tag = kMissionTag;
    shared_data.pyro.sequence = sequence;
    shared_data.pyro.channel = channel;
    shared_data.pyro.action = static_cast<uint8_t>(action);
    shared_data.pyro.result = result;
    shared_data.pyro.fault = fault;
    shared_data.pyro.armed_mask = pyro_armed_mask_;
    shared_data.pyro.continuity_mask = pyro_continuity_mask_;
    shared_data.pyro.fired_mask = pyro_fired_mask_;
    put_drop_oldest(reciver_queue_, shared_data);
}

bool CAN_task::send_pyro_arm(uint8_t channel_mask,
                             uint32_t now_ms,
                             secondary_flight_data& shared_data) {
    ++pyro_sequence_;
    if (pyro_sequence_ == 0U) {
        ++pyro_sequence_;
    }
    PYRO_ARM_Payload payload{
        channel_mask,
        PYRO_COMMAND_ARM,
        pyro_sequence_,
        kMissionTag,
        pyro_command_tag(PYRO_COMMAND_ARM, channel_mask, pyro_sequence_, kMissionTag),
    };
    CAN_Frame frame = pack_frame(CAN_ID_PYRO_ARM, payload);
    if (!send_critical_frame(frame)) {
        return false;
    }
    ++croi_status.pyro_arm_command_count;
    croi_status.pyro_last_sequence = pyro_sequence_;
    log_pyro_event(shared_data, now_ms, PyroEventAction::ArmCommand,
                   0xFFU, pyro_sequence_, 0U, 0U);
    return true;
}

bool CAN_task::send_pyro_fire(uint8_t channel,
                              uint32_t now_ms,
                              secondary_flight_data& shared_data) {
    ++pyro_sequence_;
    if (pyro_sequence_ == 0U) {
        ++pyro_sequence_;
    }
    const uint8_t command = flight_state_ == static_cast<uint8_t>(State::DROUGE)
        ? PYRO_COMMAND_FIRE_DROGUE
        : PYRO_COMMAND_FIRE_MAIN;
    PYRO_FIRE_Payload payload{
        channel,
        command,
        pyro_sequence_,
        kMissionTag,
        pyro_command_tag(command, channel, pyro_sequence_, kMissionTag),
    };
    CAN_Frame frame = pack_frame(CAN_ID_PYRO_FIRE, payload);
    if (!send_critical_frame(frame)) {
        return false;
    }
    ++croi_status.pyro_fire_command_count;
    croi_status.pyro_last_sequence = pyro_sequence_;
    croi_status.pyro_last_channel = channel;
    log_pyro_event(shared_data, now_ms, PyroEventAction::FireCommand,
                   channel, pyro_sequence_, 0U, 0U);
    return true;
}

void CAN_task::service_pyro_commands(uint32_t now_ms,
                                     secondary_flight_data& shared_data) {
    constexpr uint8_t configured_mask = configured_pyro_mask();
    if (configured_mask == 0U) {
        return;
    }

    if (flight_state_ != previous_pyro_state_) {
        previous_pyro_state_ = flight_state_;
        pyro_state_entry_ms_ = now_ms;
    }

    const bool active_flight =
        flight_state_ >= static_cast<uint8_t>(State::POWERED) &&
        flight_state_ <= static_cast<uint8_t>(State::MAIN);
    if (!active_flight) {
        if (pyro_armed_mask_ != 0U &&
            (now_ms - last_pyro_arm_ms_) >= CAN_PYRO_ARM_PERIOD_MS &&
            send_pyro_arm(0U, now_ms, shared_data)) {
            last_pyro_arm_ms_ = now_ms;
        }
        return;
    }

    const uint8_t remaining_mask =
        static_cast<uint8_t>(configured_mask & static_cast<uint8_t>(~pyro_fired_mask_));
    if ((now_ms - last_pyro_arm_ms_) >= CAN_PYRO_ARM_PERIOD_MS &&
        send_pyro_arm(remaining_mask, now_ms, shared_data)) {
        last_pyro_arm_ms_ = now_ms;
    }

#if CROI_MISSION_PYRO_DROGUE_CHANNEL >= 0
    constexpr uint8_t drogue_channel = CROI_MISSION_PYRO_DROGUE_CHANNEL;
    constexpr uint8_t drogue_bit = static_cast<uint8_t>(1U << drogue_channel);
    if (flight_state_ == static_cast<uint8_t>(State::DROUGE) &&
        (pyro_fired_mask_ & drogue_bit) == 0U &&
        (pyro_armed_mask_ & drogue_bit) != 0U &&
        (now_ms - pyro_state_entry_ms_) >= CAN_PYRO_FIRE_SETTLE_MS &&
        (last_drogue_fire_attempt_ms_ == 0U ||
         (now_ms - last_drogue_fire_attempt_ms_) >= CAN_PYRO_FIRE_RETRY_MS)) {
        last_drogue_fire_attempt_ms_ = now_ms;
        (void)send_pyro_fire(drogue_channel, now_ms, shared_data);
    }
#endif

#if CROI_MISSION_PYRO_MAIN_CHANNEL >= 0
    constexpr uint8_t main_channel = CROI_MISSION_PYRO_MAIN_CHANNEL;
    constexpr uint8_t main_bit = static_cast<uint8_t>(1U << main_channel);
    if (flight_state_ == static_cast<uint8_t>(State::MAIN) &&
        (pyro_fired_mask_ & main_bit) == 0U &&
        (pyro_armed_mask_ & main_bit) != 0U &&
        (now_ms - pyro_state_entry_ms_) >= CAN_PYRO_FIRE_SETTLE_MS &&
        (last_main_fire_attempt_ms_ == 0U ||
         (now_ms - last_main_fire_attempt_ms_) >= CAN_PYRO_FIRE_RETRY_MS)) {
        last_main_fire_attempt_ms_ = now_ms;
        (void)send_pyro_fire(main_channel, now_ms, shared_data);
    }
#endif
}

void CAN_task::service_airbrake_command(uint32_t now_ms,
                                        secondary_flight_data& shared_data) {
    if ((now_ms - last_actuator_command_ms_) < CAN_ACTUATOR_COMMAND_PERIOD_MS) {
        return;
    }

    const AirbrakeCommand command = airbrake_logic_.update(
        static_cast<State>(flight_state_), now_ms);
    ++actuator_sequence_;
    if (actuator_sequence_ == 0U) {
        ++actuator_sequence_;
    }

    ActuatorCommandPayload payload{
        command.output_index,
        static_cast<uint8_t>(command.active ? ACTUATOR_COMMAND_FLAG_ACTIVE : 0U),
        static_cast<int16_t>(command.angle_deg * 100U),
        actuator_sequence_,
        static_cast<uint16_t>(CROI_MISSION_AIRBRAKE_COMMAND_TIMEOUT_MS),
    };
    CAN_Frame frame = pack_frame(CAN_ID_ACTUATOR_COMMAND, payload);
    if (!send_frame(frame)) {
        return;
    }
    last_actuator_command_ms_ = now_ms;
    ++croi_status.actuator_command_count;
    croi_status.actuator_last_sequence = actuator_sequence_;
    croi_status.actuator_last_output = command.output_index;
    croi_status.actuator_last_angle_deg = command.angle_deg;
    croi_status.actuator_active = command.active ? 1U : 0U;

    const bool changed = !has_previous_airbrake_command_ ||
        command.active != previous_airbrake_command_.active ||
        command.output_index != previous_airbrake_command_.output_index ||
        command.angle_deg != previous_airbrake_command_.angle_deg;
    if (changed || (now_ms - last_actuator_log_ms_) >= 1000U) {
        shared_data.canards.output_index = command.output_index;
        shared_data.canards.sequence = actuator_sequence_;
        shared_data.canards.servo_angle = static_cast<float>(command.angle_deg);
        shared_data.canards.active = command.active;
        put_drop_oldest(reciver_queue_, shared_data);
        last_actuator_log_ms_ = now_ms;
    }
    previous_airbrake_command_ = command;
    has_previous_airbrake_command_ = true;
}

bool CAN_task::process_rx_frame(const CAN_Frame& frame, secondary_flight_data& shared_data) {
    if (CAN_ID_IS_HEARTBEAT(frame.id)) {
        HEARTBEAT_Payload payload{};
        if (!try_unpack_frame(frame, payload)) {
            return false;
        }
        record_heartbeat(payload, HAL_GetTick());
        return false;
    }

    switch (frame.id) {
        case CAN_ID_GPS: {
                GPS_Payload payload{};
                if (!unpack_gps(frame, payload)) {
                        return false;
                }

                shared_data.gps.latitude  = gps_decode(payload.latitude);
                shared_data.gps.longitude = gps_decode(payload.longitude);
                shared_data.gps.satellites = payload.satellites;

                return true;
        }
        case CAN_ID_ACTUATOR_COMMAND: {
                ActuatorCommandPayload payload{};
                if (!try_unpack_frame(frame, payload)) {
                        return false;
                }

                shared_data.canards.output_index = payload.output_index;
                shared_data.canards.sequence = payload.sequence;
                shared_data.canards.servo_angle = static_cast<float>(payload.angle_cdeg) / 100.0f;
                shared_data.canards.active = (payload.flags & ACTUATOR_COMMAND_FLAG_ACTIVE) != 0U;

                return true;
        }
        case CAN_ID_PYRO_STATUS: {
                PYRO_STATUS_Payload payload{};
                if (!try_unpack_frame(frame, payload)) {
                    return false;
                }
                pyro_armed_mask_ = payload.armed;
                pyro_continuity_mask_ = payload.cont_check;
                pyro_fired_mask_ = payload.fired;
                if (pyro_sequence_newer(payload.last_sequence, pyro_sequence_)) {
                    pyro_sequence_ = payload.last_sequence;
                }
                ++croi_status.pyro_status_count;
                croi_status.pyro_last_sequence = payload.last_sequence;
                croi_status.pyro_armed_mask = payload.armed;
                croi_status.pyro_continuity_mask = payload.cont_check;
                croi_status.pyro_fired_mask = payload.fired;
                log_pyro_event(shared_data, HAL_GetTick(), PyroEventAction::Status,
                               0xFFU, payload.last_sequence, 0U, payload.faults1);
                return false;
        }
        case CAN_ID_PYRO_ACK: {
                PYRO_ACK_Payload payload{};
                if (!try_unpack_frame(frame, payload) || payload.mission_tag != kMissionTag) {
                    return false;
                }
                if (pyro_sequence_newer(payload.sequence, pyro_sequence_)) {
                    pyro_sequence_ = payload.sequence;
                }
                ++croi_status.pyro_ack_count;
                croi_status.pyro_last_sequence = payload.sequence;
                croi_status.pyro_last_channel = payload.channel;
                croi_status.pyro_last_result = payload.result;
                croi_status.pyro_last_fault = payload.fault_code;
                if (payload.channel < 4U &&
                    payload.result == static_cast<uint8_t>(PyroResult::FIRED) &&
                    pyro_is_fire_command(payload.command)) {
                    pyro_fired_mask_ |= static_cast<uint8_t>(1U << payload.channel);
                    croi_status.pyro_fired_mask = pyro_fired_mask_;
                }
                log_pyro_event(shared_data, HAL_GetTick(), PyroEventAction::Acknowledgement,
                               payload.channel, payload.sequence,
                               payload.result, payload.fault_code);
                return false;
        }

        default:
            return false;
    }
}

void CAN_task::send_flight_data(const flight_data& data) {
    const uint16_t timestamp =
        static_cast<uint16_t>(data.core_data.time & 0xFFFFU);

    KALMANN_Payload kalman_payload{
        static_cast<int16_t>(data.prediction.acceleration * 100.0f),
        static_cast<int16_t>(data.prediction.altitude),
        static_cast<int16_t>(data.prediction.velocity * 100.0f),
        timestamp
    };
    CAN_Frame kalman_frame = pack_frame(CAN_ID_KALMANN, kalman_payload);
    send_frame(kalman_frame);

    BARO_Payload baro_payload{
        static_cast<uint32_t>(data.core_data.barometer.pressure),
        static_cast<int16_t>(data.core_data.barometer.temperature * 100.0f),
        static_cast<int16_t>(data.core_data.barometer.altitude * 10.0f)
    };
    CAN_Frame baro_frame = pack_frame(CAN_ID_BARO, baro_payload);
    send_frame(baro_frame);

    IMU_ACCEL_Payload accel_payload{
    static_cast<int16_t>(data.core_data.imu.acceleration.x * 100.0f),
    static_cast<int16_t>(data.core_data.imu.acceleration.y * 100.0f),
    static_cast<int16_t>(data.core_data.imu.acceleration.z * 100.0f),
    timestamp
    };

    CAN_Frame accel_frame = pack_frame(CAN_ID_IMU_ACCEL, accel_payload);
    send_frame(accel_frame);

     FLIGHT_STATE_Payload state_payload{
        static_cast<uint8_t>(data.state),
        0U,
        timestamp
    };
    CAN_Frame state_frame = pack_frame(CAN_ID_FLIGHT_STATE, state_payload);
    send_frame(state_frame);

    IMU_GYRO_Payload gyro_payload{
        static_cast<int16_t>(data.core_data.imu.gyro.x * 100.0f),
        static_cast<int16_t>(data.core_data.imu.gyro.y * 100.0f),
        static_cast<int16_t>(data.core_data.imu.gyro.z * 100.0f),
        timestamp
    };

    CAN_Frame gyro_frame = pack_frame(CAN_ID_IMU_GYRO, gyro_payload);
    send_frame(gyro_frame);
}

void CAN_task::send_heartbeat(uint32_t now_ms) {
    uint8_t err = 0U;
    if (canbus_.is_bus_off()) {
        err |= CAN_HEARTBEAT_ERR_BUS_OFF;
    }
    if (canbus_.error() != 0U) {
        err |= CAN_HEARTBEAT_ERR_CAN_ERROR;
    }
    if (tx_retry_drops_ != 0U) {
        err |= CAN_HEARTBEAT_ERR_TX_DROP;
    }
    if (node_timeout_count_ != 0U) {
        err |= CAN_HEARTBEAT_ERR_NODE_TIMEOUT;
    }

    HEARTBEAT_Payload payload{
        node_id_,
        flight_state_,
        err,
        static_cast<uint8_t>((now_ms / 1000U) & 0xFFU)
    };
    CAN_Frame frame = pack_frame(CAN_ID_HEARTBEAT, payload);
    send_frame(frame);
}

void CAN_task::service_bus_health(uint32_t now_ms) {
#if CAN_AUTO_RECOVER_BUS_OFF
    if (!canbus_.is_bus_off()) {
        return;
    }

    if ((now_ms - last_bus_recovery_ms_) < CAN_BUS_RECOVERY_PERIOD_MS) {
        return;
    }

    (void)canbus_.recover_from_bus_off();
    last_bus_recovery_ms_ = now_ms;
#else
    (void)now_ms;
#endif
}

void CAN_task::flush_tx_queue() {
    for (uint8_t sent = 0U;
         sent < CAN_TX_DRAIN_BUDGET_PER_LOOP && critical_tx_count_ > 0U;
         ++sent) {
        CAN_Frame& frame = critical_tx_queue_[critical_tx_head_];
        if (frame.id == CAN_ID_PYRO_FIRE) {
            PYRO_FIRE_Payload payload{};
            if (!try_unpack_frame(frame, payload) ||
                pyro_fire_expected_state(payload.command) != flight_state_) {
                critical_tx_head_ = static_cast<uint8_t>(
                    (critical_tx_head_ + 1U) % CAN_CRITICAL_TX_QUEUE_LEN);
                --critical_tx_count_;
                ++critical_tx_drops_;
                continue;
            }
        }
        if (!canbus_.send(&frame)) {
            return;
        }
        critical_tx_head_ = static_cast<uint8_t>(
            (critical_tx_head_ + 1U) % CAN_CRITICAL_TX_QUEUE_LEN);
        --critical_tx_count_;
    }

    for (uint8_t sent = 0U;
         sent < CAN_TX_DRAIN_BUDGET_PER_LOOP && tx_retry_count_ > 0U;
         ++sent) {
        CAN_Frame& frame = tx_retry_queue_[tx_retry_head_];
        if (!canbus_.send(&frame)) {
            return;
        }

        tx_retry_head_ = static_cast<uint8_t>(
            (tx_retry_head_ + 1U) % CAN_TX_RETRY_QUEUE_LEN);
        --tx_retry_count_;
    }
}

bool CAN_task::queue_critical_frame(const CAN_Frame& frame) {
    if (critical_tx_count_ >= CAN_CRITICAL_TX_QUEUE_LEN) {
        ++critical_tx_drops_;
        return false;
    }
    critical_tx_queue_[critical_tx_tail_] = frame;
    critical_tx_tail_ = static_cast<uint8_t>(
        (critical_tx_tail_ + 1U) % CAN_CRITICAL_TX_QUEUE_LEN);
    ++critical_tx_count_;
    return true;
}

bool CAN_task::queue_tx_frame(const CAN_Frame& frame) {
    if (tx_retry_count_ >= CAN_TX_RETRY_QUEUE_LEN) {
        tx_retry_head_ = static_cast<uint8_t>(
            (tx_retry_head_ + 1U) % CAN_TX_RETRY_QUEUE_LEN);
        --tx_retry_count_;
        ++tx_retry_drops_;
    }

    tx_retry_queue_[tx_retry_tail_] = frame;
    tx_retry_tail_ = static_cast<uint8_t>(
        (tx_retry_tail_ + 1U) % CAN_TX_RETRY_QUEUE_LEN);
    ++tx_retry_count_;
    return true;
}

void CAN_task::record_heartbeat(const HEARTBEAT_Payload& heartbeat,
                                uint32_t now_ms) {
    if (heartbeat.node_id == node_id_ || heartbeat.node_id == 0U) {
        return;
    }

    for (NodeStatus& node : nodes_) {
        if (node.active && node.node_id == heartbeat.node_id) {
            node.state = heartbeat.state;
            node.err = heartbeat.err;
            node.tx_error_count = 0U;
            node.rx_error_count = 0U;
            node.tx_queue_depth = 0U;
            node.uptime_s = heartbeat.uptime_s;
            node.last_seen_ms = now_ms;
            ++node.rx_count;
            return;
        }
    }

    for (NodeStatus& node : nodes_) {
        if (!node.active) {
            node.active = true;
            node.node_id = heartbeat.node_id;
            node.state = heartbeat.state;
            node.err = heartbeat.err;
            node.tx_error_count = 0U;
            node.rx_error_count = 0U;
            node.tx_queue_depth = 0U;
            node.uptime_s = heartbeat.uptime_s;
            node.last_seen_ms = now_ms;
            node.rx_count = 1U;
            return;
        }
    }
}

void CAN_task::expire_node_status(uint32_t now_ms) {
    for (NodeStatus& node : nodes_) {
        if (!node.active) {
            continue;
        }

        if ((now_ms - node.last_seen_ms) >= CAN_NODE_TIMEOUT_MS) {
            node.active = false;
            node.err = 1U;
            ++node_timeout_count_;
        }
    }
}

bool CAN_task::send_frame(CAN_Frame& frame) {
    if (tx_retry_count_ != 0U) {
        return queue_tx_frame(frame);
    }

    if (canbus_.send(&frame)) {
        return true;
    }

    return queue_tx_frame(frame);
}

bool CAN_task::send_critical_frame(CAN_Frame& frame) {
    if (critical_tx_count_ != 0U) {
        return queue_critical_frame(frame);
    }
    if (canbus_.send(&frame)) {
        return true;
    }
    return queue_critical_frame(frame);
}

void CAN_task::update_status(uint32_t now_ms) {
    uint32_t active_nodes = 0U;
    for (const NodeStatus& node : nodes_) {
        if (node.active) {
            ++active_nodes;
        }
    }

    croi_status.uptime_ms = now_ms;
    croi_status.can_task_heartbeat_ms = now_ms;
    croi_status.can_bus_off = canbus_.is_bus_off() ? 1U : 0U;
    croi_status.can_error = canbus_.error();
    croi_status.can_tx_retry_depth = tx_retry_count_;
    croi_status.can_tx_retry_drops = tx_retry_drops_;
    croi_status.can_node_timeout_count = node_timeout_count_;
    croi_status.can_active_nodes = active_nodes;
    croi_status.can_last_heartbeat_ms = last_heartbeat_ms_;
    croi_status.can_stack_free_bytes =
        osThreadGetStackSpace(taskHandle_) * sizeof(StackType_t);
    croi_status.rtos_heap_free_bytes = xPortGetFreeHeapSize();
    croi_status.pyro_critical_tx_drops = critical_tx_drops_;
}

}
