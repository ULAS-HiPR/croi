#pragma once

#include <cstdint>

constexpr uint32_t CROI_STATUS_MAGIC = 0x43524F49U; // CROI
constexpr uint32_t CROI_STATUS_VERSION = 9U;

extern "C" {

struct CroiStatus {
    uint32_t magic;
    uint32_t version;
    uint32_t uptime_ms;
    uint32_t init_ok;
    uint32_t imu_init_ok;
    uint32_t baro_init_ok;
    uint32_t can_init_ok;
    uint32_t flash_init_ok;
    uint32_t logger_fault_latched;
    uint32_t logger_logging_stopped;
    uint32_t logger_fault;
    uint32_t logger_flash_status;
    uint32_t logger_run_id;
    uint32_t logger_records_written;
    uint32_t logger_used_bytes;
    uint32_t can_bus_off;
    uint32_t can_error;
    uint32_t can_tx_retry_depth;
    uint32_t can_tx_retry_drops;
    uint32_t can_node_timeout_count;
    uint32_t can_active_nodes;
    uint32_t can_last_heartbeat_ms;
    uint32_t flight_state;
    uint32_t baro_pressure_pa;
    int32_t baro_temperature_c_x100;
    int32_t baro_altitude_m_x100;
    int32_t prediction_altitude_m_x100;
    int32_t prediction_velocity_m_s_x100;
    int32_t prediction_accel_m_s2_x100;
    int32_t imu_accel_x_g_x1000;
    int32_t imu_accel_y_g_x1000;
    int32_t imu_accel_z_g_x1000;
    int32_t imu_gyro_x_dps;
    int32_t imu_gyro_y_dps;
    int32_t imu_gyro_z_dps;
    uint32_t flash_wipe_state;
    uint32_t flash_wipe_progress_percent;
    uint32_t flash_wipe_address;
    uint32_t fsm_stack_free_bytes;
    uint32_t can_stack_free_bytes;
    uint32_t logger_stack_free_bytes;
    uint32_t rtos_heap_free_bytes;
    uint32_t mission_config_magic;
    uint32_t mission_config_schema_version;
    uint32_t mission_config_crc32;
    uint32_t sensor_sample_valid;
    uint32_t imu_read_failures;
    uint32_t baro_read_failures;
    uint32_t imu_last_ok_ms;
    uint32_t baro_last_ok_ms;
    uint32_t can_queue_drops;
    uint32_t logger_queue_drops;
    uint32_t state_transition_count;
    uint32_t state_entry_ms;
    uint32_t actuator_command_count;
    uint32_t actuator_last_sequence;
    uint32_t actuator_last_output;
    uint32_t actuator_last_angle_deg;
    uint32_t actuator_active;
    uint32_t fsm_task_heartbeat_ms;
    uint32_t can_task_heartbeat_ms;
    uint32_t logger_task_heartbeat_ms;
    uint32_t watchdog_init_ok;
    uint32_t watchdog_refresh_count;
    uint32_t watchdog_missed_count;
    uint32_t reset_flags;
    uint32_t pyro_arm_command_count;
    uint32_t pyro_fire_command_count;
    uint32_t pyro_ack_count;
    uint32_t pyro_status_count;
    uint32_t pyro_last_sequence;
    uint32_t pyro_last_channel;
    uint32_t pyro_last_result;
    uint32_t pyro_last_fault;
    uint32_t pyro_armed_mask;
    uint32_t pyro_continuity_mask;
    uint32_t pyro_fired_mask;
    uint32_t pyro_critical_tx_drops;
};

static_assert(sizeof(CroiStatus) == 312U, "CroiStatus wire contract changed");

extern volatile CroiStatus croi_status;

}
