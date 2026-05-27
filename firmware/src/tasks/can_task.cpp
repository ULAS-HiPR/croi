#include "CAN_task.h"

namespace task {

void CAN_task::run() {
    taskHandle_ = osThreadNew(&CAN_task::StartCANEntry,
                              this,
                              &task_attributes);
}

void CAN_task::StartCANEntry(void *argument) {
    auto *self = static_cast<CAN_task*>(argument);

    if (self) {
        self->StartCAN();
    }
}

void CAN_task::StartCAN() {

    printf("CAN task started\n");

    CAN_Frame rx_frame;

    flight_data shared_data{};
    canards_raw canards_data{};

    for (;;) {
        if (canbus_.receive(&rx_frame)) {
            switch (rx_frame.id) {
                case CAN_ID_IMU_ACCEL: {

                    if (rx_frame.dlc != sizeof(IMU_ACCEL_Payload)) {
                        break;
                    }

                    IMU_ACCEL_Payload accel{};
                    unpack_frame(rx_frame, accel);

                    shared_data.core_data.imu.acceleration.x = accel.ax / 100.0f;
                    shared_data.core_data.imu.acceleration.y = accel.ay / 100.0f;
                    shared_data.core_data.imu.acceleration.z = accel.az / 100.0f;

                    break;
                }

                case CAN_ID_IMU_GYRO: {

                    if (rx_frame.dlc != sizeof(IMU_GYRO_Payload)) {
                        break;
                    }

                    IMU_GYRO_Payload gyro{};
                    unpack_frame(rx_frame, gyro);

                    shared_data.core_data.imu.gyro.x = gyro.gx / 100.0f;
                    shared_data.core_data.imu.gyro.y = gyro.gy / 100.0f;
                    shared_data.core_data.imu.gyro.z = gyro.gz / 100.0f;

                    break;
                }

                case CAN_ID_BARO: {

                    if (rx_frame.dlc != sizeof(BARO_Payload)) {
                        break;
                    }

                    BARO_Payload baro{};
                    unpack_frame(rx_frame, baro);

                    shared_data.core_data.barometer.pressure = static_cast<float>(baro.pressure);
                    shared_data.core_data.barometer.temperature =  baro.temp / 100.0f;

                    break;
                }
                case CAN_ID_KALMANN: {
                    if (rx_frame.dlc != sizeof(KALMANN_Payload)) {
                        break;
                    }

                    KALMANN_Payload kalman{};
                    unpack_frame(rx_frame, kalman);

                    shared_data.prediction.altitude =  kalman.altitude_m;
                    shared_data.prediction.velocity = kalman.vspeed_cms / 100.0f;
                    shared_data.prediction.acceleration = kalman.accleration / 100.0f;

                    break;
                }
                case CAN_ID_FLIGHT_STATE: {
                    if (rx_frame.dlc != sizeof(FLIGHT_STATE_Payload)) {
                        break;
                    }

                    FLIGHT_STATE_Payload state{};
                    unpack_frame(rx_frame, state);

                    shared_data.state = state.state;

                    break;
                }

                default:
                    break;
            }
        }
        // --- IGNORE THIS TEST CODE ---/
            shared_data.core_data.imu.gyro.x = 100.0f; 
            shared_data.core_data.imu.gyro.y = 0.0f;
            shared_data.core_data.imu.gyro.z = 0.0f;
            shared_data.core_data.barometer.pressure = 101325.0f;
            shared_data.core_data.barometer.temperature = 20.0f;
            shared_data.prediction.altitude = 100.0f;
            shared_data.prediction.velocity = 50.0f;
            shared_data.prediction.acceleration = -9.8f;
            shared_data.state = 1;
            osMessageQueuePut(can_queue_, &shared_data, 0, 10U);
        //}

        if (osMessageQueueGet(logger_queue_,&canards_data, 0, 0U) == osOK) {
            TX_STATUS_Payload tx{}; // temp payload struct until canards implemented
            tx.rssi = static_cast<int8_t>(canards_data.kp);
            tx.snr = static_cast<int8_t>(canards_data.kd);
            tx.tx_queue = static_cast<uint8_t>(canards_data.servo_angle);
            tx.flags = 0;

            CAN_Frame tx_frame = pack_frame(CAN_ID_TX_STATUS, tx);
            canbus_.send(&tx_frame);
        }

        // always send hearbeat
        HEARTBEAT_Payload hb{};
        hb.node_id = NODE_LAMH;
        hb.state   = 1;
        hb.err     = 0;
        hb.uptime_s = 0;
        CAN_Frame hb_frame =
        pack_frame(CAN_ID_HEARTBEAT, hb);
        canbus_.send(&hb_frame);
        
        osDelay(CAN_DELAY_MS);
    }
}

}