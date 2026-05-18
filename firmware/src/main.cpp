#include "data.h"
#if F4
#include "stm32f4xx_hal.h"
#include "platform/stm_f4.h"
#elif F0
#include "stm32f0xx_hal.h"
#include "platform/stm_f0.h"
#elif defined(LINUX)
#include "platform/linux.h"
#endif
#include "cmsis_os.h"
#include <data.h>
#include "tools/state_machine.h"
#include "tools/kalman_filter.h"

#include <IMU/IMU.h>
#include <IMU/MPU6050.h>
#include <sensor.h>
#include <Baro/baro.h>
#include <Baro/BMP390.h>

#include <I2C/I2C_STM.h>
#include <SPI/SPI_STM.h>
#include <CAN/CAN_Handler.h>
#include <CAN/CAN_Frames.h>


void SystemClock_Config(void);
void Error_Handler(void);

struct FSM_TaskArgs {
    IMU* imu;
    Baro* baro;
    KalmanFilter* kalman;
    flash_internal_data settings;
    CAN_Handler* can;
};

void StartFSM(void *argument)
{
    auto* args = static_cast<FSM_TaskArgs*>(argument);

    IMU* imu = args->imu;
    Baro* baro = args->baro;
    KalmanFilter* kalman_filter = args->kalman;
    CAN_Handler* can = args->can;
    StateMachine* state_machine = new StateMachine(args->settings);

    flight_data raw_data;
    flight_data old_data;
    flight_data processed_data;
    imu_data imu_data;

    uint8_t uptime_s = 0;
    uint32_t last_1hz = HAL_GetTick();
    uint32_t time = HAL_GetTick();
    float time_diff = 0;
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

        printf("data %lu %f %f %f %d\n", time, raw_data.prediction.altitude, raw_data.prediction.velocity, raw_data.prediction.acceleration, state_machine->current_state);
        raw_data.state = state_machine->current_state;
        old_data = raw_data;

        // CAN TX
        if (can) {
            uint16_t ts = static_cast<uint16_t>(time & 0xFFFF);

            IMU_ACCEL_Payload acc{};
            acc.ax           = static_cast<int16_t>(imu_data.acceleration.x * 100.0f);
            acc.ay           = static_cast<int16_t>(imu_data.acceleration.y * 100.0f);
            acc.az           = static_cast<int16_t>(imu_data.acceleration.z * 100.0f);
            acc.timestamp_ms = ts;
            CAN_Frame acc_frame = pack_frame(CAN_ID_IMU_ACCEL, acc);
            can->send(&acc_frame);

            BARO_Payload baro_p{};
            baro_p.pressure = static_cast<uint32_t>(raw_data.core_data.barometer.pressure);
            baro_p.temp     = static_cast<int16_t>(raw_data.core_data.barometer.temperature * 10.0f);
            baro_p.seq      = 0;
            baro_p.flags    = 0;
            CAN_Frame baro_frame = pack_frame(CAN_ID_BARO, baro_p);
            can->send(&baro_frame);

            FLIGHT_STATE_Payload fs{};
            fs.state        = static_cast<uint8_t>(raw_data.state);
            fs.flags        = 0;
            fs.timestamp_ms = ts;
            CAN_Frame fs_frame = pack_frame(CAN_ID_FLIGHT_STATE, fs);
            can->send(&fs_frame);

            KALMANN_Payload kl{};
            kl.accleration   = static_cast<int16_t>(raw_data.prediction.acceleration * 100.0f);
            kl.altitude_m    = static_cast<int16_t>(raw_data.prediction.altitude);
            kl.vspeed_cms    = static_cast<int16_t>(raw_data.prediction.velocity * 100.0f);
            kl.timestamp_ms  = ts;
            CAN_Frame kl_frame = pack_frame(CAN_ID_KALMANN, kl);
            can->send(&kl_frame);

            if (time - last_1hz >= 1000) {
                last_1hz = time;
                uptime_s++;

                HEARTBEAT_Payload hb{};
                hb.node_id  = NODE_CROI;
                hb.state    = static_cast<uint8_t>(raw_data.state);
                hb.err      = 0;
                hb.uptime_s = uptime_s;
                CAN_Frame hb_frame = pack_frame(CAN_ID_HEARTBEAT, hb);
                can->send(&hb_frame);

                SYNC_Payload sync{};
                sync.timestamp_ms = time;
                CAN_Frame sync_frame = pack_frame(CAN_ID_SYNC, sync);
                can->send(&sync_frame);
            }

            // RX: drain incoming frames
            CAN_Frame rx{};
            while (can->receive(&rx)) {
                if (rx.id == CAN_ID_TX_STATUS) {
                    TX_STATUS_Payload ts_p{};
                    unpack_frame(rx, ts_p);
                    printf("[croi] RX TX_STATUS rssi=%d snr=%d\n", ts_p.rssi, ts_p.snr);
                } else if (rx.id == CAN_ID_PYRO_ACK) {
                    PYRO_ACK_Payload ack{};
                    unpack_frame(rx, ack);
                    printf("[croi] RX PYRO_ACK ch=%u result=%u\n", ack.channel, ack.result);
                } else if (rx.id == CAN_ID_CONFIG_CMD) {
                    CONFIG_CMD_Payload cfg{};
                    unpack_frame(rx, cfg);
                    printf("[croi] RX CONFIG_CMD id=%u\n", cfg.cmd_id);
                }
            }
        }

      osDelay(1000);
    }
}

osThreadId_t blinkTaskHandle;

const osThreadAttr_t blinkTask_attributes = {
    "FSMTask",          // name
    0,                    // attr_bits
    nullptr,              // cb_mem
    0,                    // cb_size
    nullptr,              // stack_mem
    256 * 4,              // stack_size
    osPriorityNormal,     // priority
    0,                    // tz_module
    0                     // reserved
};


int main(void)
{
    int* leaked_int = new int(42);
    HAL_Init();
    SystemClock_Config();
    osKernelInitialize();

#ifndef LINUX
    I2C_Handler* i2c_handler = new I2C_STM(&hi2c1, 0x68 << 1);
    SPI_Handler* spi_handler = new SPI_STM(&hspi1, BARO_CS_PORT, BARO_CS_PIN);
#else
    I2C_Handler* i2c_handler = new I2C_Mock();
    SPI_Handler* spi_handler = new SPI_Mock();
#endif
    IMU* imu = new MPU6050(*i2c_handler);
    Baro* baro = new BMP390(*spi_handler);
    KalmanFilter* kalman = new KalmanFilter();
    static flash_internal_data settings {
        .main_height = 200,
        .drouge_delay = 0,
        .liftoff_thresh = 20
    };

    //StateMachine* state_machine = new StateMachine(settings);

    CAN_Handler* can = nullptr;
#ifdef LINUX
    g_can.init();
    can = &g_can;
#endif

    static FSM_TaskArgs fsm_args;

    fsm_args.imu = imu;
    fsm_args.baro = baro;
    fsm_args.kalman = kalman;
    fsm_args.settings = settings;
    fsm_args.can = can;

    osThreadNew(StartFSM, &fsm_args, &blinkTask_attributes);

      osKernelStart();
      // never get here 
      while (1)
      {
        HAL_Delay(1000);
      }
    }



#ifndef LINUX
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) { Error_Handler(); }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) { HAL_IncTick(); }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
#endif /* LINUX */
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
