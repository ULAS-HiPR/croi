#include "data.h"
#if F4
#include "stm32f4xx_hal.h"
#include "platform/stm_f4.h"
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


void SystemClock_Config(void);
void Error_Handler(void);

struct FSM_TaskArgs {
    IMU* imu;
    Baro* baro;
    KalmanFilter* kalman;
    flash_internal_data settings;
};

void StartFSM(void *argument)
{
    auto* args = static_cast<FSM_TaskArgs*>(argument);

    IMU* imu = args->imu;
    Baro* baro = args->baro;
    KalmanFilter* kalman_filter = args->kalman;
    StateMachine* state_machine = new StateMachine(args->settings);

    flight_data raw_data;
    flight_data old_data;
    flight_data processed_data;
    imu_data imu_data;

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

      osDelay(1000);
    }
}

osThreadId_t blinkTaskHandle;

const osThreadAttr_t blinkTask_attributes = {
    "blinkTask",          // name
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

    I2C_Handler* i2c_handler = new I2C_STM(&hi2c1, 0x68 << 1);
    SPI_Handler* spi_handler = new SPI_STM(&hspi1, BARO_CS_PORT, BARO_CS_PIN);
    IMU* imu = new MPU6050(*i2c_handler);
    Baro* baro = new BMP390(*spi_handler);
    KalmanFilter* kalman = new KalmanFilter();
    static flash_internal_data settings {
        .main_height = 200,
        .drouge_delay = 0,
        .liftoff_thresh = 20
    };

    //StateMachine* state_machine = new StateMachine(settings);

    static FSM_TaskArgs fsm_args;

    fsm_args.imu = imu;
    fsm_args.baro = baro;
    fsm_args.kalman = kalman;
    fsm_args.settings = settings;

    osThreadNew(StartFSM, &fsm_args, &blinkTask_attributes);

      osKernelStart();
      // never get here 
      while (1)
      {
        HAL_Delay(1000);
      }
    }



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    // stay here
  }
}