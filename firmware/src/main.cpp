#include "data.h"
#include <cstdint>
#if F4
#include "stm32f4xx_hal.h"
#include "platform/stm_f4.h"
#endif
#if F0
#include "stm32f0xx_hal.h"
#include "platform/stm_f0.h"
#endif
#include "cmsis_os.h"
#include <data.h>

#include "tools/kalman_filter.h"

#include "tasks/state_machine.h"
#include "tasks/can.h"
#include "tasks/logger.h"

//Generic henders
#include <sensor.h>
#include <IMU/IMU.h>
#include <Baro/baro.h>
#include <Flash/flash.h>
#include <I2C/I2C_STM.h>
#include <SPI/SPI_STM.h>

//Specific sensors
#include <IMU/LSM6DSO32.h>
#include <Baro/MS5607.h>
#include <Flash/MX25L128.h>
//#inlcude <IMU/ADXL347.h>
//#include <Env/BME280.h>


void SystemClock_Config(void);
void Error_Handler(void);

const osMessageQueueAttr_t canRQueue_attributes = {
  .name = "canReciverQueue"
};

const osMessageQueueAttr_t canSQueue_attributes = {
  .name = "canSenderQueue"
};

const osMessageQueueAttr_t loggingQueue_attributes = {
  .name = "loggingQueue"
};


int main(void)
{
    HAL_Init();
    SystemClock_Config();
    osKernelInitialize();

    osMessageQueueId_t canReciverQueueHandle = osMessageQueueNew(16, sizeof(flight_data), &canRQueue_attributes);
    osMessageQueueId_t canSenderQueueHandle = osMessageQueueNew(16, sizeof(flight_data), &canSQueue_attributes);
    osMessageQueueId_t loggingQueueHandle = osMessageQueueNew(16, sizeof(flight_data), &loggingQueue_attributes);

    SPI_Handler* spi_handler_baro = new SPI_STM(&hspi1, BARO_CS_PORT, BARO_CS_PIN);
    SPI_Handler* spi_handler_imu = new SPI_STM(&hspi1, IMU_CS_PORT, IMU_CS_PIN);

    bool init_status = true;
    Flash* flash_memory = new MX25L128();
    init_status &= flash_memory->init();
    
    IMU* imu = new LSM6DSO32(*spi_handler_imu);
    Baro* baro = new MS5607(*spi_handler_baro);

    init_status &= imu->init();
    init_status &= baro->init();

    //fake memory location
    uint8_t buffer[sizeof(flash_internal_data)];
    bool read_correctly = flash_memory->read(0x00, &buffer[0], sizeof(flash_internal_data));
    flash_internal_data* settings = reinterpret_cast<flash_internal_data*>(buffer);

    //CAN* canbus = new CAN();
    //canbus->init(); 
    // this should check the stack on boards on can, and load in what messages need to be send based on boards & settings


    static task::StateMachine state_machine(imu, baro, settings, canSenderQueueHandle, loggingQueueHandle);
    static task::CAN can_task(canSenderQueueHandle, canReciverQueueHandle);
    static task::Logger logger(flash_memory, loggingQueueHandle);

    can_task.run();
    logger.run();
    state_machine.run();


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
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
