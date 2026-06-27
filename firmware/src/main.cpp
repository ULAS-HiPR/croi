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
#include "platform/error_handler.h"
#include "cmsis_os.h"

#include "tasks/CAN_task.h"

#if F4
#include <CAN/CAN_Mock.h>
#elif F0
#include <CAN/CAN_STM.h>
#endif

#if defined(READING) && defined(F0)
#include "tasks/flash_readout_task.h"
#include <Flash/MX25L128.h>
#include <SPI/SPI_STM.h>
#endif


void SystemClock_Config(void);
void Error_Handler(void);

const osMessageQueueAttr_t canRQueue_attributes = {
  .name = "canReciverQueue"
};

const osMessageQueueAttr_t canSQueue_attributes = {
  .name = "canSenderQueue"
};

int main(void)
 {
    HAL_Init();
    SystemClock_Config();
    osKernelInitialize();

#if defined(READING) && defined(F0)
    MX_GPIO_Init();
    MX_SPI1_Init();
    static SPI_STM flash_spi(&hspi1, FLASH_CS_GPIO_Port, FLASH_CS_Pin);
    static MX25L128 flash(flash_spi);
    static task::FlashReadoutTask flash_readout_task(flash);
    flash_readout_task.run();
#else
    osMessageQueueId_t canReciverQueueHandle = osMessageQueueNew(4, sizeof(flight_data), &canRQueue_attributes);
    osMessageQueueId_t canSenderQueueHandle = osMessageQueueNew(4, sizeof(flight_data), &canSQueue_attributes);
    if (canReciverQueueHandle == nullptr || canSenderQueueHandle == nullptr) {
      Error_Handler();
    }

    #if F4
      static CAN_MOCK canbus;
      if (!canbus.init()) {
        Error_Handler();
      }
      static task::CAN_task can_task(canbus, canSenderQueueHandle, canReciverQueueHandle, NODE_CROI);
      can_task.run();
    #elif F0
      MX_CAN_Init();
      static CAN_STM canbus(&hcan);
      if (!canbus.init()) {
        Error_Handler();
      }
      static task::CAN_task can_task(canbus, canSenderQueueHandle, canReciverQueueHandle, NODE_CROI);
      can_task.run();
    #endif
#endif

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

#ifdef F0
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
      Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
      Error_Handler();
  }
  SystemCoreClockUpdate();
#else
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
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
#endif
}

#ifdef F0
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
#endif // F0

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
