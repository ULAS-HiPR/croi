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
#include <task.h>
#include <data.h>

#include "tasks/state_machine.h"
#include "tasks/can_task.h"
#include "tasks/logger.h"
#include "tasks/watchdog_task.h"
#include "croi_status.h"
#include "croi_mission_config.h"

//Generic henders
#include <sensor.h>
#include <IMU/IMU.h>
#include <Baro/baro.h>
#include <Flash/flash.h>
#include <Buzzer/buzzer.h>
#include <SPI/SPI_STM.h>
#include <CAN/CAN_Handler.h>
#if F4
#include <CAN/CAN_Mock.h>
#elif F0
#include <CAN/CAN_STM.h>
#endif

//Specific sensors
#include <IMU/LSM6DSO32.h>
#include <Baro/MS5607.h>
#include <Flash/MX25L128.h>
#include <Buzzer/buzzer_stm.h>
//#inlcude <IMU/ADXL347.h>
//#include <Env/BME280.h>

extern "C" {

struct OgmaBoardIdentity {
    uint32_t magic;
    uint16_t schema_version;
    uint16_t struct_size;
    uint32_t board_id;
    uint32_t capabilities;
    uint32_t firmware_version;
    uint32_t firmware_build;
    uint32_t reserved0;
    uint32_t reserved1;
};

__attribute__((used)) volatile OgmaBoardIdentity ogma_board_identity{
    0x4F474944U,
    1U,
    sizeof(OgmaBoardIdentity),
    0x01U,
    0x03U,
    20260714U,
    0U,
    0U,
    0U,
};

__attribute__((used)) volatile CroiStatus croi_status{
    CROI_STATUS_MAGIC,
    CROI_STATUS_VERSION,
};

}


void SystemClock_Config(void);
void Error_Handler(void);

extern "C" void vApplicationMallocFailedHook(void)
{
    Error_Handler();
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t, char *)
{
    Error_Handler();
}

namespace {

static_assert(CROI_MISSION_CONFIG_MAGIC == 0x4F474D43U,
              "invalid Croí mission config magic");
static_assert(CROI_MISSION_CONFIG_SCHEMA_VERSION == 6U,
              "unsupported Croí mission config schema");
static_assert(CROI_MISSION_LIFTOFF_ACCEL_M_S2_X100 >= 100U &&
              CROI_MISSION_LIFTOFF_ACCEL_M_S2_X100 <= 20000U,
              "liftoff threshold must be 1 to 200 m/s^2");
static_assert(CROI_MISSION_MAIN_DEPLOY_ALTITUDE_M <= 20000U,
              "main deployment altitude is out of range");
static_assert(CROI_MISSION_DROGUE_DELAY_MS <= 600000U,
              "drogue delay is out of range");
static_assert(CROI_MISSION_IMU_VERTICAL_AXIS <= 2U,
              "IMU vertical axis is out of range");
static_assert(CROI_MISSION_IMU_VERTICAL_SIGN == -1 ||
              CROI_MISSION_IMU_VERTICAL_SIGN == 1,
              "IMU vertical sign must be -1 or 1");

constexpr uint32_t CAN_RECEIVER_QUEUE_DEPTH = 4U;
constexpr uint32_t CAN_SENDER_QUEUE_DEPTH = 4U;
constexpr uint32_t LOGGER_QUEUE_DEPTH = 4U;

StaticQueue_t can_receiver_queue_control_block{};
alignas(uint32_t) uint8_t can_receiver_queue_storage[
    CAN_RECEIVER_QUEUE_DEPTH * sizeof(secondary_flight_data)]{};
const osMessageQueueAttr_t can_receiver_queue_attributes{
    "canReceiverQueue",
    0U,
    &can_receiver_queue_control_block,
    sizeof(can_receiver_queue_control_block),
    can_receiver_queue_storage,
    sizeof(can_receiver_queue_storage),
};

StaticQueue_t can_sender_queue_control_block{};
alignas(uint32_t) uint8_t can_sender_queue_storage[
    CAN_SENDER_QUEUE_DEPTH * sizeof(flight_data)]{};
const osMessageQueueAttr_t can_sender_queue_attributes{
    "canSenderQueue",
    0U,
    &can_sender_queue_control_block,
    sizeof(can_sender_queue_control_block),
    can_sender_queue_storage,
    sizeof(can_sender_queue_storage),
};

StaticQueue_t logging_queue_control_block{};
alignas(uint32_t) uint8_t logging_queue_storage[
    LOGGER_QUEUE_DEPTH * sizeof(flight_data)]{};
const osMessageQueueAttr_t logging_queue_attributes{
    "loggingQueue",
    0U,
    &logging_queue_control_block,
    sizeof(logging_queue_control_block),
    logging_queue_storage,
    sizeof(logging_queue_storage),
};

}


int main(void)
  {
    (void)ogma_board_identity.magic;
    (void)croi_status.magic;
    croi_status.reset_flags = RCC->CSR;
    __HAL_RCC_CLEAR_RESET_FLAGS();
    croi_status.mission_config_magic = CROI_MISSION_CONFIG_MAGIC;
    croi_status.mission_config_schema_version = CROI_MISSION_CONFIG_SCHEMA_VERSION;
    croi_status.mission_config_crc32 = CROI_MISSION_CONFIG_CRC32;
    HAL_Init();
    SystemClock_Config();
    
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_TIM3_Init();
    osKernelInitialize();

    static SPI_STM spi_handler_baro(&hspi1, BARO_CS_PORT, BARO_CS_PIN);
    static SPI_STM spi_handler_imu(&hspi1, IMU_CS_PORT, IMU_CS_PIN);
    static SPI_STM spi_handler_flash(&hspi1, FLASH_CS_PORT, FLASH_CS_PIN);
    static MX25L128 flash_memory(spi_handler_flash);
    static LSM6DSO32 imu(spi_handler_imu);
    static MS5607 baro(spi_handler_baro);
    static Buzzer_STM buzzer(&htim3, TIM_CHANNEL_3);
    static flash_internal_data settings{
        .main_height_m = static_cast<int>(CROI_MISSION_MAIN_DEPLOY_ALTITUDE_M),
        .drogue_delay_ms = CROI_MISSION_DROGUE_DELAY_MS,
        .liftoff_accel_m_s2_x100 = CROI_MISSION_LIFTOFF_ACCEL_M_S2_X100,
    };

    bool init_status = true;
    buzzer.init();

    const bool imu_ok = imu.init();
    const bool baro_ok = baro.init();
    croi_status.imu_init_ok = imu_ok ? 1U : 0U;
    croi_status.baro_init_ok = baro_ok ? 1U : 0U;
    init_status &= imu_ok;
    init_status &= baro_ok;
    croi_status.init_ok = init_status ? 1U : 0U;
    if (!init_status) {
        Error_Handler();
    }
   
    printf("Init status: %s\n", init_status ? "OK" : "FAIL");

    static LoggerHealth logger_health;

    #if F4
      static CAN_MOCK canbus;
      croi_status.can_init_ok = 1U;
    #elif F0
      MX_CAN_Init();
      static CAN_STM canbus(&hcan);
      const bool can_ok = canbus.init();
      croi_status.can_init_ok = can_ok ? 1U : 0U;
      if (!can_ok) {
         Error_Handler();
      }
    #endif
    osMessageQueueId_t canReciverQueueHandle = osMessageQueueNew(
        CAN_RECEIVER_QUEUE_DEPTH,
        sizeof(secondary_flight_data),
        &can_receiver_queue_attributes);
    osMessageQueueId_t canSenderQueueHandle = osMessageQueueNew(
        CAN_SENDER_QUEUE_DEPTH,
        sizeof(flight_data),
        &can_sender_queue_attributes);
    osMessageQueueId_t loggingQueueHandle = osMessageQueueNew(
        LOGGER_QUEUE_DEPTH,
        sizeof(flight_data),
        &logging_queue_attributes);
    if (canReciverQueueHandle == nullptr || canSenderQueueHandle == nullptr ||
        loggingQueueHandle == nullptr) {
        Error_Handler();
    }

    static task::StateMachine state_machine(&imu, &baro, &settings, canSenderQueueHandle, loggingQueueHandle);
    static task::CAN_task can_task(canbus, canSenderQueueHandle, canReciverQueueHandle, NODE_CROI);
    static task::Logger logger(&flash_memory, loggingQueueHandle, canReciverQueueHandle, &logger_health);
    static task::WatchdogTask watchdog_task;

    #if defined(CROI_WIPE_FLASH_ON_BOOT)
    if (!logger.run()) {
      Error_Handler();
    }
    #elif !defined(READING)
    if (!can_task.run() || !state_machine.run() || !logger.run() || !watchdog_task.run()) {
      Error_Handler();
    }
    #else
    if (!logger.run()) {
      Error_Handler();
    }
    #endif
    //size_t freeHeap = xPortGetFreeHeapSize();
    //printf("Free heap size: %u bytes\n", freeHeap);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
      Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;


  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
      Error_Handler();
  }
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
