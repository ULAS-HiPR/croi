#ifdef F0
#ifndef STM_F072xB_H
#define STM_F072xB_H

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_spi.h"
#include "stm32f0xx_hal_gpio.h"
#include "error_handler.h"
#define LED_PIN GPIO_PIN_5 
#define LED_GPIO_PORT GPIOA 
#define LED_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE() 

#define SPI_SCK_PIN GPIO_PIN_5 
#define SPI_MISO_PIN GPIO_PIN_6 
#define SPI_MOSI_PIN GPIO_PIN_7 
#define SPI_GPIO_PORT GPIOA 
#define SPI_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE() 

#define IMU_CS_Pin GPIO_PIN_11
#define IMU_CS_GPIO_Port GPIOB

#define FLASH_CS_Pin GPIO_PIN_12
#define FLASH_CS_GPIO_Port GPIOB

#define BME_CS_Pin GPIO_PIN_14
#define BME_CS_GPIO_Port GPIOB

#define ADXL_CS_Pin GPIO_PIN_15
#define ADXL_CS_GPIO_Port GPIOB

#define BARO_CS_Pin GPIO_PIN_4
#define BARO_CS_GPIO_Port GPIOB

extern SPI_HandleTypeDef hspi1;
extern CAN_HandleTypeDef hcan;

// Initialization functions
void MX_SPI1_Init();
void MX_CAN_Init();
void MX_GPIO_Init();

#endif // STM_F4_H
#endif // F4