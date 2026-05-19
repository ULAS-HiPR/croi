#include "stm32f0xx_hal.h"

volatile uint32_t rescue_signature = 0x5AFE0001UL;
volatile uint32_t rescue_counter = 0;

void SystemClock_Config(void);
void Error_Handler(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    while (1) {
        ++rescue_counter;
        HAL_Delay(100);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef rcc_osc = {0};
    RCC_ClkInitTypeDef rcc_clk = {0};

    rcc_osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    rcc_osc.HSIState = RCC_HSI_ON;
    rcc_osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    rcc_osc.PLL.PLLState = RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(&rcc_osc) != HAL_OK) {
        Error_Handler();
    }

    rcc_clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    rcc_clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    rcc_clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    rcc_clk.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&rcc_clk, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
