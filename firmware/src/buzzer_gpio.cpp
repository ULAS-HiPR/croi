#include "stm32f0xx_hal.h"

#define BUZZER_GPIO_PORT GPIOB
#define BUZZER_GPIO_PIN GPIO_PIN_0

void SystemClock_Config(void);
void Error_Handler(void);

static void DelayCycles(volatile uint32_t cycles)
{
    while (cycles-- > 0U) {
        __NOP();
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = BUZZER_GPIO_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BUZZER_GPIO_PORT, &gpio);

    while (1) {
        BUZZER_GPIO_PORT->BSRR = BUZZER_GPIO_PIN;
        DelayCycles(7000U);
        BUZZER_GPIO_PORT->BRR = BUZZER_GPIO_PIN;
        DelayCycles(7000U);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef rcc_osc = {0};
    RCC_ClkInitTypeDef rcc_clk = {0};

    rcc_osc.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
    rcc_osc.HSI48State = RCC_HSI48_ON;
    rcc_osc.PLL.PLLState = RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(&rcc_osc) != HAL_OK) {
        Error_Handler();
    }

    rcc_clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    rcc_clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
    rcc_clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    rcc_clk.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&rcc_clk, FLASH_LATENCY_1) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
