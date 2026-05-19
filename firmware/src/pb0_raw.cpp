#include "stm32f0xx.h"

static void delay(volatile uint32_t cycles)
{
    while (cycles-- > 0U) {
        __NOP();
    }
}

int main(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    (void)RCC->AHBENR;

    GPIOB->MODER &= ~(3UL << (0U * 2U));
    GPIOB->MODER |= (1UL << (0U * 2U));

    GPIOB->OTYPER &= ~GPIO_OTYPER_OT_0;
    GPIOB->PUPDR &= ~(3UL << (0U * 2U));
    GPIOB->OSPEEDR |= (3UL << (0U * 2U));

    while (1) {
        GPIOB->BSRR = GPIO_BSRR_BS_0;
        delay(800000U);

        GPIOB->BRR = GPIO_BRR_BR_0;
        delay(800000U);
    }
}
