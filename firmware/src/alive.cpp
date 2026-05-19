#include "stm32f072xb.h"

volatile uint32_t alive_signature = 0xA117E5A1UL;
volatile uint32_t alive_counter = 0;

static void delay(volatile uint32_t ticks)
{
    while (ticks-- > 0U) {
        __NOP();
    }
}

int main(void)
{
    while (1) {
        ++alive_counter;
        delay(100000U);
    }
}
