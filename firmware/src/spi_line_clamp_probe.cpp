#include "stm32f0xx.h"

#include <cstdint>

namespace {

constexpr std::uint32_t kSysClockHz = 48000000U;
constexpr std::uint32_t kTimerTickHz = 1000000U;

constexpr std::uint16_t kSck = 1U << 5U;       // PA5
constexpr std::uint16_t kMisoNet = 1U << 6U;   // PA6
constexpr std::uint16_t kMosiNet = 1U << 7U;   // PA7
constexpr std::uint16_t kBuzzer = 1U << 0U;    // PB0
constexpr std::uint16_t kBaroCs = 1U << 4U;    // PB4
constexpr std::uint16_t kImuCs = 1U << 11U;    // PB11
constexpr std::uint16_t kFlashCs = 1U << 12U;  // PB12
constexpr std::uint16_t kBme680Cs = 1U << 14U; // PB14
constexpr std::uint16_t kAdxl375Cs = 1U << 15U;// PB15
constexpr std::uint16_t kAllCs = kBaroCs | kImuCs | kFlashCs | kBme680Cs | kAdxl375Cs;

void pin_high(GPIO_TypeDef* port, std::uint16_t pin)
{
    port->BSRR = pin;
}

void pin_low(GPIO_TypeDef* port, std::uint16_t pin)
{
    port->BRR = pin;
}

void clock_init()
{
    RCC->CR2 |= RCC_CR2_HSI48ON;
    while ((RCC->CR2 & RCC_CR2_HSI48RDY) == 0U) {}

    FLASH->ACR |= FLASH_ACR_LATENCY;

    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_HSI48;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI48) {}
}

void timebase_init()
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    (void)RCC->APB1ENR;

    TIM3->PSC = (kSysClockHz / kTimerTickHz) - 1U;
    TIM3->ARR = 0xFFFFU;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 = TIM_CR1_CEN;
}

void delay_us(std::uint32_t us)
{
    while (us > 0U) {
        const std::uint32_t chunk = us > 60000U ? 60000U : us;
        TIM3->CNT = 0U;
        while (TIM3->CNT < chunk) {}
        us -= chunk;
    }
}

void delay_ms(std::uint32_t ms)
{
    while (ms-- > 0U) {
        delay_us(1000U);
    }
}

void beep(std::uint16_t frequency_hz, std::uint16_t duration_ms)
{
    const std::uint32_t half_period_us = 500000UL / frequency_hz;
    const std::uint32_t total_us = static_cast<std::uint32_t>(duration_ms) * 1000U;
    std::uint32_t elapsed_us = 0U;

    while ((elapsed_us + (half_period_us * 2U)) <= total_us) {
        pin_high(GPIOB, kBuzzer);
        delay_us(half_period_us);
        pin_low(GPIOB, kBuzzer);
        delay_us(half_period_us);
        elapsed_us += half_period_us * 2U;
    }

    pin_low(GPIOB, kBuzzer);
}

void rest(std::uint16_t ms)
{
    pin_low(GPIOB, kBuzzer);
    delay_ms(ms);
}

void marker(std::uint8_t count)
{
    rest(600U);
    for (std::uint8_t i = 0U; i < count; ++i) {
        beep(1047U, 90U);
        rest(110U);
    }
    rest(350U);
}

void gpio_init()
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
    (void)RCC->AHBENR;

    GPIOB->MODER &= ~((3UL << (0U * 2U)) |
                      (3UL << (4U * 2U)) |
                      (3UL << (11U * 2U)) |
                      (3UL << (12U * 2U)) |
                      (3UL << (14U * 2U)) |
                      (3UL << (15U * 2U)));
    GPIOB->MODER |= (1UL << (0U * 2U)) |
                    (1UL << (4U * 2U)) |
                    (1UL << (11U * 2U)) |
                    (1UL << (12U * 2U)) |
                    (1UL << (14U * 2U)) |
                    (1UL << (15U * 2U));
    GPIOB->OTYPER &= ~(kBuzzer | kAllCs);
    GPIOB->PUPDR &= ~((3UL << (0U * 2U)) |
                      (3UL << (4U * 2U)) |
                      (3UL << (11U * 2U)) |
                      (3UL << (12U * 2U)) |
                      (3UL << (14U * 2U)) |
                      (3UL << (15U * 2U)));
    GPIOB->OSPEEDR |= (3UL << (0U * 2U)) |
                      (3UL << (4U * 2U)) |
                      (3UL << (11U * 2U)) |
                      (3UL << (12U * 2U)) |
                      (3UL << (14U * 2U)) |
                      (3UL << (15U * 2U));

    GPIOA->MODER &= ~((3UL << (5U * 2U)) |
                      (3UL << (6U * 2U)) |
                      (3UL << (7U * 2U)));
    GPIOA->MODER |= (1UL << (5U * 2U)) |
                    (1UL << (6U * 2U)) |
                    (1UL << (7U * 2U));
    GPIOA->OTYPER &= ~(kSck | kMisoNet | kMosiNet);
    GPIOA->PUPDR &= ~((3UL << (5U * 2U)) |
                      (3UL << (6U * 2U)) |
                      (3UL << (7U * 2U)));
    GPIOA->OSPEEDR |= (3UL << (5U * 2U)) |
                      (3UL << (6U * 2U)) |
                      (3UL << (7U * 2U));

    pin_high(GPIOB, kAllCs);
    pin_low(GPIOB, kBuzzer);
    GPIOA->BRR = kSck | kMisoNet | kMosiNet;
}

void set_lines(bool sck, bool miso, bool mosi)
{
    pin_high(GPIOB, kAllCs);

    if (sck) {
        pin_high(GPIOA, kSck);
    } else {
        pin_low(GPIOA, kSck);
    }

    if (miso) {
        pin_high(GPIOA, kMisoNet);
    } else {
        pin_low(GPIOA, kMisoNet);
    }

    if (mosi) {
        pin_high(GPIOA, kMosiNet);
    } else {
        pin_low(GPIOA, kMosiNet);
    }
}

void hold_section(std::uint8_t count, bool sck, bool miso, bool mosi)
{
    set_lines(false, false, false);
    marker(count);
    set_lines(sck, miso, mosi);
    delay_ms(2500U);
    set_lines(false, false, false);
}

void toggle_section(std::uint8_t count, bool toggle_miso, bool toggle_mosi)
{
    set_lines(false, false, false);
    marker(count);

    for (std::uint16_t i = 0U; i < 80U; ++i) {
        set_lines(false, toggle_miso, toggle_mosi);
        delay_ms(25U);
        set_lines(false, false, false);
        delay_ms(25U);
    }
}

} // namespace

int main()
{
    clock_init();
    gpio_init();
    timebase_init();

    beep(523U, 100U);
    rest(60U);
    beep(784U, 100U);
    rest(60U);
    beep(1047U, 140U);

    while (true) {
        hold_section(1U, false, false, true); // PA7 / MOSI high, all CS high.
        hold_section(2U, false, true, false); // PA6 / MISO high, all CS high.
        hold_section(3U, false, true, true);  // Both data lines high, all CS high.
        hold_section(4U, true, false, false); // PA5 / SCK high, all CS high.
        toggle_section(5U, false, true);      // PA7 / MOSI square wave, all CS high.
        toggle_section(6U, true, false);      // PA6 / MISO square wave, all CS high.
        toggle_section(7U, true, true);       // Both data lines square wave, all CS high.
        rest(1200U);
    }
}
