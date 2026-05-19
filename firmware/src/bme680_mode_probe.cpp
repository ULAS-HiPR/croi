#include "stm32f0xx.h"

#include <cstdint>

namespace {

constexpr std::uint32_t kSysClockHz = 48000000U;
constexpr std::uint32_t kTimerTickHz = 1000000U;

constexpr std::uint8_t kSckIndex = 5U;       // PA5
constexpr std::uint8_t kMisoIndex = 6U;      // PA6
constexpr std::uint8_t kMosiIndex = 7U;      // PA7
constexpr std::uint16_t kSck = 1U << kSckIndex;
constexpr std::uint16_t kMisoNet = 1U << kMisoIndex;
constexpr std::uint16_t kMosiNet = 1U << kMosiIndex;

constexpr std::uint16_t kBuzzer = 1U << 0U;    // PB0
constexpr std::uint16_t kBaroCs = 1U << 4U;    // PB4
constexpr std::uint16_t kImuCs = 1U << 11U;    // PB11
constexpr std::uint16_t kFlashCs = 1U << 12U;  // PB12
constexpr std::uint16_t kBme680Cs = 1U << 14U; // PB14
constexpr std::uint16_t kAdxl375Cs = 1U << 15U;// PB15
constexpr std::uint16_t kAllCs = kBaroCs | kImuCs | kFlashCs | kBme680Cs | kAdxl375Cs;

enum class TestResult : std::uint8_t {
    Pass = 1U,
    FailsHigh = 2U,
    FailsLow = 3U,
    BothBad = 4U,
};

void pin_high(GPIO_TypeDef* port, std::uint16_t pin)
{
    port->BSRR = pin;
}

void pin_low(GPIO_TypeDef* port, std::uint16_t pin)
{
    port->BRR = pin;
}

bool pin_reads_high(GPIO_TypeDef* port, std::uint16_t pin)
{
    return (port->IDR & pin) != 0U;
}

void bme_cs_earliest_low()
{
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    (void)RCC->AHBENR;

    GPIOB->MODER &= ~(3UL << (14U * 2U));
    GPIOB->MODER |= 1UL << (14U * 2U);
    GPIOB->OTYPER &= ~kBme680Cs;
    GPIOB->PUPDR &= ~(3UL << (14U * 2U));
    GPIOB->OSPEEDR |= 3UL << (14U * 2U);
    pin_low(GPIOB, kBme680Cs);
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

void chirp_start()
{
    beep(523U, 80U);
    rest(45U);
    beep(784U, 80U);
    rest(45U);
    beep(1047U, 120U);
    rest(500U);
}

void phase_sound(bool bme_low)
{
    if (bme_low) {
        beep(330U, 120U);
        rest(60U);
        beep(330U, 120U);
    } else {
        beep(988U, 120U);
        rest(60U);
        beep(988U, 120U);
    }
    rest(350U);
}

void marker(std::uint8_t count)
{
    rest(550U);
    for (std::uint8_t i = 0U; i < count; ++i) {
        beep(1175U, 70U);
        rest(90U);
    }
    rest(240U);
}

void pass_sound()
{
    beep(880U, 90U);
    rest(45U);
    beep(1320U, 110U);
}

void skip_sound()
{
    beep(660U, 320U);
}

void fail_sound(TestResult result)
{
    const std::uint8_t count = static_cast<std::uint8_t>(result);
    for (std::uint8_t i = 0U; i < count; ++i) {
        beep(220U, 160U);
        rest(140U);
    }
}

void report(std::uint8_t section, TestResult result)
{
    marker(section);
    if (result == TestResult::Pass) {
        pass_sound();
    } else {
        fail_sound(result);
    }
    rest(500U);
}

void report_skip(std::uint8_t section)
{
    marker(section);
    skip_sound();
    rest(500U);
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
    GPIOA->PUPDR &= ~((3UL << (5U * 2U)) |
                      (3UL << (6U * 2U)) |
                      (3UL << (7U * 2U)));
    GPIOA->OTYPER &= ~(kSck | kMisoNet | kMosiNet);
    GPIOA->OSPEEDR |= (3UL << (5U * 2U)) |
                      (3UL << (6U * 2U)) |
                      (3UL << (7U * 2U));

    pin_high(GPIOB, kAllCs);
    pin_low(GPIOB, kBuzzer);
    pin_low(GPIOA, kSck | kMisoNet | kMosiNet);
}

void configure_pa_output(std::uint8_t pin_index)
{
    const std::uint32_t shift = static_cast<std::uint32_t>(pin_index) * 2U;
    GPIOA->MODER &= ~(3UL << shift);
    GPIOA->MODER |= 1UL << shift;
    GPIOA->OTYPER &= ~(1U << pin_index);
    GPIOA->PUPDR &= ~(3UL << shift);
    GPIOA->OSPEEDR |= 3UL << shift;
}

void configure_pa_input(std::uint8_t pin_index, std::uint8_t pull)
{
    const std::uint32_t shift = static_cast<std::uint32_t>(pin_index) * 2U;
    GPIOA->MODER &= ~(3UL << shift);
    GPIOA->PUPDR &= ~(3UL << shift);
    GPIOA->PUPDR |= static_cast<std::uint32_t>(pull) << shift;
}

void release_data_lines()
{
    configure_pa_input(kMosiIndex, 0U);
    configure_pa_input(kMisoIndex, 0U);
    pin_low(GPIOA, kSck);
}

void set_bme_condition(bool bme_low)
{
    pin_high(GPIOB, kAllCs);
    pin_low(GPIOA, kSck);
    if (bme_low) {
        pin_low(GPIOB, kBme680Cs);
    }
    delay_ms(30U);
}

TestResult weak_pull_test(std::uint8_t pin_index, std::uint16_t pin_mask, bool bme_low)
{
    release_data_lines();
    set_bme_condition(bme_low);

    configure_pa_input(pin_index, 2U); // Pull-down.
    delay_ms(30U);
    const bool low_ok = !pin_reads_high(GPIOA, pin_mask);

    configure_pa_input(pin_index, 1U); // Pull-up.
    delay_ms(30U);
    const bool high_ok = pin_reads_high(GPIOA, pin_mask);

    configure_pa_input(pin_index, 0U);
    pin_high(GPIOB, kBme680Cs);

    if (low_ok && high_ok) {
        return TestResult::Pass;
    }
    if (low_ok && !high_ok) {
        return TestResult::FailsHigh;
    }
    if (!low_ok && high_ok) {
        return TestResult::FailsLow;
    }
    return TestResult::BothBad;
}

TestResult drive_read_test(std::uint8_t pin_index, std::uint16_t pin_mask, bool bme_low)
{
    release_data_lines();
    set_bme_condition(bme_low);
    configure_pa_output(pin_index);

    pin_low(GPIOA, pin_mask);
    delay_ms(20U);
    const bool low_ok = !pin_reads_high(GPIOA, pin_mask);

    pin_high(GPIOA, pin_mask);
    delay_ms(20U);
    const bool high_ok = pin_reads_high(GPIOA, pin_mask);

    pin_low(GPIOA, pin_mask);
    configure_pa_input(pin_index, 0U);
    pin_high(GPIOB, kBme680Cs);

    if (low_ok && high_ok) {
        return TestResult::Pass;
    }
    if (low_ok && !high_ok) {
        return TestResult::FailsHigh;
    }
    if (!low_ok && high_ok) {
        return TestResult::FailsLow;
    }
    return TestResult::BothBad;
}

void run_phase(bool bme_low)
{
    phase_sound(bme_low);

    report(1U, weak_pull_test(kMosiIndex, kMosiNet, bme_low));
    report(2U, weak_pull_test(kMisoIndex, kMisoNet, bme_low));
    report(3U, drive_read_test(kMosiIndex, kMosiNet, bme_low));

    if (bme_low) {
        // BME SDO may be active while selected, so do not hard-drive PA6 here.
        report_skip(4U);
    } else {
        report(4U, drive_read_test(kMisoIndex, kMisoNet, bme_low));
    }
}

} // namespace

int main()
{
#ifdef BME680_CS_BOOT_LOW
    bme_cs_earliest_low();
#endif

    clock_init();
    timebase_init();

#ifdef BME680_CS_BOOT_LOW
    delay_ms(250U);
#endif

    gpio_init();
    chirp_start();

    while (true) {
        run_phase(false); // Two high tones: BME CS high.
        rest(900U);
        run_phase(true);  // Two low tones: BME CS low.
        rest(1800U);
    }
}
