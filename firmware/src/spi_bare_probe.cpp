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

constexpr std::uint32_t kW25q128Jedec = 0xEF4018U;
constexpr std::uint32_t kMx25l128Jedec = 0xC22018U;
constexpr std::uint8_t kLsm6dso32Whoami = 0x6CU;
constexpr std::uint8_t kBme680ChipId = 0x61U;
constexpr std::uint8_t kAdxl375DeviceId = 0xE5U;

enum class WireMode : std::uint8_t {
    Normal,
    Swapped,
};

enum class SpiMode : std::uint8_t {
    Mode0,
    Mode3,
};

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
    if (frequency_hz == 0U) {
        pin_low(GPIOB, kBuzzer);
        delay_ms(duration_ms);
        return;
    }

    const std::uint32_t half_period_us = 500000UL / frequency_hz;
    const std::uint32_t total_us = static_cast<std::uint32_t>(duration_ms) * 1000U;
    const std::uint32_t period_us = half_period_us * 2U;
    std::uint32_t elapsed_us = 0U;

    while ((elapsed_us + period_us) <= total_us) {
        pin_high(GPIOB, kBuzzer);
        delay_us(half_period_us);
        pin_low(GPIOB, kBuzzer);
        delay_us(half_period_us);
        elapsed_us += period_us;
    }

    pin_low(GPIOB, kBuzzer);
    if (elapsed_us < total_us) {
        delay_us(total_us - elapsed_us);
    }
}

void rest(std::uint16_t ms)
{
    beep(0U, ms);
}

void startup_chirp()
{
    beep(523U, 110U);
    rest(45U);
    beep(784U, 110U);
    rest(45U);
    beep(1047U, 150U);
    rest(300U);
}

void marker(std::uint8_t count)
{
    rest(650U);
    for (std::uint8_t i = 0U; i < count; ++i) {
        beep(1047U, 95U);
        rest(120U);
    }
    rest(350U);
}

void result(std::uint8_t count)
{
    rest(300U);
    for (std::uint8_t i = 0U; i < count; ++i) {
        beep(440U, 230U);
        rest(260U);
    }
    rest(850U);
}

void report(std::uint8_t section, std::uint8_t code)
{
    marker(section);
    result(code);
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

    pin_high(GPIOB, kAllCs);
    pin_low(GPIOB, kBuzzer);
}

std::uint16_t tx_pin(WireMode wire)
{
    return wire == WireMode::Normal ? kMosiNet : kMisoNet;
}

std::uint16_t rx_pin(WireMode wire)
{
    return wire == WireMode::Normal ? kMisoNet : kMosiNet;
}

void bus_config(WireMode wire, SpiMode mode)
{
    pin_high(GPIOB, kAllCs);

    const std::uint16_t out = static_cast<std::uint16_t>(kSck | tx_pin(wire));
    const std::uint16_t in = rx_pin(wire);
    (void)in;

    GPIOA->MODER &= ~((3UL << (5U * 2U)) |
                      (3UL << (6U * 2U)) |
                      (3UL << (7U * 2U)));
    GPIOA->MODER |= (1UL << (5U * 2U));
    if (wire == WireMode::Normal) {
        GPIOA->MODER |= (1UL << (7U * 2U)); // PA7 out, PA6 in.
    } else {
        GPIOA->MODER |= (1UL << (6U * 2U)); // PA6 out, PA7 in.
    }

    GPIOA->OTYPER &= ~(kSck | kMisoNet | kMosiNet);
    GPIOA->PUPDR &= ~((3UL << (5U * 2U)) |
                      (3UL << (6U * 2U)) |
                      (3UL << (7U * 2U)));
    GPIOA->OSPEEDR |= (3UL << (5U * 2U)) |
                      (3UL << (6U * 2U)) |
                      (3UL << (7U * 2U));

    GPIOA->BRR = out;
    if (mode == SpiMode::Mode3) {
        pin_high(GPIOA, kSck);
    } else {
        pin_low(GPIOA, kSck);
    }
}

void cs_low(std::uint16_t cs)
{
    pin_high(GPIOB, kAllCs);
    delay_us(30U);
    pin_low(GPIOB, cs);
    delay_us(30U);
}

void cs_high(std::uint16_t cs, SpiMode mode)
{
    delay_us(30U);
    pin_high(GPIOB, cs);
    if (mode == SpiMode::Mode3) {
        pin_high(GPIOA, kSck);
    } else {
        pin_low(GPIOA, kSck);
    }
    delay_us(30U);
}

void drive_data(WireMode wire, bool high)
{
    if (high) {
        pin_high(GPIOA, tx_pin(wire));
    } else {
        pin_low(GPIOA, tx_pin(wire));
    }
}

bool sample_data(WireMode wire)
{
    return (GPIOA->IDR & rx_pin(wire)) != 0U;
}

std::uint8_t spi_byte(WireMode wire, SpiMode mode, std::uint8_t tx)
{
    std::uint8_t rx = 0U;

    for (std::uint8_t mask = 0x80U; mask != 0U; mask >>= 1U) {
        drive_data(wire, (tx & mask) != 0U);
        delay_us(35U);

        if (mode == SpiMode::Mode3) {
            pin_low(GPIOA, kSck);
            delay_us(35U);
            pin_high(GPIOA, kSck);
            delay_us(35U);
        } else {
            pin_high(GPIOA, kSck);
            delay_us(35U);
        }

        if (sample_data(wire)) {
            rx |= mask;
        }

        if (mode == SpiMode::Mode0) {
            pin_low(GPIOA, kSck);
            delay_us(35U);
        }
    }

    return rx;
}

std::uint8_t read_register(std::uint16_t cs,
                           WireMode wire,
                           SpiMode mode,
                           std::uint8_t command,
                           bool take_second_dummy)
{
    bus_config(wire, mode);
    cs_low(cs);
    (void)spi_byte(wire, mode, command);
    const std::uint8_t first = spi_byte(wire, mode, 0x00U);
    const std::uint8_t second = take_second_dummy ? spi_byte(wire, mode, 0x00U) : first;
    cs_high(cs, mode);
    return take_second_dummy ? second : first;
}

std::uint32_t flash_jedec(WireMode wire, SpiMode mode)
{
    bus_config(wire, mode);
    cs_low(kFlashCs);
    (void)spi_byte(wire, mode, 0xABU);
    cs_high(kFlashCs, mode);
    delay_ms(2U);

    cs_low(kFlashCs);
    (void)spi_byte(wire, mode, 0x9FU);
    const std::uint8_t b0 = spi_byte(wire, mode, 0x00U);
    const std::uint8_t b1 = spi_byte(wire, mode, 0x00U);
    const std::uint8_t b2 = spi_byte(wire, mode, 0x00U);
    cs_high(kFlashCs, mode);

    return (static_cast<std::uint32_t>(b0) << 16U) |
           (static_cast<std::uint32_t>(b1) << 8U) |
           static_cast<std::uint32_t>(b2);
}

bool flash_expected(WireMode wire, SpiMode mode)
{
    const std::uint32_t id = flash_jedec(wire, mode);
    return id == kW25q128Jedec || id == kMx25l128Jedec;
}

bool flash_weird_nonzero(WireMode wire, SpiMode mode)
{
    const std::uint32_t id = flash_jedec(wire, mode);
    return id != 0x000000U && id != 0xFFFFFFU;
}

std::uint8_t mode_code(bool mode0_ok, bool mode3_ok)
{
    if (mode0_ok && mode3_ok) {
        return 4U;
    }
    if (mode0_ok) {
        return 2U;
    }
    if (mode3_ok) {
        return 3U;
    }
    return 1U;
}

std::uint8_t flash_code(WireMode wire)
{
    const bool mode0_ok = flash_expected(wire, SpiMode::Mode0);
    const bool mode3_ok = flash_expected(wire, SpiMode::Mode3);
    const std::uint8_t code = mode_code(mode0_ok, mode3_ok);
    if (code != 1U) {
        return code;
    }

    if (flash_weird_nonzero(wire, SpiMode::Mode0) ||
        flash_weird_nonzero(wire, SpiMode::Mode3)) {
        return 5U;
    }

    return 1U;
}

bool imu_visible(SpiMode mode)
{
    const std::uint8_t whoami = read_register(kImuCs, WireMode::Normal, mode, 0x8FU, false);
    return whoami == kLsm6dso32Whoami;
}

bool bme680_visible(SpiMode mode)
{
    const std::uint8_t id_1 = read_register(kBme680Cs, WireMode::Normal, mode, 0xD0U | 0x80U, false);
    const std::uint8_t id_2 = read_register(kBme680Cs, WireMode::Normal, mode, 0xD0U | 0x80U, true);
    return id_1 == kBme680ChipId || id_2 == kBme680ChipId;
}

bool adxl375_visible(SpiMode mode)
{
    const std::uint8_t devid = read_register(kAdxl375Cs, WireMode::Normal, mode, 0x80U, false);
    return devid == kAdxl375DeviceId;
}

std::uint16_t ms5607_prom_word(SpiMode mode, std::uint8_t index)
{
    bus_config(WireMode::Normal, mode);
    cs_low(kBaroCs);
    (void)spi_byte(WireMode::Normal, mode, static_cast<std::uint8_t>(0xA0U + (index * 2U)));
    const std::uint8_t high = spi_byte(WireMode::Normal, mode, 0x00U);
    const std::uint8_t low = spi_byte(WireMode::Normal, mode, 0x00U);
    cs_high(kBaroCs, mode);

    return static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(high) << 8U) |
        static_cast<std::uint16_t>(low));
}

std::uint8_t ms5607_crc4(const std::uint16_t* prom)
{
    std::uint16_t local_prom[8] = {};
    for (std::uint8_t i = 0U; i < 8U; ++i) {
        local_prom[i] = prom[i];
    }
    local_prom[7] = static_cast<std::uint16_t>(local_prom[7] & 0xFF00U);

    std::uint16_t remainder = 0U;
    constexpr std::uint16_t polynomial = 0x3000U;

    for (std::uint8_t index = 0U; index < 16U; ++index) {
        const std::uint8_t prom_index = index / 2U;
        if ((index & 1U) == 0U) {
            remainder = static_cast<std::uint16_t>(
                remainder ^ static_cast<std::uint16_t>(local_prom[prom_index] >> 8U));
        } else {
            remainder = static_cast<std::uint16_t>(
                remainder ^ static_cast<std::uint16_t>(local_prom[prom_index] & 0x00FFU));
        }

        for (std::uint8_t bit = 0U; bit < 8U; ++bit) {
            if ((remainder & 0x8000U) != 0U) {
                remainder = static_cast<std::uint16_t>(
                    static_cast<std::uint16_t>(remainder << 1U) ^ polynomial);
            } else {
                remainder = static_cast<std::uint16_t>(remainder << 1U);
            }
        }
    }

    return static_cast<std::uint8_t>((remainder >> 12U) & 0x0FU);
}

std::uint8_t ms5607_one_mode_code(SpiMode mode)
{
    bus_config(WireMode::Normal, mode);
    cs_low(kBaroCs);
    (void)spi_byte(WireMode::Normal, mode, 0x1EU);
    cs_high(kBaroCs, mode);
    delay_ms(4U);

    std::uint16_t prom[8] = {};
    std::uint8_t plausible = 0U;
    for (std::uint8_t i = 0U; i < 8U; ++i) {
        prom[i] = ms5607_prom_word(mode, i);
        if (prom[i] != 0x0000U && prom[i] != 0xFFFFU) {
            ++plausible;
        }
    }

    if (plausible < 4U) {
        return 0U;
    }

    const std::uint8_t stored_crc = static_cast<std::uint8_t>(prom[7] & 0x0FU);
    return ms5607_crc4(prom) == stored_crc ? 2U : 1U;
}

std::uint8_t ms5607_code()
{
    const std::uint8_t mode0 = ms5607_one_mode_code(SpiMode::Mode0);
    const std::uint8_t mode3 = ms5607_one_mode_code(SpiMode::Mode3);

    if (mode0 == 2U && mode3 == 2U) {
        return 4U;
    }
    if (mode0 == 2U) {
        return 2U;
    }
    if (mode3 == 2U) {
        return 3U;
    }
    if (mode0 != 0U || mode3 != 0U) {
        return 5U;
    }
    return 1U;
}

void scope_burst(std::uint8_t section, std::uint16_t cs, WireMode wire)
{
    marker(section);
    bus_config(wire, SpiMode::Mode0);

    for (std::uint8_t burst = 0U; burst < 8U; ++burst) {
        cs_low(cs);
        for (std::uint8_t i = 0U; i < 48U; ++i) {
            (void)spi_byte(wire, SpiMode::Mode0, (i & 1U) == 0U ? 0xA5U : 0x5AU);
        }
        cs_high(cs, SpiMode::Mode0);
        delay_ms(120U);
    }
}

} // namespace

int main()
{
    clock_init();
    gpio_init();
    timebase_init();
    startup_chirp();

    while (true) {
        report(1U, flash_code(WireMode::Normal));
        report(2U, flash_code(WireMode::Swapped));
        report(3U, mode_code(imu_visible(SpiMode::Mode0), imu_visible(SpiMode::Mode3)));
        report(4U, ms5607_code());
        report(5U, mode_code(bme680_visible(SpiMode::Mode0), bme680_visible(SpiMode::Mode3)));
        report(6U, mode_code(adxl375_visible(SpiMode::Mode0), adxl375_visible(SpiMode::Mode3)));
        rest(1800U);

        scope_burst(1U, kFlashCs, WireMode::Normal);
        scope_burst(2U, kFlashCs, WireMode::Swapped);
        scope_burst(3U, kImuCs, WireMode::Normal);
        scope_burst(4U, kBaroCs, WireMode::Normal);
        scope_burst(5U, kBme680Cs, WireMode::Normal);
        scope_burst(6U, kAdxl375Cs, WireMode::Normal);
        rest(2200U);
    }
}
