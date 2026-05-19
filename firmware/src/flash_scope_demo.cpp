#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"

#include <cstdint>

SPI_HandleTypeDef hspi1;

extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
}

namespace {

constexpr std::uint32_t kSysClockHz = 48000000U;
constexpr std::uint32_t kTimerTickHz = 1000000U;

constexpr std::uint32_t kSck = GPIO_BSRR_BS_5;
constexpr std::uint32_t kMisoNet = GPIO_BSRR_BS_6;
constexpr std::uint32_t kMosiNet = GPIO_BSRR_BS_7;
constexpr std::uint32_t kSckReset = GPIO_BRR_BR_5;
constexpr std::uint32_t kMisoNetReset = GPIO_BRR_BR_6;
constexpr std::uint32_t kMosiNetReset = GPIO_BRR_BR_7;

constexpr std::uint32_t kBuzzerSet = GPIO_BSRR_BS_0;
constexpr std::uint32_t kBuzzerReset = GPIO_BRR_BR_0;

constexpr std::uint16_t kBaroCs = GPIO_PIN_4;
constexpr std::uint16_t kImuCs = GPIO_PIN_11;
constexpr std::uint16_t kFlashCs = GPIO_PIN_12;
constexpr std::uint16_t kBme680Cs = GPIO_PIN_14;
constexpr std::uint16_t kAdxl375Cs = GPIO_PIN_15;

constexpr std::uint32_t kW25q128Jedec = 0xEF4018U;
constexpr std::uint32_t kMx25l128Jedec = 0xC22018U;
constexpr std::uint8_t kLsm6dso32Whoami = 0x6CU;
constexpr std::uint8_t kBme680ChipId = 0x61U;
constexpr std::uint8_t kAdxl375DeviceId = 0xE5U;

enum class FlashMode {
    Normal,
    Swapped,
};

enum class SpiClockMode {
    Mode0,
    Mode3,
};

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

void buzzer_low()
{
    GPIOB->BRR = kBuzzerReset;
}

void buzzer_high()
{
    GPIOB->BSRR = kBuzzerSet;
}

void play_tone(std::uint16_t frequency_hz, std::uint16_t duration_ms)
{
    if (frequency_hz == 0U) {
        buzzer_low();
        delay_ms(duration_ms);
        return;
    }

    const std::uint32_t half_period_us = 500000UL / frequency_hz;
    const std::uint32_t total_us = static_cast<std::uint32_t>(duration_ms) * 1000U;
    const std::uint32_t period_us = half_period_us * 2U;
    std::uint32_t elapsed_us = 0U;

    while ((elapsed_us + period_us) <= total_us) {
        buzzer_high();
        delay_us(half_period_us);
        buzzer_low();
        delay_us(half_period_us);
        elapsed_us += period_us;
    }

    buzzer_low();
    if (elapsed_us < total_us) {
        delay_us(total_us - elapsed_us);
    }
}

void rest(std::uint16_t ms)
{
    play_tone(0U, ms);
}

void beep_count(std::uint8_t count)
{
    rest(450U);
    for (std::uint8_t i = 0U; i < count; ++i) {
        play_tone(440U, 260U);
        rest(280U);
    }
    rest(750U);
}

void marker_count(std::uint8_t count)
{
    rest(650U);
    for (std::uint8_t i = 0U; i < count; ++i) {
        play_tone(1047U, 110U);
        rest(120U);
    }
    rest(350U);
}

void report_section(std::uint8_t marker, std::uint8_t result)
{
    marker_count(marker);
    beep_count(result);
}

void startup_chirp()
{
    play_tone(523U, 120U);
    rest(45U);
    play_tone(784U, 120U);
    rest(45U);
    play_tone(1047U, 160U);
    rest(200U);
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
    GPIOB->OTYPER &= ~(GPIO_PIN_0 | kBaroCs | kImuCs | kFlashCs | kBme680Cs | kAdxl375Cs);
    GPIOB->OSPEEDR |= (3UL << (0U * 2U)) |
                      (3UL << (4U * 2U)) |
                      (3UL << (11U * 2U)) |
                      (3UL << (12U * 2U)) |
                      (3UL << (14U * 2U)) |
                      (3UL << (15U * 2U));
    GPIOB->PUPDR &= ~((3UL << (0U * 2U)) |
                      (3UL << (4U * 2U)) |
                      (3UL << (11U * 2U)) |
                      (3UL << (12U * 2U)) |
                      (3UL << (14U * 2U)) |
                      (3UL << (15U * 2U)));

    GPIOB->BSRR = kBaroCs | kImuCs | kFlashCs | kBme680Cs | kAdxl375Cs;
    buzzer_low();
}

void spi1_gpio_af_init()
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    (void)RCC->AHBENR;

    GPIOA->MODER &= ~((3UL << (5U * 2U)) |
                      (3UL << (6U * 2U)) |
                      (3UL << (7U * 2U)));
    GPIOA->MODER |= (2UL << (5U * 2U)) |
                    (2UL << (6U * 2U)) |
                    (2UL << (7U * 2U));
    GPIOA->OTYPER &= ~(GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
    GPIOA->OSPEEDR |= (3UL << (5U * 2U)) |
                      (3UL << (6U * 2U)) |
                      (3UL << (7U * 2U));
    GPIOA->PUPDR &= ~((3UL << (5U * 2U)) |
                      (3UL << (6U * 2U)) |
                      (3UL << (7U * 2U)));
    GPIOA->AFR[0] &= ~((0xFUL << (5U * 4U)) |
                       (0xFUL << (6U * 4U)) |
                       (0xFUL << (7U * 4U)));
}

bool spi1_init(SpiClockMode mode)
{
    __HAL_RCC_SPI1_CLK_ENABLE();
    spi1_gpio_af_init();

    hspi1.Instance = SPI1;
    HAL_SPI_DeInit(&hspi1);

    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = mode == SpiClockMode::Mode3
        ? SPI_POLARITY_HIGH
        : SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = mode == SpiClockMode::Mode3
        ? SPI_PHASE_2EDGE
        : SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;

    return HAL_SPI_Init(&hspi1) == HAL_OK;
}

bool spi1_transfer(const std::uint8_t* tx, std::uint8_t* rx, std::uint16_t len)
{
    return HAL_SPI_TransmitReceive(
        &hspi1,
        const_cast<std::uint8_t*>(tx),
        rx,
        len,
        20U) == HAL_OK;
}

void configure_spi_gpio(FlashMode mode)
{
    GPIOB->BSRR = kBaroCs | kImuCs | kFlashCs | kBme680Cs | kAdxl375Cs;

    GPIOA->MODER &= ~((3UL << (5U * 2U)) |
                      (3UL << (6U * 2U)) |
                      (3UL << (7U * 2U)));
    GPIOA->MODER |= (1UL << (5U * 2U));

    if (mode == FlashMode::Normal) {
        GPIOA->MODER |= (1UL << (7U * 2U)); // PA7 drives board SPI_MOSI.
    } else {
        GPIOA->MODER |= (1UL << (6U * 2U)); // PA6 drives board SPI_MISO, flash pin 5 on this PCB.
    }

    GPIOA->OTYPER &= ~(GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
    GPIOA->OSPEEDR |= (3UL << (5U * 2U)) |
                      (3UL << (6U * 2U)) |
                      (3UL << (7U * 2U));
    GPIOA->PUPDR &= ~((3UL << (5U * 2U)) |
                      (3UL << (6U * 2U)) |
                      (3UL << (7U * 2U)));
    GPIOA->BRR = kSckReset | kMisoNetReset | kMosiNetReset;
}

void cs_low()
{
    GPIOB->BRR = GPIO_BRR_BR_12;
    delay_us(5U);
}

void cs_high()
{
    delay_us(5U);
    GPIOB->BSRR = kFlashCs;
    GPIOA->BRR = kSckReset | kMisoNetReset | kMosiNetReset;
}

void drive_out(FlashMode mode, bool high)
{
    if (mode == FlashMode::Normal) {
        if (high) {
            GPIOA->BSRR = kMosiNet;
        } else {
            GPIOA->BRR = kMosiNetReset;
        }
    } else {
        if (high) {
            GPIOA->BSRR = kMisoNet;
        } else {
            GPIOA->BRR = kMisoNetReset;
        }
    }
}

bool read_in(FlashMode mode)
{
    if (mode == FlashMode::Normal) {
        return (GPIOA->IDR & GPIO_PIN_6) != 0U;
    }

    return (GPIOA->IDR & GPIO_PIN_7) != 0U;
}

std::uint8_t spi_byte(FlashMode mode, std::uint8_t tx)
{
    std::uint8_t rx = 0U;

    for (std::uint8_t mask = 0x80U; mask != 0U; mask >>= 1U) {
        drive_out(mode, (tx & mask) != 0U);
        delay_us(8U);
        GPIOA->BSRR = kSck;
        delay_us(8U);

        if (read_in(mode)) {
            rx |= mask;
        }

        GPIOA->BRR = kSckReset;
        delay_us(8U);
    }

    return rx;
}

void wake_flash(FlashMode mode)
{
    configure_spi_gpio(mode);
    cs_low();
    spi_byte(mode, 0xABU);
    cs_high();
    delay_ms(2U);
}

std::uint32_t read_jedec(FlashMode mode)
{
    configure_spi_gpio(mode);
    cs_low();
    spi_byte(mode, 0x9FU);
    const std::uint8_t b0 = spi_byte(mode, 0x00U);
    const std::uint8_t b1 = spi_byte(mode, 0x00U);
    const std::uint8_t b2 = spi_byte(mode, 0x00U);
    cs_high();

    return (static_cast<std::uint32_t>(b0) << 16U) |
           (static_cast<std::uint32_t>(b1) << 8U) |
           static_cast<std::uint32_t>(b2);
}

bool expected_jedec(std::uint32_t id)
{
    return id == kW25q128Jedec || id == kMx25l128Jedec;
}

bool interesting_jedec(std::uint32_t id)
{
    return id != 0x000000U && id != 0xFFFFFFU;
}

std::uint8_t classify(std::uint32_t normal_id, std::uint32_t swapped_id)
{
    if (expected_jedec(normal_id)) {
        return 2U;
    }

    if (expected_jedec(swapped_id)) {
        return 3U;
    }

    const bool normal_interesting = interesting_jedec(normal_id);
    const bool swapped_interesting = interesting_jedec(swapped_id);

    if (normal_interesting && swapped_interesting) {
        return 6U;
    }

    if (normal_interesting) {
        return 4U;
    }

    if (swapped_interesting) {
        return 5U;
    }

    return 1U;
}

bool hw_read_bytes(SpiClockMode mode,
                   std::uint16_t cs_pin,
                   const std::uint8_t* tx,
                   std::uint8_t* rx,
                   std::uint16_t len)
{
    if (!spi1_init(mode)) {
        return false;
    }

    GPIOB->BSRR = kBaroCs | kImuCs | kFlashCs | kBme680Cs | kAdxl375Cs;
    delay_us(5U);
    GPIOB->BRR = cs_pin;
    delay_us(5U);
    const bool ok = spi1_transfer(tx, rx, len);
    delay_us(5U);
    GPIOB->BSRR = cs_pin;
    delay_us(5U);
    return ok;
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

bool imu_visible(SpiClockMode mode)
{
    const std::uint8_t tx[2] = {0x8FU, 0x00U};
    std::uint8_t rx[2] = {};
    return hw_read_bytes(mode, kImuCs, tx, rx, sizeof(tx)) && rx[1] == kLsm6dso32Whoami;
}

std::uint8_t imu_code()
{
    return mode_code(imu_visible(SpiClockMode::Mode0), imu_visible(SpiClockMode::Mode3));
}

std::uint16_t ms5607_prom_word(SpiClockMode mode, std::uint8_t index)
{
    const std::uint8_t tx[3] = {
        static_cast<std::uint8_t>(0xA0U + (index * 2U)),
        0x00U,
        0x00U,
    };
    std::uint8_t rx[3] = {};
    if (!hw_read_bytes(mode, kBaroCs, tx, rx, sizeof(tx))) {
        return 0U;
    }

    return static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(rx[1]) << 8U) |
        static_cast<std::uint16_t>(rx[2]));
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

bool ms5607_visible(SpiClockMode mode, bool* crc_ok)
{
    const std::uint8_t reset_tx[1] = {0x1EU};
    std::uint8_t reset_rx[1] = {};
    (void)hw_read_bytes(mode, kBaroCs, reset_tx, reset_rx, sizeof(reset_tx));
    delay_ms(3U);

    std::uint16_t prom[8] = {};
    std::uint8_t plausible_words = 0U;
    for (std::uint8_t i = 0U; i < 8U; ++i) {
        prom[i] = ms5607_prom_word(mode, i);
        if (prom[i] != 0x0000U && prom[i] != 0xFFFFU) {
            ++plausible_words;
        }
    }

    if (plausible_words < 4U) {
        if (crc_ok != nullptr) {
            *crc_ok = false;
        }
        return false;
    }

    const std::uint8_t stored_crc = static_cast<std::uint8_t>(prom[7] & 0x0FU);
    if (crc_ok != nullptr) {
        *crc_ok = ms5607_crc4(prom) == stored_crc;
    }
    return true;
}

std::uint8_t ms5607_code()
{
    bool crc0 = false;
    bool crc3 = false;
    const bool mode0_ok = ms5607_visible(SpiClockMode::Mode0, &crc0);
    const bool mode3_ok = ms5607_visible(SpiClockMode::Mode3, &crc3);

    if ((mode0_ok && crc0) && (mode3_ok && crc3)) {
        return 4U;
    }

    if (mode0_ok && crc0) {
        return 2U;
    }

    if (mode3_ok && crc3) {
        return 3U;
    }

    if (mode0_ok || mode3_ok) {
        return 5U;
    }

    return 1U;
}

bool bme680_visible(SpiClockMode mode)
{
    const std::uint8_t tx[3] = {static_cast<std::uint8_t>(0xD0U | 0x80U), 0x00U, 0x00U};
    std::uint8_t rx[3] = {};
    return hw_read_bytes(mode, kBme680Cs, tx, rx, sizeof(tx)) &&
           (rx[1] == kBme680ChipId || rx[2] == kBme680ChipId);
}

std::uint8_t bme680_code()
{
    return mode_code(bme680_visible(SpiClockMode::Mode0), bme680_visible(SpiClockMode::Mode3));
}

bool adxl375_visible(SpiClockMode mode)
{
    const std::uint8_t tx[2] = {0x80U, 0x00U};
    std::uint8_t rx[2] = {};
    return hw_read_bytes(mode, kAdxl375Cs, tx, rx, sizeof(tx)) && rx[1] == kAdxl375DeviceId;
}

std::uint8_t adxl375_code()
{
    return mode_code(adxl375_visible(SpiClockMode::Mode0), adxl375_visible(SpiClockMode::Mode3));
}

void hardware_spi_scope_burst(std::uint16_t cs_pin)
{
    static constexpr std::uint16_t kBurstLength = 96U;
    std::uint8_t tx[kBurstLength] = {};
    std::uint8_t rx[kBurstLength] = {};

    for (std::uint16_t i = 0U; i < kBurstLength; ++i) {
        tx[i] = (i & 1U) == 0U ? 0xA5U : 0x5AU;
    }

    (void)spi1_init(SpiClockMode::Mode0);
    GPIOB->BSRR = kBaroCs | kImuCs | kFlashCs | kBme680Cs | kAdxl375Cs;
    delay_ms(1U);
    GPIOB->BRR = cs_pin;
    delay_ms(1U);
    (void)spi1_transfer(tx, rx, kBurstLength);
    delay_ms(1U);
    GPIOB->BSRR = cs_pin;
    delay_ms(120U);
}

void scope_section(std::uint8_t marker, std::uint16_t cs_pin)
{
    marker_count(marker);
    for (std::uint8_t i = 0U; i < 10U; ++i) {
        hardware_spi_scope_burst(cs_pin);
    }
}

} // namespace

int main()
{
    clock_init();
    SystemCoreClockUpdate();
    HAL_Init();
    gpio_init();
    timebase_init();
    startup_chirp();

    wake_flash(FlashMode::Normal);
    wake_flash(FlashMode::Swapped);

    const std::uint32_t normal_id = read_jedec(FlashMode::Normal);
    const std::uint32_t swapped_id = read_jedec(FlashMode::Swapped);

    while (true) {
        report_section(1U, classify(normal_id, swapped_id));
        report_section(2U, imu_code());
        report_section(3U, ms5607_code());
        report_section(4U, bme680_code());
        report_section(5U, adxl375_code());
        rest(1800U);

        scope_section(1U, kFlashCs);
        scope_section(2U, kImuCs);
        scope_section(3U, kBaroCs);
        scope_section(4U, kBme680Cs);
        scope_section(5U, kAdxl375Cs);
        rest(2200U);
    }
}
