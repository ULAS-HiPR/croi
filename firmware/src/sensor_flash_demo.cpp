#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"
#include "platform/stm_f0.h"

#include <Baro/MS5607.h>
#include <Flash/MX25L128.h>
#include <IMU/LSM6DSO32.h>
#include <SPI/SPI_STM.h>
#include <data.h>

#include <cstddef>
#include <cstdint>
#include <cstring>

void Error_Handler(void);

extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
}

#define BUZZER_GPIO_PORT GPIOB
#define BUZZER_PIN_MASK GPIO_BSRR_BS_0
#define BUZZER_PIN_RESET_MASK GPIO_BRR_BR_0

namespace {

constexpr std::uint32_t kTimerTickHz = 1000000U;
constexpr std::uint32_t kSysClockHz = 48000000U;
constexpr std::uint32_t kLogRegionSize = 64U * 1024U;
constexpr std::uint32_t kLogRegionStart = MX25_FLASH_SIZE - kLogRegionSize;
constexpr std::uint32_t kLogDurationMs = 5000U;
constexpr std::uint32_t kSamplePeriodMs = 50U;
constexpr std::uint32_t kBaroPeriodMs = 200U;
constexpr std::uint32_t kMaxRecords = kLogDurationMs / kSamplePeriodMs;

constexpr std::uint32_t kHeaderMagic = 0x43524F49U; // CROI
constexpr std::uint32_t kFooterMagic = 0x444F4E45U; // DONE
constexpr std::uint32_t kLogVersion = 1U;
constexpr std::uint32_t kFnvOffset = 2166136261UL;
constexpr std::uint32_t kFnvPrime = 16777619UL;

constexpr std::uint32_t kStatusFlashOk = 1U << 0U;
constexpr std::uint32_t kStatusImuOk = 1U << 1U;
constexpr std::uint32_t kStatusBaroOk = 1U << 2U;
constexpr std::uint32_t kStatusImuSampleOk = 1U << 8U;
constexpr std::uint32_t kStatusBaroSampleOk = 1U << 9U;

constexpr std::uint8_t kFailNone = 0U;
constexpr std::uint8_t kFailFlashInit = 1U;
constexpr std::uint8_t kFailImu = 2U;
constexpr std::uint8_t kFailBaro = 3U;
constexpr std::uint8_t kFailLogVerify = 4U;

void delay_ms_blocking(std::uint32_t ms);

#pragma pack(push, 1)
struct LogHeader {
    std::uint32_t magic;
    std::uint32_t version;
    std::uint32_t header_size;
    std::uint32_t record_size;
    std::uint32_t region_start;
    std::uint32_t region_size;
    std::uint32_t sample_period_ms;
    std::uint32_t duration_ms;
    std::uint32_t status_flags;
};

struct LogRecord {
    std::uint32_t sequence;
    std::uint32_t timestamp_ms;
    std::uint32_t status_flags;
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    std::int16_t gyro_x_dps;
    std::int16_t gyro_y_dps;
    std::int16_t gyro_z_dps;
    std::int16_t imu_temp_c;
    std::int32_t pressure_pa;
    float baro_temp_c;
    float altitude_m;
};

struct LogFooter {
    std::uint32_t magic;
    std::uint32_t record_count;
    std::uint32_t checksum;
    std::uint32_t status_flags;
};
#pragma pack(pop)

static_assert(sizeof(LogRecord) <= 64U, "LogRecord unexpectedly large");

float abs_float(float value)
{
    return value < 0.0f ? -value : value;
}

float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }

    if (value > maximum) {
        return maximum;
    }

    return value;
}

void spi_bit_delay()
{
    for (volatile std::uint32_t i = 0U; i < 24U; ++i) {
        __NOP();
    }
}

void restore_spi1_af_pins()
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    const std::uint32_t pins = SPI_SCK_PIN | SPI_MISO_PIN | SPI_MOSI_PIN;
    GPIOA->MODER &= ~((3UL << (5U * 2U)) |
                      (3UL << (6U * 2U)) |
                      (3UL << (7U * 2U)));
    GPIOA->MODER |= (2UL << (5U * 2U)) |
                    (2UL << (6U * 2U)) |
                    (2UL << (7U * 2U));
    GPIOA->OTYPER &= ~pins;
    GPIOA->PUPDR &= ~((3UL << (5U * 2U)) |
                      (3UL << (6U * 2U)) |
                      (3UL << (7U * 2U)));
    GPIOA->OSPEEDR |= (3UL << (5U * 2U)) |
                      (3UL << (6U * 2U)) |
                      (3UL << (7U * 2U));
    GPIOA->AFR[0] &= ~((0xFUL << (5U * 4U)) |
                       (0xFUL << (6U * 4U)) |
                       (0xFUL << (7U * 4U)));
}

void configure_flash_swapped_gpio()
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIOB->BSRR = BARO_CS_PIN | IMU_CS_PIN | BME680_CS_PIN | ADXL375_CS_PIN;

    GPIOA->MODER &= ~((3UL << (5U * 2U)) |
                      (3UL << (6U * 2U)) |
                      (3UL << (7U * 2U)));
    GPIOA->MODER |= (1UL << (5U * 2U)) |
                    (1UL << (6U * 2U));
    GPIOA->OTYPER &= ~(SPI_SCK_PIN | SPI_MISO_PIN | SPI_MOSI_PIN);
    GPIOA->PUPDR &= ~((3UL << (5U * 2U)) |
                      (3UL << (6U * 2U)) |
                      (3UL << (7U * 2U)));
    GPIOA->OSPEEDR |= (3UL << (5U * 2U)) |
                      (3UL << (6U * 2U)) |
                      (3UL << (7U * 2U));
    GPIOA->BRR = SPI_SCK_PIN | SPI_MISO_PIN;
}

class FlashSwappedSPI : public SPI_Handler {
public:
    FlashSwappedSPI(GPIO_TypeDef* cs_port, std::uint16_t cs_pin)
        : cs_port_(cs_port), cs_pin_(cs_pin)
    {
        cs_high();
    }

    bool write(int, std::uint8_t reg, std::uint8_t* buf, std::uint16_t len) override
    {
        cs_low();
        const bool ok = transmit(&reg, 1U) && transmit(buf, len);
        cs_high();
        return ok;
    }

    bool read(int, std::uint8_t reg, std::uint8_t* buf, std::uint16_t len) override
    {
        cs_low();
        const bool ok = read_no_cs(reg, buf, len);
        cs_high();
        return ok;
    }

    bool read_no_cs(std::uint8_t reg, std::uint8_t* buf, std::uint16_t len) override
    {
        const std::uint8_t addr = reg | 0x80U;
        return transmit(&addr, 1U) && receive(buf, len);
    }

    bool write_no_cs(std::uint8_t reg, const std::uint8_t* buf, std::uint16_t len) override
    {
        return transmit(&reg, 1U) && transmit(buf, len);
    }

    bool transmit(const std::uint8_t* data, std::size_t len) override
    {
        while (len-- > 0U) {
            clock_byte(*data++, nullptr);
        }
        return true;
    }

    bool receive(std::uint8_t* buf, std::size_t len) override
    {
        while (len-- > 0U) {
            std::uint8_t value = 0U;
            clock_byte(0x00U, &value);
            *buf++ = value;
        }
        return true;
    }

    bool transfer(const std::uint8_t* tx, std::uint8_t* rx, std::size_t len) override
    {
        while (len-- > 0U) {
            clock_byte(*tx++, rx++);
        }
        return true;
    }

    void cs_low() override
    {
        configure_flash_swapped_gpio();
        GPIOA->BRR = SPI_SCK_PIN | SPI_MISO_PIN;
        cs_port_->BRR = cs_pin_;
    }

    void cs_high() override
    {
        cs_port_->BSRR = cs_pin_;
        GPIOA->BRR = SPI_SCK_PIN | SPI_MISO_PIN;
        restore_spi1_af_pins();
    }

    void cs_select(int) override
    {
        cs_low();
    }

    void cs_deselect(int) override
    {
        cs_high();
    }

    void delay_ms(int ms) override
    {
        if (ms > 0) {
            delay_ms_blocking(static_cast<std::uint32_t>(ms));
        }
    }

private:
    void clock_byte(std::uint8_t tx, std::uint8_t* rx)
    {
        std::uint8_t in = 0U;

        for (std::uint8_t mask = 0x80U; mask != 0U; mask >>= 1U) {
            if ((tx & mask) != 0U) {
                GPIOA->BSRR = SPI_MISO_PIN;
            } else {
                GPIOA->BRR = SPI_MISO_PIN;
            }

            spi_bit_delay();
            GPIOA->BSRR = SPI_SCK_PIN;
            spi_bit_delay();

            if ((GPIOA->IDR & SPI_MOSI_PIN) != 0U) {
                in |= mask;
            }

            GPIOA->BRR = SPI_SCK_PIN;
            spi_bit_delay();
        }

        if (rx != nullptr) {
            *rx = in;
        }
    }

    GPIO_TypeDef* cs_port_;
    std::uint16_t cs_pin_;
};

std::uint32_t fnv1a_update(std::uint32_t hash, const void* data, std::size_t length)
{
    const std::uint8_t* bytes = static_cast<const std::uint8_t*>(data);

    for (std::size_t i = 0; i < length; ++i) {
        hash ^= bytes[i];
        hash *= kFnvPrime;
    }

    return hash;
}

void pb0_high()
{
    BUZZER_GPIO_PORT->BSRR = BUZZER_PIN_MASK;
}

void pb0_low()
{
    BUZZER_GPIO_PORT->BRR = BUZZER_PIN_RESET_MASK;
}

void early_clock_init()
{
    RCC->CR2 |= RCC_CR2_HSI48ON;
    while ((RCC->CR2 & RCC_CR2_HSI48RDY) == 0U) {}

    FLASH->ACR |= FLASH_ACR_LATENCY;

    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_HSI48;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI48) {}
}

void early_buzzer_gpio_init()
{
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    (void)RCC->AHBENR;

    GPIOB->MODER &= ~(3UL << (0U * 2U));
    GPIOB->MODER |= (1UL << (0U * 2U));
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT_0;
    GPIOB->PUPDR &= ~(3UL << (0U * 2U));
    GPIOB->OSPEEDR |= (3UL << (0U * 2U));

    pb0_low();
}

void early_timebase_init()
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

void delay_ms_blocking(std::uint32_t ms)
{
    while (ms-- > 0U) {
        delay_us(1000U);
    }
}

void tone_off()
{
    pb0_low();
}

void play_tone(std::uint16_t frequency_hz, std::uint16_t duration_ms)
{
    if (frequency_hz == 0U) {
        tone_off();
        delay_ms_blocking(duration_ms);
        return;
    }

    const std::uint32_t half_period_us = 500000UL / frequency_hz;
    const std::uint32_t total_us = static_cast<std::uint32_t>(duration_ms) * 1000U;
    const std::uint32_t period_us = half_period_us * 2U;
    std::uint32_t elapsed_us = 0U;

    while ((elapsed_us + period_us) <= total_us) {
        pb0_high();
        delay_us(half_period_us);
        pb0_low();
        delay_us(half_period_us);
        elapsed_us += period_us;
    }

    tone_off();

    if (elapsed_us < total_us) {
        delay_us(total_us - elapsed_us);
    }
}

void rest(std::uint16_t duration_ms)
{
    play_tone(0U, duration_ms);
}

void play_startup_chirp()
{
    play_tone(660U, 220U);
    rest(70U);
    play_tone(880U, 220U);
    rest(70U);
    play_tone(1175U, 260U);
    rest(60U);
}

void play_early_boot_marker()
{
    play_tone(523U, 140U);
    rest(45U);
    play_tone(784U, 140U);
    rest(45U);
    play_tone(1047U, 180U);
    rest(120U);
}

void play_spi_ready_chirp()
{
    play_tone(880U, 80U);
    rest(45U);
    play_tone(880U, 80U);
    rest(120U);
}

void play_pass_chirp(std::uint16_t base_hz)
{
    play_tone(base_hz, 45U);
    rest(25U);
    play_tone(static_cast<std::uint16_t>(base_hz + 180U), 55U);
    rest(70U);
}

void play_imu_fail_alarm()
{
    play_tone(247U, 120U);
    rest(70U);
    play_tone(185U, 180U);
    rest(150U);
}

void play_baro_fail_alarm()
{
    play_tone(294U, 80U);
    rest(60U);
    play_tone(247U, 80U);
    rest(60U);
    play_tone(220U, 160U);
    rest(150U);
}

void play_success_chime()
{
    play_tone(523U, 90U);
    rest(35U);
    play_tone(659U, 90U);
    rest(35U);
    play_tone(784U, 140U);
}

void play_failure_chime()
{
    play_tone(330U, 130U);
    rest(60U);
    play_tone(262U, 220U);
}

void play_failure_count(std::uint8_t count)
{
    rest(900U);

    for (std::uint8_t i = 0U; i < count; ++i) {
        play_tone(440U, 320U);
        rest(300U);
    }

    rest(1800U);
}

std::uint8_t failure_code(bool flash_ok, bool log_ok, bool imu_ok, bool baro_ok)
{
    if (!flash_ok) {
        return kFailFlashInit;
    }

    if (!imu_ok) {
        return kFailImu;
    }

    if (!baro_ok) {
        return kFailBaro;
    }

    if (!log_ok) {
        return kFailLogVerify;
    }

    return kFailNone;
}

void play_verdict_code(std::uint8_t code)
{
    if (code == kFailNone) {
        play_success_chime();
    } else {
        play_failure_count(code);
    }
}

void play_motion_pip(const imu_data& imu)
{
    const float activity =
        abs_float(imu.acceleration.x) +
        abs_float(imu.acceleration.y) +
        abs_float(imu.acceleration.z);
    const float clipped = clamp_float(activity, 0.0f, 8.0f);
    const auto frequency = static_cast<std::uint16_t>(420.0f + (clipped * 170.0f));
    play_tone(frequency, 12U);
}

void play_logging_heartbeat()
{
    play_tone(360U, 10U);
}

class FlashPageWriter {
public:
    FlashPageWriter(MX25L128& flash, std::uint32_t start, std::uint32_t capacity)
        : flash_(flash), start_(start), capacity_(capacity), address_(start)
    {
        std::memset(page_, 0xFF, sizeof(page_));
    }

    bool append(const void* data, std::size_t length)
    {
        const std::uint8_t* bytes = static_cast<const std::uint8_t*>(data);

        while (length > 0U) {
            if ((address_ - start_ + page_used_ + length) > capacity_) {
                ok_ = false;
                return false;
            }

            const std::size_t available = sizeof(page_) - page_used_;
            const std::size_t chunk = length < available ? length : available;
            std::memcpy(&page_[page_used_], bytes, chunk);
            page_used_ += chunk;
            bytes += chunk;
            length -= chunk;

            if (page_used_ == sizeof(page_)) {
                if (!flush()) {
                    return false;
                }
            }
        }

        return true;
    }

    bool flush()
    {
        if (page_used_ == 0U) {
            return ok_;
        }

        if (!flash_.write(address_, page_, page_used_)) {
            ok_ = false;
            return false;
        }

        address_ += static_cast<std::uint32_t>(page_used_);
        page_used_ = 0U;
        std::memset(page_, 0xFF, sizeof(page_));
        return true;
    }

    bool ok() const
    {
        return ok_;
    }

private:
    MX25L128& flash_;
    std::uint32_t start_;
    std::uint32_t capacity_;
    std::uint32_t address_;
    std::uint8_t page_[MX25_PAGE_SIZE];
    std::size_t page_used_{0U};
    bool ok_{true};
};

bool verify_log(MX25L128& flash,
                std::uint32_t expected_records,
                std::uint32_t expected_checksum,
                std::uint32_t expected_status)
{
    LogHeader header = {};
    if (!flash.read(kLogRegionStart, reinterpret_cast<std::uint8_t*>(&header), sizeof(header))) {
        return false;
    }

    if ((header.magic != kHeaderMagic) ||
        (header.version != kLogVersion) ||
        (header.header_size != sizeof(LogHeader)) ||
        (header.record_size != sizeof(LogRecord)) ||
        (header.region_start != kLogRegionStart) ||
        (header.region_size != kLogRegionSize)) {
        return false;
    }

    std::uint32_t checksum = kFnvOffset;
    std::uint32_t read_address = kLogRegionStart + sizeof(LogHeader);

    for (std::uint32_t index = 0U; index < expected_records; ++index) {
        LogRecord record = {};
        if (!flash.read(read_address, reinterpret_cast<std::uint8_t*>(&record), sizeof(record))) {
            return false;
        }

        checksum = fnv1a_update(checksum, &record, sizeof(record));
        read_address += sizeof(record);
    }

    LogFooter footer = {};
    if (!flash.read(read_address, reinterpret_cast<std::uint8_t*>(&footer), sizeof(footer))) {
        return false;
    }

    return (footer.magic == kFooterMagic) &&
           (footer.record_count == expected_records) &&
           (footer.checksum == expected_checksum) &&
           (footer.checksum == checksum) &&
           (footer.status_flags == expected_status);
}

} // namespace

int main()
{
    early_clock_init();
    SystemCoreClockUpdate();
    early_buzzer_gpio_init();
    early_timebase_init();
    play_early_boot_marker();

    HAL_Init();
    early_timebase_init();
    early_buzzer_gpio_init();

    play_startup_chirp();

    MX_SPI1_Init();
    play_spi_ready_chirp();

    FlashSwappedSPI flash_spi(FLASH_CS_PORT, FLASH_CS_PIN);
    SPI_STM imu_spi(&hspi1, IMU_CS_PORT, IMU_CS_PIN);
    SPI_STM baro_spi(&hspi1, BARO_CS_PORT, BARO_CS_PIN);

    MX25L128 flash(flash_spi);
    LSM6DSO32 imu(imu_spi);
    MS5607 baro(baro_spi);

    std::uint32_t status_flags = 0U;
    play_tone(494U, 90U);
    rest(40U);
    const bool flash_ok = flash.init();
    if (flash_ok) {
        status_flags |= kStatusFlashOk;
        play_pass_chirp(740U);
    } else {
        play_failure_chime();
    }

    const bool imu_ok = imu.init();
    if (imu_ok) {
        status_flags |= kStatusImuOk;
        play_pass_chirp(860U);
    } else {
        play_imu_fail_alarm();
    }

    const bool baro_ok = baro.init();
    if (baro_ok) {
        status_flags |= kStatusBaroOk;
        play_pass_chirp(980U);
    } else {
        play_baro_fail_alarm();
    }

    play_tone(440U, 90U);
    rest(45U);
    play_tone(330U, 90U);
    rest(80U);
    bool log_ok = flash_ok;
    if (log_ok) {
        log_ok = flash.erase(kLogRegionStart, kLogRegionSize);
        if (log_ok) {
            play_pass_chirp(620U);
        } else {
            play_failure_chime();
        }
    }

    FlashPageWriter writer(flash, kLogRegionStart, kLogRegionSize);

    const LogHeader header{
        kHeaderMagic,
        kLogVersion,
        sizeof(LogHeader),
        sizeof(LogRecord),
        kLogRegionStart,
        kLogRegionSize,
        kSamplePeriodMs,
        kLogDurationMs,
        status_flags,
    };

    log_ok = log_ok && writer.append(&header, sizeof(header));

    imu_data latest_imu = {};
    baro_data latest_baro = {};
    bool latest_baro_valid = false;
    std::uint32_t checksum = kFnvOffset;
    std::uint32_t record_count = 0U;
    std::uint32_t last_baro_ms = 0U;
    const std::uint32_t start_ms = HAL_GetTick();

    for (std::uint32_t sequence = 0U; sequence < kMaxRecords && log_ok; ++sequence) {
        const std::uint32_t sample_start_ms = HAL_GetTick();
        const std::uint32_t elapsed_ms = sample_start_ms - start_ms;
        std::uint32_t record_status = status_flags;

        if (imu_ok && imu.update(&latest_imu)) {
            record_status |= kStatusImuSampleOk;
        }

        if (baro_ok && ((sequence == 0U) || ((sample_start_ms - last_baro_ms) >= kBaroPeriodMs))) {
            if (baro.update(&latest_baro)) {
                latest_baro_valid = true;
            }
            last_baro_ms = sample_start_ms;
        }

        if (latest_baro_valid) {
            record_status |= kStatusBaroSampleOk;
        }

        const LogRecord record{
            sequence,
            elapsed_ms,
            record_status,
            latest_imu.acceleration.x,
            latest_imu.acceleration.y,
            latest_imu.acceleration.z,
            latest_imu.gyro.x,
            latest_imu.gyro.y,
            latest_imu.gyro.z,
            static_cast<std::int16_t>(latest_imu.temperature),
            latest_baro.pressure,
            latest_baro.temperature,
            latest_baro.altitude,
        };

        checksum = fnv1a_update(checksum, &record, sizeof(record));
        log_ok = writer.append(&record, sizeof(record));
        ++record_count;

        if ((sequence % 2U) == 0U) {
            if (imu_ok) {
                play_motion_pip(latest_imu);
            } else {
                play_logging_heartbeat();
            }
        }

        const std::uint32_t target_next_ms = sample_start_ms + kSamplePeriodMs;
        while ((static_cast<std::int32_t>(target_next_ms - HAL_GetTick()) > 0) && log_ok) {}
    }

    const LogFooter footer{
        kFooterMagic,
        record_count,
        checksum,
        status_flags,
    };

    log_ok = log_ok && writer.append(&footer, sizeof(footer));
    log_ok = log_ok && writer.flush();
    log_ok = log_ok && writer.ok();
    log_ok = log_ok && verify_log(flash, record_count, checksum, status_flags);

    const std::uint8_t final_code = failure_code(flash_ok, log_ok, imu_ok, baro_ok);
    const bool demo_ok = final_code == kFailNone;
    if (demo_ok) {
        play_success_chime();
    } else {
        play_failure_chime();
        play_verdict_code(final_code);
    }

    while (true) {
        if (demo_ok) {
            rest(900U);
            play_success_chime();
        } else {
            play_verdict_code(final_code);
        }
    }
}

void Error_Handler(void)
{
    __disable_irq();
    early_buzzer_gpio_init();
    early_timebase_init();
    while (true) {
        play_tone(165U, 250U);
        rest(120U);
    }
}
