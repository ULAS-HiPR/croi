#include "stm32f0xx_hal.h"
#include "platform/stm_f0.h"

#include <Baro/MS5607.h>
#include <Flash/MX25L128.h>
#include <IMU/LSM6DSO32.h>
#include <SPI/SPI_STM.h>
#include <data.h>

#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

UART_HandleTypeDef huart1;

extern "C" volatile std::uint32_t g_csv_ready = 0U;
extern "C" volatile std::uint32_t g_csv_len = 0U;
extern "C" volatile std::uint32_t g_csv_error = 0U;
extern "C" char g_csv_dump[12U * 1024U] = {};

extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
}

namespace {

constexpr std::uint32_t kLogStart = MX25_FLASH_SIZE - (32U * 1024U);
constexpr std::uint32_t kLogSize = 32U * 1024U;
constexpr std::uint32_t kSampleMs = 100U;
constexpr std::uint32_t kDurationMs = 5000U;
constexpr std::uint32_t kDumpDelayMs = 5000U;

void SystemClock_Config();
void MX_USART1_UART_Init();

void buzzer_off()
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {};
    gpio.Pin = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}

void uart_write(const char* text, std::size_t len)
{
    if (len == 0U) {
        return;
    }
    (void)HAL_UART_Transmit(&huart1,
                            reinterpret_cast<std::uint8_t*>(const_cast<char*>(text)),
                            static_cast<std::uint16_t>(len),
                            HAL_MAX_DELAY);
}

void uart_write(const char* text)
{
    uart_write(text, std::strlen(text));
}

void uart_printf(const char* fmt, ...)
{
    char buffer[192];
    va_list args;
    va_start(args, fmt);
    const int used = std::vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (used <= 0) {
        return;
    }

    const std::size_t len = used < static_cast<int>(sizeof(buffer))
        ? static_cast<std::size_t>(used)
        : sizeof(buffer) - 1U;
    uart_write(buffer, len);
}

std::int32_t to_scaled(float value, float scale)
{
    const float scaled = value * scale;
    return static_cast<std::int32_t>(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
}

class CsvFlashWriter {
public:
    CsvFlashWriter(MX25L128& flash, std::uint32_t start, std::uint32_t size)
        : flash_(flash), start_(start), end_(start + size), address_(start)
    {
        std::memset(page_, 0xFF, sizeof(page_));
    }

    bool append(const char* data)
    {
        return append(data, std::strlen(data));
    }

    bool append(const char* data, std::size_t len)
    {
        while (len > 0U) {
            if ((address_ + page_used_) >= end_) {
                ok_ = false;
                return false;
            }

            const std::size_t room = sizeof(page_) - page_used_;
            const std::size_t chunk = len < room ? len : room;
            std::memcpy(&page_[page_used_], data, chunk);
            page_used_ += chunk;
            data += chunk;
            len -= chunk;
            total_ += static_cast<std::uint32_t>(chunk);

            if (page_used_ == sizeof(page_) && !flush()) {
                return false;
            }
        }

        return true;
    }

    bool flush()
    {
        if (page_used_ == 0U) {
            return ok_;
        }

        const bool wrote = flash_.write(address_, page_, page_used_);
        if (!wrote) {
            ok_ = false;
            return false;
        }

        address_ += static_cast<std::uint32_t>(page_used_);
        page_used_ = 0U;
        std::memset(page_, 0xFF, sizeof(page_));
        return true;
    }

    std::uint32_t bytes() const
    {
        return total_;
    }

    bool ok() const
    {
        return ok_;
    }

private:
    MX25L128& flash_;
    std::uint32_t start_;
    std::uint32_t end_;
    std::uint32_t address_;
    std::uint8_t page_[MX25_PAGE_SIZE];
    std::size_t page_used_{0U};
    std::uint32_t total_{0U};
    bool ok_{true};
};

void dump_flash(MX25L128& flash, std::uint32_t start, std::uint32_t len)
{
    std::uint8_t buffer[128];
    std::uint32_t offset = 0U;

    while (offset < len) {
        const std::uint32_t remaining = len - offset;
        const std::uint32_t chunk = remaining < sizeof(buffer) ? remaining : sizeof(buffer);
        if (!flash.read(start + offset, buffer, chunk)) {
            uart_write("\r\nDUMP_READ_FAIL\r\n");
            return;
        }
        uart_write(reinterpret_cast<const char*>(buffer), chunk);
        offset += chunk;
    }
}

void copy_flash_to_ram(MX25L128& flash, std::uint32_t start, std::uint32_t len)
{
    const std::uint32_t capacity = sizeof(g_csv_dump) - 1U;
    if (len > capacity) {
        len = capacity;
        g_csv_error |= 1U;
    }

    if (len > 0U && !flash.read(start, reinterpret_cast<std::uint8_t*>(g_csv_dump), len)) {
        g_csv_error |= 2U;
        g_csv_len = 0U;
        g_csv_ready = 1U;
        return;
    }

    g_csv_dump[len] = '\0';
    g_csv_len = len;
    g_csv_ready = 1U;
}

void append_sample_line(CsvFlashWriter& writer,
                        std::uint32_t ms,
                        const imu_data& imu,
                        const baro_data& baro,
                        bool imu_ok,
                        bool baro_ok)
{
    char line[192];
    const int used = std::snprintf(
        line,
        sizeof(line),
        "%lu,%ld,%ld,%ld,%d,%d,%d,%d,%ld,%ld,%ld,%u,%u\r\n",
        static_cast<unsigned long>(ms),
        static_cast<long>(to_scaled(imu.acceleration.x, 1000.0f)),
        static_cast<long>(to_scaled(imu.acceleration.y, 1000.0f)),
        static_cast<long>(to_scaled(imu.acceleration.z, 1000.0f)),
        static_cast<int>(imu.gyro.x),
        static_cast<int>(imu.gyro.y),
        static_cast<int>(imu.gyro.z),
        static_cast<int>(imu.temperature),
        static_cast<long>(baro.pressure),
        static_cast<long>(to_scaled(baro.temperature, 100.0f)),
        static_cast<long>(to_scaled(baro.altitude, 1000.0f)),
        imu_ok ? 1U : 0U,
        baro_ok ? 1U : 0U);

    if (used > 0) {
        writer.append(line, used < static_cast<int>(sizeof(line))
            ? static_cast<std::size_t>(used)
            : sizeof(line) - 1U);
    }
}

} // namespace

int main()
{
    HAL_Init();
    SystemClock_Config();
    buzzer_off();
    MX_USART1_UART_Init();
    MX_SPI1_Init();

    uart_write("\r\nCROI_SENSOR_CSV_DEMO\r\n");
    uart_write("uart: USART1 PA9 TX 115200\r\n");
    uart_write("buzzer: off\r\n");

    SPI_STM flash_spi(&hspi1, FLASH_CS_PORT, FLASH_CS_PIN);
    SPI_STM imu_spi(&hspi1, IMU_CS_PORT, IMU_CS_PIN);
    SPI_STM baro_spi(&hspi1, BARO_CS_PORT, BARO_CS_PIN);

    MX25L128 flash(flash_spi);
    LSM6DSO32 imu(imu_spi);
    MS5607 baro(baro_spi);

    const bool flash_ok = flash.init();
    const bool imu_init_ok = imu.init();
    const bool baro_init_ok = baro.init();

    uart_printf("flash,%u,0x%06lX\r\n",
                flash_ok ? 1U : 0U,
                static_cast<unsigned long>(flash.jedec_id()));
    uart_printf("imu,%u\r\n", imu_init_ok ? 1U : 0U);
    uart_printf("baro,%u\r\n", baro_init_ok ? 1U : 0U);

    if (!flash_ok) {
        uart_write("STOP,FLASH_FAIL\r\n");
        while (true) {
            buzzer_off();
            HAL_Delay(1000);
        }
    }

    uart_write("erase,start\r\n");
    const bool erase_ok = flash.erase(kLogStart, kLogSize);
    uart_printf("erase,%u\r\n", erase_ok ? 1U : 0U);
    if (!erase_ok) {
        uart_write("STOP,ERASE_FAIL\r\n");
        while (true) {
            buzzer_off();
            HAL_Delay(1000);
        }
    }

    CsvFlashWriter writer(flash, kLogStart, kLogSize);
    writer.append("ms,ax_mg,ay_mg,az_mg,gx_dps,gy_dps,gz_dps,imu_temp_c,pressure_pa,baro_temp_centi,altitude_mm,imu_ok,baro_ok\r\n");

    imu_data imu_sample = {};
    baro_data baro_sample = {};
    bool last_imu_ok = false;
    bool last_baro_ok = false;
    std::uint32_t sample_count = 0U;
    const std::uint32_t start = HAL_GetTick();

    uart_write("sample,start\r\n");
    while ((HAL_GetTick() - start) < kDurationMs && writer.ok()) {
        const std::uint32_t now = HAL_GetTick();
        last_imu_ok = imu_init_ok && imu.update(&imu_sample);
        last_baro_ok = baro_init_ok && baro.update(&baro_sample);

        append_sample_line(writer,
                           now - start,
                           imu_sample,
                           baro_sample,
                           last_imu_ok,
                           last_baro_ok);
        ++sample_count;

        const std::uint32_t next = now + kSampleMs;
        while (static_cast<std::int32_t>(next - HAL_GetTick()) > 0) {
            buzzer_off();
        }
    }
    const bool flush_ok = writer.flush();

    uart_printf("sample,done,%lu,bytes,%lu,flash_write,%u\r\n",
                static_cast<unsigned long>(sample_count),
                static_cast<unsigned long>(writer.bytes()),
                (writer.ok() && flush_ok) ? 1U : 0U);

    copy_flash_to_ram(flash, kLogStart, writer.bytes());

    HAL_Delay(kDumpDelayMs);
    uart_write("CSV_BEGIN\r\n");
    dump_flash(flash, kLogStart, writer.bytes());
    uart_write("CSV_END\r\n");

    while (true) {
        buzzer_off();
        HAL_Delay(1000);
    }
}

namespace {

void SystemClock_Config()
{
    RCC_OscInitTypeDef osc = {};
    RCC_ClkInitTypeDef clk = {};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
    osc.HSI48State = RCC_HSI48_ON;
    osc.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        while (true) {}
    }

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1) != HAL_OK) {
        while (true) {}
    }
}

void MX_USART1_UART_Init()
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {};
    gpio.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF1_USART1;
    HAL_GPIO_Init(GPIOA, &gpio);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        while (true) {}
    }
}

} // namespace

extern "C" int _write(int, char* ptr, int len)
{
    uart_write(ptr, static_cast<std::size_t>(len));
    return len;
}
