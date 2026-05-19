#include "stm32f0xx_hal.h"

#include <cstddef>
#include <cstdint>

#define BUZZER_GPIO_PORT GPIOB
#define BUZZER_GPIO_PIN GPIO_PIN_0
#define BUZZER_TIMER_TICK_HZ 1000000U

TIM_HandleTypeDef htim3;

struct Note {
    uint16_t frequency_hz;
    uint16_t duration_ms;
};

static constexpr uint16_t REST = 0;
static constexpr uint16_t NOTE_C4 = 262;
static constexpr uint16_t NOTE_D4 = 294;
static constexpr uint16_t NOTE_DS4 = 311;
static constexpr uint16_t NOTE_E4 = 330;
static constexpr uint16_t NOTE_F4 = 349;
static constexpr uint16_t NOTE_G4 = 392;
static constexpr uint16_t NOTE_A4 = 440;
static constexpr uint16_t NOTE_B4 = 494;
static constexpr uint16_t NOTE_C5 = 523;
static constexpr uint16_t NOTE_D5 = 587;
static constexpr uint16_t NOTE_DS5 = 622;
static constexpr uint16_t NOTE_E5 = 659;
static constexpr uint16_t NOTE_F5 = 698;
static constexpr uint16_t NOTE_G5 = 784;
static constexpr uint16_t NOTE_A5 = 880;

#define ARRAY_LEN(array) (sizeof(array) / sizeof((array)[0]))

static const Note startup_chime[] = {
    {2400, 80}, {REST, 100}, {2400, 80}, {REST, 100}, {2400, 80}, {REST, 300},
};

static const Note diagnostic_ladder[] = {
    {300, 220}, {600, 220}, {900, 220}, {1200, 220},
    {1600, 220}, {2000, 220}, {2400, 360}, {2800, 220},
    {3200, 220}, {2400, 500}, {REST, 500},
};

static const Note ode_to_joy[] = {
    {NOTE_E4, 260}, {NOTE_E4, 260}, {NOTE_F4, 260}, {NOTE_G4, 260},
    {NOTE_G4, 260}, {NOTE_F4, 260}, {NOTE_E4, 260}, {NOTE_D4, 260},
    {NOTE_C4, 260}, {NOTE_C4, 260}, {NOTE_D4, 260}, {NOTE_E4, 260},
    {NOTE_E4, 390}, {NOTE_D4, 130}, {NOTE_D4, 520},
    {NOTE_E4, 260}, {NOTE_E4, 260}, {NOTE_F4, 260}, {NOTE_G4, 260},
    {NOTE_G4, 260}, {NOTE_F4, 260}, {NOTE_E4, 260}, {NOTE_D4, 260},
    {NOTE_C4, 260}, {NOTE_C4, 260}, {NOTE_D4, 260}, {NOTE_E4, 260},
    {NOTE_D4, 390}, {NOTE_C4, 130}, {NOTE_C4, 520},
};

static const Note drunken_sailor[] = {
    {NOTE_D4, 180}, {NOTE_D4, 180}, {NOTE_D4, 180}, {NOTE_D4, 180},
    {NOTE_D4, 180}, {NOTE_D4, 180}, {NOTE_D4, 180}, {NOTE_F4, 180},
    {NOTE_A4, 360}, {REST, 90},
    {NOTE_C5, 180}, {NOTE_C5, 180}, {NOTE_C5, 180}, {NOTE_C5, 180},
    {NOTE_C5, 180}, {NOTE_C5, 180}, {NOTE_C5, 180}, {NOTE_E5, 180},
    {NOTE_G5, 360}, {REST, 90},
    {NOTE_D5, 180}, {NOTE_D5, 180}, {NOTE_D5, 180}, {NOTE_D5, 180},
    {NOTE_D5, 180}, {NOTE_C5, 180}, {NOTE_A4, 180}, {NOTE_F4, 180},
    {NOTE_D4, 520},
};

static const Note can_can[] = {
    {NOTE_G4, 130}, {NOTE_G4, 130}, {NOTE_A4, 130}, {NOTE_B4, 130},
    {NOTE_C5, 260}, {NOTE_B4, 130}, {NOTE_A4, 130}, {NOTE_G4, 260},
    {NOTE_F4, 130}, {NOTE_F4, 130}, {NOTE_G4, 130}, {NOTE_A4, 130},
    {NOTE_B4, 260}, {NOTE_A4, 130}, {NOTE_G4, 130}, {NOTE_F4, 260},
    {NOTE_E4, 130}, {NOTE_E4, 130}, {NOTE_F4, 130}, {NOTE_G4, 130},
    {NOTE_A4, 260}, {NOTE_G4, 130}, {NOTE_F4, 130}, {NOTE_E4, 260},
    {NOTE_D4, 130}, {NOTE_E4, 130}, {NOTE_F4, 130}, {NOTE_G4, 130},
    {NOTE_A4, 130}, {NOTE_B4, 130}, {NOTE_C5, 520},
};

static const Note entertainer[] = {
    {NOTE_D4, 120}, {NOTE_DS4, 120}, {NOTE_E4, 120}, {NOTE_C5, 240},
    {NOTE_E4, 120}, {NOTE_C5, 240}, {NOTE_E4, 120}, {NOTE_C5, 360},
    {NOTE_C5, 120}, {NOTE_D5, 120}, {NOTE_DS5, 120}, {NOTE_E5, 120},
    {NOTE_C5, 120}, {NOTE_D5, 120}, {NOTE_E5, 240}, {NOTE_B4, 120},
    {NOTE_D5, 240}, {NOTE_C5, 520},
};

struct Song {
    const Note* notes;
    std::size_t note_count;
    uint16_t pause_ms;
};

static const Song playlist[] = {
    {ode_to_joy, ARRAY_LEN(ode_to_joy), 800},
    {drunken_sailor, ARRAY_LEN(drunken_sailor), 800},
    {can_can, ARRAY_LEN(can_can), 800},
    {entertainer, ARRAY_LEN(entertainer), 1200},
};

void SystemClock_Config(void);
void Error_Handler(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);

static void MX_TIM3_Init(void);

static void Buzzer_Off(void)
{
    if (htim3.Instance != nullptr) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    }
}

static void Buzzer_SetFrequency(uint16_t frequency_hz)
{
    if (frequency_hz == REST) {
        Buzzer_Off();
        return;
    }

    uint32_t period = BUZZER_TIMER_TICK_HZ / frequency_hz;
    if (period < 2U) {
        period = 2U;
    } else if (period > 0xFFFFU) {
        period = 0xFFFFU;
    }

    __HAL_TIM_SET_AUTORELOAD(&htim3, period - 1U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, period / 2U);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}

static void PlayNote(const Note& note)
{
    Buzzer_SetFrequency(note.frequency_hz);

    if (note.duration_ms > 30U && note.frequency_hz != REST) {
        HAL_Delay(note.duration_ms - 20U);
        Buzzer_Off();
        HAL_Delay(20U);
    } else {
        HAL_Delay(note.duration_ms);
        Buzzer_Off();
    }
}

static void PlayNotes(const Note* notes, std::size_t note_count)
{
    for (std::size_t i = 0; i < note_count; ++i) {
        PlayNote(notes[i]);
    }
}

static void Sweep(uint16_t start_hz, uint16_t end_hz, int16_t step_hz, uint16_t hold_ms)
{
    int32_t frequency = start_hz;

    while ((step_hz > 0 && frequency <= end_hz) ||
           (step_hz < 0 && frequency >= end_hz)) {
        Buzzer_SetFrequency(static_cast<uint16_t>(frequency));
        HAL_Delay(hold_ms);
        frequency += step_hz;
    }

    Buzzer_Off();
    HAL_Delay(150);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_TIM3_Init();

    while (1) {
        Buzzer_Off();
        HAL_Delay(5000);

        Buzzer_SetFrequency(2400);
        HAL_Delay(5000);
    }
}

static void MX_TIM3_Init(void)
{
    TIM_OC_InitTypeDef config = {0};

    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();
    uint32_t prescaler = timer_clock / BUZZER_TIMER_TICK_HZ;
    if (prescaler == 0U) {
        prescaler = 1U;
    }

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = prescaler - 1U;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 1000U - 1U;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }

    config.OCMode = TIM_OCMODE_PWM1;
    config.Pulse = 0;
    config.OCPolarity = TIM_OCPOLARITY_HIGH;
    config.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim3, &config, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&htim3);

    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }

    Buzzer_Off();
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

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM3) {
        __HAL_RCC_TIM3_CLK_ENABLE();
    }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
    if (htim->Instance != TIM3) {
        return;
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = BUZZER_GPIO_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF1_TIM3;
    HAL_GPIO_Init(BUZZER_GPIO_PORT, &gpio);
}

void Error_Handler(void)
{
    __disable_irq();
    Buzzer_Off();
    while (1) {}
}
