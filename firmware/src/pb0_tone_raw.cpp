#include "stm32f0xx.h"

#define BUZZER_GPIO_PORT GPIOB
#define BUZZER_GPIO_PIN GPIO_BSRR_BS_0
#define BUZZER_GPIO_PIN_RESET GPIO_BRR_BR_0

static constexpr uint32_t SYSCLK_HZ = 48000000U;
static constexpr uint32_t TIMER_TICK_HZ = 1000000U;
static constexpr uint32_t NOTE_REST = 0U;
static constexpr uint32_t REPEATED_NOTE_GAP_MS = 4U;

static constexpr uint16_t SHORT_MS = 156U;
static constexpr uint16_t MEDIUM_MS = 273U;
static constexpr uint16_t END_MS = 546U;
static constexpr uint16_t PHRASE_REST_MS = 286U;
static constexpr uint16_t RICK_SHORT_MS = 156U;
static constexpr uint16_t RICK_MEDIUM_MS = 273U;
static constexpr uint16_t RICK_LONG_MS = 520U;
static constexpr uint16_t RICK_LINE_REST_MS = 250U;

static constexpr uint32_t NOTE_A3 = 220000U;
static constexpr uint32_t NOTE_C4 = 261630U;
static constexpr uint32_t NOTE_D4 = 293660U;
static constexpr uint32_t NOTE_E4 = 329630U;
static constexpr uint32_t NOTE_F4 = 349230U;
static constexpr uint32_t NOTE_G4 = 392000U;
static constexpr uint32_t NOTE_A4 = 440000U;
static constexpr uint32_t NOTE_AS4 = 466160U;
static constexpr uint32_t NOTE_B4 = 493880U;
static constexpr uint32_t NOTE_CS5 = 554370U;
static constexpr uint32_t NOTE_D5 = 587330U;
static constexpr uint32_t NOTE_E5 = 659250U;
static constexpr uint32_t NOTE_FS5 = 739990U;
static constexpr uint32_t NOTE_A5 = 880000U;

struct MelodyNote {
    uint32_t frequency_millihz;
    uint16_t duration_ms;
};

static const MelodyNote fanfare[] = {
    {NOTE_A3, SHORT_MS},
    {NOTE_C4, SHORT_MS},
    {NOTE_D4, SHORT_MS},
    {NOTE_D4, SHORT_MS},
    {NOTE_D4, SHORT_MS},
    {NOTE_E4, MEDIUM_MS},
    {NOTE_F4, SHORT_MS},
    {NOTE_F4, SHORT_MS},
    {NOTE_F4, SHORT_MS},
    {NOTE_G4, MEDIUM_MS},
    {NOTE_E4, SHORT_MS},
    {NOTE_E4, SHORT_MS},
    {NOTE_D4, SHORT_MS},
    {NOTE_C4, SHORT_MS},
    {NOTE_D4, END_MS},
    {NOTE_REST, PHRASE_REST_MS},

    {NOTE_A3, SHORT_MS},
    {NOTE_C4, SHORT_MS},
    {NOTE_D4, SHORT_MS},
    {NOTE_D4, SHORT_MS},
    {NOTE_D4, SHORT_MS},
    {NOTE_F4, MEDIUM_MS},
    {NOTE_G4, SHORT_MS},
    {NOTE_G4, SHORT_MS},
    {NOTE_G4, SHORT_MS},
    {NOTE_A4, SHORT_MS},
    {NOTE_AS4, SHORT_MS},
    {NOTE_AS4, SHORT_MS},
    {NOTE_A4, SHORT_MS},
    {NOTE_G4, SHORT_MS},
    {NOTE_A4, MEDIUM_MS},
    {NOTE_D4, SHORT_MS},
    {NOTE_D4, SHORT_MS},
    {NOTE_E4, SHORT_MS},
    {NOTE_F4, SHORT_MS},
    {NOTE_F4, SHORT_MS},
    {NOTE_G4, SHORT_MS},
    {NOTE_A4, SHORT_MS},
    {NOTE_D4, SHORT_MS},
    {NOTE_D4, END_MS},
    {NOTE_REST, PHRASE_REST_MS},

    {NOTE_D4, SHORT_MS},
    {NOTE_F4, SHORT_MS},
    {NOTE_E4, SHORT_MS},
    {NOTE_E4, SHORT_MS},
    {NOTE_F4, SHORT_MS},
    {NOTE_D4, SHORT_MS},
    {NOTE_E4, MEDIUM_MS},
    {NOTE_A3, SHORT_MS},
    {NOTE_C4, SHORT_MS},
    {NOTE_D4, SHORT_MS},
    {NOTE_D4, SHORT_MS},
    {NOTE_D4, SHORT_MS},
    {NOTE_E4, MEDIUM_MS},
    {NOTE_F4, SHORT_MS},
    {NOTE_F4, SHORT_MS},
    {NOTE_F4, SHORT_MS},
    {NOTE_G4, MEDIUM_MS},
    {NOTE_E4, SHORT_MS},
    {NOTE_E4, SHORT_MS},
    {NOTE_D4, SHORT_MS},
    {NOTE_C4, SHORT_MS},
    {NOTE_C4, SHORT_MS},
    {NOTE_D4, END_MS},
};

static const MelodyNote second_melody[] = {
    {NOTE_A4, RICK_SHORT_MS},
    {NOTE_B4, RICK_SHORT_MS},
    {NOTE_D5, RICK_MEDIUM_MS},
    {NOTE_B4, RICK_SHORT_MS},
    {NOTE_FS5, RICK_MEDIUM_MS},
    {NOTE_FS5, RICK_MEDIUM_MS},
    {NOTE_E5, RICK_LONG_MS},
    {NOTE_REST, RICK_LINE_REST_MS},

    {NOTE_A4, RICK_SHORT_MS},
    {NOTE_B4, RICK_SHORT_MS},
    {NOTE_D5, RICK_MEDIUM_MS},
    {NOTE_B4, RICK_SHORT_MS},
    {NOTE_E5, RICK_MEDIUM_MS},
    {NOTE_E5, RICK_MEDIUM_MS},
    {NOTE_D5, RICK_SHORT_MS},
    {NOTE_CS5, RICK_SHORT_MS},
    {NOTE_B4, RICK_LONG_MS},
    {NOTE_REST, RICK_LINE_REST_MS},

    {NOTE_A4, RICK_SHORT_MS},
    {NOTE_B4, RICK_SHORT_MS},
    {NOTE_D5, RICK_MEDIUM_MS},
    {NOTE_B4, RICK_SHORT_MS},
    {NOTE_D5, RICK_MEDIUM_MS},
    {NOTE_E5, RICK_SHORT_MS},
    {NOTE_CS5, RICK_MEDIUM_MS},
    {NOTE_A4, RICK_SHORT_MS},
    {NOTE_A4, RICK_SHORT_MS},
    {NOTE_E5, RICK_MEDIUM_MS},
    {NOTE_D5, RICK_LONG_MS},
    {NOTE_REST, RICK_LINE_REST_MS},

    {NOTE_A4, RICK_SHORT_MS},
    {NOTE_B4, RICK_SHORT_MS},
    {NOTE_D5, RICK_MEDIUM_MS},
    {NOTE_B4, RICK_SHORT_MS},
    {NOTE_FS5, RICK_MEDIUM_MS},
    {NOTE_FS5, RICK_MEDIUM_MS},
    {NOTE_E5, RICK_LONG_MS},
    {NOTE_REST, RICK_LINE_REST_MS},

    {NOTE_A4, RICK_SHORT_MS},
    {NOTE_B4, RICK_SHORT_MS},
    {NOTE_D5, RICK_MEDIUM_MS},
    {NOTE_B4, RICK_SHORT_MS},
    {NOTE_A5, RICK_MEDIUM_MS},
    {NOTE_CS5, RICK_SHORT_MS},
    {NOTE_D5, RICK_SHORT_MS},
    {NOTE_CS5, RICK_SHORT_MS},
    {NOTE_B4, RICK_LONG_MS},
    {NOTE_REST, RICK_LINE_REST_MS},

    {NOTE_A4, RICK_SHORT_MS},
    {NOTE_B4, RICK_SHORT_MS},
    {NOTE_D5, RICK_MEDIUM_MS},
    {NOTE_B4, RICK_SHORT_MS},
    {NOTE_D5, RICK_MEDIUM_MS},
    {NOTE_E5, RICK_MEDIUM_MS},
    {NOTE_CS5, RICK_MEDIUM_MS},
    {NOTE_A4, RICK_SHORT_MS},
    {NOTE_A4, RICK_SHORT_MS},
    {NOTE_E5, RICK_MEDIUM_MS},
    {NOTE_D5, RICK_LONG_MS},
};

static void pb0_high()
{
    BUZZER_GPIO_PORT->BSRR = BUZZER_GPIO_PIN;
}

static void pb0_low()
{
    BUZZER_GPIO_PORT->BRR = BUZZER_GPIO_PIN_RESET;
}

static void clock_init()
{
    RCC->CR2 |= RCC_CR2_HSI48ON;
    while ((RCC->CR2 & RCC_CR2_HSI48RDY) == 0U) {}

    FLASH->ACR |= FLASH_ACR_LATENCY;

    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_HSI48;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI48) {}
}

static void gpio_init()
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

static void timebase_init()
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    (void)RCC->APB1ENR;

    TIM3->PSC = (SYSCLK_HZ / TIMER_TICK_HZ) - 1U;
    TIM3->ARR = 0xFFFFU;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 = TIM_CR1_CEN;
}

static void delay_us(uint32_t us)
{
    while (us > 0U) {
        const uint32_t chunk = (us > 60000U) ? 60000U : us;
        TIM3->CNT = 0U;
        while (TIM3->CNT < chunk) {}
        us -= chunk;
    }
}

static void delay_ms(uint32_t ms)
{
    while (ms-- > 0U) {
        delay_us(1000U);
    }
}

static void pwm_off()
{
    pb0_low();
}

static void play_frequency(uint32_t frequency_millihz, uint16_t duration_ms)
{
    if (frequency_millihz == NOTE_REST) {
        pwm_off();
        delay_ms(duration_ms);
        return;
    }

    const uint32_t half_period_us =
        (500000000UL + (frequency_millihz / 2U)) / frequency_millihz;
    const uint32_t total_us = static_cast<uint32_t>(duration_ms) * 1000U;
    const uint32_t period_us = half_period_us * 2U;
    uint32_t elapsed_us = 0U;

    while ((elapsed_us + period_us) <= total_us) {
        pb0_high();
        delay_us(half_period_us);
        pb0_low();
        delay_us(half_period_us);
        elapsed_us += period_us;
    }

    pwm_off();

    if (elapsed_us < total_us) {
        delay_us(total_us - elapsed_us);
    }
}

static void play_melody(const MelodyNote* melody, uint32_t note_count)
{
    uint32_t previous_frequency = NOTE_REST;

    for (uint32_t i = 0U; i < note_count; ++i) {
        const MelodyNote& note = melody[i];

        if (note.frequency_millihz != NOTE_REST &&
            note.frequency_millihz == previous_frequency) {
            pwm_off();
            delay_ms(REPEATED_NOTE_GAP_MS);
        }

        play_frequency(note.frequency_millihz, note.duration_ms);
        previous_frequency = note.frequency_millihz;
    }
}

int main(void)
{
    clock_init();
    gpio_init();
    timebase_init();

    while (1) {
        play_melody(fanfare, sizeof(fanfare) / sizeof(fanfare[0]));
        pwm_off();
        delay_ms(250U);
        play_melody(second_melody, sizeof(second_melody) / sizeof(second_melody[0]));
        pwm_off();
        delay_ms(1200U);
    }
}
