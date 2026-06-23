#include "stm32f0xx_hal.h"

#include <cstdint>

namespace {

constexpr uint32_t kDebugMagic = 0x43414E31U; // CAN1
constexpr uint32_t kCanBitrate = 500000U;
constexpr uint32_t kTxPeriodMs = 250U;

CAN_HandleTypeDef hcan;

struct CanDebug {
    uint32_t magic;
    uint32_t version;
    uint32_t clock_status;
    uint32_t hse_ready;
    uint32_t system_hz;
    uint32_t pclk1_hz;
    uint32_t can_bitrate;
    uint32_t can_init_status;
    uint32_t filter_status;
    uint32_t start_status;
    uint32_t uid[3];
    uint32_t node_id;
    uint32_t loop_count;
    uint32_t tx_attempts;
    uint32_t tx_queued;
    uint32_t tx_ok;
    uint32_t tx_error;
    uint32_t arbitration_lost;
    uint32_t rx_total;
    uint32_t rx_valid;
    uint32_t peer_id;
    uint32_t peer_counter;
    uint32_t fifo_overruns;
    uint32_t hal_error;
    uint32_t esr;
    uint32_t tsr;
    uint32_t msr;
    uint32_t tec;
    uint32_t rec;
    uint32_t bus_off;
    uint32_t last_rx_id;
    uint32_t last_rx_dlc;
    uint8_t last_rx_data[8];
};

uint32_t hash_uid(uint32_t a, uint32_t b, uint32_t c) {
    uint32_t hash = 2166136261U;
    const uint32_t words[] = {a, b, c};
    for (uint32_t word : words) {
        for (uint32_t shift = 0; shift < 32U; shift += 8U) {
            hash ^= (word >> shift) & 0xFFU;
            hash *= 16777619U;
        }
    }
    return hash;
}

HAL_StatusTypeDef configure_hse_clock() {
    RCC_OscInitTypeDef oscillator{};
    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    oscillator.HSEState = RCC_HSE_ON;
    oscillator.PLL.PLLState = RCC_PLL_ON;
    oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    oscillator.PLL.PREDIV = RCC_PREDIV_DIV1;
    oscillator.PLL.PLLMUL = RCC_PLL_MUL6;

    HAL_StatusTypeDef status = HAL_RCC_OscConfig(&oscillator);
    if (status != HAL_OK) {
        return status;
    }

    RCC_ClkInitTypeDef clocks{};
    clocks.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1;
    clocks.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clocks.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clocks.APB1CLKDivider = RCC_HCLK_DIV1;
    status = HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_1);
    SystemCoreClockUpdate();
    return status;
}

void configure_can_gpio() {
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio{};
    gpio.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF4_CAN;
    HAL_GPIO_Init(GPIOA, &gpio);
}

HAL_StatusTypeDef configure_can() {
    __HAL_RCC_CAN1_CLK_ENABLE();
    configure_can_gpio();

    hcan.Instance = CAN;
    hcan.Init.Prescaler = 6;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = ENABLE;
    hcan.Init.AutoWakeUp = DISABLE;
    hcan.Init.AutoRetransmission = ENABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;
    return HAL_CAN_Init(&hcan);
}

HAL_StatusTypeDef configure_accept_all_filter() {
    CAN_FilterTypeDef filter{};
    filter.FilterIdHigh = 0;
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = 0;
    filter.FilterMaskIdLow = 0;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    return HAL_CAN_ConfigFilter(&hcan, &filter);
}

void record_mailbox_result(volatile CanDebug& debug, uint32_t tsr,
                           uint32_t rqcp, uint32_t txok,
                           uint32_t alst, uint32_t terr) {
    if ((tsr & rqcp) == 0U) {
        return;
    }

    if ((tsr & txok) != 0U) {
        ++debug.tx_ok;
    }
    if ((tsr & alst) != 0U) {
        ++debug.arbitration_lost;
    }
    if ((tsr & terr) != 0U) {
        ++debug.tx_error;
    }
    hcan.Instance->TSR = rqcp;
}

void poll_tx(volatile CanDebug& debug) {
    const uint32_t tsr = hcan.Instance->TSR;
    record_mailbox_result(debug, tsr, CAN_TSR_RQCP0, CAN_TSR_TXOK0,
                          CAN_TSR_ALST0, CAN_TSR_TERR0);
    record_mailbox_result(debug, tsr, CAN_TSR_RQCP1, CAN_TSR_TXOK1,
                          CAN_TSR_ALST1, CAN_TSR_TERR1);
    record_mailbox_result(debug, tsr, CAN_TSR_RQCP2, CAN_TSR_TXOK2,
                          CAN_TSR_ALST2, CAN_TSR_TERR2);
}

void poll_rx(volatile CanDebug& debug) {
    if (__HAL_CAN_GET_FLAG(&hcan, CAN_FLAG_FOV0) != RESET) {
        ++debug.fifo_overruns;
        __HAL_CAN_CLEAR_FLAG(&hcan, CAN_FLAG_FOV0);
    }

    while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) != 0U) {
        CAN_RxHeaderTypeDef header{};
        uint8_t data[8]{};
        if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) {
            break;
        }

        ++debug.rx_total;
        debug.last_rx_id = header.IDE == CAN_ID_STD ? header.StdId : header.ExtId;
        debug.last_rx_dlc = header.DLC;
        for (uint32_t i = 0; i < 8U; ++i) {
            debug.last_rx_data[i] = data[i];
        }

        if (header.IDE == CAN_ID_STD && header.RTR == CAN_RTR_DATA &&
            header.DLC == 8U && data[0] == 0xCAU && data[1] == 0x4EU) {
            const uint32_t peer_id = static_cast<uint32_t>(data[2]) |
                                     (static_cast<uint32_t>(data[3]) << 8U);
            if (peer_id != debug.node_id) {
                ++debug.rx_valid;
                debug.peer_id = peer_id;
                debug.peer_counter = static_cast<uint32_t>(data[4]) |
                                     (static_cast<uint32_t>(data[5]) << 8U) |
                                     (static_cast<uint32_t>(data[6]) << 16U) |
                                     (static_cast<uint32_t>(data[7]) << 24U);
            }
        }
    }
}

void send_heartbeat(volatile CanDebug& debug, uint32_t counter) {
    ++debug.tx_attempts;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 3U) {
        return;
    }

    CAN_TxHeaderTypeDef header{};
    header.StdId = debug.node_id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8;
    header.TransmitGlobalTime = DISABLE;

    uint8_t data[8] = {
        0xCAU,
        0x4EU,
        static_cast<uint8_t>(debug.node_id),
        static_cast<uint8_t>(debug.node_id >> 8U),
        static_cast<uint8_t>(counter),
        static_cast<uint8_t>(counter >> 8U),
        static_cast<uint8_t>(counter >> 16U),
        static_cast<uint8_t>(counter >> 24U),
    };
    uint32_t mailbox = 0;
    if (HAL_CAN_AddTxMessage(&hcan, &header, data, &mailbox) == HAL_OK) {
        ++debug.tx_queued;
    } else {
        ++debug.tx_error;
    }
}

void snapshot_can_state(volatile CanDebug& debug) {
    debug.hal_error = HAL_CAN_GetError(&hcan);
    debug.esr = hcan.Instance->ESR;
    debug.tsr = hcan.Instance->TSR;
    debug.msr = hcan.Instance->MSR;
    debug.tec = (debug.esr & CAN_ESR_TEC_Msk) >> CAN_ESR_TEC_Pos;
    debug.rec = (debug.esr & CAN_ESR_REC_Msk) >> CAN_ESR_REC_Pos;
    debug.bus_off = (debug.esr & CAN_ESR_BOFF) != 0U;
}

} // namespace

extern "C" {
volatile CanDebug g_can_debug = {};
}

extern "C" void SysTick_Handler(void) {
    HAL_IncTick();
}

int main() {
    HAL_Init();

    g_can_debug.magic = kDebugMagic;
    g_can_debug.version = 1;
    g_can_debug.uid[0] = *reinterpret_cast<const uint32_t*>(UID_BASE);
    g_can_debug.uid[1] = *reinterpret_cast<const uint32_t*>(UID_BASE + 4U);
    g_can_debug.uid[2] = *reinterpret_cast<const uint32_t*>(UID_BASE + 8U);
    const uint32_t uid_hash = hash_uid(g_can_debug.uid[0], g_can_debug.uid[1],
                                       g_can_debug.uid[2]);
    g_can_debug.node_id = 0x400U | (uid_hash & 0x3FFU);

    g_can_debug.clock_status = configure_hse_clock();
    g_can_debug.hse_ready = __HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) != RESET;
    g_can_debug.system_hz = HAL_RCC_GetSysClockFreq();
    g_can_debug.pclk1_hz = HAL_RCC_GetPCLK1Freq();
    g_can_debug.can_bitrate = kCanBitrate;

    if (g_can_debug.clock_status != HAL_OK || g_can_debug.system_hz != 48000000U) {
        while (true) {
            ++g_can_debug.loop_count;
        }
    }

    g_can_debug.can_init_status = configure_can();
    if (g_can_debug.can_init_status == HAL_OK) {
        g_can_debug.filter_status = configure_accept_all_filter();
    }
    if (g_can_debug.filter_status == HAL_OK) {
        g_can_debug.start_status = HAL_CAN_Start(&hcan);
    }

    if (g_can_debug.can_init_status != HAL_OK ||
        g_can_debug.filter_status != HAL_OK ||
        g_can_debug.start_status != HAL_OK) {
        while (true) {
            ++g_can_debug.loop_count;
        }
    }

    uint32_t counter = 0;
    uint32_t next_tx = HAL_GetTick() + 100U + (uid_hash % 100U);
    while (true) {
        ++g_can_debug.loop_count;
        poll_tx(g_can_debug);
        poll_rx(g_can_debug);
        snapshot_can_state(g_can_debug);

        const uint32_t now = HAL_GetTick();
        if (static_cast<int32_t>(now - next_tx) >= 0) {
            send_heartbeat(g_can_debug, counter++);
            next_tx += kTxPeriodMs;
            if (static_cast<int32_t>(now - next_tx) >= 0) {
                next_tx = now + kTxPeriodMs;
            }
        }
    }
}
