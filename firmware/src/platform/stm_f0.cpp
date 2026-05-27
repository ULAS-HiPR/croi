#if F0
#include "stm_f0.h"

CAN_HandleTypeDef hcan;
SPI_HandleTypeDef hspi1;

void MX_CAN_Init()
{
    hcan.Instance = CAN;

    hcan.Init.Prescaler = 16;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_2TQ;

    HAL_CAN_Init(&hcan);
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{
    if (hcan->Instance != CAN) return;

    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_CAN;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
#endif // F0
