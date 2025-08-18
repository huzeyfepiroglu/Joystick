/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN)
  {
  /* USER CODE BEGIN CAN_MspInit 0 */

  /* USER CODE END CAN_MspInit 0 */
    /* CAN clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN interrupt Init */
    HAL_NVIC_SetPriority(CAN_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN_RX0_IRQn);
  /* USER CODE BEGIN CAN_MspInit 1 */

  /* USER CODE END CAN_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN)
  {
  /* USER CODE BEGIN CAN_MspDeInit 0 */

  /* USER CODE END CAN_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN_RX0_IRQn);
  /* USER CODE BEGIN CAN_MspDeInit 1 */

  /* USER CODE END CAN_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/* -------- Tüm ID’leri FIFO0’a kabul eden filtre ------- */
//void CAN_Filter_All_To_FIFO0(void)
//{
//  CAN_FilterTypeDef f = {0};
//  f.FilterBank = 0;
//  f.FilterMode = CAN_FILTERMODE_IDMASK;
//  f.FilterScale = CAN_FILTERSCALE_32BIT;
//  f.FilterIdHigh = 0x0000;
//  f.FilterIdLow  = 0x0000;
//  f.FilterMaskIdHigh = 0x0000;
//  f.FilterMaskIdLow  = 0x0000;      // hepsini kabul
//  f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//  f.FilterActivation = ENABLE;
//  HAL_CAN_ConfigFilter(&hcan, &f);
//}

/* Tüm standart ID'leri kabul eden basit mask filtre (FIFO0) */
void CAN_ConfigFilter(void)
{
  CAN_FilterTypeDef sFilter = {0};

  sFilter.FilterBank = 0;                          // 0..27 arası
  sFilter.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilter.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;

  /* Accept-all: ID=0, MASK=0 */
  sFilter.FilterIdHigh      = 0x0000;
  sFilter.FilterIdLow       = 0x0000;
  sFilter.FilterMaskIdHigh  = 0x0000;
  sFilter.FilterMaskIdLow   = 0x0000;

  sFilter.FilterActivation = ENABLE;

  if (HAL_CAN_ConfigFilter(&hcan, &sFilter) != HAL_OK) {
    Error_Handler();
  }
}

/* CAN'i başlat + RX interrupt'larını aktive et */
void CAN_StartIT(void)
{
  if (HAL_CAN_Start(&hcan) != HAL_OK) {
    Error_Handler();
  }

  /* FIFO0'da mesaj gelince kesme, ayrıca TX mailbox boşalması vs. */
  if (HAL_CAN_ActivateNotification(&hcan,
      CAN_IT_RX_FIFO0_MSG_PENDING |
      CAN_IT_TX_MAILBOX_EMPTY |
      CAN_IT_ERROR_WARNING |
      CAN_IT_BUSOFF) != HAL_OK)
  {
    Error_Handler();
  }
}

/* Örnek bir 8-byte standart CAN çerçevesi gönderimi */
void CAN_SendExample(void)
{
  CAN_TxHeaderTypeDef txHeader = {0};
  uint8_t data[8] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88};
  uint32_t txMailbox;

  txHeader.StdId = 0x321;
  txHeader.IDE   = CAN_ID_STD;
  txHeader.RTR   = CAN_RTR_DATA;
  txHeader.DLC   = 8;
  txHeader.TransmitGlobalTime = DISABLE;

  if (HAL_CAN_AddTxMessage(&hcan, &txHeader, data, &txMailbox) != HAL_OK) {
    // TX kuyruğu dolu vs. durumunda hata yönetimi
  }
}
/* USER CODE END 1 */
