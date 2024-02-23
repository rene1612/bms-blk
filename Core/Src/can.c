/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "main.h"
#include "neey.h"

extern const _DEV_CONFIG_REGS* pDevConfig;

CAN_TxHeaderTypeDef	TxHeader, ReplayHeader;
uint8_t				CanTxData[8];
uint32_t            TxMailbox;
uint8_t				can_task_scheduler;
CAN_RxHeaderTypeDef RxHeader;
uint8_t				can_replay_msg;
uint8_t             CanRxData[8];

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = pDevConfig->app_can_bitrate;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
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
  can_task_scheduler = PROCESS_NO_TASK;

  main_regs.can_rx_cmd_id = (pDevConfig->dev_id<<4) + CANRX_SA;
  main_regs.can_tx_data_id = (pDevConfig->dev_id<<4) + CANTX_SA;
  main_regs.can_tx_heartbeat_id = (pDevConfig->dev_id<<4) + CANTX_HA;
  main_regs.can_filterMask = RXFILTERMASK;
  main_regs.can_filterID = (pDevConfig->dev_id<<8); // Only accept bootloader CAN message ID

  TxHeader.DLC = 5;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.StdId = main_regs.can_tx_data_id;
  TxHeader.RTR = CAN_RTR_DATA;

  ReplayHeader.DLC = 2;
  ReplayHeader.IDE = CAN_ID_STD;
  ReplayHeader.StdId = main_regs.can_tx_data_id;
  ReplayHeader.RTR = CAN_RTR_DATA;


	/* config_can_filter ---------------------------------------------------------*/
	/* Setup Can-Filter                                                           */
   CAN_FilterTypeDef sFilterConfig;

  /*##-2- Configure the CAN Filter ###########################################*/
  //sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
  sFilterConfig.FilterBank = 13;

  sFilterConfig.FilterIdHigh = main_regs.can_filterID << 5;
  sFilterConfig.FilterIdLow = 0;

  sFilterConfig.FilterMaskIdHigh = main_regs.can_filterMask << 5;
  sFilterConfig.FilterMaskIdLow = 0;
  sFilterConfig.SlaveStartFilterBank = 14;  // how many filters to assign to the CAN1 (master can)

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
	Error_Handler();
  }

  /* USER CODE END CAN_Init 2 */

}


void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_CAN1_2();

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}


void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}


/* USER CODE BEGIN 1 */
uint8_t	process_CAN(void)
{
	uint8_t sys_reg;
	//uint16_t reg;

	if (can_task_scheduler & PROCESS_CAN_SEND_NEW_NEEY_DATA)
	{
		if (!HAL_CAN_IsTxMessagePending(&hcan, TxMailbox))
		{
			if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, CanTxData, &TxMailbox) != HAL_OK)
			{
				Error_Handler ();
			}
		}
		else
			return can_task_scheduler;

		can_task_scheduler &= ~PROCESS_CAN_SEND_NEW_NEEY_DATA;

		//can_task_scheduler &= ~PROCESS_CAN_SEND_NEW_ADC_DATA;
		//return can_task_scheduler;
	}


	if (can_task_scheduler & PROCESS_CAN_ON_MSG)
	{
		switch (CanRxData[0])
		{
		case ALIVE_CMD:
			alive_timer = main_regs.alive_timeout;
			break;

		case SYS_RESET_CMD:
			//printf("SYS_RESET\n");
			HAL_NVIC_SystemReset();
			break;

		case SYS_APP_RESET_CMD:
			//printf("APP_RESET\n");
			JumpToApp();
			break;

		case SYS_BOOT_CMD:
			//printf("SYS_BOOT\n");
			//leave a message in a bottle for the bootloader,
			//so the btld will not start app again and stay in btld-mode
			*(uint32_t *)_MAGIC_RAM_ADDRESS_ = _MAGIC_RAM_DWORD_;
			JumpToBtld();
			break;

		case SYS_READ_REG_CMD:
			//printf("ADC_READ_REG_CMD\n");
			sys_reg = CanRxData[1];

			if (sys_reg < sizeof(main_regs))
			{
				CanTxData[0] = REPLAY_DATA_CMD;
				CanTxData[1] = sys_reg;
				CanTxData[2] = *(((uint8_t *)&main_regs)+sys_reg);
				ReplayHeader.DLC = 3;
			}
			else
			{
				CanTxData[0] = REPLAY_AKC_NACK_CMD;
				CanTxData[1] = NACK;
				ReplayHeader.DLC = 2;
			}
			can_task_scheduler |= PROCESS_CAN_SEND_REPLAY;
			break;

		case SYS_WRITE_REG_CMD:
			//printf("WRITE_REG_CMD\n");
			sys_reg = CanRxData[1];

			if (sys_reg < sizeof(main_regs))
			{
				*(((uint8_t *)&main_regs)+sys_reg) = CanRxData[2];
				CanTxData[1] = ACK;
			}
			else
			{
				CanTxData[1] = NACK;
			}
			ReplayHeader.DLC = 2;
			CanTxData[0] = REPLAY_AKC_NACK_CMD;
			can_task_scheduler |= PROCESS_CAN_SEND_REPLAY;
			break;

		default:
			break;
		}
		can_task_scheduler &= ~PROCESS_CAN_ON_MSG;
	}


	if (can_task_scheduler & PROCESS_CAN_SEND_REPLAY)
	{
		if (!HAL_CAN_IsTxMessagePending(&hcan, TxMailbox))
		{
			if (HAL_CAN_AddTxMessage(&hcan, &ReplayHeader, CanTxData, &TxMailbox) != HAL_OK)
			{
				Error_Handler ();
			}

			can_task_scheduler &= ~PROCESS_CAN_SEND_REPLAY;
		}
	}

	return can_task_scheduler;
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, CanRxData) != HAL_OK)
	{
		Error_Handler();
	}

	if ((RxHeader.StdId == 0x446))
	{
		can_task_scheduler |= PROCESS_CAN_ON_MSG;
    	main_task_scheduler |= PROCESS_CAN;
	}
}
/* USER CODE END 1 */
