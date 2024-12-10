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
#include <string.h>
#include "main.h"
#include "neey.h"
#include "passive_balancer.h"

#include "dallas_temperature.h"

#ifdef __WS2812B__
#include "ws2812b.h"
#endif

extern const _DEV_CONFIG_REGS* pDevConfig;
extern DallasTemperatureData dt;

CAN_TxHeaderTypeDef	TxHeader, ReplayHeader, TripHeader, AllertHeader, BroadcastHeader;
uint8_t				CanTxData[8];
uint32_t            TxMailbox;
uint8_t				can_task_scheduler;
CAN_RxHeaderTypeDef RxHeader;
uint8_t				can_replay_msg;
uint8_t             CanRxData[8];
uint8_t				current_cell_2_send;
uint8_t				current_blk_data_2_send;
_BMS_CELL_DATA		bms_cell_data[NEEY_CHANNEL_COUNT];
_BMS_BLK_DATA1		bms_blk_data1;
_BMS_BLK_DATA2		bms_blk_data2;
_BMS_BLK_DATA3		bms_blk_data3;

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */
  hcan.Init.Prescaler = pDevConfig->app_can_bitrate;
  hcan.Instance = CAN1;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  /* USER CODE END CAN_Init 1 */
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
  main_regs.can_filterID = (pDevConfig->dev_id<<4); // Only accept  CAN message ID
  main_regs.can_rx_brdc_cmd_id = (pDevConfig->can_broadcast_id<<4) + 0xF;

  TxHeader.DLC = 5;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.StdId = main_regs.can_tx_data_id;
  TxHeader.RTR = CAN_RTR_DATA;

  ReplayHeader.DLC = 2;
  ReplayHeader.IDE = CAN_ID_STD;
  ReplayHeader.StdId = main_regs.can_tx_data_id;
  ReplayHeader.RTR = CAN_RTR_DATA;

  TripHeader.DLC = 3;
  TripHeader.IDE = CAN_ID_STD;
  TripHeader.StdId = (pDevConfig->can_trip_id<<4) + CANRX_SA;
  TripHeader.RTR = CAN_RTR_DATA;

  AllertHeader.DLC = 2;
  AllertHeader.IDE = CAN_ID_STD;
  AllertHeader.StdId = main_regs.can_tx_data_id;
  AllertHeader.RTR = CAN_RTR_DATA;

  BroadcastHeader.DLC = 1;
  BroadcastHeader.IDE = CAN_ID_STD;
  BroadcastHeader.StdId = (pDevConfig->can_broadcast_id<<4)+0xF;
  BroadcastHeader.RTR = CAN_RTR_DATA;

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

uint8_t prepare_BMS_CellData()
{
	uint8_t cell_counter;

	if (neey_ctrl.data_lock)
		return 0;

	neey_ctrl.data_lock = 1;

	//for (cell_counter=0;cell_counter<neey_ctrl.neey_dev_info.CellCount;cell_counter++)
	for (cell_counter=0;cell_counter<MAX_LF280K_CELL_COUNT;cell_counter++)
	{
		bms_cell_data[cell_counter].bms_data_type = BMS_GET_CELL_DATA_CMD;
		bms_cell_data[cell_counter].flags_ch_number = (uint8_t)(cell_counter | (neey_ctrl.cell_data[cell_counter].flag << 5));
		bms_cell_data[cell_counter].cell_voltage = neey_ctrl.cell_data[cell_counter].voltage;
		bms_cell_data[cell_counter].cell_resistance = neey_ctrl.cell_data[cell_counter].resistance;
		//		bms_cell_data[cell_counter].cell_temperature = Temp[cell_counter];
		//bms_cell_data[cell_counter].cell_temperature = (int16_t)(temperatures[cell_counter]*100);
		bms_cell_data[cell_counter].cell_temperature = (int16_t)(dt.temp[cell_counter]*100);

		//bms_cell_data[cell_counter].cell_flags = neey_ctrl.cell_data[cell_counter].flag;
	}

	neey_ctrl.data_lock = 0;

	current_cell_2_send=0;
	//current_blk_data_2_send=0;

	return 1;
}

uint8_t prepare_BMS_BLKData()
{
	if (neey_ctrl.data_lock)
		return 0;

	neey_ctrl.data_lock = 1;

	bms_blk_data1.bms_data_type=BMS_GET_BLK_DATA_CMD;
	bms_blk_data1.flags_ch_number=1;
	bms_blk_data1.amt_voltage=neey_ctrl.neey_dev_data.AmtVol;

	bms_blk_data2.bms_data_type=BMS_GET_BLK_DATA_CMD;
	bms_blk_data2.flags_ch_number=2;
	bms_blk_data2.ave_voltage=neey_ctrl.neey_dev_data.AveVol;
	bms_blk_data2.div_voltage=neey_ctrl.neey_dev_data.DiffVol;

	bms_blk_data3.bms_data_type=BMS_GET_BLK_DATA_CMD;
	bms_blk_data3.flags_ch_number=3;
	bms_blk_data3.bal_current=neey_ctrl.neey_dev_data.BalCurrent;
	bms_blk_data3.neey_temperatur=neey_ctrl.neey_dev_data.Temperatur;
	bms_blk_data3.heat_sink_temperatur=(int16_t)(dt.temp[22]*100);;

	neey_ctrl.data_lock = 0;

	current_blk_data_2_send=0;

	return 1;
}


uint8_t	process_CAN(void)
{
	uint8_t len;
	//int16_t temperature;
	uint16_t sys_reg, max_sys_reg_size;
	uint8_t* p_sys_reg_offset;


	/* PROCESS_CAN_SEND_NEW_NEEY_DATA --------------------------------------------------------*/
	if (can_task_scheduler & PROCESS_CAN_SEND_NEW_NEEY_DATA)
	{
		if (main_regs.cfg_regs.neey_cfg_data.auto_run) {

			if (prepare_BMS_CellData()) {
				can_task_scheduler |= PROCESS_CAN_SEND_NEW_CELL_DATA;
			}

			if (prepare_BMS_BLKData()) {
				can_task_scheduler |= PROCESS_CAN_SEND_NEW_BLK_DATA;
			}

		}

		can_task_scheduler &= ~PROCESS_CAN_SEND_NEW_NEEY_DATA;
	}

	/* PROCESS_CAN_SEND_NEW_CELL_DATA --------------------------------------------------------*/
	if (can_task_scheduler & PROCESS_CAN_SEND_NEW_CELL_DATA) {
		if (!HAL_CAN_IsTxMessagePending(&hcan, TxMailbox)) {
			TxHeader.DLC=sizeof(_BMS_CELL_DATA);
			memcpy(CanTxData, (uint8_t*)&bms_cell_data[current_cell_2_send],sizeof(_BMS_CELL_DATA));

			if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, CanTxData, &TxMailbox) != HAL_OK) {
				Error_Handler ();
			}
			else {
				//if (++current_cell_2_send >= neey_ctrl.neey_dev_info.CellCount) {
				if (++current_cell_2_send >= MAX_LF280K_CELL_COUNT) {
					can_task_scheduler &= ~PROCESS_CAN_SEND_NEW_CELL_DATA;
				}
				set_signal_led(BLUE_LED, LED_200MS_FLASH);
			}
		}
		else {
			return can_task_scheduler;
		}
	}

	/* PROCESS_CAN_SEND_NEW_BLK_DATA --------------------------------------------------------*/
	if (can_task_scheduler & PROCESS_CAN_SEND_NEW_BLK_DATA) {
		if (!HAL_CAN_IsTxMessagePending(&hcan, TxMailbox)) {

			if(current_blk_data_2_send==0){
				TxHeader.DLC=sizeof(_BMS_BLK_DATA1);
				memcpy(CanTxData, (uint8_t*)&bms_blk_data1,sizeof(_BMS_BLK_DATA1));
			}
			else if (current_blk_data_2_send==1){
				TxHeader.DLC=sizeof(_BMS_BLK_DATA2);
				memcpy(CanTxData, (uint8_t*)&bms_blk_data2,sizeof(_BMS_BLK_DATA2));
			}
			else if (current_blk_data_2_send==2){
				TxHeader.DLC=sizeof(_BMS_BLK_DATA3);
				memcpy(CanTxData, (uint8_t*)&bms_blk_data3,sizeof(_BMS_BLK_DATA3));
			}

			if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, CanTxData, &TxMailbox) != HAL_OK) {
				Error_Handler ();
			}
			else {
				if (++current_blk_data_2_send>=3){
					can_task_scheduler &= ~PROCESS_CAN_SEND_NEW_BLK_DATA;
				}
				set_signal_led(BLUE_LED, LED_200MS_FLASH);
			}
		}else {
			return can_task_scheduler;
		}
	}

	/* PROCESS_CAN_ON_BRDC_MSG --------------------------------------------------------*/
	if (can_task_scheduler & PROCESS_CAN_ON_BRDC_MSG)
	{
		if (CanRxData[0] <= ALIVE_CMD) {
			can_task_scheduler |= PROCESS_CAN_ON_MSG;
		}
		can_task_scheduler &= ~PROCESS_CAN_ON_BRDC_MSG;
	}


	/* PROCESS_CAN_ON_MSG --------------------------------------------------------*/
	if (can_task_scheduler & PROCESS_CAN_ON_MSG)
	{
		switch (CanRxData[0])
		{
		/*****************************************************/
		case ALIVE_CMD:
			alive_timer = main_regs.alive_timeout;
			break;

		/*****************************************************/
		case SYS_RESET_CMD:
			//printf("SYS_RESET\n");
			HAL_NVIC_SystemReset();
			break;

		/*****************************************************/
		case SYS_APP_RESET_CMD:
			//printf("APP_RESET\n");
			JumpToApp();
			break;

		/*****************************************************/
		case SYS_BOOT_CMD:
			//printf("SYS_BOOT\n");
			//leave a message in a bottle for the bootloader,
			//so the btld will not start app again and stay in btld-mode
			*(uint32_t *)_MAGIC_RAM_ADDRESS_ = _MAGIC_RAM_DWORD_;
			JumpToBtld();
			break;

		/*****************************************************/
		case SYS_READ_REG_CMD:
			//printf("ADC_READ_REG_CMD\n");
			sys_reg = (CanRxData[2]<<7)+CanRxData[3];

			len=CanRxData[4];

			if(!len || len >7)
				len=1;

			switch(CanRxData[1]) {
				case GET_SW_INFO_REGS:
					p_sys_reg_offset=(uint8_t *)&main_regs.sw_info;
					max_sys_reg_size=sizeof(_SW_INFO_REGS);
					break;

				case GET_DEV_CFG_REGS:
					p_sys_reg_offset=(uint8_t *)&main_regs.dev_config;
					max_sys_reg_size=sizeof(_DEV_CONFIG_REGS);
					break;

				case GET_BRD_INFO_REGS:
					p_sys_reg_offset=(uint8_t *)&main_regs.board_info;
					max_sys_reg_size=sizeof(_BOARD_INFO_STRUCT);
					break;

				case GET_MAIN_REGS:
				default:
					p_sys_reg_offset=(uint8_t *)&main_regs;
					max_sys_reg_size=sizeof(_MAIN_REGS)-sizeof(_SW_INFO_REGS)-sizeof(_DEV_CONFIG_REGS)-sizeof(_BOARD_INFO_STRUCT);
					break;
			}

			if ((sys_reg + 1) < max_sys_reg_size) {
				if ((sys_reg + len) >= max_sys_reg_size) {
					len = max_sys_reg_size - sys_reg;
				}
				CanTxData[0] = REPLAY_DATA_CMD;
				//CanTxData[1] = sys_reg;
				memcpy((uint8_t *)&CanTxData[1],(p_sys_reg_offset+sys_reg),len);
				//CanTxData[2] = *(((uint8_t *)&main_regs)+sys_reg);
				ReplayHeader.DLC = len+1;
			}
			else {
				CanTxData[0] = REPLAY_AKC_NACK_CMD;
				CanTxData[1] = NACK;
				ReplayHeader.DLC = 2;
			}

			can_task_scheduler |= PROCESS_CAN_SEND_REPLAY;
			break;

		/*****************************************************/
		case SYS_WRITE_REG_CMD:
			//printf("WRITE_REG_CMD\n");
			sys_reg = CanRxData[1];

			if (sys_reg < sizeof(main_regs)) {
				*(((uint8_t *)&main_regs)+sys_reg) = CanRxData[2];
				CanTxData[1] = ACK;
			}
			else {
				CanTxData[1] = NACK;
			}
			ReplayHeader.DLC = 2;
			CanTxData[0] = REPLAY_AKC_NACK_CMD;
			can_task_scheduler |= PROCESS_CAN_SEND_REPLAY;
			break;

		/*****************************************************/
		case PB_SET_CMD:
			//printf("WRITE_REG_CMD\n");
			uint8_t channal = CanRxData[1];
			uint8_t value = CanRxData[2];

			PBalancer_set(channal, value);
			CanTxData[1] = ACK;
			ReplayHeader.DLC = 2;
			CanTxData[0] = REPLAY_AKC_NACK_CMD;
			can_task_scheduler |= PROCESS_CAN_SEND_REPLAY;
			break;

		/*****************************************************/
		case PB_SET_OE_CMD:
			//printf("WRITE_REG_CMD\n");

			if (CanRxData[1])
				PB_OutputEnable(GPIO_PIN_SET);
			else
				PB_OutputEnable(GPIO_PIN_RESET);

			CanTxData[1] = ACK;
			ReplayHeader.DLC = 2;
			CanTxData[0] = REPLAY_AKC_NACK_CMD;
			can_task_scheduler |= PROCESS_CAN_SEND_REPLAY;
			break;

		/*****************************************************/
		case NEEY_SET_CMD:
			//printf("WRITE_REG_CMD\n");

			if (Send_Param_to_neey(CanRxData[1], (uint8_t*)&CanRxData[2] , RxHeader.DLC-2)==HAL_OK) {
				CanTxData[1] = ACK;
			}
			else {
				CanTxData[1] = NACK;
			}

			ReplayHeader.DLC = 2;
			CanTxData[0] = REPLAY_AKC_NACK_CMD;
			can_task_scheduler |= PROCESS_CAN_SEND_REPLAY;
			break;

		/*****************************************************/
		case NEEY_GET_INFO_CMD:
			//printf("ADC_READ_REG_CMD\n");
			sys_reg = (CanRxData[1]<<7)+CanRxData[2];
			len=CanRxData[3];

			if(!len || len >7)
				len=1;

			if (sys_reg < sizeof(_NEEY_INFO)) {
				CanTxData[0] = REPLAY_DATA_CMD;
				//CanTxData[1] = sys_reg;
				memcpy((uint8_t *)&CanTxData[1],(((uint8_t *)&neey_ctrl.neey_dev_info)+sys_reg),len);
				//CanTxData[2] = *(((uint8_t *)&main_regs)+sys_reg);
				ReplayHeader.DLC = len+1;
			}
			else {
				CanTxData[0] = REPLAY_AKC_NACK_CMD;
				CanTxData[1] = NACK;
				ReplayHeader.DLC = 2;
			}
			can_task_scheduler |= PROCESS_CAN_SEND_REPLAY;
			break;

#ifdef __WS2812B__
		/*****************************************************/
		case WS2815_SET_CMD:
			//printf("WRITE_REG_CMD\n");
			setLedValues(CanRxData[1], CanRxData[2], CanRxData[3], CanRxData[4]);
			//setLedValues(uint16_t led, uint8_t r, uint8_t g, uint8_t b);

			CanTxData[1] = ACK;

			ReplayHeader.DLC = 2;
			CanTxData[0] = REPLAY_AKC_NACK_CMD;
			can_task_scheduler |= PROCESS_CAN_SEND_REPLAY;
			break;

			/*****************************************************/
			case WS2815_ENABLE_CMD:
				//printf("WRITE_REG_CMD\n");
				if (enableWS2812b_Leds(CanRxData[1])==HAL_OK) {
					CanTxData[1] = ACK;
				}else {
					CanTxData[1] = NACK;
				}

				ReplayHeader.DLC = 2;
				CanTxData[0] = REPLAY_AKC_NACK_CMD;
				can_task_scheduler |= PROCESS_CAN_SEND_REPLAY;
				break;


#endif
		/*****************************************************/
		case BMS_GET_CELL_DATA_CMD:
			if (prepare_BMS_CellData()) {
				can_task_scheduler |= PROCESS_CAN_SEND_NEW_CELL_DATA;
			}
			else {
				return can_task_scheduler;
			}
			break;

		/*****************************************************/
		case BMS_GET_BLK_DATA_CMD:
			if (prepare_BMS_BLKData()) {
				can_task_scheduler |= PROCESS_CAN_SEND_NEW_BLK_DATA;
			}
			else {
				return can_task_scheduler;
			}
			break;

		/*****************************************************/
		default:
			break;
		}
		can_task_scheduler &= ~PROCESS_CAN_ON_MSG;
	}


	/* PROCESS_CAN_SEND_REPLAY --------------------------------------------------------*/
	if (can_task_scheduler & PROCESS_CAN_SEND_REPLAY)
	{
		if (!HAL_CAN_IsTxMessagePending(&hcan, TxMailbox))
		{
			if (HAL_CAN_AddTxMessage(&hcan, &ReplayHeader, CanTxData, &TxMailbox) != HAL_OK)
			{
				Error_Handler ();
			}

			set_signal_led(BLUE_LED, LED_200MS_FLASH);
			can_task_scheduler &= ~PROCESS_CAN_SEND_REPLAY;
		}
	}

	return can_task_scheduler;
}


/* HAL_CAN_RxFifo0MsgPendingCallback -----------------------------------------*/
/* Interrupt callback to manage incomming can-messages                        */
/*----------------------------------------------------------------------------*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, CanRxData) != HAL_OK)
	{
		Error_Handler();
	}

	if ((RxHeader.StdId == main_regs.can_rx_cmd_id))
	{
		can_task_scheduler |= PROCESS_CAN_ON_MSG;
    	main_task_scheduler |= PROCESS_CAN;
	}else if (RxHeader.StdId == main_regs.can_rx_brdc_cmd_id) {
		can_task_scheduler |= PROCESS_CAN_ON_BRDC_MSG;
    	main_task_scheduler |= PROCESS_CAN;
	}
}


/* HAL_CAN_ErrorCallback -----------------------------------------------------*/
/* Interrupt callback to manage Can Errors                                    */
/*----------------------------------------------------------------------------*/
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *phcan){
	//uint32_t transmitmailbox;
	int retry=0;

	if((phcan->ErrorCode & HAL_CAN_ERROR_TX_ALST0) || (phcan->ErrorCode & HAL_CAN_ERROR_TX_TERR0)){
		HAL_CAN_AbortTxRequest(phcan,CAN_TX_MAILBOX0);
		retry=1;
	}
	if((phcan->ErrorCode & HAL_CAN_ERROR_TX_ALST1) || (phcan->ErrorCode & HAL_CAN_ERROR_TX_TERR1)){
		HAL_CAN_AbortTxRequest(phcan,CAN_TX_MAILBOX1);
		retry=1;
	}
	if((phcan->ErrorCode & HAL_CAN_ERROR_TX_ALST2) || (phcan->ErrorCode & HAL_CAN_ERROR_TX_TERR2)){
		HAL_CAN_AbortTxRequest(phcan,CAN_TX_MAILBOX2);
		retry=1;
	}

	HAL_CAN_ResetError(phcan);

	if(retry==1){
		HAL_CAN_DeactivateNotification(phcan,CAN_IT_TX_MAILBOX_EMPTY);
		HAL_CAN_ActivateNotification(phcan,CAN_IT_TX_MAILBOX_EMPTY);
		//HAL_CAN_AddTxMessage(phcan,&(CanTxList.SendMsgBuff.header),CanTxList.SendMsgBuff.Data,&transmitmailbox);

		//todo: alarm/error-handling
	}
}

/* HAL_CAN_TxCpltCallback ----------------------------------------------------*/
/* Interrupt callback to manage Can Tx Ready                                  */
/*----------------------------------------------------------------------------*/
void CAN_TX_Cplt(CAN_HandleTypeDef* phcan){

}


void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef* phcan){
	CAN_TX_Cplt(phcan);
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef* phcan){
	CAN_TX_Cplt(phcan);
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef* phcan){
	CAN_TX_Cplt(phcan);
}

/* send_broadcast_msg ---------------------------------------------------------*/
/*                                                                            */
/*---------------------------------------------------------------------------*/
void can_send_brdc_msg(uint8_t* p_msg, uint8_t len){

	assert(len<8);
	//CanTxData[0] = SYS_ALLERT_MSG;
	memcpy(CanTxData, p_msg, len);
	AllertHeader.DLC=len;

	HAL_CAN_AddTxMessage(&hcan, &BroadcastHeader, CanTxData, &TxMailbox);

	set_signal_led(BLUE_LED, LED_100MS_FLASH);
}


/* send_allert_msg ---------------------------------------------------------*/
/*                                                                           */
/*----------------------------------------------------------------------------*/
void can_send_allert_msg(uint8_t* p_allert_msg, uint8_t len){

	assert(len<7);
	CanTxData[0] = SYS_ALERT_MSG;
	memcpy(CanTxData+1, p_allert_msg, len);
	AllertHeader.DLC=len+1;

	HAL_CAN_AddTxMessage(&hcan, &AllertHeader, CanTxData, &TxMailbox);

	set_signal_led(BLUE_LED, LED_100MS_FLASH);
}


/* send_trip_msg ---------------------------------------------------------*/
/*                                                                          */
/*----------------------------------------------------------------------------*/
void can_send_trip_msg(void){

	CanTxData[0] = SET_RELAY_CMD;
	CanTxData[1] = 01;
	CanTxData[2] = GPIO_PIN_SET;

	HAL_CAN_AddTxMessage(&hcan, &TripHeader, CanTxData, &TxMailbox);

	set_signal_led(BLUE_LED, LED_100MS_FLASH);
}


/* USER CODE END 1 */
