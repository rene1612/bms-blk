/**
  ******************************************************************************
  * @file    neey.c
  * @brief   This file provides code for the ctrl an rading of the neey-balancer
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 René Schoenrock.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include "neey.h"
#include "string.h"
#include "usart.h"


NEEY_HandleTypeDef hneey;

uint8_t neey_task_scheduler;


//UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

//#define UART2_RX_DATA_SIZE	512
#define UART2_RX_BUF_SIZE	(sizeof(_NEEY_RecDataTypeDef)+2)
uint8_t	RxBuf[UART2_RX_BUF_SIZE];
//uint8_t RxDataUART2[UART2_RX_DATA_SIZE];
//uint16_t oldPos=0;
//uint16_t newPos=0;
const char ConnectString[]={'+','C','O','N','N','E','C','T','E','D',0x0D,0x0A,0x00};
const char DisConnectString[]={'+','D','I','S','C','O','N','N','E','C','T','E','D',0x0D,0x0A,0x00};
const char ATString[]={'A','T',0x0D,0x0A,0x00};
const char ATStringOK[]={'O','K',0x0D,0x0A,0x00};

_NEEY_CTRL neey_ctrl;

uint8_t neey_start_DMA (void){
	HAL_StatusTypeDef returnedERR=HAL_OK;
	/* start the DMA again */
	returnedERR = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *) RxBuf, UART2_RX_BUF_SIZE);
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

	return returnedERR;
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART2)
	{
//		if (neey_ctrl.neey_state == AT_MODE) {
//
//		}
		neey_ctrl.cur_rx_transfer.rx_byte_count = Size;

		if (Size >= sizeof(_NEEY_RecDataTypeDef)) {//so we have a fulle data packet?
		  main_task_scheduler |= PROCESS_NEEY;
		  neey_task_scheduler |= PROCESS_NEEY_DATA;
		} else if (Size >= sizeof(_NEEY_RecDevInfoTypeDef)) {
			  main_task_scheduler |= PROCESS_NEEY;
			  neey_task_scheduler |= PROCESS_NEEY_INFO;

		} else {

			neey_start_DMA();
		}

//		oldPos = newPos;  // Update the last position before copying new data
//
//		/* If the data in large and it is about to exceed the buffer size, we have to route it to the start of the buffer
//		 * This is to maintain the circular buffer
//		 * The old data in the main buffer will be overlapped
//		 */
//		if (oldPos+Size > UART2_RX_DATA_SIZE)  // If the current position + new data size is greater than the main buffer
//		{
//			uint16_t datatocopy = UART2_RX_DATA_SIZE-oldPos;  // find out how much space is left in the main buffer
//			memcpy ((uint8_t *)RxDataUART2+oldPos, RxBuf, datatocopy);  // copy data in that remaining space
//
//			oldPos = 0;  // point to the start of the buffer
//			memcpy ((uint8_t *)RxDataUART2, (uint8_t *)RxBuf+datatocopy, (Size-datatocopy));  // copy the remaining data
//			newPos = (Size-datatocopy);  // update the position
//		}
//
//		/* if the current position + new data size is less than the main buffer
//		 * we will simply copy the data into the buffer and update the position
//		 */
//		else
//		{
//			memcpy ((uint8_t *)RxDataUART2+oldPos, RxBuf, Size);
//			newPos = Size+oldPos;
//		}

	}
}

/* NEEY init function */
void send_to_neey(uint16_t addr, _NEEY_PKT_TYPE pkt_type, uint8_t sub_type, uint8_t* p_data, uint8_t len)
{
	_NEEY_SendDataTypeDef neey_data_pkt;
	uint8_t cs=0;
	uint8_t index;
	uint8_t* p_pkt = (uint8_t*)&neey_data_pkt;

	neey_data_pkt.PacketStart = NEEY_PACKET_START_TX;
	if (pkt_type==NEEY_PACKET_TYPE_cmd) {
		neey_data_pkt.Address = NEEY_ADDR; //adresse ist immer 0x1100???
	}
	else {
		neey_data_pkt.Address = NEEY_ADDR + (1<<8); //adresse ist immer 0x1101???
	}
	neey_data_pkt.PacketType = pkt_type;
	neey_data_pkt.PacketSubType = sub_type;
	neey_data_pkt.PacketLength = 20;
	memset(&neey_data_pkt.data, 0x00, sizeof(neey_data_pkt.data));
	memcpy (&neey_data_pkt.data, p_data, len);

	for(index=0; index<18;index++)
		cs ^= *(p_pkt+index);

	neey_data_pkt.Checksum = cs;
	neey_data_pkt.PacketEnd = 0xFF;

	HAL_UART_Transmit(&huart2, &neey_data_pkt, neey_data_pkt.PacketLength, 1000);
}


/* NEEY init function */
void MX_NEEY_Init(void)
{
	uint8_t init_buffer[10]={};

	neey_ctrl.neey_state = AT_MODE;
	neey_ctrl.cur_rx_transfer.rx_byte_count = 0;
	neey_ctrl.cur_rx_transfer.state = SCAN_PKT_START;
	neey_ctrl.data_pkt_counter = 0;
	neey_task_scheduler = PROCESS_NEEY_NO_TASK;


	neey_start_DMA();

	//printf(ConnectString);
	//printf(DisConnectString);

//	HAL_UART_Transmit(&huart2, ATStringOK, strlen(ATStringOK), 1000);
//	HAL_Delay(20);

	HAL_UART_Transmit(&huart2, ConnectString, strlen(ConnectString), 1000);
	HAL_Delay(20);

	send_to_neey(NEEY_ADDR, NEEY_PACKET_TYPE_info, 0, init_buffer , 10);
	HAL_Delay(20);
	send_to_neey(NEEY_ADDR, NEEY_PACKET_TYPE_unknown, 0, init_buffer , 10);
	HAL_Delay(20);
	send_to_neey(NEEY_ADDR, NEEY_PACKET_TYPE_data, 0, init_buffer , 10);
//	send_to_neey(NEEY_ADDR, NEEY_PACKET_TYPE_data, 0, init_buffer , 10);

	if (main_regs.cfg_regs.neey_cfg_data.cell_count) { //do we need to send neey config (from app config) to neey??
		HAL_Delay(20);
		send_to_neey(NEEY_ADDR, NEEY_PACKET_TYPE_cmd, NEEY_SUB_TYPE_cellcount, (uint8_t*)&main_regs.cfg_regs.neey_cfg_data.cell_count , sizeof(main_regs.cfg_regs.neey_cfg_data.cell_count));
		HAL_Delay(20);
		send_to_neey(NEEY_ADDR, NEEY_PACKET_TYPE_cmd, NEEY_SUB_TYPE_StartVol, (uint8_t*)&main_regs.cfg_regs.neey_cfg_data.start_voltage , sizeof(main_regs.cfg_regs.neey_cfg_data.start_voltage));
		HAL_Delay(20);
		send_to_neey(NEEY_ADDR, NEEY_PACKET_TYPE_cmd, NEEY_SUB_TYPE_MaxBalCurrent, (uint8_t*)&main_regs.cfg_regs.neey_cfg_data.max_balance_current , sizeof(main_regs.cfg_regs.neey_cfg_data.max_balance_current));
		HAL_Delay(20);
		send_to_neey(NEEY_ADDR, NEEY_PACKET_TYPE_cmd, NEEY_SUB_TYPE_SleepVol, (uint8_t*)&main_regs.cfg_regs.neey_cfg_data.sleep_voltage , sizeof(main_regs.cfg_regs.neey_cfg_data.sleep_voltage));
		HAL_Delay(20);
		send_to_neey(NEEY_ADDR, NEEY_PACKET_TYPE_cmd, NEEY_SUB_TYPE_Buzzer, (uint8_t*)&main_regs.cfg_regs.neey_cfg_data.buzzer , sizeof(main_regs.cfg_regs.neey_cfg_data.buzzer));
		HAL_Delay(20);
		send_to_neey(NEEY_ADDR, NEEY_PACKET_TYPE_cmd, NEEY_SUB_TYPE_BatType, (uint8_t*)&main_regs.cfg_regs.neey_cfg_data.cell_type , sizeof(main_regs.cfg_regs.neey_cfg_data.cell_type));
		HAL_Delay(20);
		send_to_neey(NEEY_ADDR, NEEY_PACKET_TYPE_cmd, NEEY_SUB_TYPE_BatCap, (uint8_t*)&main_regs.cfg_regs.neey_cfg_data.cell_cpacity , sizeof(main_regs.cfg_regs.neey_cfg_data.cell_cpacity));
		HAL_Delay(20);
		send_to_neey(NEEY_ADDR, NEEY_PACKET_TYPE_cmd, NEEY_SUB_TYPE_EquVol, (uint8_t*)&main_regs.cfg_regs.neey_cfg_data.equalization_voltage , sizeof(main_regs.cfg_regs.neey_cfg_data.equalization_voltage));
	}
//	hneey.Instance = NEEY;
//hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
//hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
//if (HAL_RTC_Init(&hrtc) != HAL_OK)
//	{
//	  Error_Handler();
//	}

}



void HAL_NEEY_MspInit(NEEY_HandleTypeDef* neeyHandle)
{

 // if(neeyHandle->Instance==NEEY)
  {
//    HAL_PWR_EnableBkUpAccess();
    /* Enable BKP CLK enable for backup registers */
//    __HAL_RCC_BKP_CLK_ENABLE();
    /* RTC clock enable */
//    __HAL_RCC_RTC_ENABLE();
  }
}


void HAL_NEEY_MspDeInit(NEEY_HandleTypeDef* neeyHandle)
{

 // if(neeyHandle->Instance==NEEY)
  {
    /* Peripheral clock disable */
//    __HAL_RCC_RTC_DISABLE();
  }
}


uint8_t	check_data_pkt_NEEY(void* p_pkt_buf) {

	uint8_t cs=0;
	uint8_t i;
	uint16_t index;
	uint8_t* p_pkt = (uint8_t*)p_pkt_buf;
	_NEEY_RecDataTypeDef* p_rec_data_pkt = (_NEEY_RecDataTypeDef*)p_pkt_buf;

	if (p_rec_data_pkt->PacketStart != NEEY_PACKET_START_RX)
		return HAL_ERROR;

	for(index=0; index < sizeof(_NEEY_RecDataTypeDef)-2; index++)
		cs += *(p_pkt+index);

	if(p_rec_data_pkt->Checksum != cs || p_rec_data_pkt->PacketEnd != NEEY_PACKET_END || p_rec_data_pkt->PacketType != NEEY_PACKET_TYPE_data )
		return HAL_ERROR;

	if (!neey_ctrl.data_lock) {

		neey_ctrl.data_lock = 1;

		for(i=0; i < NEEY_CHANNEL_COUNT; i++) {
			neey_ctrl.cell_data[i].voltage =(uint16_t)(p_rec_data_pkt->CellVoltage[i]*1000);
			neey_ctrl.cell_data[i].resistance =(uint16_t)(p_rec_data_pkt->CellValue[i]*1000);
		}

		neey_ctrl.neey_dev_data.AmtVol = (uint16_t)(p_rec_data_pkt->AmtVol*1000);
		neey_ctrl.neey_dev_data.AveVol = (uint16_t)(p_rec_data_pkt->AveVol*1000);
		neey_ctrl.neey_dev_data.DiffVol = (uint16_t)(p_rec_data_pkt->DiffVol*1000);
		neey_ctrl.neey_dev_data.Temperatur = (int16_t)(p_rec_data_pkt->Temperatur1*100);
		neey_ctrl.neey_dev_data.BalCurrent = (int16_t)(p_rec_data_pkt->BalCurrent*1000);

		neey_ctrl.data_pkt_counter++;
		if (!(neey_ctrl.data_pkt_counter % main_regs.cfg_regs.neey_cfg_data.data_send_intervall)){
		  //main_task_scheduler |= PROCESS_CAN;
		//	can_task_scheduler |= PROCESS_CAN_SEND_NEW_NEEY_DATA;
		  //neey_task_scheduler |= PROCESS_NEEY_INFO;
			//send the data via can-bus
		}
		neey_ctrl.data_lock = 0;
	}

	return HAL_OK;
}


uint8_t	check_info_pkt_NEEY(void* p_pkt_buf) {

	uint8_t cs=0;
	uint16_t index;
	uint8_t* p_pkt = (uint8_t*)p_pkt_buf;
	_NEEY_RecDevInfoTypeDef* p_rec_info_pkt = (_NEEY_RecDevInfoTypeDef*)p_pkt_buf;


	if (p_rec_info_pkt->PacketStart != NEEY_PACKET_START_RX)
		return HAL_ERROR;

	for(index=0; index < sizeof(_NEEY_RecDevInfoTypeDef)-2; index++)
		cs += *(p_pkt+index);


	if(p_rec_info_pkt->Checksum != cs || p_rec_info_pkt->PacketEnd != NEEY_PACKET_END)
		return HAL_ERROR;

	memcpy(neey_ctrl.neey_dev_info.NeeyDevType, p_rec_info_pkt->NeeyDevType, sizeof(p_rec_info_pkt->NeeyDevType));
	memcpy(neey_ctrl.neey_dev_info.NeeyMFD, p_rec_info_pkt->NeeyMFD, sizeof(p_rec_info_pkt->NeeyMFD));
	memcpy(neey_ctrl.neey_dev_info.NeeyVersions, p_rec_info_pkt->NeeyVersions, sizeof(p_rec_info_pkt->NeeyVersions));

	return HAL_OK;

}

uint8_t	process_NEEY(void)
{

	if (neey_task_scheduler & PROCESS_NEEY_DATA)
	{

		if(check_data_pkt_NEEY((void*)RxBuf) == HAL_OK) {
			neey_start_DMA ();
		}

		neey_task_scheduler &= ~PROCESS_NEEY_DATA;
	}

	if (neey_task_scheduler & PROCESS_NEEY_INFO)
	{

		if(check_info_pkt_NEEY((void*)RxBuf) == HAL_OK) {
			neey_start_DMA ();
		}

		neey_task_scheduler &= ~PROCESS_NEEY_INFO;
	}

	return neey_task_scheduler;
}
