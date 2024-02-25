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
#include "can.h"


#define UART2_RX_BUF_SIZE	(sizeof(_NEEY_RecDataTypeDef)+2)


NEEY_HandleTypeDef hneey;

uint8_t neey_task_scheduler;

//UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

uint8_t	RxBuf[UART2_RX_BUF_SIZE];
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
	_NEEY_RX_HEADER* p_hdr = (_NEEY_RX_HEADER *)RxBuf;

	if (huart->Instance == USART2)
	{
		neey_ctrl.cur_rx_transfer.rx_byte_count = Size;

		if (Size >= sizeof(_NEEY_RX_HEADER)) {

			switch(p_hdr->PacketType) {

				case data:
					if (Size == sizeof(_NEEY_RecDataTypeDef)) {//so we have a full data packet?
						  main_task_scheduler |= PROCESS_NEEY;
						  neey_task_scheduler |= PROCESS_NEEY_DATA;
					}else
						neey_start_DMA();
					break;

				case info:
					if (Size == sizeof(_NEEY_RecDevInfoTypeDef)) {//so we have an info packet?
					  main_task_scheduler |= PROCESS_NEEY;
					  neey_task_scheduler |= PROCESS_NEEY_INFO;
					}else
						neey_start_DMA();
					break;

				case param:
					if (Size == sizeof(_NEEY_RecDevParmTypeDef)) {//so we have an info packet?
					  main_task_scheduler |= PROCESS_NEEY;
					  neey_task_scheduler |= PROCESS_NEEY_PARAM;
					}else
						neey_start_DMA();
					break;

				default:
					neey_start_DMA();
					break;
			}

		}
		else { //if the size does not match we ignore the data and start dma again

		// maybe we got cmd-response from neey here???

		//		if (neey_ctrl.neey_state == AT_MODE) {
		//
		//		}
			neey_start_DMA();
		}
	}
}


/* NEEY helper function */
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

	for(index=0; index<18; index++)
		cs ^= *(p_pkt+index);

	neey_data_pkt.Checksum = cs;
	neey_data_pkt.PacketEnd = NEEY_PACKET_END;

	HAL_UART_Transmit(&huart2, (uint8_t *)&neey_data_pkt, neey_data_pkt.PacketLength, 1000);
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

	HAL_UART_Transmit(&huart2, (uint8_t *)ConnectString, strlen(ConnectString), 1000);
	HAL_Delay(20);

	send_to_neey(NEEY_ADDR, NEEY_PACKET_TYPE_info, 0, init_buffer , 10);
	//HAL_Delay(100);


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


void check_and_send_config_data_NEEY(void) {

	uint8_t init_buffer[10]={};

	//todo: better to check the current config from neey direct
	//neey sends info packet after connect message (maybe config is included there)???
	if (main_regs.cfg_regs.neey_cfg_data.config_at_start) { //do we need to send neey config (from app config) to neey??

		if (neey_ctrl.neey_dev_info.Buzzer==0 || neey_ctrl.neey_dev_info.Buzzer!=main_regs.cfg_regs.neey_cfg_data.buzzer) {
			send_to_neey(NEEY_ADDR, NEEY_PACKET_TYPE_cmd, NEEY_SUB_TYPE_Buzzer, (uint8_t*)&main_regs.cfg_regs.neey_cfg_data.buzzer , sizeof(main_regs.cfg_regs.neey_cfg_data.buzzer));
			HAL_Delay(100);
		}

		if (neey_ctrl.neey_dev_info.CellCount==0 || neey_ctrl.neey_dev_info.CellCount!=main_regs.cfg_regs.neey_cfg_data.cell_count) {
			send_to_neey(NEEY_ADDR, NEEY_PACKET_TYPE_cmd, NEEY_SUB_TYPE_cellcount, (uint8_t*)&main_regs.cfg_regs.neey_cfg_data.cell_count , sizeof(main_regs.cfg_regs.neey_cfg_data.cell_count));
			//maybe the neey sends answer to command and better to wait for it???
			HAL_Delay(100);
		}

		if (neey_ctrl.neey_dev_info.StartVol==0 || neey_ctrl.neey_dev_info.StartVol!=main_regs.cfg_regs.neey_cfg_data.start_voltage) {
			send_to_neey(NEEY_ADDR, NEEY_PACKET_TYPE_cmd, NEEY_SUB_TYPE_StartVol, (uint8_t*)&main_regs.cfg_regs.neey_cfg_data.start_voltage , sizeof(main_regs.cfg_regs.neey_cfg_data.start_voltage));
			HAL_Delay(100);
		}

		if (neey_ctrl.neey_dev_info.MaxBalCurrent==0 || neey_ctrl.neey_dev_info.MaxBalCurrent!=main_regs.cfg_regs.neey_cfg_data.max_balance_current) {
			send_to_neey(NEEY_ADDR, NEEY_PACKET_TYPE_cmd, NEEY_SUB_TYPE_MaxBalCurrent, (uint8_t*)&main_regs.cfg_regs.neey_cfg_data.max_balance_current , sizeof(main_regs.cfg_regs.neey_cfg_data.max_balance_current));
			HAL_Delay(100);
		}

		if (neey_ctrl.neey_dev_info.SleepVol==0 || neey_ctrl.neey_dev_info.SleepVol!=main_regs.cfg_regs.neey_cfg_data.sleep_voltage) {
			send_to_neey(NEEY_ADDR, NEEY_PACKET_TYPE_cmd, NEEY_SUB_TYPE_SleepVol, (uint8_t*)&main_regs.cfg_regs.neey_cfg_data.sleep_voltage , sizeof(main_regs.cfg_regs.neey_cfg_data.sleep_voltage));
			HAL_Delay(100);
		}

		if (neey_ctrl.neey_dev_info.BatType==0 || neey_ctrl.neey_dev_info.BatType!=main_regs.cfg_regs.neey_cfg_data.cell_type) {
			send_to_neey(NEEY_ADDR, NEEY_PACKET_TYPE_cmd, NEEY_SUB_TYPE_BatType, (uint8_t*)&main_regs.cfg_regs.neey_cfg_data.cell_type , sizeof(main_regs.cfg_regs.neey_cfg_data.cell_type));
			HAL_Delay(100);
		}

		if (neey_ctrl.neey_dev_info.BatCap==0 || neey_ctrl.neey_dev_info.BatCap!=main_regs.cfg_regs.neey_cfg_data.cell_cpacity) {
			send_to_neey(NEEY_ADDR, NEEY_PACKET_TYPE_cmd, NEEY_SUB_TYPE_BatCap, (uint8_t*)&main_regs.cfg_regs.neey_cfg_data.cell_cpacity , sizeof(main_regs.cfg_regs.neey_cfg_data.cell_cpacity));
			HAL_Delay(100);
		}

		if (neey_ctrl.neey_dev_info.EquVol==0 || neey_ctrl.neey_dev_info.EquVol!=main_regs.cfg_regs.neey_cfg_data.equalization_voltage) {
			send_to_neey(NEEY_ADDR, NEEY_PACKET_TYPE_cmd, NEEY_SUB_TYPE_EquVol, (uint8_t*)&main_regs.cfg_regs.neey_cfg_data.equalization_voltage , sizeof(main_regs.cfg_regs.neey_cfg_data.equalization_voltage));
			HAL_Delay(100);
		}
	}

	//timeout???
//	  uint32_t tickstart;
//	  /* Get tick */
//	  tickstart = HAL_GetTick();
//
//	  /* Wait initialisation acknowledge */
//	  while ((hcan->Instance->MSR & CAN_MSR_INAK) == 0U)
//	  {
//	    if ((HAL_GetTick() - tickstart) > CAN_TIMEOUT_VALUE)
//	    {
//	      /* Update error code */
//	      return HAL_ERROR;
//	    }

	//start cmd for sending neey-data???
	send_to_neey(NEEY_ADDR, NEEY_PACKET_TYPE_data, 0, init_buffer , 10);
//	send_to_neey(NEEY_ADDR, NEEY_PACKET_TYPE_data, 0, init_buffer , 10);

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

		//todo
		// get the other data that are unknown by now
		// f.e. the cells that are in balancing process

		neey_ctrl.data_pkt_counter++;

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


uint8_t	check_param_pkt_NEEY(void* p_pkt_buf) {

	uint8_t cs=0;
	uint16_t index;
	uint8_t* p_pkt = (uint8_t*)p_pkt_buf;
	_NEEY_RecDevParmTypeDef* p_rec_param_pkt = (_NEEY_RecDevParmTypeDef*)p_pkt_buf;


	if (p_rec_param_pkt->pkt_header.PacketStart != NEEY_PACKET_START_RX)
		return HAL_ERROR;

	for(index=0; index < sizeof(_NEEY_RecDevParmTypeDef)-2; index++)
		cs += *(p_pkt+index);

	if(p_rec_param_pkt->Checksum != cs || p_rec_param_pkt->PacketEnd != NEEY_PACKET_END)
		return HAL_ERROR;


	neey_ctrl.neey_dev_info.CellCount 		= p_rec_param_pkt->cell_count;
	neey_ctrl.neey_dev_info.StartVol 		= p_rec_param_pkt->start_voltage;
	neey_ctrl.neey_dev_info.MaxBalCurrent 	= p_rec_param_pkt->max_balance_current;
	neey_ctrl.neey_dev_info.SleepVol 		= p_rec_param_pkt->sleep_voltage;
	neey_ctrl.neey_dev_info.Buzzer 			= p_rec_param_pkt->buzzer;
	neey_ctrl.neey_dev_info.BatType 		= p_rec_param_pkt->bat_type;
	neey_ctrl.neey_dev_info.BatCap 			= p_rec_param_pkt->bat_capacity;
	neey_ctrl.neey_dev_info.EquVol 			= p_rec_param_pkt->equ_voltage;

	return HAL_OK;

}


uint8_t	process_NEEY(void)
{
	uint8_t init_buffer[10]={};

	if (neey_task_scheduler & PROCESS_NEEY_DATA)
	{

		if(check_data_pkt_NEEY((void*)RxBuf) == HAL_OK) {

			//check if we need to send new neey data via can
			if (!(neey_ctrl.data_pkt_counter % main_regs.cfg_regs.neey_cfg_data.data_send_intervall)) {

				main_task_scheduler |= PROCESS_CAN;
				can_task_scheduler |= PROCESS_CAN_SEND_NEW_NEEY_DATA;
			  //neey_task_scheduler |= PROCESS_NEEY_INFO;
				//send the data via can-bus
			}
		}

		neey_start_DMA ();	//start over with serial rec of neey data

		neey_task_scheduler &= ~PROCESS_NEEY_DATA;
	}


	if (neey_task_scheduler & PROCESS_NEEY_INFO)
	{

		if(check_info_pkt_NEEY((void*)RxBuf) == HAL_OK) {

			//what to do with new and valid neey-info data
			//send via can???
			//send config to neey if info data differs from cur neey-conig

			send_to_neey(NEEY_ADDR, NEEY_PACKET_TYPE_param, 0, init_buffer , 10);
		}

		neey_start_DMA ();	//start over with serial rec of neey data

		neey_task_scheduler &= ~PROCESS_NEEY_INFO;
	}


	if (neey_task_scheduler & PROCESS_NEEY_PARAM)
	{

		if(check_param_pkt_NEEY((void*)RxBuf) == HAL_OK) {

			//what to do with new and valid neey-info data
			//send via can???
			//send config to neey if info data differs from cur neey-conig

			check_and_send_config_data_NEEY();
		}

		neey_start_DMA ();	//start over with serial rec of neey data

		neey_task_scheduler &= ~PROCESS_NEEY_PARAM;
	}

	return neey_task_scheduler;
}
