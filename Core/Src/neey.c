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
#include <stdio.h>
#include "neey.h"
#include "string.h"
#include "usart.h"


NEEY_HandleTypeDef hneey;


//UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

#define UART2_RX_DATA_SIZE	512
#define UART2_RX_BUF_SIZE	272
uint8_t	RxBuf[UART2_RX_BUF_SIZE];
uint8_t RxDataUART2[UART2_RX_DATA_SIZE];
uint16_t oldPos=0;
uint16_t newPos=0;
const char ConnectString[]={'+','C','O','N','N','E','C','T',0x0D,0x0A};
const char DisConnectString[]={'+','D','I','S','C','O','N','N','E','C','T',0x0D,0x0A};


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART2)
	{
		oldPos = newPos;  // Update the last position before copying new data

		/* If the data in large and it is about to exceed the buffer size, we have to route it to the start of the buffer
		 * This is to maintain the circular buffer
		 * The old data in the main buffer will be overlapped
		 */
		if (oldPos+Size > UART2_RX_DATA_SIZE)  // If the current position + new data size is greater than the main buffer
		{
			uint16_t datatocopy = UART2_RX_DATA_SIZE-oldPos;  // find out how much space is left in the main buffer
			memcpy ((uint8_t *)RxDataUART2+oldPos, RxBuf, datatocopy);  // copy data in that remaining space

			oldPos = 0;  // point to the start of the buffer
			memcpy ((uint8_t *)RxDataUART2, (uint8_t *)RxBuf+datatocopy, (Size-datatocopy));  // copy the remaining data
			newPos = (Size-datatocopy);  // update the position
		}

		/* if the current position + new data size is less than the main buffer
		 * we will simply copy the data into the buffer and update the position
		 */
		else
		{
			memcpy ((uint8_t *)RxDataUART2+oldPos, RxBuf, Size);
			newPos = Size+oldPos;
		}

		/* start the DMA again */
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *) RxBuf, UART2_RX_BUF_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	}
}


/* NEEY init function */
void MX_NEEY_Init(void)
{

  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuf, UART2_RX_BUF_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

  printf(ConnectString);
  printf(DisConnectString);

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


uint8_t	process_NEEY(void)
{
	return 0;
}
