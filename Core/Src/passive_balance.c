/**
  ******************************************************************************
  * @file    passive_balance.c
  * @brief   This file provides code for the ctrl an rading of the passive-balancer
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 René Schoenrock.
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
#include "main.h"
#include "spi.h"
#include "gpio.h"


uint8_t spi_buffer[3]={0x00,0x00,0x00};


void PB_OutputEnable(GPIO_PinState PinState) {
	HAL_GPIO_WritePin(SPI1_OE_GPIO_Port, SPI1_OE_Pin, !PinState);
}

void PBalancer_init(void)
{

	PB_OutputEnable(0);

	//send {0x00, 0x00, 0x00} to passive balancer (shut off everything)
	HAL_SPI_Transmit_IT(&hspi1, spi_buffer, 3);
	HAL_GPIO_WritePin(SPI1_DATA_STROBE_GPIO_Port, SPI1_DATA_STROBE_Pin, GPIO_PIN_SET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(SPI1_DATA_STROBE_GPIO_Port, SPI1_DATA_STROBE_Pin, GPIO_PIN_RESET);


}



uint8_t	process_PBalancer(void)
{
	return 0;
}
