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
#include "neey.h"


NEEY_HandleTypeDef hneey;

/* NEEY init function */
void MX_NEEY_Init(void)
{


//	hneey.Instance = NEEY;
  //hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  //hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  //if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

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

