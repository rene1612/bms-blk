/**
  ******************************************************************************
  * @file    neey.h
  * @brief   This file contains all the function prototypes for
  *          the neey.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 Rene Schoenrock.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NEEY_H__
#define __NEEY_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/**
  * @brief  NEEY Configuration Structure definition
  */
typedef struct
{
  uint32_t AsynchPrediv;    /*!< Specifies the RTC Asynchronous Predivider value.
                                 This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFFFFF  or RTC_AUTO_1_SECOND
                                 If RTC_AUTO_1_SECOND is selected, AsynchPrediv will be set automatically to get 1sec timebase */

  uint32_t OutPut;          /*!< Specifies which signal will be routed to the RTC Tamper pin.
                                 This parameter can be a value of @ref RTC_output_source_to_output_on_the_Tamper_pin */

} NEEY_InitTypeDef;


/**
  * @brief  Time Handle Structure definition
  */
typedef struct __RTC_HandleTypeDef
{
  //NEEY_TypeDef                 *Instance;  /*!< Register base address    */

  NEEY_InitTypeDef             Init;       /*!< RTC required parameters  */

  //RTC_DateTypeDef             DateToUpdate;       /*!< Current date set by user and updated automatically  */

  //HAL_LockTypeDef             Lock;       /*!< RTC locking object       */

  //__IO HAL_RTCStateTypeDef    State;      /*!< Time communication state */

#if (USE_HAL_NEEY_REGISTER_CALLBACKS == 1)
  void (* AlarmAEventCallback)(struct __RTC_HandleTypeDef *hrtc);           /*!< RTC Alarm A Event callback         */

  void (* Tamper1EventCallback)(struct __RTC_HandleTypeDef *hrtc);          /*!< RTC Tamper 1 Event callback        */

  void (* MspInitCallback)(struct __RTC_HandleTypeDef *hrtc);               /*!< RTC Msp Init callback              */

  void (* MspDeInitCallback)(struct __RTC_HandleTypeDef *hrtc);             /*!< RTC Msp DeInit callback            */

#endif /* (USE_HAL_NEEY_REGISTER_CALLBACKS) */

} NEEY_HandleTypeDef;

extern NEEY_HandleTypeDef hneey;



void MX_NEEY_Init(void);


#ifdef __cplusplus
}
#endif

#endif /* __NEEY_H__ */

