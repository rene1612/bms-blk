/**
  ******************************************************************************
  * @file    passive_balance.h
  * @brief   This file contains all the function prototypes for
  *          the passive_balance.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 Rene Schoenrock.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PASSIVE_BALANCER_H__
#define __PASSIVE_BALANCER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"



typedef enum
{
	NO_PB,
	PB_RUN_MODE,
	PB_MODE_END
}_PBALANCER_STATE;



void PBalancer_init(void);

uint8_t	process_PBalancer(void);


#ifdef __cplusplus
}
#endif

#endif /* __NEEY_H__ */

