/******************************************************************************
 * @file           : ws2812b.h
 * @brief          : Ws2812b library header
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 Lars Boegild Thomsen <lbthomsen@gmail.com>.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#ifndef __WS2812B_H
#define __WS2812B_H

#include "main.h"

#define WS2812B_MAX_CHANNEL 22
#define LED_CNT WS2812B_MAX_CHANNEL

// Buffer allocated will be twice this
#define BUFFER_SIZE 24

// LED on/off counts.  PWM timer is running 104 counts.  LED_CNT need to be set to the total counts in the PWM.
#define LED_OFF 1 * LED_CNT / 3 - 1
#define LED_ON 2 * LED_CNT / 3 + 2
#define LED_RESET_CYCLES 10 // Full 24-bit cycles

// Define LED driver state machine states
#define LED_RES 0 // Reset
#define LED_IDL 1 // Idle
#define LED_DAT 2 // Transfer data

#define GL 0 // Green LED
#define RL 1 // Red LED
#define BL 2 // Blue LED

#define WS2812B_INIT_OK 0
#define WS2812B_INIT_MEM 1

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim);

typedef struct {
    TIM_HandleTypeDef *timer0;
    TIM_HandleTypeDef *timer1;
    uint32_t channel0;
    uint32_t channel1;
} ws2823b_init_TypeDef;

uint8_t ws2812b_init(TIM_HandleTypeDef *init_timer, uint32_t init_channel, uint16_t init_leds);

// Set all led values to zero
void zeroLedValues();

// Set a single led value
void setLedValue(uint16_t led, uint8_t color, uint8_t value);

// Set values of all 3 leds
void setLedValues(uint16_t led, uint8_t r, uint8_t g, uint8_t b);

uint8_t enableWS2812b_Leds(uint8_t enable);







typedef enum
{
	NO_WS2812,
	WS2812_RUN_MODE,
	WS2812_MODE_END
}_WS2812_STATE;

/**
 * @struct	REG
 * @brief	Registersatz des Controllers.
 *
 * @note	Der Registersatz wird im RAM und im EEProm gehalten
 */
 typedef struct
 {
	 uint8_t	red_led;				//
	 uint8_t	green_led;				//
	 uint8_t	blue_led;				//
 }_RGB_LED;


uint8_t WS2812_Set(uint8_t led_nr, _RGB_LED rgb_value);


#endif // _WS2812B_H

/*
 * vim: ts=4 nowrap
 */



