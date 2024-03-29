/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint8_t main_task_scheduler;
extern uint8_t alive_timer;


typedef enum
{
	NO_CMD,
	SYS_READ_REG_CMD,
	SYS_WRITE_REG_CMD,
	SYS_RESET_CMD,
	SYS_BOOT_CMD,
	SET_RELAY_CMD,
	ALIVE_CMD,
	REPLAY_AKC_NACK_CMD,
	REPLAY_DATA_CMD,
	END_CMD
}_CAN_CMD;


typedef enum
{
	SYS_OK,
	SYS_ACTIVE_START,
	SYS_ACTIVE_FC,
	SYS_ACTIVE_SBC,
	SYS_ACTIVE_STOP,
	SYS_ERROR,
}_SYS_STATE;


/**
 * @struct	REG
 * @brief	Registersatz des Controllers.
 *
 * @note	Der Registersatz wird im RAM und im EEProm gehalten
 */
 typedef struct
 {
 /**
  * @var	unsigned char ctrl
  * @brief	Steuer-Register für den Controller
  *
  *	- Bit 0 -> Bit zum Ein-/Ausschalten
  *	- Bit 7 -> Reset des Controllers auslösen
  *
  * @see	CTRL_REG
  */
  uint8_t			ctrl;

  _SYS_STATE		sys_state;

  uint8_t			monitor_led_state;

  uint8_t			alive_timeout;


 /**
  * @var	uint32_t		can_bitrate
  * @brief
  */
  uint32_t			can_bitrate;


 /**
  * @var	unsigned char dev_signature
  * @brief	Register mit der Gerätekennung
  * @see	__DEV_SIGNATURE__
  * @see	DEV_SIGNATURE_REG
  * @see	config.h
  */
  unsigned char		dev_signature;

 /**
  * @var	unsigned int sw_release
  * @brief	Register mit der Softwareversion
  * @see	__SW_RELEASE__
  * @see	SW_REL_REG
  * @see	config.h
  */
  unsigned int		sw_release;

 /**
  * @var	unsigned int sw_release_date
  * @brief	Register mit dem Datum der Softwareversion
  * Formatierung:
  *	- Byte 0 -> Tag
  *	- BYTE 1 -> Monat
  *	- BYTE 2 -> Jahr
  *	- BYTE 3 -> Jahr
  * @see	__SW_RELEASE_DATE__
  * @see	SW_REL_DATE_REG
  * @see	config.h
  */
  unsigned long		sw_release_date;
 }_MAIN_REGS;


 /**
  * @def		REG_ADDR_AUTO_INC
  * @brief	Flag im Registeradressbyte, welches eine automatische Incrementierung
  *			der Registeradresse nach einer Lese- oder Schreibaktion bewirkt.
  *
  * @note	Die Registeradresse wird vom Master bei jeder Schreibaktion als erstes Byte gesendet.
  * @see		fco_reg_addr
  * @see		TWI-WRITE-Mode
  */
  #define REG_ADDR_AUTO_INC	0x80


extern  _MAIN_REGS main_regs;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* Funktionen(Prototypes) --------------------------------------------------------*/
void set_sys_state (_SYS_STATE sys_state);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI1_DATA_STROBE_Pin GPIO_PIN_0
#define SPI1_DATA_STROBE_GPIO_Port GPIOB
#define SPI1_OE_Pin GPIO_PIN_1
#define SPI1_OE_GPIO_Port GPIOB
#define OneWire_Pin GPIO_PIN_10
#define OneWire_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_12
#define LED_GREEN_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_13
#define LED_RED_GPIO_Port GPIOB
#define LED_BLUE_Pin GPIO_PIN_14
#define LED_BLUE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define PROCESS_NO_TASK			0x00
#define PROCESS_NEEY			0x01
#define PROCESS_CAN				0x02
#define PROCESS_10_MS_TASK		0x04
#define PROCESS_100_MS_TASK		0x08
#define PROCESS_STATUS			0x10

#define ALIVE_TIMEOUT_10MS		15
#define APP_CAN_BITRATE			500000UL

#define __DEV_SIGNATURE__			0x12
#define __SW_RELEASE__				0x0100
#define SW_RELEASE_DAY				12
#define SW_RELEASE_MONTH			12
#define SW_RELEASE_YEAR				2023
#define __SW_RELEASE_DATE__			((SW_RELEASE_DAY<<24 ) | (SW_RELEASE_MONTH<<18) | SW_RELEASE_YEAR)


 /**
  * Bit-Defines für das Controllregister
  */
  #define REG_CTRL_ACTIVATE			0
  #define REG_CTRL_DEACTIVATE		1
  #define REG_CTRL_CRIT_ALLERT		2
  #define REG_CTRL_RESET			7	//!<Reset des Controllers auslösen

 /**
  * Zustände
  */
#define STATE_OFF					0x00	//!<Keine Blinken (LED aus)

#define STATE_OK					0x4F	//!<Zustand alles OK (gleichmäßiges "langsames" Blinken Tastverhältnis 50/50)

#define STATE_WARN					0xCC	//!<Zustand Warnung (gleichmäßiges "schnelles" Blinken Tastverhältnis 50/50)

#define STATE_ERR_UNKNOWN			0x1F	//!<Zustand unbekannter Fehler (gleichmäßiges "sehr schnelles" Blinken Tastverhältnis 50/50)

#define STATE_ERR_HEADSINK_TEMP		0x11	//!<Zustand Fehler Kühlkörper-Temperatur zu hoch (einmal kurzes blinken)


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
