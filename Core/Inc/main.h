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
#include "neey.h"
#include "OneWire.h"


#ifndef __BOARD_TYPE__
	#define __BOARD_TYPE__				BMS_BLK_BOARD
#endif

//
#define __BRD_ID__						0x05

#ifndef __DEV_ID__
	#define __DEV_ID__					(__BOARD_TYPE__ + __BRD_ID__)
#endif

#include <dev_config.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint8_t main_task_scheduler;
extern uint8_t alive_timer;

/* USER CODE BEGIN Private defines */

#define PROCESS_NO_TASK			0x00
#define PROCESS_NEEY			0x01
#define PROCESS_CAN				0x02
#define PROCESS_10_MS_TASK		0x04
#define PROCESS_100_MS_TASK		0x08
#define PROCESS_STATUS			0x10
#define PROCESS_PBALANCER		0x20
#define PROCESS_OW				0x40

#define ALIVE_TIMEOUT_10MS		15
#define APP_CAN_BITRATE			500000UL

#define __DEV_SIGNATURE__		0x12
#define __SW_RELEASE__			0x0101
#define SW_RELEASE_DAY			25
#define SW_RELEASE_MONTH		11
#define SW_RELEASE_YEAR			2024
#define __SW_RELEASE_DATE__		((SW_RELEASE_DAY<<24 ) | (SW_RELEASE_MONTH<<18) | SW_RELEASE_YEAR)
#define __SW_NAME__				"BMS-BLK-APP"



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

#define MAX_LF280K_CELL_COUNT		22

/* USER CODE END Private defines */

//typedef enum
//{
//	ACK = 0x11,
//	NACK= 0x13
//}_REPLAY_TYPE;


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
	 uint16_t	capacity;				//capacity in [Wh]
	 const char manufacturers[16];
	 const char cell_types[8];
	 const char model_codes[8];
	 const char capacity_ah[6];
	 const char energy_wh[6];
	 const char production_date[12];
	 const char original_qr_content[24];	//
 }_LF280K_QR_INFO_STRUCT;

 /**
   * @brief  NEEY Data Structure definition ()
   */
 typedef struct
 {
 	float		max_balance_current;
 	float		start_voltage;
 	float		equalization_voltage;
 	float		sleep_voltage;
 	uint8_t		cell_count;
 	uint8_t		cell_type;
 	uint8_t		buzzer;
 	uint8_t		auto_run;
 	uint16_t	cell_cpacity;
 	uint8_t		data_send_intervall;	//send intervall in 500ms steps
 	uint8_t		config_at_start;		//send config to neey at start
} _NEEY_CONFIG_DATA;

 /**
 * @struct	REG
 * @brief	Registersatz des Controllers.
 *
 * @note	Der Registersatz wird im RAM und im EEProm gehalten
 */
 typedef struct
 {
	 _NEEY_CONFIG_DATA			neey_cfg_data;
	 uint64_t 					temp_sensor_lookup_table[MAXDEVICES_ON_THE_BUS];
	 _LF280K_QR_INFO_STRUCT		lf280k_qr_info[MAX_LF280K_CELL_COUNT];
 }_BMS_BLK_CONFIG_REGS;


/**
 * @struct	REG
 * @brief	Registersatz des Controllers.
 *
 * @note	Der Registersatz wird im RAM und im EEProm gehalten
 */
 typedef struct
 {
	uint8_t						ctrl;
	_SYS_STATE					sys_state;
	uint8_t						monitor_led_state;
	uint8_t						alive_timeout;
	uint32_t					can_rx_cmd_id;
	uint32_t					can_tx_data_id;
	uint32_t					can_tx_heartbeat_id;
	uint32_t 					can_filterMask;
	uint32_t 					can_filterID; // Only accept bootloader CAN message ID
	_BMS_BLK_CONFIG_REGS		cfg_regs;
 }_MAIN_REGS;


#pragma pack(push,1)

typedef struct
{
uint8_t			bms_data_type;
uint8_t			flags_ch_number;
uint16_t		cell_voltage;
uint16_t		cell_resistance;
uint16_t		cell_temperature;
}_BMS_CELL_DATA;

typedef struct
{
uint8_t			bms_data_type;
uint8_t			flags_ch_number;
uint32_t		amt_voltage;
}_BMS_BLK_DATA1;

typedef struct
{
uint8_t			bms_data_type;
uint8_t			flags_ch_number;
uint16_t		ave_voltage;
uint16_t		div_voltage;
}_BMS_BLK_DATA2;

typedef struct
{
uint8_t			bms_data_type;
uint8_t			flags_ch_number;
uint16_t		bal_current;
int16_t			neey_temperatur;
}_BMS_BLK_DATA3;

#pragma pack(pop)


typedef union
{
 uint8_t			array_data[sizeof(_BMS_CELL_DATA)];

_BMS_CELL_DATA		bms_cell_data;
}_BMS_CELL_DATA_UNION;


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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Funktionen(Prototypes)
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void set_sys_state (_SYS_STATE sys_state);
void JumpToBtld(void);
void JumpToApp(void);

/* Private typedef -----------------------------------------------------------*/
typedef void (*pFunction)(void);


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
#define FAN_SPEED_Pin GPIO_PIN_1
#define FAN_SPEED_GPIO_Port GPIOA
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


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
