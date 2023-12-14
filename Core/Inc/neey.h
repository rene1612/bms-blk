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



#define NEEY_CHANNEL_COUNT		24
#define NEEY_PACKET_START_TX	0x55AA
#define NEEY_PACKET_START_RX	0xAA55
#define NEEY_PACKET_END			0xFF


/**
  * Packet-Type defines
  * */
#define	NEEY_PACKET_TYPE_info 			0x01
#define	NEEY_PACKET_TYPE_data 			0x02
#define	NEEY_PACKET_TYPE_unknown		0x04
#define	NEEY_PACKET_TYPE_cmd 			0x05


/**
  * Sub-Type defines
  * */
#define	NEEY_SUB_TYPE_cellcount 		0x01
#define	NEEY_SUB_TYPE_StartVol 			0x02
#define	NEEY_SUB_TYPE_MaxBalCurrent 	0x03
#define	NEEY_SUB_TYPE_SleepVol 			0x04
#define	NEEY_SUB_TYPE_Buzzer 			0x14
#define	NEEY_SUB_TYPE_BatType 			0x15
#define	NEEY_SUB_TYPE_BatCap 			0x16
#define	NEEY_SUB_TYPE_EquVol 			0x17
#define	NEEY_SUB_TYPE_run 				0x0D


/**
  * Bettery-Type defines
  * */
#define	NEEY_BATTERY_TYPE_NCM 			0x01
#define	NEEY_BATTERY_TYPE_LFP 			0x02
#define	NEEY_BATTERY_TYPE_LTO 			0x03
#define	NEEY_BATTERY_TYPE_PBAC			0x04


/**
  * CMD-ON/OFF-Type defines
  * */
#define	NEEY_ON 						0x01
#define	NEEY_OFF 						0x00


/**
  * CMD-ON/OFF-Type defines
  * */
#define PROCESS_NEEY_NO_TASK			0x00
#define PROCESS_NEEY_SEND					0x01
#define PROCESS_NEEY_					0x02
#define PROCESS_NEEY_					0x04


typedef enum
{
	NO_NEEY,
	AT_MODE,
	RUN_MODE,
	NEEY_MODE_END
}_NEEY_STATE;


#pragma pack(push,1)

/**
  * @brief  NEEY Data Structure definition (from NEEY to MC)
  */
typedef struct
{
  uint16_t 	PacketStart;    					/*!< Start of the data packet, always 0x55AA */

  uint16_t 	Address;							/*!< adress??? */

  uint8_t 	PacketType;							/*!< Type, 0x02 (Daten), 0x01 (info?), 0x04 (???), 0x05 (CMD?)*/

  uint16_t 	PacketLength;						/*!< Länge des gesamten Datenpacketes (eigentlich immer 300) */

  uint8_t 	PacketCount;						/*!< wird einfach für jedes empf. Datenpaket hochgezählt */

  float 	CellVoltage[NEEY_CHANNEL_COUNT];	/*!< Cell-voltage */

  float		CellValue[NEEY_CHANNEL_COUNT];		/*!< the big unkonwn, strange float values releated to cell ca. 0,26???   */

  float 	AmtVol;								/*!< pack voltage of all cells  */

  float 	AveVol;								/*!< average voltage of all cells  */

  float 	DiffVol;							/*!< diff-voltage max cell volt. - min cell volt.  */

  float		Unknown1;							/*!< dont konw  */

  float 	BalCurrent;							/*!< current balance current  */

  float 	Temperatur1;						/*!< neey temperatur  */

  float 	Temperatur2;						/*!< neey temperatur again??? */

  uint8_t	dummy[69];							/*!< the big unkonwn 2  */

  uint8_t	Checksum;							/*!< einfache 8-Bit Überlauf-Prüfsumme */

  uint8_t	PacketEnd;							/*!< End of the data packet, always 0xFF */

} _NEEY_RecDataTypeDef;


/**
  * @brief  NEEY Dev Info Structure definition (from NEEY to MC)
  */
typedef struct
{
  uint16_t 	PacketStart;    					/*!< Start of the data packet, always 0x55AA */

  uint16_t 	Address;							/*!< adress??? */

  uint8_t 	PacketType;							/*!< Type, 0x02 (Daten), 0x01 (info?), 0x04 (???), 0x05 (CMD?)*/

  uint16_t 	PacketLength;						/*!< Länge des gesamten Datenpacketes (eigentlich immer 100) */

  char		NeeyDevType[16];

  char		NeeyVersions[24];					/*!< string mit versionsnummern HW...,ZH....,V....  */

  char		NeeyMFD[12];						/*!< string mit dem Manufacturing Date YYYYMMDD  */

  uint8_t	dummy[38];							/*!< the big unkonwn 2  */

  uint8_t	Checksum;							/*!< einfache 8-Bit Überlauf-Prüfsumme */

  uint8_t	PacketEnd;							/*!< End of the data packet, always 0xFF */

} _NEEY_RecDevInfoTypeDef;



/**
  * @brief  NEEY Data Structure definition (from MC to NEEY)
  */
typedef struct
{
  uint16_t 	PacketStart;    					/*!< Start of the data packet, always 0xAA55 */

  uint16_t 	Address;							/*!< adress??? */

  uint8_t 	PacketType;							/*!< Type, 0x02 (Daten), 0x01 (info?), 0x04 (???), 0x05 (CMD?)*/

  uint8_t 	PacketSubType;						/*!< SubType, @seeSub-Type defines */

  uint16_t 	PacketLength;						/*!< Länge des gesamten Datenpacketes (eigentlich immer 20) */

  uint8_t 	data[10];								/*!< data to be send to neey (format see subtype */

  uint8_t	Checksum;							/*!< einfache 8-Bit Xor-Prüfsumme */

  uint8_t	PacketEnd;							/*!< End of the data packet, always 0xFF */

} _NEEY_SendDataTypeDef;

#pragma pack(pop)


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
typedef struct __NEEY_HandleTypeDef
{
  //NEEY_TypeDef                 *Instance;  /*!< Register base address    */

  NEEY_InitTypeDef             Init;       /*!< RTC required parameters  */

  //RTC_DateTypeDef             DateToUpdate;       /*!< Current date set by user and updated automatically  */

  //HAL_LockTypeDef             Lock;       /*!< RTC locking object       */

  //__IO HAL_RTCStateTypeDef    State;      /*!< Time communication state */

#if (USE_HAL_NEEY_REGISTER_CALLBACKS == 1)
  void (* AlarmAEventCallback)(struct __RTC_HandleTypeDef *hrtc);           /*!< Neey Alarm A Event callback         */

  void (* Tamper1EventCallback)(struct __RTC_HandleTypeDef *hrtc);          /*!< Neey Tamper 1 Event callback        */

  void (* MspInitCallback)(struct __RTC_HandleTypeDef *hrtc);               /*!< Neey Msp Init callback              */

  void (* MspDeInitCallback)(struct __RTC_HandleTypeDef *hrtc);             /*!< Neey Msp DeInit callback            */

#endif /* (USE_HAL_NEEY_REGISTER_CALLBACKS) */

} NEEY_HandleTypeDef;

extern NEEY_HandleTypeDef hneey;



void MX_NEEY_Init(void);

uint8_t	process_NEEY(void);


#ifdef __cplusplus
}
#endif

#endif /* __NEEY_H__ */

