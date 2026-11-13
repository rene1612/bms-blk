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
#define NEEY_ADDR				0x11


/**
  * Packet-Type defines
  * */
#define	NEEY_PACKET_TYPE_info 			0x01
#define	NEEY_PACKET_TYPE_data 			0x02
#define	NEEY_PACKET_TYPE_param			0x04
#define	NEEY_PACKET_TYPE_cmd 			0x05


typedef enum {
	info =		NEEY_PACKET_TYPE_info,
	data =		NEEY_PACKET_TYPE_data,
	param =		NEEY_PACKET_TYPE_param,
	cmd	=		NEEY_PACKET_TYPE_cmd,
}_NEEY_PKT_TYPE;

/**
  * Sub-Type defines
  * */
#define	NEEY_SUB_TYPE_cellcount 		0x01	/*!<Qty (S) 1Byte 1-Byte unsignedint set range 2-24S */
#define	NEEY_SUB_TYPE_StartVol 			0x02	/*!<Startup DifVol(V) 4Byte 4-Byte float *shown in Notes */
#define	NEEY_SUB_TYPE_MaxBalCurrent 	0x03	/*!<Max Equcur(A) 4Byte 4-Byte float * */
#define	NEEY_SUB_TYPE_SleepVol 			0x04	/*!<StopVol(V) 4Byte 4-Byte float * */
#define	NEEY_SUB_TYPE_BLE_Name 			0x13	/*!<Modifying Bluetooth Name 10Byte Max 10-Byte str The name can only contain letters, numbers,and '-' */
#define	NEEY_SUB_TYPE_Buzzer 			0x14	/*!<Buzzerringing mode 1Byte 1-Byte unsignedint 1: close 2: once */
#define	NEEY_SUB_TYPE_BatType 			0x15	/*!<Battery typesetting 1Byte 1-Byte unsignedint 1:NCM 2:LFP 3:LTO 4:PB*/
#define	NEEY_SUB_TYPE_BatCap 			0x16	/*!<Soc(AH) 4Byte 4-Byte unsignedint * */
#define	NEEY_SUB_TYPE_EquVol 			0x17	/*!<RunVol(V) 4Byte 4-Byte float * */
#define	NEEY_SUB_TYPE_run 				0x0D	/*!<Equalizationswitch control 1Byte 1-Byte unsignedint 0: close 1:open */


typedef enum {
	cellcount =			NEEY_SUB_TYPE_cellcount,
	StartVol =			NEEY_SUB_TYPE_StartVol,
	MaxBalCurrent =		NEEY_SUB_TYPE_MaxBalCurrent,
	SleepVol	=		NEEY_SUB_TYPE_SleepVol,
	Buzzer =			NEEY_SUB_TYPE_Buzzer,
	BatType =			NEEY_SUB_TYPE_BatType,
	BatCap =			NEEY_SUB_TYPE_BatCap,
	EquVol	=			NEEY_SUB_TYPE_EquVol,
	run	=				NEEY_SUB_TYPE_run,
}_NEEY_PKT_SUB_TYPE;


/**
  * Bettery-Type defines
  * */
#define	NEEY_BATTERY_TYPE_NCM 			0x01
#define	NEEY_BATTERY_TYPE_LFP 			0x02
#define	NEEY_BATTERY_TYPE_LTO 			0x03
#define	NEEY_BATTERY_TYPE_PBAC			0x04


/**
  * Buzzer-Type defines
  * */
#define	NEEY_BUZZER_TYPE_NC 			0x00
#define	NEEY_BUZZER_TYPE_SHUT 			0x01
#define	NEEY_BUZZER_TYPE_SINGLE 		0x02
#define	NEEY_BUZZER_TYPE_LOOP 			0x03

/**
  * CMD-ON/OFF-Type defines
  * */
#define	NEEY_ON 						0x01
#define	NEEY_OFF 						0x00


/**
  * Neey flag defines for cell-state
 * */
#define	NEEY_CELL_CHARGING				0x01
#define	NEEY_CELL_DISCHARGING			0x02
#define NEEY_CELL_VOLTAGE_VALID			0x04
#define NEEY_CELL_RESITANCE_VALID		0x08
#define	NEEY_CELL_HIGHEST_VOLTAGE		0x10
#define	NEEY_CELL_LOWEST_VOLTAGE		0x20


/**
  * Neey Task-Scheduler defines
  * */
#define PROCESS_NEEY_NO_TASK			0x00
#define PROCESS_NEEY_SEND				0x01
#define PROCESS_NEEY_DATA				0x02
#define PROCESS_NEEY_AT					0x04
#define PROCESS_NEEY_ALIVE				0x10
#define PROCESS_NEEY_INFO				0x20
#define PROCESS_NEEY_PARAM				0x40
#define PROCESS_NEEY_TIMER				0x80


typedef enum
{
	NO_NEEY,
	AT_MODE,
	INIT_MODE,
	RUN_MODE,
	NEEY_MODE_END
}_NEEY_STATE;


typedef enum
{
	SCAN_PKT_START,
	READ_HEADER,
	READ_PKT_DATA,
}_NEEY_RX_TRANSFER_STATE;



/**
  * @brief  NEEY Data Structure definition ()
  */
typedef struct
{
	uint16_t					rx_byte_count;    					/*!< Start of the data packet, always 0x55AA */
	_NEEY_RX_TRANSFER_STATE		state;
} _NEEY_RX_TRANSFER;


/**
  * @brief  NEEY Data Structure definition ()
  */
typedef struct
{
	char			NeeyDevType[16];
	char			NeeyVersions[24];					/*!< string mit versionsnummern HW...,ZH....,V....  */
	char			NeeyMFD[12];						/*!< string mit dem Manufacturing Date YYYYMMDD  */
	char			BLE_Name[10];
	float			StartVol;
	float			MaxBalCurrent;
	float			SleepVol;
	float			EquVol;
	uint8_t			CellCount;
	uint8_t			Buzzer;
	uint8_t			BatType;
	uint16_t		BatCap;
} _NEEY_INFO;


/**
  * @brief  NEEY Data Structure definition ()
  */
typedef struct
{
	uint16_t		voltage;					/*!< Spannung der Zelle  */
	uint16_t		resistance;					/*!< Leitungswiderstand  */
	uint8_t			flag;						/*!< laden, entladen ....  */
} _NEEY_CELL_DATA;


typedef enum
{
	numberof_units_doesnot_match_the_setting 	= 1, //The numberof units doesnot match the setting
	lineresistance_detection_inprogress 		= 2, //EQU lineresistance detection inprogress
	resistance_of_line_is_too_large 			= 3, //The resistanceof the EQU lineis too large
	Systemready 								= 4, //Systemready
	EQU_run 									= 5, //EQU run
	EQU_completed 								= 6, //EQU completed
	Battery_low_voltage 						= 7, //Battery low voltage
	temperature_is_too_high 					= 8, //Mainengine temperature istoo high
	Hostfailure 								= 9, //Hostfailure, pleasecontact the agent
	Low_battery_voltage 						= 10, //Low battery voltage,EQUstop
	temperature_is_too_high_EQUstop 			= 11, //Mainengine temperature istoo high, EQUstop
	Self_checkcompleted 						= 12, //Self-checkcompleted, systemwaiting
}_NEEY_DEV_RUNNING_STATE;



/**
  * @brief  NEEY Data Structure definition ()
  */
typedef struct
{
	uint32_t 				AmtVol;								/*!< pack voltage of all cells  */
	uint16_t 				AveVol;								/*!< average voltage of all cells  */
	uint16_t 				DiffVol;							/*!< diff-voltage max cell volt. - min cell volt.  */
	int16_t 				BalCurrent;							/*!< current balance current  */
	int16_t 				Temperatur1;						/*!< neey temperatur  */
	int16_t 				Temperatur2;						/*!< neey temperatur  */
	_NEEY_DEV_RUNNING_STATE running_state;
	uint32_t				RunTime;
	uint8_t					highest_voltage_cell_nr;
	uint8_t					lowest_voltage_cell_nr;
	uint8_t					AlarmInfo[25];
} _NEEY_DEV_DATA;

/**
  * @brief  NEEY Data Structure definition ()
  */
typedef struct
{
	_NEEY_RX_TRANSFER 		cur_rx_transfer;    					/*!<  */
	_NEEY_INFO				neey_dev_info;
	_NEEY_CELL_DATA			cell_data[NEEY_CHANNEL_COUNT];
	_NEEY_DEV_DATA			neey_dev_data;
	_NEEY_STATE				neey_state;
	uint8_t					data_pkt_counter;
	uint8_t					last_checked_data_pkt_counter;
	uint8_t					data_lock;
} _NEEY_CTRL;


#pragma pack(push,1)

/**
  * @brief  NEEY Data Structure definition (from NEEY to MC)
  */
typedef struct
{
	  uint16_t 	PacketStart;    					/*!< Start of the data packet, always 0x55AA */

	  uint16_t 	Address;							/*!< adress??? */

	  uint8_t 	PacketType;							/*!< Type, 0x02 (Daten), 0x01 (info?), 0x04 (???), 0x05 (CMD?)*/

	  uint8_t 	PacketSubType;						/*!< SubType, @seeSub-Type defines */

	  uint16_t 	PacketLength;						/*!< Länge des gesamten Datenpacketes (eigentlich immer 100) */

} _NEEY_RX_HEADER;



/**
  * @brief  NEEY Data Structure definition (from NEEY to MC)
  * @length	0x12C (300 Byte)
  * @brief	we be transmitted by neey every 1 second
  */
typedef struct
{
  uint16_t 	PacketStart;    					/*!< Start of the data packet, always 0x55AA */

  uint16_t 	Address;							/*!< adress??? */

  uint8_t 	PacketType;							/*!< Type, 0x02 (Daten), 0x01 (info?), 0x04 (???), 0x05 (CMD?)*/

  uint8_t 	PacketSubType;						/*!< SubType, @seeSub-Type defines */

  uint16_t 	PacketLength;						/*!< Länge des gesamten Datenpacketes (eigentlich immer 300) */

  uint8_t 	PacketCount;						/*!< wird einfach für jedes empf. Datenpaket hochgezählt */

  float 	CellVoltage[NEEY_CHANNEL_COUNT];	/*!< Cell-voltage */

  float		CellValue[NEEY_CHANNEL_COUNT];		/*!< the big unkonwn, strange float values releated to cell ca. 0,26, maybe line resistance in [Ohm]???   */

  float 	AmtVol;								/*!< pack voltage of all cells  */

  float 	AveVol;								/*!< average voltage of all cells  */

  float 	DiffVol;							/*!< diff-voltage max cell volt. - min cell volt.  */

  uint8_t	HighestCellvoltageNumber;  	  	  	/*!<213 Highest cellvoltage serial number 1Byte 0-23 */

  uint8_t	MinCellvoltageNumber;  	  	  	  	/*!<214 Minimum cellvoltage serial number 1Byte 0-23 */

  uint8_t  	Undefined;  	  	  	  			/*!<215 Undefined 1Byte Undefined */

  uint8_t	DevRunningStatusFlag;  	  	  	  	/*!<216 Devicerunning status flag 1Byte shown in Table 3 */

  float 	BalCurrent;							/*!< current balance current  */

  float 	Temperatur1;						/*!< neey temperatur  */

  float 	Temperatur2;						/*!< neey temperatur again??? */

  uint8_t	AlarmInfo[25];						/*!<229-253 Alarm information 25Byte shown in Table 4 */

  uint32_t	RunTime;							/*!<254-257 Run time 4Byte 4-Byte unsignedint */

  uint8_t	SystemReserved[40];					/*!<258-297 SystemReserved 40Byte Write all undefined bytes to 0 */

  uint8_t	Checksum;							/*!< simple 8-Bit Überlauf-Prüfsumme */

  uint8_t	PacketEnd;							/*!< End of the data packet, always 0xFF */

} _NEEY_RecDataTypeDef;


/**
  * @brief  NEEY Dev Info Structure definition (from NEEY to MC)
  * @length	0x64 (100 Byte)
  */
typedef struct
{
  uint16_t 	PacketStart;    					/*!< Start of the data packet, always 0x55AA */

  uint16_t 	Address;							/*!< adress??? */

  uint8_t 	PacketType;							/*!< Type, 0x02 (Daten), 0x01 (info?), 0x04 (???), 0x05 (CMD?)*/

  uint8_t 	PacketSubType;						/*!< SubType, @seeSub-Type defines */

  uint16_t 	PacketLength;						/*!< Länge des gesamten Datenpacketes (eigentlich immer 100) */

  char		NeeyDevType[16];

  char		NeeyVersions[24];					/*!< string mit versionsnummern HW...,ZH....,V....  */

  char		NeeyMFD[12];						/*!< string mit dem Manufacturing Date YYYYMMDD  */

  uint8_t	dummy[38];							/*!< the big unkonwn (maybe and hopefully the current config???)  */

  uint8_t	Checksum;							/*!< einfache 8-Bit Überlauf-Prüfsumme */

  uint8_t	PacketEnd;							/*!< End of the data packet, always 0xFF */

} _NEEY_RecDevInfoTypeDef;


/**
  * @brief  NEEY Dev Parms Structure definition (from NEEY to MC)
  * @length	0x64 (100 Byte)
  */
typedef struct
{
	_NEEY_RX_HEADER		pkt_header;

	uint8_t				cell_count;
	float				start_voltage;
	float				max_balance_current;
	float				sleep_voltage;

	uint8_t				dummy1;

	uint8_t				buzzer;
	uint8_t				bat_type;
	uint16_t 			bat_capacity;						/*!< Länge des gesamten Datenpacketes (eigentlich immer 100) */

	uint8_t				dummy2;
	uint8_t				dummy3;

	float				equ_voltage;

	uint8_t				empty[66];

	uint8_t				Checksum;							/*!< einfache 8-Bit Überlauf-Prüfsumme */

	uint8_t				PacketEnd;							/*!< End of the data packet, always 0xFF */

} _NEEY_RecDevParmTypeDef;


/**
  * @brief  NEEY Data Structure definition (from MC to NEEY)
  * @length	0x14 (20 Byte)
  */
typedef struct
{
  uint16_t 	PacketStart;    					/*!< Start of the data packet, always 0xAA55 */

  uint8_t 	Address;							/*!< adress??? */

  uint8_t 	RW_Flag;							/*!< read/write flag??? */

  uint8_t 	PacketType;							/*!< Type, 0x02 (Daten), 0x01 (info?), 0x04 (???), 0x05 (CMD?)*/

  uint8_t 	PacketSubType;						/*!< SubType, @seeSub-Type defines */

  uint16_t 	PacketLength;						/*!< Länge des gesamten Datenpacketes (eigentlich immer 20) */

  uint8_t 	data[10];							/*!< data to be send to neey (format see subtype */

  uint8_t	Checksum;							/*!< einfache 8-Bit Xor-Prüfsumme */

  uint8_t	PacketEnd;							/*!< End of the data packet, always 0xFF */

} _NEEY_SendDataTypeDef;

#pragma pack(pop)



extern _NEEY_CTRL neey_ctrl;

void MX_NEEY_Init(void);

uint8_t	process_NEEY(void);

uint8_t Send_Param_to_neey(uint8_t sub_type, uint8_t* p_data, uint8_t len);

extern uint8_t		neey_task_scheduler;


#ifdef __cplusplus
}
#endif

#endif /* __NEEY_H__ */

