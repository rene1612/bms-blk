/*
 * DALLAS_TEMPERATURE.c
 *
 *  Created on: Dec 01, 2021
 *      Author: Shpegun60
 */
#include "dallas_temperature.h"
#include "string.h"



/***********************************************************************************************
 * DT_IsConnected_ScratchPad
 * @brief	dallas temperature CRC8 check
 * @param scratchPad
 * @return crc
 */
static uint8_t DT_IsConnected_ScratchPad(uint8_t* scratchPad)
{
	return ((OW_Crc8(scratchPad, 8) == scratchPad[SCRATCHPAD_CRC]));
}


/***********************************************************************************************
 * @fn		DT_MillisToWaitForConversion
 * @brief
 * @param 	dt
 * @param 	time
 * @return 	millis to wait for Conversion
 */
static uint8_t DT_MillisToWaitForConversion(DallasTemperatureData* dt, uint32_t time)
{
	switch (dt->resolution) {
		case TEMP_9_BIT:
			return (uint8_t)((time - dt->lastTime) > 94);
		case TEMP_10_BIT:
			return (uint8_t)((time - dt->lastTime) > 188);
		case TEMP_11_BIT:
			return (uint8_t)((time - dt->lastTime) > 375);
		case TEMP_12_BIT:
		default:
			return (uint8_t)((time - dt->lastTime) > 750);
	}
}


/***********************************************************************************************
 * @fn		DT_SetOneWire
 * @brief	dallas temperature CRC8 check
 * @param 	dt		DallasTemperatureData Handle
 * @param 	ow		UartOneWire_HandleTypeDef Handle
 * @return 	void
 */
void DT_SetOneWire(DallasTemperatureData* dt, UartOneWire_HandleTypeDef* ow)
{
	if(!ow || !dt) {
		return;
	}

	dt->ow = ow;
	dt->state = 0;
	dt->devicesCount = 0;
	dt->resolution = 0;
	dt->lastTime = 0;
	dt->data_lock = 0;
	DT_Search(dt);
}


/***********************************************************************************************
 * @fn		DT_Search
 * @brief	search for dallas temperature sensors
 * @param 	dt		DallasTemperatureData Handle
 * @return  device count
 */
uint8_t DT_Search(DallasTemperatureData* dt) {
	if(!dt) {
		return OW_NO_DEVICE;
	}

	for(uint8_t i = 0; i < DS_MAX_SENSORS; ++i) {
		for(uint8_t j = 0; j < 8; j++) {
			dt->id[i][j] = 0;
		}
	}

	dt->devicesCount = 0;
	dt->state = 0;
	dt->counteRead = 0;

	for(uint8_t i = 0; i < DS_MAX_SENSORS; ++i) {
		dt->temp[i] = 0.0;
	}

	if(!dt->ow) {
		return OW_NO_DEVICE;
	}

	OW_ClearStates(dt->ow);

	uint8_t state = 0;

	uint8_t searchAttempt = 5;
	while(searchAttempt) {

		while(1) {
			state = OW_Reset(dt->ow);
			if(state != OW_WAIT) {
				break;
			}
		}

		dt->devicesCount = OW_SearchBlock(dt->ow, (uint8_t *)&dt->id, DS_MAX_SENSORS);
		if(dt->devicesCount) {
			break;
		}
		searchAttempt--;
	}

	return OW_OK;
}


/***********************************************************************************************
 * @fn		DT_init
 * @brief	Init function vor Tempearture Sensor from Dallas
 * @param 	dt		DallasTemperatureData Handle
 * @return 	void
 */
void DT_init(DallasTemperatureData* dt, uint8_t resolution) {
	if(!dt->ow || !dt) {
		return;
	}

	uint8_t query[5]={SkipROM, WRITESCRATCH, DS_AlarmTH, DS_AlarmTL, resolution};
	while(1) {
		if(OW_Send(dt->ow, query, 5, NULL, 0, OW_NO_READ) != OW_WAIT) {
			break;
		}
	}

	dt->resolution = resolution;

	return;
}


/***********************************************************************************************
 * @fn		DT_ContiniousProceed
 * @brief	Worker Function for continious conversion of all temp sensors, needs to executed in
 * 			main while loop
 * @param 	dt		DallasTemperatureData Handle
 * @param 	time	current timer tick as time reference
 * @return 	OW_NO_DEVICE or OW_OK
 */
uint8_t DT_ContiniousProceed(DallasTemperatureData* dt, uint32_t time) {

	if(!dt->devicesCount || !dt || !dt->ow) {
		return OW_NO_DEVICE;
	}
	//	if((time - dt->lastTime) > 1000) {
	//		DT_Search(dt);
	//		dt->lastTime = time;
	//	}
	//
	//	return 0;


	switch(dt->state) {

		case 0:
			dt->counteRead = 0;
			dt->wrdata[0] = SkipROM;
			dt->wrdata[1] = STARTCONVO;
			dt->state++;
//			break;

		case 1: {
			uint8_t val = OW_Send(dt->ow, dt->wrdata, 2, (uint8_t *) NULL, 0, OW_NO_READ);

			if(val == OW_NO_DEVICE) {
				dt->state = 0;
				return OW_NO_DEVICE;
			} else if(val == OW_OK) {
				dt->lastTime = time;
				dt->state++;
			}
			break;
		}

		case 2:
			if(DT_MillisToWaitForConversion(dt, time)) {
				dt->state++;
			}
			break;

		case 3:
			dt->wrdata[0] = MatchROM;
			memcpy(&dt->wrdata[1], dt->id[dt->counteRead], 8);
			dt->wrdata[9] = READSCRATCH;

			for(uint8_t i = 10; i < 19; ++i) {
				dt->wrdata[i] = 0xff;
			}
			dt->state++;
			break;

		case 4: {
			uint8_t val = OW_Send(dt->ow, dt->wrdata, 19, dt->rddata, 9, 10);
			if(val == OW_NO_DEVICE) {
				dt->state = 0;
				return OW_NO_DEVICE;
			} else if(val == OW_OK) {
				dt->state++;
			}
			break;
		}

		case 5: {
			if(DT_IsConnected_ScratchPad(dt->rddata)) {
				int16_t TemperatureData = ((int16_t)(dt->rddata[TEMP_MSB] << 8)) | ((int16_t)(dt->rddata[TEMP_LSB]));
				dt->temp[dt->counteRead] = (float) ((float) TemperatureData * 0.0625); // The resolution is 0.0625 degrees
			}

			if(dt->counteRead < (dt->devicesCount - 1)) {
				dt->counteRead++;
				dt->state = 3;
			} else {
				dt->counteRead = 0;
				dt->state++;
			}
			break;
		}

		case 6: {
			uint8_t index;

			if (!dt->data_lock) {
				for (index=0; index < dt->devicesCount; index++) {
					dt->fixpoint_temp[index] = (int16_t)(dt->temp[index] * 100);
				}
				dt->state = 0;
			}

			break;
		}

	}
	return OW_OK;
}


/***********************************************************************************************
 * @fn		getTemperatureByPosition_Celsius
 * @brief	Read temeratures from internal array
 * @param 	dt			DallasTemperatureData Handle
 * @param 	position	Position/Sensor-Nr. to read
 * @return 	float temperature value in °C
 */
float getTemperatureByPosition_Celsius(DallasTemperatureData* dt, uint8_t position) {
	if(position >= dt->devicesCount  || !dt) {
		return 0.0;
	}
	return dt->temp[position];
}


/***********************************************************************************************
 * @fn		getTemperatureByROM_Celsius
 * @brief	Read temeratures from internal array
 * @param 	dt			DallasTemperatureData Handle
 * @param 	p_rom_array	array with rom code to read from
 * @return 	int16 temperature value in 1/100°C
 */
int16_t getTemperatureByROM_Celsius(DallasTemperatureData* dt, uint8_t* p_rom_array) {
	uint8_t index;
	//uint8_t* p_rom_index;

	if(!p_rom_array  || !dt) {
		return 0.0;
	}

	for (index=0; index<dt->devicesCount;index++){
		if (memcmp(p_rom_array, (uint8_t*)dt->id[index], 8)==0){
			return dt->fixpoint_temp[index];
		}
	}

	return 0.0;
}


