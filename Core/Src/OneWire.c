/**
    @author Stanislav Lakhtin
    @date   11.07.2016
    @brief  Реализация протокола 1wire на базе библиотеки libopencm3 для микроконтроллера STM32F103
            Возможно, библиотека будет корректно работать и на других uK (требуется проверка).
            Проверка необходима, чтобы убедиться в корректности настройки UART/USART для работы
            в полудуплексном режиме
            Общая идея заключается в использовании аппаратного USART uK для иммитации работы 1wire.
            Подключение устройств осуществляется на выбранный USART к TX пину, который должен быть подтянут к линии питания сопротивлением 4.7К.
            Реализация библиотеки осуществляет замыкание RX на TX внутри uK, оставляя ножку RX доступной для использования в других задачах.
            Реализация библиотеки предполагает возможную одновременную работу как с независимыми шинами сразу со всеми
            возможными UART/USART в микроконтроллере. При этом все шины (до 5 штук) будут адресоваться и опрашиваться индивидуально
 */

#include "main.h"

#ifdef __OW_ONEWIRE_OLD__

#include "OneWire.h"
#include "stm32f1xx_hal.h"
#include "stdio.h"
#include "string.h"
#include "gpio.h"


extern UART_HandleTypeDef huart3;
#define ow_uart huart3
#define OW_USART USART3
#define MAXDEVICES_ON_THE_BUS 23

/*****************************************************************************/
//volatile uint8_t recvFlag;
//volatile uint16_t rc_buffer[5];

int16_t 	Temp[MAX_TEMP_DEV_ON_THE_BUS];
uint8_t 	devices;
OneWire 	ow;
uint32_t 	pDelay = 300, i;
uint8_t 	sensor;
DEVInfo 	devInfo;
Temperature t;
char*		crcOK;

uint8_t 	ow_task_scheduler;
uint8_t 	current_temp_device;



/****************************************************************************
 * HAL_UART_TxCpltCallback
 *
 * @param void
 * @return ow_prcess_mask
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == OW_USART) {
		ow.txFlag &= ~(1 << 0);					//сбрасываем флаг ответ получен после
		main_task_scheduler |= PROCESS_OW;
		ow_task_scheduler |= OW_PROCESS_TX_CPLT;
	}
}

//uint16_t USART_ReceiveData(USART_TypeDef* USARTx)
//{
//	/* Check the parameters */
//	assert_param(IS_USART_ALL_PERIPH(USARTx));
//
//	/* Receive Data */
//	return (uint16_t)(USARTx->DR & (uint16_t)0x01FF);
//}


/****************************************************************************
 * USART_SendData
 *
 * @param void
 * @return ow_prcess_mask
 */
void USART_SendData(UART_HandleTypeDef *huart, uint8_t* pData, uint8_t len)
{
	/* Check the parameters */
	assert_param(IS_USART_ALL_PERIPH(huart->Instance));
	assert_param(IS_USART_DATA(Data));
	assert(len<=sizeof(ow.tx_buffer));

	//HAL_UART_Transmit_IT();
	HAL_UART_Transmit_DMA(huart, pData, len);
	/* Transmit Data */
	//USARTx->DR = (Data & (uint16_t)0x01FF);

	ow.txFlag |= (1 << 0);//сбрасываем флаг ответ получен после
}


/****************************************************************************
 * usart_setup
 *
 * @param void
 * @return ow_prcess_mask
 */
void usart_setup(uint32_t baud) {

	ow_uart.Instance = OW_USART;
	ow_uart.Init.BaudRate = baud;
	ow_uart.Init.WordLength = UART_WORDLENGTH_8B;
	ow_uart.Init.StopBits = UART_STOPBITS_1;
	ow_uart.Init.Parity = UART_PARITY_NONE;
	ow_uart.Init.Mode = UART_MODE_TX_RX;
	ow_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	ow_uart.Init.OverSampling = UART_OVERSAMPLING_16;

	if (HAL_HalfDuplex_Init(&ow_uart) != HAL_OK)
	{
		Error_Handler();
		__asm__("NOP");
	}

	__HAL_UART_ENABLE_IT(&ow_uart, UART_IT_RXNE);
}


/**********************************************************************************************
 * owInit
 *
 * @param ow
 * @return none
 */
void owInit(OneWire *ow) {
	int i=0, k = 0;

	for (; i < MAXDEVICES_ON_THE_BUS; i++) {
		uint8_t *r = (uint8_t *)&ow->ids[i];

		k=0;
		for (; k < 8; k++)
			r[k] = 0;
	}

	k=0;
	for (; k < 8; k++) {
		ow->lastROM[k] = 0x00;
	}

	ow->lastDiscrepancy = 64;
	ow->state = ow_state_none;
	ow_task_scheduler = OW_NO_TASK;
	current_temp_device = 0;
}


/***********************************************************************************************
 * owReadHandler
 *
 * @param void
 * @return ow_prcess_mask
 */
void owReadHandler() { //обработчик прерыания USART

	/* Проверяем, что мы вызвали прерывание из-за RXNE. */
	if (((OW_USART->CR1 & USART_CR1_RXNEIE) != 0) &&
		((OW_USART->SR & UART_FLAG_RXNE) != (uint16_t)RESET)) {

		/* Получаем данные из периферии и сбрасываем флаг*/
		//while ((OW_USART->SR & UART_FLAG_RXNE) == (uint16_t)RESET){;}

		/* Check the parameters */
		assert_param(IS_USART_ALL_PERIPH(OW_USART));

		/* Receive Data */
		ow.rx_buffer = (uint8_t)(OW_USART->DR & (uint16_t)0x00FF);
		ow.recvFlag &= ~(1 << 0);//сбрасываем флаг ответ получен после

		main_task_scheduler |= PROCESS_OW;
		ow_task_scheduler |= OW_PROCESS_RX_CPLT;
	}
}


/**********************************************************************************************
 * owSend
 *
 * @param data Data to send
 * @return none
 */
void owSend(uint8_t data) {
	ow.recvFlag |= (1 << 0);//устанавливаем флаг если попадем в обработчик прерывания там он сбросится

	ow.tx_buffer[0] = data;
	USART_SendData(&ow_uart, ow.tx_buffer, 1);//отправляем данные

	//HAL_GPIO_WritePin(FAN_SPEED_GPIO_Port, FAN_SPEED_Pin, GPIO_PIN_SET);
//	while(__HAL_UART_GET_FLAG(&ow_uart, UART_FLAG_TC) == RESET);//wait for tx to complete
	while(ow.txFlag & (1 << 0));//wait for tx to complete
	//HAL_GPIO_WritePin(FAN_SPEED_GPIO_Port, FAN_SPEED_Pin, GPIO_PIN_RESET);
}


/**********************************************************************************************
 * owEchoRead
 *
 * @param void
 * @return ow_prcess_mask
 */
uint8_t owEchoRead() {//
	uint16_t pause = 1000;

	while (ow.recvFlag & (1 << 0) && pause--);// ждем пока кто-то не ответит но не больше паузы

	return ow.rx_buffer;//в зависимости от используемого номера UART
}


/** Реализация RESET на шине 1wire
 *
 * @param N usart -- выбранный для реализации 1wire usart
 * @return Возвращает 1 если на шине кто-то есть и 0 в противном случае
 */
uint8_t owResetCmd() {
	uint8_t owPresence;
	
	usart_setup(9600);

	ow.state = ow_reset;
	owSend(0xF0); // Send RESET отправляем импуль сброса
	owPresence = owEchoRead(); // Ждём PRESENCE на шине и вовзращаем, что есть

	usart_setup(115200);// перенастраиваем скорость UART

	return owPresence;
}


/*******************************************************************************************
 * owReadSlot
 *
 * @param void
 * @return ow_prcess_mask
 */
uint8_t owReadSlot(uint8_t data) {//читаем у нас пришла единица или ноль в ответ
	return (data == OW_READ) ? 1 : 0; //если пришло 0xFF, то бит = 1, что то другое бит = 0
}


/*******************************************************************************************
 * byteToBits
 *
 * @param void
 * @return ow_prcess_mask
 */
uint8_t *byteToBits(uint8_t ow_byte, uint8_t *bits) {//разлагаем 1 байт на 8 байт ,кодируем так скасказать в посылку для 1wire
	uint8_t i;

	for (i = 0; i < 8; i++) {

		if (ow_byte & 0x01) {//если текущий бит в байте ==1 то
			*bits = WIRE_1; //заменяем на число которое при передаче по USART для 1wire будет единцией t.e 0xFF
		}
		else {
			*bits = WIRE_0;// тоже самое только для 0
		}

		bits++;
		ow_byte = ow_byte >> 1; //сдвигаем обработанный бит
	}

	return bits; //возвращае массив для передачи
}


/*******************************************************************************************
 * Метод пересылает последовательно 8 байт по одному на каждый бит в data
 * @param usart -- выбранный для эмуляции 1wire UART
 * @param d -- данные
 */
void owSendByte(uint8_t d) {

	byteToBits(d, ow.tx_buffer);	//

	USART_SendData(&ow_uart, ow.tx_buffer, 8);
}


/********************************************************************************************
 * bitsToByte
 *
 * @param void
 * @return ow_prcess_mask
 */
uint8_t bitsToByte(uint8_t *bits) {//принимает "кодированый" массив байт полученный по UART и делает из него байт))
	uint8_t target_byte, i;

	target_byte = 0;

	for (i = 0; i < 8; i++) {
		target_byte = target_byte >> 1;

		if (*bits == WIRE_1) {//если пришло  по USART 0xFF то это у нас пришла 1ца
			target_byte |= 0x80;//устанавливаем в 1 старший бит
		}

		bits++;//передвигаемся к следующему байту который является либо 0=0x00 или 1=0xFF
	}

	return target_byte; //возвращаем полученный байт
}


/* Подсчет CRC8 массива mas длиной Len */
/**************************************************************************************************
 * owCRC
 *
 * @param void
 * @return ow_prcess_mask
 */
uint8_t owCRC(uint8_t *mas, uint8_t Len) {
	uint8_t i, dat, crc, fb, st_byt;

	st_byt = 0;
	crc = 0;

	do {
		dat = mas[st_byt];

		for (i = 0; i < 8; i++) {  // счетчик битов в байте
			fb = crc ^ dat;
			fb &= 1;
			crc >>= 1;
			dat >>= 1;

			if (fb == 1) crc ^= 0x8c; // полином
		}

		st_byt++;
	} while (st_byt < Len); // счетчик байтов в массиве

	return crc;
}


/****************************************************************************
 * owCRC8
 *
 * @param void
 * @return ow_prcess_mask
 */
uint8_t owCRC8(RomCode *rom){
	return owCRC((uint8_t*)rom, 7);
}


/*********************************************************************************************
 * return 1 if has got one more address
 * return 0 if hasn't
 * return -1 if error reading happened
 *
 * переделать на функции обратного вызова для реакции на ошибки
 */
int hasNextRom(OneWire *ow, uint8_t *ROM) {//
	uint8_t ui32BitNumber = 0;
	int zeroFork = -1;
	uint8_t i = 0;

	if (owResetCmd() == ONEWIRE_NOBODY) { //есть ли кто на шине
		return 0;
	}

	owSendByte(ONEWIRE_SEARCH);//

//	HAL_GPIO_WritePin(FAN_SPEED_GPIO_Port, FAN_SPEED_Pin, GPIO_PIN_SET);
	while(ow->txFlag & (1 << 0));//lets busy wait here
//	HAL_GPIO_WritePin(FAN_SPEED_GPIO_Port, FAN_SPEED_Pin, GPIO_PIN_RESET);

	do {
		uint8_t answerBit = 0;
		int byteNum = ui32BitNumber / 8;
		uint8_t *current = (ROM) + byteNum;
		uint8_t cB, cmp_cB, searchDirection = 0;

		owSend(OW_READ); // чтение прямого бита
		cB = owReadSlot(owEchoRead());//ответ от датчика
		owSend(OW_READ); // чтение инверсного бита
		cmp_cB = owReadSlot(owEchoRead());//ответ от датчика

		if (cB == cmp_cB && cB == 1)//сравниваем два ответа
			return -1;//ошибка никто не ответил

		if (cB != cmp_cB) { //нормальная ситуация пришло либо 10 либо 01
			searchDirection = cB;//выбираем в каком направлении будем двигатся дальше
		}
		else {//колизия пришло 00 т.е текущий бит у ROM-ов разный
			if (ui32BitNumber == ow->lastDiscrepancy) {//если текущая позиция колизии равна прошлой
				searchDirection = 1;//выбираем в каком направлении будем двигатся дальше
			}
			else {
				if (ui32BitNumber > ow->lastDiscrepancy) {//если мы зашили дальше
					searchDirection = 0;//выбираем в каком направлении будем двигатся дальше
				}
				else {
					searchDirection = (uint8_t) ((ow->lastROM[byteNum] >> ui32BitNumber % 8) & 0x01);
				}

				if (searchDirection == 0) {
					zeroFork = ui32BitNumber;//запоминаем развилку
				}
			}
		}

		// сохраняем бит
		if (searchDirection)
			*(current) |= 1 << ui32BitNumber % 8;//выставляем бит в текущем байте байте

		answerBit = (uint8_t) ((searchDirection == 0) ? WIRE_0 : WIRE_1);// решаем кого отключить
		owSend(answerBit);//вырубаем "мешающие" устройсва
		ui32BitNumber++;//ищем следующий бит

	} while (ui32BitNumber < 64);//пока не найден весь ROM все биты

	ow->lastDiscrepancy = zeroFork;//запоминаем развилку

	for (; i < 7; i++)
		ow->lastROM[i] = ROM[i];//запоминаем последний ROM

	return ow->lastDiscrepancy > 0;
}



/*******************************************************************************************************
 * Method for Возвращает количество устройств на шине или код ошибки, если значение меньше 0
 * @param ow -- OneWire pointer
 * @return data
 */
int owSearchCmd(OneWire *ow) {
	int device = 0, nextROM;
	owInit(ow);

	do {
		nextROM = hasNextRom(ow, (uint8_t*)(&ow->ids[device])); //передаем указатель на структуру куда положить след.ROM
		if (nextROM<0)
			return -1;

		device++;
	} while (nextROM && device < MAXDEVICES_ON_THE_BUS);//ищем пока кто-то есть и этих кто-то не больше дефайна

	return device;//возвращаем порядковый номер датчика (устройства) на шине
}


/**
 * Method for
 * @param ow -- OneWire pointer
 * @return data
 */
void owSkipRomCmd(OneWire *ow) {//отправляет команду пропуска ROM после этого следующая команда будет
	owResetCmd();                 //для всех устройств на шине

	owSendByte(ONEWIRE_SKIP_ROM);

//	HAL_GPIO_WritePin(FAN_SPEED_GPIO_Port, FAN_SPEED_Pin, GPIO_PIN_SET);

//	while(__HAL_UART_GET_FLAG(&ow_uart, UART_FLAG_TC) == RESET);//wait for tx to complete
	while(ow->txFlag & (1 << 0));//wait for tx to complete

//	HAL_GPIO_WritePin(FAN_SPEED_GPIO_Port, FAN_SPEED_Pin, GPIO_PIN_RESET);
}


/**********************************************************************************************************
 * Method for
 * @param rom -- selected device on the bus
 * @param cmd -- command to operate
 * @return none
 */
void owMatchRomCmd(RomCode *rom, uint8_t cmd) {//позволяет мастеру обращаться к конкретному  ведомому устройству
	int i = 0;
	uint8_t buf_ptr=0;

	owResetCmd();

	byteToBits(ONEWIRE_MATCH_ROM, ow.tx_buffer);//преобразовываем байт в биты "массив байт для  передачи UART и эмуляции 1WIRE"
	buf_ptr += 8;


	//owSendByte(ONEWIRE_MATCH_ROM);//обращаемся к конкретному устройсву

	for (; i < 8; i++) {
		byteToBits(*(((uint8_t *) rom) + i), ow.tx_buffer+buf_ptr);
		buf_ptr += 8;
		//owSendByte(*(((uint8_t *) rom) + i));//"перебираемся по структуре как по массиву" первой звездочкой получаем i тый байт из структуры
	}


	byteToBits(cmd, ow.tx_buffer+buf_ptr);//преобразовываем байт в биты "массив байт для  передачи UART и эмуляции 1WIRE"
	buf_ptr += 8;


	USART_SendData(&ow_uart, ow.tx_buffer, buf_ptr);
}


/**
 * Method for
 * @param ow -- OneWire pointer
 * @param rom -- selected device on the bus
 * @return none
 */
void owConvertTemperatureCmd(OneWire *ow, RomCode *rom) {
	owMatchRomCmd(rom, ONEWIRE_CONVERT_TEMPERATURE);//позволяет мастеру обращаться к конкретному  ведомому устройству

	ow->state=ow_convert_temperature;
}


/************************************************************************************************
 * Method for reading scratchad DS18B20 OR DS18S20
 * If sensor DS18B20 then data MUST be at least 9 byte
 * If sensor DS18S20 then data MUST be at least 2 byte
 * @param ow -- OneWire pointer
 * @param rom -- selected device on the bus
 * @param data -- buffer for data
 * @return data
 */
uint8_t *owReadScratchpadCmd(OneWire *ow, RomCode *rom, uint8_t *data) {//читаем память датчика
	uint16_t b = 0, p;

	switch (rom->family) {
		case DS18B20:
		case DS18S20:
			p = 72;  //9*8 =72
			break;

		default:
			return data;
	}

	//owMatchRomCmd(rom);
	//owSendByte(ONEWIRE_READ_SCRATCHPAD);//отправляем команду на чтение памяти

	//owMatchRomCmd(rom, ONEWIRE_READ_SCRATCHPAD);//
	//ow->state=ow_read_scratchpad;


	while (b < p) {// пока мы не обработали 9 байт
		//uint8_t pos = (uint8_t) ((p - 8) / 8 - (b / 8)); //позиция обрабатываемого байта
		uint8_t pos = (uint8_t) (b / 8); //позиция обрабатываемого байта
		uint8_t bt;

		owSend(OW_READ);
		bt = owReadSlot(owEchoRead());//читаем данные

		if (bt == 1)
			data[pos] |= 1 << b % 8;//выставляем бит в нужной позиции
		else
			data[pos] &= ~(1 << b % 8);//сбрасываем бит в нужной позиции

		b++;//следующий бит
	}

  return data;
}


/*************************************************************************************************
 * Method for writing scratchad DS18B20 OR DS18S20
 * @param ow -- OneWire pointer
 * @param rom -- selected device on the bus
 * @param th --
 * @return tl
 * @return conf
 */
void owWriteDS18B20Scratchpad(OneWire *ow, RomCode *rom, uint8_t th, uint8_t tl, uint8_t conf) {

	if (rom->family != DS18B20)
		return;

	owMatchRomCmd(rom, ONEWIRE_WRITE_SCRATCHPAD);//

	//owMatchRomCmd(rom);//обращаемся к конкретному устройству
	//owSendByte(ONEWIRE_WRITE_SCRATCHPAD);//будем записывать в память


	owSendByte(th);//пороги для температур
	owSendByte(tl);
	owSendByte(conf);

	ow->state = ow_write_scratchpad;
}


/*******************************************************************************************************
 * Get last mesaured temperature from DS18B20 or DS18S20. These temperature MUST be measured in previous
 * opearions. If you want to measure new value you can set reSense in true. In this case next invocation
 * that method will return value calculated in that step.
 * @param ow -- OneWire bus pointer
 * @param rom -- selected device
 * @param reSense -- do you want resense temp for next time?
 * @return struct with data
 */
Temperature readTemperature(OneWire *ow, RomCode *rom, uint8_t reSense) {
	Scratchpad_DS18B20 *sp;
	Scratchpad_DS18S20 *spP;
	Temperature t;
	uint8_t pad[9];
	uint8_t crc=0;

	t.inCelsus = 0x00;
	t.frac = 0x00;
	sp = (Scratchpad_DS18B20 *) &pad;
	spP = (Scratchpad_DS18S20 *) &pad;

	switch (rom->family) {
		case DS18B20:
			owReadScratchpadCmd(ow, rom, pad);//читаем память  для DS18B20

			crc = owCRC(pad, 8);

			if (crc != sp->crc)
				break;

			t.inCelsus = (int8_t) (sp->temp_msb << 4) | (sp->temp_lsb >> 4);//целая часть
			t.frac = (uint8_t) ((((sp->temp_lsb & 0x0F)) * 10) >> 4);//дробная
			break;

		case DS18S20:
			owReadScratchpadCmd(ow, rom, pad);//читаем память  для DS18S20
			t.inCelsus = spP->temp_lsb >> 1;
			t.frac = (uint8_t) 5 * (spP->temp_lsb & 0x01);
			break;

		default:
			return t;
	}

//	if (reSense) {
//		owConvertTemperatureCmd(ow, rom);//можно сразу после как забрали данные отдаем датчику команду на преобразования температуры
//	}

	return t;
}


void owCopyScratchpadCmd(OneWire *ow, RomCode *rom) {
	owMatchRomCmd(rom, ONEWIRE_COPY_SCRATCHPAD);//
	ow->state=ow_copy_scratchpad;
}


void owRecallE2Cmd(OneWire *ow, RomCode *rom) {
	owMatchRomCmd(rom, ONEWIRE_RECALL_E2);//
	ow->state=ow_recall_E2_cmd;
}



/**************************************************************************************
 * get_ROMid
 *
 * @param void
 * @return ow_prcess_mask
 */
int get_ROMid (void) {

	if (owResetCmd() != ONEWIRE_NOBODY) {    // is anybody on the bus?

		devices = owSearchCmd(&ow);        //

		if (devices <= 0) {
			while (1) {
				pDelay = 1000000;
				for (i = 0; i < pDelay * 1; i++)    /* Wait a bit. */
					__asm__("nop");
			}
		}

		i = 0;
		for (; i < devices; i++) {//выводим в кон�?оль в�?е найденные ROM
			RomCode *r = &ow.ids[i];
			uint8_t crc = owCRC8(r);
			crcOK = (crc == r->crc)?"CRC OK":"CRC ERROR!";
			devInfo.device = i;

			sprintf(devInfo.info, "SN: %02X/%02X%02X%02X%02X%02X%02X/%02X", r->family, r->code[5], r->code[4], r->code[3],
					r->code[2], r->code[1], r->code[0], r->crc);

			if (crc != r->crc) {
				devInfo.device = i;
				sprintf (devInfo.info,"\n can't read cause CNC error");
			}
		}
	}

	pDelay = 1000000;
	for (i = 0; i < pDelay * 1; i++)
		__asm__("nop");

	if (strcmp(crcOK,"CRC OK") == 0) return 0;
	else return -1;
}





/****************************************************************************
 * get_Temperature (void)
 * @param void
 * @return none
 */
void get_Temperature (void)
{
	if (devices) {
		current_temp_device = 0;

		ow_task_scheduler |= OW_PROCESS_READ_TEMP;
		main_task_scheduler |= PROCESS_OW;
	}
}


void read_Temperatures_OW (void)
{
}


/****************************************************************************
 *
 * @param void
 * @return ow_prcess_mask
 */
uint8_t process_OW(void) {

	/***********************************************************************/
	if (ow_task_scheduler & OW_PROCESS_RX_CPLT)
	{
		switch (ow.state) {//че у нас за датчик
		case ow_reset:
			ow.state = ow_state_none;
			break;

		case ow_read_scratchpad:
			break;

		case ow_write_scratchpad:
			break;

		case ow_convert_temperature:
			break;

		default:
			// error handler
			break;
		}

		ow_task_scheduler &= ~OW_PROCESS_RX_CPLT;
	}


	/***********************************************************************/
	if (ow_task_scheduler & OW_PROCESS_TX_CPLT)
	{
		switch (ow.state) {//
		case ow_read_scratchpad:
			Temperature t;

			t = readTemperature(&ow, &ow.ids[current_temp_device], 0);
			Temp[current_temp_device] = (int16_t)t.inCelsus*10 + t.frac;

			if (++current_temp_device < devices) {
				//ow_task_scheduler |= OW_PROCESS_READ_TEMP;
				owMatchRomCmd(&ow.ids[current_temp_device], ONEWIRE_READ_SCRATCHPAD);//
				ow.state=ow_read_scratchpad;
			}
			else {
				//owConvertTemperatureCmd(&ow, &ow.ids[current_temp_device]);

//				uint8_t cB, cmp_cB = 0;
//				owResetCmd();
//				owSkipRomCmd(&ow);
//				owSend(ONEWIRE_CONVERT_TEMPERATURE);
//				owSend(OW_READ); // чтение прямого бита
//				cB = owReadSlot(owEchoRead());//ответ от датчика

//				if (cB == 1)//сравниваем два ответа
//					ow.state = ow_convert_temperature;


				//owSkipRomCmd(&ow);
				//owSend(ONEWIRE_CONVERT_TEMPERATURE);
				//ow.state = ow_convert_temperature;
			}
			break;

		case ow_write_scratchpad:
			break;

		case ow_convert_temperature:
			owMatchRomCmd(&ow.ids[current_temp_device], ONEWIRE_READ_SCRATCHPAD);//
			ow.state=ow_read_scratchpad;
			break;

		default:
			// error handler
			break;
		}

		ow_task_scheduler &= ~OW_PROCESS_TX_CPLT;
	}


	/***********************************************************************/
	if (ow_task_scheduler & OW_PROCESS_READ_TEMP)
	{
		owSkipRomCmd(&ow);
		owSend(ONEWIRE_CONVERT_TEMPERATURE);
		ow.state=ow_convert_temperature;

//		owMatchRomCmd(&ow.ids[current_temp_device], ONEWIRE_READ_SCRATCHPAD);//
//		ow.state=ow_read_scratchpad;


		ow_task_scheduler &= ~OW_PROCESS_READ_TEMP;
	}

	return ow_task_scheduler;
}

#endif
