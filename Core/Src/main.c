/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "crc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <string.h>

//#include "OneWire.h"
#include "dallas_temperature.h"
#include "UartOneWire.h"

#include "neey.h"
#include "passive_balancer.h"
#ifdef __WS2812B__
#include "ws2812b.h"
#endif

#if __has_include("gitcommit.h")
	#include "gitcommit.h"
#else
	#define __GIT_SHORT_HASH__ 0x0000000
	#define __GIT_BRANCH__ "none"
	#define __GIT_DATE_STR__ "2024-11-25"
	#define __GIT_DATE_UT__ 1732571756
#endif


/* Fash memory organisation

		0x8020000 -> +-------------+
					 | 0x801FC00   |	\
					 |    to       |	 \
					 | 0x802FFFF   |	  |- Reserved for checksum and other informations
					 |   1 KB      |	 /
					 | data        |	/
		0x801FC00 -> +-------------+
					 | 0x801F800   |	\
					 |    to       |	 \
					 | 0x801FBFF   |	  |- Bootloader and device Config-Stuff
					 |   1 KB      |	 /		(Dev-Addr->Can-ID, can-baudrate, Boardtype, Boardversion ...)
					 | data        |	/		 @see dev_config_regs
		0x801F800 -> +-------------+
					 | 0x801F000   |	\
					 |    to       |	 \
					 | 0x801F7FF   |	  |- Applicaton Configdata
					 |   2 KB      |	 /		(Tempsensor ID-Lookup table, Calibration-Data, Thresholds, ...)
					 | data        |	/		 @see app_cfg_regs
		0x801F000 -> +-------------+
					 |             |	\
					 |             |	 \
					 |             |	  |
					 |             |	  |
					 | 0x8008000   |	  |
					 |    to       |	  |
					 | 0x801EFFF   |	  |- Contain the application software (this project)
					 |   92 KB     |	  |
					 | application |	  |
					 |             |	  |
					 |             |	  |
					 |             |	 /
					 |             |	/
		0x8008000 -> +-------------+
					 |             |	\
					 | 0x8000000   |	 \
					 |    to       |	  |
					 | 0x8007FFF   |	  |- Contain bootloader software
					 |   32 KB     |	  |		@see CAN_BOOT-Projecz
					 | bootloader  |	 /
					 |             |	/
		0x8000000 -> +-------------+
 */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define LEDS 22

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t main_task_scheduler;
uint8_t adc_enable_mask;
uint8_t alive_timer;
uint16_t timer_10ms;


extern UART_HandleTypeDef huart3;
UartOneWire_HandleTypeDef ow;
DallasTemperatureData dt;

uint8_t resolution = TEMP_12_BIT;



//#define __DEBUG__

#ifdef __DEBUG__
////alles was persistend (im Flash) gespeichert werden soll, z.b. Kalibration, ...
__attribute__((__section__(".dev_config"))) const _DEV_CONFIG_REGS dev_config_regs = {
	__DEV_ID__,
	__BOARD_TYPE__,
	__BOARD_NAME__,
	__BOARD_VERSION__,
	__BOARD_MF_DATE__,
	DEAULT_BL_CAN_BITRATE,
	DEAULT_APP_CAN_BITRATE,
	DEAULT_TRIPP_CAN_ID,
	DEAULT_BROADCAST_CAN_ID
};
#endif


////Boardinfoname , ...
__attribute__((__section__(".board_info"))) _BOARD_INFO_STRUCT board_name = {__BOARD_NAME__};

////Softwareinfo, ...
__attribute__((__section__(".sw_info"))) const _SW_INFO_REGS sw_info_regs = {
	__SW_NAME__,
#ifdef __DEBUG__
	"Debug",
#else
	"Release",
#endif
	__SW_RELEASE__,
	__SW_RELEASE_DATE__,
	{	//git-info aus gitcommit.h
		__GIT_SHORT_HASH__,
		__GIT_DATE_UT__,
		__GIT_DATE_STR__,
		__GIT_BRANCH__,
	}
};


////alles was persistend (im Flash) gespeichert werden soll, z.b. Kalibration, ...
__attribute__((__section__(".app_config"))) const _BMS_BLK_CONFIG_REGS app_cfg_regs = {
	{	//neey_config_data
		4.0,						//max_balance_current
		3.45,						//start_voltage
		0.005,						//equalization_voltage
		3.25,						//sleep_voltage
		22,							//cell_count
		NEEY_BATTERY_TYPE_LFP,		//cell_type
		NEEY_BUZZER_TYPE_SINGLE,	//buzzer
		0,							//auto_run
		280,						//cell_cpacity
		2,							//send intervall in 500ms steps
		1,							//config_at_start
	},
	{	//temp_sensor_lookup_table
		#include "temp_sensor_lt_01_05.txt"
	},
	{	//lf280k_qr_info
 		#include "lf280k_qr.txt"
	},
#if defined (__DEBUG__)
	((1<<REG_ALERT_HEAT_SINK_TEMP) | (1<<REG_ALERT_NEEY) | (1<<REG_ALERT_NEEY_DATA) |(0<<REG_ALERT_CELL_VOLTAGE) |
	(0<<REG_ALERT_CELL_RESISTANCE) | (0<<REG_ALERT_CELL_TEMP) | (0<<REG_ALERT_BLK_VOLTAGE) | (0<<REG_ALERT_BLK_DIFF_VOLTAGE)), //allert_mask

	((0<<REG_ALERT_HEAT_SINK_TEMP) | (0<<REG_ALERT_NEEY) | (1<<REG_ALERT_NEEY_DATA) |(1<<REG_ALERT_CELL_VOLTAGE) |
	(0<<REG_ALERT_CELL_RESISTANCE) | (0<<REG_ALERT_CELL_TEMP) | (0<<REG_ALERT_BLK_VOLTAGE) | (0<<REG_ALERT_BLK_DIFF_VOLTAGE)), //crit_allert_mask
#else
	((1<<REG_ALERT_HEAT_SINK_TEMP) | (1<<REG_ALERT_NEEY) | (1<<REG_ALERT_NEEY_DATA) |(1<<REG_ALERT_CELL_VOLTAGE) |
	(1<<REG_ALERT_CELL_RESISTANCE) | (0<<REG_ALERT_CELL_TEMP) | (1<<REG_ALERT_BLK_VOLTAGE) | (1<<REG_ALERT_BLK_DIFF_VOLTAGE)), //allert_mask

	((0<<REG_ALERT_HEAT_SINK_TEMP) | (0<<REG_ALERT_NEEY) | (1<<REG_ALERT_NEEY_DATA) |(1<<REG_ALERT_CELL_VOLTAGE) |
	(1<<REG_ALERT_CELL_RESISTANCE) | (0<<REG_ALERT_CELL_TEMP) | (1<<REG_ALERT_BLK_VOLTAGE) | (1<<REG_ALERT_BLK_DIFF_VOLTAGE)), //crit_allert_mask
#endif
	{
			{2500,3650,(ENABLE_MAX_THRESHOLD|ENABLE_MIN_THRESHOLD)}, //cell_voltage 1/1000 Volt
			{100,400,(ENABLE_MAX_THRESHOLD|ENABLE_MIN_THRESHOLD)}, //cell_resistance mOhm
			{50,450,(ENABLE_MAX_THRESHOLD|ENABLE_MIN_THRESHOLD)}, //cell_temperature 1/10 °C
			{5500,7900,(ENABLE_MAX_THRESHOLD|ENABLE_MIN_THRESHOLD)}, //blk_voltage 1/100 Volt
			{0,200,(ENABLE_MAX_THRESHOLD)}, //blk_voltage_div 1/1000 Volt
			{50,500,(ENABLE_MAX_THRESHOLD|ENABLE_MIN_THRESHOLD)}, //heatsink_temperature 1/10 °C
	}
};

//#if defined (__DEBUG__) && defined(__ALERT_DEBUG__)

const _DEV_CONFIG_REGS* pDevConfig = (const _DEV_CONFIG_REGS*)DEV_CONFIG_FL_ADDRESS;


/**
 * @var		main_regs
 * @brief	Registersatz im Ram (Arbeitsregister)
 * @see		MAIN_REGS
 * @see		main_ee_regs
 *
 */
_MAIN_REGS main_regs = {
	//!<RW CTRL Ein-/Ausschalten usw.  (1 BYTE )
	((1<<REG_CTRL_ACTIVATE) | (1<<REG_CTRL_CRIT_ALERT) | (1<<REG_CTRL_ENABLE_PB) |
			(1<<REG_CTRL_ENABLE_OW) | (1<<REG_CTRL_ENABLE_NEEY) |  (1<<REG_CTRL_ENABLE_WS2815) ),

	SYS_OK,
	ERR_NONE,
	STATE_OFF,

	ALIVE_TIMEOUT_10MS,

	0,
	0,
	0,
	0,
	0,
	0,

	{}
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t process_10Ms_Timer(void);
void 	AlertHandler(void);
uint8_t check_AlertThresholds(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
 {
	 int DataIdx;
	 for (DataIdx = 0; DataIdx < len; DataIdx++)
	 {
		 ITM_SendChar(*ptr++);
	 }
	 return len;
 }

_LED_SIGNAL_STATE	led_signal_state={1,SLOW_FLASH,OFF,OFF};

/****************************************************************************
  * @brief  The application entry point.
  * @retval int
  */
void set_signal_led(uint8_t led, _LED_SIGNAL_MASK mask)
{
	if(led & GREEN_LED)
		led_signal_state.green_led_mask=mask;

	if(led & RED_LED)
		led_signal_state.red_led_mask=mask;

	if(led & BLUE_LED) {
		led_signal_state.blue_led_mask=mask;
		led_signal_state.mask=1;
	}

	return;
}


/****************************************************************************
  * @brief  The application entry point.
  * @retval int
  */
void signal_led_task(void)
{

	if(led_signal_state.green_led_mask & led_signal_state.mask) {
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
	}else {
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
	}

	if(led_signal_state.red_led_mask & led_signal_state.mask) {
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
	}else {
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
	}

	if(led_signal_state.blue_led_mask & led_signal_state.mask) {
		HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
		led_signal_state.blue_led_mask &= + ~led_signal_state.mask;
	}else {
		HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
	}

	led_signal_state.mask<<=1;
	if (!led_signal_state.mask) {
		led_signal_state.mask=1;
	}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t second_counter=1;
	main_task_scheduler = 0;
	alive_timer = 0;
	timer_10ms = 0;

	memcpy(&main_regs.cfg_regs, &app_cfg_regs, sizeof(_BMS_BLK_CONFIG_REGS));

	//copy dev-config from flash to ram (we will use it from ram)
	memcpy(&main_regs.dev_config, pDevConfig, sizeof(_DEV_CONFIG_REGS));

	//copy sofware-info from flash to ram (we will use it from ram)
	memcpy(&main_regs.sw_info, &sw_info_regs, sizeof(_SW_INFO_REGS));

	//copy board-name from flash to ram (we will use it from ram)
	memcpy(&main_regs.board_info, &board_name, sizeof(_BOARD_INFO_STRUCT));

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


	HAL_CAN_Start(&hcan);

	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
		Error_Handler();
	}

	//
	if (main_regs.ctrl & (1<<REG_CTRL_ENABLE_PB)) {
		PBalancer_init();
	}

	//start one-wire temperature sensors
	if (main_regs.ctrl & (1<<REG_CTRL_ENABLE_OW)) {
		OW_Init(&ow, &huart3);
		DT_SetOneWire(&dt, &ow);
		DT_init(&dt, resolution);

		main_task_scheduler |= PROCESS_OW;
	}

	// Start our 10ms timer
	HAL_TIM_Base_Start_IT(&htim4);

	//start all neey releated stuff
	MX_NEEY_Init();

#ifdef __WS2812B__
	//ws2815-led-monitor
	if (main_regs.ctrl & (1<<REG_CTRL_ENABLE_WS2815)) {
		ws2812b_init(&htim2, TIM_CHANNEL_2, LEDS);
	}
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /* PROCESS_NEEY  ------------------------------------------------------------*/
	  if (main_task_scheduler & PROCESS_NEEY) {
		  if (!process_NEEY()){
			  //check_AllertThrescholds();
			  main_task_scheduler &= ~PROCESS_NEEY;
		  }
	  }


	  /* PROCESS_PBALANCER  --------------------------------------------------------*/
	  if (main_task_scheduler & PROCESS_PBALANCER) {
		  if (main_regs.ctrl & (1<<REG_CTRL_ENABLE_PB)){

			  if (!process_PBalancer()){
				  //check_AllertThrescholds();
				  main_task_scheduler &= ~PROCESS_PBALANCER;
			  }
		  }else {
			  main_task_scheduler &= ~PROCESS_PBALANCER;
		  }
	  }


	  /* PROCESS_OW  ------------------------------------------------------------*/
	  if (main_task_scheduler & PROCESS_OW) {
		  if (main_regs.ctrl & (1<<REG_CTRL_ENABLE_OW)){

				uint32_t millis = HAL_GetTick();
				DT_ContiniousProceed(&dt, millis);

		  }else {
			  main_task_scheduler &= ~PROCESS_OW;
		  }
	  }


	  /* PROCESS_CAN  -----------------------------------------------------------*/
	  if (main_task_scheduler & PROCESS_CAN) {
		  if (!process_CAN()){
			  main_task_scheduler &= ~PROCESS_CAN;
		  }
	  }


	  /* PROCESS_10_MS_TASK ------------------------------------------------------*/
	  if (main_task_scheduler & PROCESS_10_MS_TASK) {
		  if (!process_10Ms_Timer()){
			  main_task_scheduler &= ~PROCESS_10_MS_TASK;
		  }
	  }


	  /* PROCESS_100_MS_TASK  -----------------------------------------------------*/
	  if (main_task_scheduler & PROCESS_100_MS_TASK) {
		  main_task_scheduler &= ~PROCESS_100_MS_TASK;

		  signal_led_task();
	  }


	  /* PROCESS_1000_MS_TASK  ----------------------------------------------------*/
	  if (main_task_scheduler & PROCESS_1000_MS_TASK) {
		  main_task_scheduler &= ~PROCESS_1000_MS_TASK;

		  if (!(second_counter%2)) {
			  if(main_regs.ctrl & (1<<REG_CTRL_ENABLE_NEEY)) {
				  neey_task_scheduler |= PROCESS_NEEY_ALIVE;
				  main_task_scheduler |= PROCESS_NEEY;
			  }
		  }
		second_counter++;
	  }

#ifdef __DEBUG__
	  HAL_GPIO_TogglePin(WS2815_ENABLE_GPIO_Port, WS2815_ENABLE_Pin);
#endif
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */




//*****************************************************************************
//
//! wird alle 10ms aufgerufen, getriggert durch den Timer4-overrun
//!
//! \fn uint8_t process_10Ms_Timer(void)
//!
//!
//! \return None.
//
//*****************************************************************************
uint8_t check_AlertThresholds(void)
{

	return 0;
}


//*****************************************************************************
//
//! wird alle 10ms aufgerufen, getriggert durch den Timer4-overrun
//!
//! \fn uint8_t process_10Ms_Timer(void)
//!
//!
//! \return None.
//
//*****************************************************************************
uint8_t process_10Ms_Timer(void)
{
	if (alive_timer)
	{
		if (--alive_timer == 0)
		{
			//kritisch
			alive_timer = main_regs.alive_timeout;
			uint8_t alert_msg;
			alert_msg = ERR_ALIVE;
			DoAlert(&alert_msg, 1);
		}
	}

	//HAL_GPIO_WritePin(FAN_SPEED_GPIO_Port, FAN_SPEED_Pin, GPIO_PIN_TOGLE);
	//HAL_GPIO_TogglePin(FAN_SPEED_GPIO_Port,FAN_SPEED_Pin);
	if (main_regs.ctrl & (1<<REG_CTRL_ENABLE_PB))
		process_PBalancer();

	if (!(timer_10ms % 10))
	{
		main_task_scheduler |= PROCESS_100_MS_TASK;
	}

	if (!(++timer_10ms % 100))
	{
		main_task_scheduler |= PROCESS_1000_MS_TASK;
	}

	return 0;
}


//*****************************************************************************
//
//! wird im Fehlerfall aufgerufen und aktivert die vollständige Abschaltung
//!
//! \fn void AllertHandler(void)
//!
//!
//! \return None.
//
//*****************************************************************************
void AlertHandler(void)
{
	//send Something?

	if (main_regs.ctrl & (1<<REG_CTRL_ACTIVATE))
	{
		//HAL_GPIO_WritePin(RELAY_2_GPIO_Port, RELAY_2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);

		Error_Handler();
	}
}


//*****************************************************************************
//
//! wird im Fehlerfall aufgerufen und aktivert die vollständige Abschaltung
//!
//! \fn void DoAlert(void)
//!
//!
//! \return None.
//
//*****************************************************************************
void DoAlert(uint8_t* p_msg, uint8_t len)
{

	if (main_regs.ctrl & (1<<REG_CTRL_CRIT_ALERT))
	{
		can_send_trip_msg();

		AlertHandler();
	}

	//send Something?
	can_send_allert_msg(p_msg, len);


	main_regs.sys_err=(_SYS_ERR_CODES)p_msg[0];
	main_regs.sys_state=SYS_ERROR;

}



//*****************************************************************************
//
//! Callback: timer has rolled over
//!
//! \fn void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//!
//!
//! \return None.
//
//*****************************************************************************
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim4 )
  {
    main_task_scheduler |= PROCESS_10_MS_TASK;
  }
}


/* Jump to application -------------------------------------------------------------*/
void JumpToBtld(void){
    uint32_t  JumpAddress = *(__IO uint32_t*)(DEV_BL_ADDRESS + 4);
    pFunction Jump = (pFunction)JumpAddress;


    HAL_RCC_DeInit();
    HAL_DeInit();

    //HAL_NVIC_DisableIRQ();

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

#if (SET_VECTOR_TABLE)
    SCB->VTOR = DEV_BL_ADDRESS;
#endif

    __set_MSP(*(__IO uint32_t*)DEV_BL_ADDRESS);
    Jump();
}


/* Jump to application -------------------------------------------------------------*/
void JumpToApp(void){
    uint32_t  JumpAddress = *(__IO uint32_t*)(DEV_APP_ADDRESS + 4);
    pFunction Jump = (pFunction)JumpAddress;


    HAL_RCC_DeInit();
    HAL_DeInit();

    //HAL_NVIC_DisableIRQ();

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

#if (SET_VECTOR_TABLE)
    SCB->VTOR = DEV_APP_ADDRESS;
#endif

    __set_MSP(*(__IO uint32_t*)DEV_APP_ADDRESS);
    Jump();
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
