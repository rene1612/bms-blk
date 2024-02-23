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
//#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "OneWire.h"
#include "neey.h"
#include "passive_balancer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern float Temp[MAXDEVICES_ON_THE_BUS];
uint8_t main_task_scheduler;
uint8_t adc_enable_mask;
uint8_t alive_timer;
uint16_t timer_10ms;

__attribute__((__section__(".board_info"))) const unsigned char BOARD_NAME[16] = "BMS-BLK-APP";

__attribute__((__section__(".sw_info"))) const _SW_INFO_REGS sw_info_regs = {
		__SW_RELEASE__,
		__SW_RELEASE_DATE__,
		0x3cd7450ab698ad24,
		"no tag"
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
		#include "temp_sensor_lt.txt"
	},
	{	//lf280k_qr_info
 		#include "lf280k_qr.txt"
	},
};


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
	((1<<REG_CTRL_ACTIVATE) | (1<<REG_CTRL_CRIT_ALLERT)),

	SYS_OK,
	STATE_OFF,

	ALIVE_TIMEOUT_10MS,

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
void 	AllertHandler(void);
uint8_t check_AllertThrescholds(void);
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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	main_task_scheduler = 0;
	//adc_enable_mask =
	//adc_enable_mask = (0x01<<ADC_CH5);
	alive_timer = 0;
	timer_10ms = 0;

	memcpy(&main_regs.cfg_regs, &app_cfg_regs, sizeof(_BMS_BLK_CONFIG_REGS));

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
  /* USER CODE BEGIN 2 */

  //
  PBalancer_init();

  HAL_CAN_Start(&hcan);

  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
	  Error_Handler();
  }

  //start one-wire temperature sensors
  get_ROMid();

  // Start our 10ms timer
  HAL_TIM_Base_Start_IT(&htim4);

  //start all neey releated stuff
  MX_NEEY_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if (main_task_scheduler & PROCESS_NEEY)
	  {
		  if (!process_NEEY())
		  {
			  //check_AllertThrescholds();
			  main_task_scheduler &= ~PROCESS_NEEY;
		  }
	  }


	  if (main_task_scheduler & PROCESS_PBALANCER)
	  {
		  if (!process_PBalancer())
		  {
			  //check_AllertThrescholds();
			  main_task_scheduler &= ~PROCESS_PBALANCER;
		  }
	  }


	  if (main_task_scheduler & PROCESS_CAN)
	  {
		  if (!process_CAN())
			  main_task_scheduler &= ~PROCESS_CAN;
	  }


	  if (main_task_scheduler & PROCESS_10_MS_TASK)
	  {
		  if (!process_10Ms_Timer())
			  main_task_scheduler &= ~PROCESS_10_MS_TASK;
	  }


	  if (main_task_scheduler & PROCESS_100_MS_TASK)
	  {
		  main_task_scheduler &= ~PROCESS_100_MS_TASK;

		  get_Temperature();

		  HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
	  }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV128;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
uint8_t check_AllertThrescholds(void)
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
		}
	}

	if (!(++timer_10ms % 10))
	{
		main_task_scheduler |= PROCESS_100_MS_TASK;
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
void AllertHandler(void)
{
	//send Something?

	if (main_regs.ctrl & (1<<REG_CTRL_ACTIVATE))
	{
		//HAL_GPIO_WritePin(RELAY_2_GPIO_Port, RELAY_2_Pin, GPIO_PIN_SET);

		while(1){}
	}
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
