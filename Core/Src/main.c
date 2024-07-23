/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct 
{
	uint32_t tick_inicial;
	uint32_t time_lapse;
}stimer;

typedef enum
{
	counting = 0,
	off
}typeTimerStatus;
typedef enum 
{
	false = 0,
	true
}typeBoolStatus;

typedef enum
{
	release = 0,
	push
}typePushBottonState;
typedef enum 
{
	edge_detect = 0,
	filter_bouncing
}typeEdgeDetectState;
typedef enum
{
	fall = 0,
	rise
}typeEdge;

typedef struct
{
	typePushBottonState ant;
	typePushBottonState atu;
	typeEdgeDetectState edge_detect_state;
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
	stimer dbtimer;
}sBot;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t a = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void stimer_init(stimer* pTimer, uint32_t tLapse);
void stimer_reload(stimer* pTimer);
typeTimerStatus stimer_is_off(stimer* pTimer);
typeBoolStatus is_Rise_Edge(sBot* pBot);
void edge_detect_init(sBot* pBot, GPIO_TypeDef* GPIOz, uint16_t zPin, typeEdge edge, uint32_t dbouncing);
typeBoolStatus is_Fall_Edge(sBot* pBot);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  /* USER CODE BEGIN 2 */
	//HAL_GPIO_WritePin(LED_Orange_GPIO_Port, LED_Orange_Pin, GPIO_PIN_SET);
	/*uint32_t tickctrl, tickctrlold, timelapse;
	tickctrlold = HAL_GetTick();
	timelapse = 500;*/
	stimer stimer1, stimer2;
	stimer_init(&stimer1, 500);
	stimer_init(&stimer2, 1000);
	sBot bluebot;
	typeBoolStatus OrangeLedBlinkOn = false;
	
	edge_detect_init(&bluebot,Bot_Blue_GPIO_Port, Bot_Blue_Pin, rise, 100);
	
	//uint16_t bstatus_old, bstatus;
	//bstatus = HAL_GPIO_ReadPin(Bot_Blue_GPIO_Port, Bot_Blue_Pin);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  
	  
	  if (true == is_Rise_Edge(&bluebot))
	  {
		  OrangeLedBlinkOn = !OrangeLedBlinkOn;
		  a++;
	  }
	  
	  if (true == OrangeLedBlinkOn)
	  {
		  if (off == stimer_is_off(&stimer1))
		  {
			  HAL_GPIO_TogglePin(LED_Orange_GPIO_Port, LED_Orange_Pin);
			  stimer_reload(&stimer1);
		  }
	  }
	 
	  
	  if (stimer_is_off(&stimer2))
	  {
		  HAL_GPIO_TogglePin(LED_Blue_GPIO_Port, LED_Blue_Pin);
		  stimer_reload(&stimer2);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void stimer_init(stimer* pTimer, uint32_t tLapse)
{
	pTimer->tick_inicial = HAL_GetTick();
	pTimer->time_lapse = tLapse;
	
}
void stimer_reload(stimer* pTimer)
{
	pTimer->tick_inicial += pTimer->time_lapse;	
}
typeTimerStatus stimer_is_off(stimer* pTimer)
{
	uint32_t Delta = HAL_GetTick() - pTimer->tick_inicial;
	return (Delta >= pTimer->time_lapse);
	
}

typeBoolStatus is_Rise_Edge(sBot* pBot)
{
	pBot->ant = pBot->atu;
	pBot->atu = (typePushBottonState)HAL_GPIO_ReadPin(pBot->GPIOx, pBot->GPIO_Pin);
	if (pBot->atu == push && pBot->ant == release && pBot->edge_detect_state == edge_detect) 
	{
		stimer_reload(&(pBot->dbtimer));
		pBot->edge_detect_state = filter_bouncing;
	}
	if (stimer_is_off(&(pBot->dbtimer)) && pBot->edge_detect_state == filter_bouncing)
	{
		pBot->atu = (typePushBottonState)HAL_GPIO_ReadPin(pBot->GPIOx, pBot->GPIO_Pin);
		pBot->edge_detect_state = edge_detect;
		return (pBot->atu == push);	 
	}	
	return false;
}
void edge_detect_init(sBot* pBot, GPIO_TypeDef* GPIOz, uint16_t zPin, typeEdge edge, uint32_t dbtimelapse)
{
	pBot->GPIOx = GPIOz;
	pBot->GPIO_Pin = zPin;
	pBot->edge_detect_state = edge_detect;
	stimer_init(&(pBot->dbtimer), dbtimelapse);
	
	if (rise == edge)
	{
		pBot->ant = push;
		pBot->atu = push;
	}
	else 
	{
		pBot->ant = release;
		pBot->atu = release;
	}
	
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
