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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "flash_driver.h"
#include "crc_driver.h"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//button states
#define RELAY1_ON 1
#define RELAY1_OFF 2
#define RELAY2_ON 3
#define RELAY2_OFF 4
#define RELAY3_ON 5
#define RELAY3_OFF 6
#define RELAY4_ON 7
#define RELAY4_OFF 8

//flash addresses
#define RELAY1_ADDRESS 0x0803F800	//page 127
#define RELAY2_ADDRESS 0x0803F000	//page 126
#define RELAY3_ADDRESS 0x0803E800	//page 125
#define RELAY4_ADDRESS 0x0803E000	//page 124
#define TEST_ADDRESS 0x0803D800		//page 123

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

WWDG_HandleTypeDef hwwdg;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_WWDG_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint64_t RxData[3];
bool rx_flag = false;
uint8_t errors = 0;
bool timer_flag = false;
bool error_flag = false;

//counts used for testing
uint32_t relay1_count = 0;
uint32_t relay2_count = 0;
uint32_t relay3_count = 0;
uint32_t relay4_count = 0;

//UART messages to communicate updates
uint8_t err_max[50] = "CRC ERROR detected over 10 times within 5 sec\n\r";
uint8_t relay1_msg[20] = "RELAY1 ON\n\r";
uint8_t relay2_msg[20] = "RELAY2 ON\n\r";
uint8_t relay3_msg[20] = "RELAY3 ON\n\r";
uint8_t relay4_msg[20] = "RELAY4 ON\n\r";
uint8_t wwdg_msg[20] = "Watchdog init\n\r";


/**
 * receives data with crc from transmitter every 10ms
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t*)RxData, sizeof(RxData));
	rx_flag = true;
}

/**
 * decodes crc value and if the remainder is not zero, track an error in the data sent
 */
void crc_decode(){
	rx_flag = false;
	int shift = 60;
	int position = 0;
	uint64_t appended_data = RxData[1];
	uint64_t dividend = appended_data & 0xF000000000000000;
	dividend = dividend >> shift;
	uint64_t ans = crc_xor(dividend);

	uint64_t remain = crc_division(appended_data, position, shift, ans);
	if(remain != 0){
		errors++;
		if(!error_flag){
			LL_TIM_EnableCounter(TIM16);
			error_flag = true;
		}
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();

  /* USER CODE BEGIN 2 */
	LL_TIM_EnableCounter(TIM17);
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t*)RxData, sizeof(RxData));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		//watchdog calibration
		if(LL_TIM_IsActiveFlag_UPDATE(TIM17)){
			LL_TIM_ClearFlag_UPDATE(TIM17);
			LL_TIM_DisableCounter(TIM17);
			MX_WWDG_Init();
			timer_flag = true;
			HAL_UART_Transmit(&huart2, wwdg_msg, 20, 10);
		}

		//relay logic
		if(rx_flag){
			if(timer_flag){
				HAL_WWDG_Refresh(&hwwdg);		//receiving timeout
			}
			crc_decode();

			switch(RxData[0]){
			case RELAY1_ON:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
				HAL_UART_Transmit(&huart2, relay1_msg, 20, 10);
				relay1_count++;
				flash_write(RELAY1_ADDRESS, relay1_count, timer_flag, &hwwdg);
				break;
			case RELAY1_OFF:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
				break;
			case RELAY2_ON:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_UART_Transmit(&huart2, relay2_msg, 20, 10);
				relay2_count++;
				flash_write(RELAY2_ADDRESS, relay2_count, timer_flag, &hwwdg);
				break;
			case RELAY2_OFF:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
				break;
			case RELAY3_ON:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
				HAL_UART_Transmit(&huart2, relay3_msg, 20, 10);
				relay3_count++;
				flash_write(RELAY3_ADDRESS, relay3_count, timer_flag, &hwwdg);
				break;
			case RELAY3_OFF:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
				break;
			case RELAY4_ON:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
				HAL_UART_Transmit(&huart2, relay4_msg, 20, 10);
				relay4_count++;
				flash_write(RELAY4_ADDRESS, relay4_count, timer_flag, &hwwdg);
				break;
			case RELAY4_OFF:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
				break;
			default:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
				break;
			}
		}

		//error reset
		if(LL_TIM_IsActiveFlag_UPDATE(TIM16)){
			LL_TIM_ClearFlag_UPDATE(TIM16);
			LL_TIM_DisableCounter(TIM16);
			error_flag = false;
			errors = 0;
		}
		if(errors > 10){
			HAL_UART_Transmit(&huart2, err_max, 50, 10);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_TIM16|RCC_PERIPHCLK_TIM17;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  PeriphClkInit.Tim17ClockSelection = RCC_TIM17CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */
	//f = 8Mhz / PSC				PSC = 615
	//T = (1 / f) * period = 5s		Period = 64934
  /* USER CODE END TIM16_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM16);

  /* TIM16 interrupt Init */
  NVIC_SetPriority(TIM1_UP_TIM16_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  TIM_InitStruct.Prescaler = 615;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 64934;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM16, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM16);
  /* USER CODE BEGIN TIM16_Init 2 */

	LL_TIM_GenerateEvent_UPDATE(TIM16);

	LL_TIM_ClearFlag_UPDATE(TIM16);
  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */
	//f = 8MHz / PSC					PSC = 367
	//T = (1 / f) * period = 3s			Period = 65216
  /* USER CODE END TIM17_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM17);

  /* TIM17 interrupt Init */
  NVIC_SetPriority(TIM1_TRG_COM_TIM17_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  TIM_InitStruct.Prescaler = 367;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65216;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM17, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM17);
  /* USER CODE BEGIN TIM17_Init 2 */

	LL_TIM_GenerateEvent_UPDATE(TIM17);

	LL_TIM_ClearFlag_UPDATE(TIM17);
  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief WWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_WWDG_Init(void)
{

  /* USER CODE BEGIN WWDG_Init 0 */
	//watch dog counter for window of 5 to 15 ms
  /* USER CODE END WWDG_Init 0 */

  /* USER CODE BEGIN WWDG_Init 1 */
	//counter = ((max_time * clk) / (4096 * prescalar)) + 64			= ((.015 * 8M) / (4096 * 4)) + 64 = 79
	//window = counter - ((min_time * clk) / (4096 * prescalar))		= 72 - ((0.005 * 8M) / (4096 * 4)) = 77
	// 5-30ms window for watchdog reset
  /* USER CODE END WWDG_Init 1 */
  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_4;
  hwwdg.Init.Window = 77;
  hwwdg.Init.Counter = 79;
  hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN WWDG_Init 2 */

  /* USER CODE END WWDG_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pins : PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
