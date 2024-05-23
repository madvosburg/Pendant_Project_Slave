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
#define LED1_ON 1
#define LED1_OFF 2
#define LED2_ON 3
#define LED2_OFF 4
#define LED3_ON 5
#define LED3_OFF 6
#define LED4_ON 7
#define LED4_OFF 8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

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
uint64_t crc_key = 0xD;
bool rx_flag = false;
uint8_t errors = 0;
bool timer_flag = false;
bool wwdg_flag = false;
bool error_flag = false;

uint8_t err_max[50] = "ERROR detected over 10 times within 5 sec\n\r";

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t*)RxData, sizeof(RxData));
	rx_flag = true;
}

/**
 * XOR logic used to divide data by key
 */
uint64_t crc_xor(uint64_t div_data){
	uint64_t ans = div_data;
	if(ans & 0b1000){
		ans = ans ^ crc_key;			//if leftmost bit is 1, perform xor with key
	}else{
		ans = ans ^ 0b0000;				//if leftmost bit is 0, perform xor with all zeros
	}
	return ans;
}

/**
 * divides data by key to get remainder
 *
 * takes 4 bits at a time and XORs them until 4 bit remainder is left
 */
uint64_t crc_division(uint64_t data, int curs_pos, int shift_pos, uint64_t answer){
	int cursor = curs_pos;
	int bit_shift = shift_pos;
	uint64_t remain = answer;
	uint64_t dividend = 0;

	while(bit_shift > 0){
		bit_shift--;
		dividend = data & (0x0800000000000000 >> cursor);
		dividend = dividend >> bit_shift;
		remain = remain << 1;
		remain += dividend;
		remain = crc_xor(remain);
		cursor++;
	}
	return remain;
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
			HAL_TIM_Base_Start_IT(&htim16);
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
	HAL_TIM_Base_Start_IT(&htim17);

	HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t*)RxData, sizeof(RxData));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(wwdg_flag){
			MX_WWDG_Init();
			wwdg_flag = false;
		}
		if(rx_flag){
			if(timer_flag){
				HAL_WWDG_Refresh(&hwwdg);		//receiving timeout
			}
			crc_decode();
			rx_flag = false;

			switch(RxData[0]){
				case LED1_ON:
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
					break;
				case LED1_OFF:
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
					break;
				case LED2_ON:
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
					break;
				case LED2_OFF:
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
					break;
				case LED3_ON:
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
					break;
				case LED3_OFF:
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
					break;
				case LED4_ON:
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
					break;
				case LED4_OFF:
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

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 615;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 64934;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

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

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 367;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65216;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

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
	//counter = ((max_time * clk) / (4096 * prescalar)) + 64			= ((.015 * 8M) / (4096 * 4)) + 64 = 72
	//window = counter - ((min_time * clk) / (4096 * prescalar))		= 72 - ((0.005 * 8M) / (4096 * prescalar)) = 70
  /* USER CODE END WWDG_Init 1 */
  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_4;
  hwwdg.Init.Window = 70;
  hwwdg.Init.Counter = 72;
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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	UNUSED(htim);

	if(htim == &htim16){
		error_flag = false;
		errors = 0;
		HAL_TIM_Base_Stop_IT(&htim16);
	}else if(htim == &htim17){
		timer_flag = true;
		wwdg_flag = true;
		HAL_TIM_Base_Stop_IT(&htim17);
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
