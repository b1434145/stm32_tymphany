/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Buff_len (200)
#define TIM2_CNT_NUM (100)
#define SD_CNT_NUM (10)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char tx_buff[Buff_len];

uint8_t rx_data[1] = {0};

typedef union {
	#define FloatSize (2)
	float f4Byte[FloatSize];
	char  Byte4float[4*FloatSize];
} xu_floatChar;
xu_floatChar PostAvgOffsetValue = {0};
#define SAVE_ON_FLOAT  PostAvgOffsetValue.f4Byte[0]
#define SAVE_OFF_FLOAT PostAvgOffsetValue.f4Byte[1]

typedef union {
	char u8Byte;
	struct {
		unsigned bit0:1;
		unsigned bit1:1;
		unsigned bit2:1;
		unsigned bit3:1;
		unsigned bit4:1;
		unsigned bit5:1;
		unsigned bit6:1;
	} sbit;
} xu_sbit;
xu_sbit flag[1] = {0};
#define BTN_blue flag[0].sbit.bit0

typedef enum {
	xePR_State_Empty,
	xePR_State_Ready,
	xePR_State_Idle,
} xe_ProximityReady;
xe_ProximityReady ProximityReady = xePR_State_Empty;

typedef enum {
	xePOCFD_PowerOn,
	xePOCFD_FreeRun,
} xe_PowerOn_Check_Freq_Directly;
xe_PowerOn_Check_Freq_Directly xe_WearValueState = xePOCFD_FreeRun;

typedef enum {
	xeWS_WearInitial,
	xeWS_WearOn,
	xeWS_WearOff,
	xeWS_Normal,
} xe_WearState;
static xe_WearState xeWearState = xeWS_WearInitial;

typedef enum {
	xeCalibrationOn,
	xeCalibrationOff,
} xe_CalibrationState;
xe_CalibrationState xeCalibrationState = xeCalibrationOn;

float SD = 0;
float SD_sqrt = 0;
float SD_avg = 0;
float present = 0;
uint32_t avg = 0;
calibration_check = 1;

static unsigned int save_TIM2[TIM2_CNT_NUM] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void task1ms(void);
void task5ms(void);
void task10ms(void);
void task50ms(void);
void task100ms(void);
void task500ms(void);
void task1000ms(void);
void task2000ms(void);
void task5000ms(void);
float ffnPostAvg_0d05(float u32Input);
float ffnPostAvg_0d25(float u32Input);
void calibration_task(void);
float present_freq(uint32_t);
uint32_t avg_freq(uint32_t);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void task1ms(void) {

}
void task5ms(void) {

}
void task10ms(void) {
	uint32_t TIM2CNT = __HAL_TIM_GET_COUNTER(&htim2);
	__HAL_TIM_SET_COUNTER(&htim2, 0);

//		snprintf(tx_buff, Buff_len, "counter: %d \r\n",  TIM2CNT);
//		HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff, strlen(tx_buff), HAL_MAX_DELAY);

		unsigned int freq = 0; //can speed up
		float mean = 0;
		for(int i = TIM2_CNT_NUM-1; i > 0; i--) {
			save_TIM2[i] = save_TIM2[i-1];
		}
		save_TIM2[0] = TIM2CNT;
		for(int i = 0; i < TIM2_CNT_NUM; i++) {
			freq += save_TIM2[i];
		}
		SD = 0;
		mean = freq / TIM2_CNT_NUM;
		for (int i = 0; i < 10; i++) {
		    SD += pow(save_TIM2[i] - mean, 2);
		}
		SD_sqrt = sqrt(SD / TIM2_CNT_NUM);

//		snprintf(tx_buff, Buff_len, "SD: %f \r\n",  SD_sqrt);
//		HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff, strlen(tx_buff), HAL_MAX_DELAY);

		float ftmp;
		//Plot Frequency post avg (0.25)
		ftmp = freq;
		float Get0d25PostAvg = ffnPostAvg_0d25(ftmp);

		//Plot Frequency post avg (0.05)
		float Get0d05PostAvg = ffnPostAvg_0d05(ftmp);
		present = present_freq(TIM2CNT);
		avg = avg_freq(freq);
		float offset = SD_avg*100;

		const float cfOffsetValue = 250.0;
		float fMaxBoundary = Get0d05PostAvg+cfOffsetValue;
		float fMinBoundary = Get0d05PostAvg-cfOffsetValue;
		float fPostAvgFrequency = Get0d25PostAvg-fMinBoundary;

//		snprintf(tx_buff, Buff_len, "present: %f avg: %f offset: %f \r\n", present, avg, offset);
//		HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff, strlen(tx_buff), HAL_MAX_DELAY);

//		snprintf(tx_buff, Buff_len, "freq: %d \r\n", freq);
//		HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff, strlen(tx_buff), HAL_MAX_DELAY);

		switch(ProximityReady) {
			case xePR_State_Empty:
				break;
			case xePR_State_Ready:
				break;
			case xePR_State_Idle: //can be added
				break;
		}

		float TransferMax = 2*cfOffsetValue;
		float TransferMin = 0.0;

		if (ProximityReady == xePR_State_Idle) {
			switch(xe_WearValueState) {
				case xePOCFD_FreeRun:
						//Normal
						if(abs(present - avg) < 13*offset) {
							if(xeWearState != xeWS_WearInitial) {
								xeWearState = xeWS_WearInitial;
							}
						}

						//Wear on
						if(present - avg < (-13)*offset) {
							if(xeWearState != xeWS_WearOn) {
								xeWearState = xeWS_WearOn;

								SAVE_ON_FLOAT = Get0d25PostAvg;
							}
						}

						//Wear off
						if(present - avg > 13*offset) {
							if(xeWearState != xeWS_WearOff) {
								xeWearState = xeWS_WearOff;

								SAVE_OFF_FLOAT = Get0d25PostAvg;
							}
						}
						break;
			}
		}

		switch(xeWearState) {
			case xeWS_WearInitial:
					break;
			case xeWS_Normal:
					break;
			case xeWS_WearOn:
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
					break;
			case xeWS_WearOff:
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
					break;
		}


}
void task50ms(void) {

}
void task100ms(void) {
//	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	if(xeWearState == xeWS_WearInitial) {
		static float save_SD[SD_CNT_NUM] = {0};
		float SD_sum = 0;
		for(int i = SD_CNT_NUM-1; i > 0; i--) {
					save_SD[i] = save_SD[i-1];
					SD_sum += save_SD[i];
		}
		save_SD[0] = SD_sqrt;
		SD_avg = SD_sum / (SD_CNT_NUM-1);
	}
}
void task500ms(void) {
	if (!(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)))  {
		BTN_blue = 0;
		snprintf(tx_buff, Buff_len, "BlueButton Press\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff, strlen(tx_buff), HAL_MAX_DELAY);
	}

}
void task1000ms(void) {

}

void task2000ms(void) {
	switch(ProximityReady) {
		case xePR_State_Empty:
			ProximityReady = xePR_State_Ready;
		case xePR_State_Ready:
			ProximityReady = xePR_State_Idle;
		default:
			break;
	}

}

void task5000ms(void) {
}

void calibration_away_task(void) {

	snprintf(tx_buff, Buff_len, "Enter Calibration Mode\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff, strlen(tx_buff), HAL_MAX_DELAY);

	snprintf(tx_buff, Buff_len, "Please keep away from headphone\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff, strlen(tx_buff), HAL_MAX_DELAY);
	HAL_Delay(3000);

	snprintf(tx_buff, Buff_len, "Calibration will start in 3 s\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff, strlen(tx_buff), HAL_MAX_DELAY);
	HAL_Delay(1000);

	snprintf(tx_buff, Buff_len, "Calibration will start in 2 s\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff, strlen(tx_buff), HAL_MAX_DELAY);
	HAL_Delay(1000);

	snprintf(tx_buff, Buff_len, "Calibration will start in 1 s\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff, strlen(tx_buff), HAL_MAX_DELAY);
	HAL_Delay(1000);

	snprintf(tx_buff, Buff_len, "freq = %d\r\n", avg);
	HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff, strlen(tx_buff), HAL_MAX_DELAY);
	HAL_Delay(1000);
}

void calibration_close_task(void) {

	snprintf(tx_buff, Buff_len, "Enter Calibration Mode\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff, strlen(tx_buff), HAL_MAX_DELAY);

	snprintf(tx_buff, Buff_len, "Please be close to headphone\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff, strlen(tx_buff), HAL_MAX_DELAY);
	HAL_Delay(3000);

	snprintf(tx_buff, Buff_len, "Calibration will start in 3 s\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff, strlen(tx_buff), HAL_MAX_DELAY);
	HAL_Delay(1000);

	snprintf(tx_buff, Buff_len, "Calibration will start in 2 s\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff, strlen(tx_buff), HAL_MAX_DELAY);
	HAL_Delay(1000);

	snprintf(tx_buff, Buff_len, "Calibration will start in 1 s\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff, strlen(tx_buff), HAL_MAX_DELAY);
	HAL_Delay(1000);

	snprintf(tx_buff, Buff_len, "freq = %d\r\n", avg);
	HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff, strlen(tx_buff), HAL_MAX_DELAY);
	HAL_Delay(1000);

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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(1) {
	  		  static unsigned int   polling1ms = 0;
	  		  static unsigned char  polling5ms = 0, polling10ms = 0, polling50ms = 0, polling100ms = 0;
	  		  static unsigned short polling500ms = 0, polling1000ms = 0, polling2000ms = 0, polling5000ms = 0;

	  		  if(polling1ms != HAL_GetTick() ) {
				  polling1ms++;
				  task1ms();

				  polling5ms++;
				  polling10ms++;
				  polling50ms++;
				  polling100ms++;
				  polling500ms++;
				  polling1000ms++;
				  polling2000ms++;
				  polling5000ms++;
	  		  }
	  		  if(polling5ms >= 5) {
	  			  polling5ms = 0;
	  			  task5ms();
	  		  }

	  		  if(polling10ms>=10) {
	  			  polling10ms = 0;
	  			  task10ms();
	  		  }
	  		  if(polling50ms>=50) {
	  			  polling50ms = 0;
	  			  task50ms();
	  		  }
	  		  if(polling100ms>=100) {
	  			  polling100ms = 0;
	  		  	  task100ms();
	  		  }
	  		  if(polling500ms>=500) {
	  			  polling500ms = 0;
	  			  task500ms();
	  		  }
	  		  if(polling1000ms>=1000) {
	  			  polling1000ms = 0;
	  			  task1000ms();
	  		  }
	  		if(polling2000ms>=2000) {
				  polling2000ms = 0;
				  task2000ms();
			  }
	  		if(polling5000ms>=5000) {
				  task5000ms();
				  if(calibration_check == 2) {
					  calibration_close_task();
					  calibration_check++;
					  for(int i = 0; i < TIM2_CNT_NUM; i++) {
						save_TIM2[i] = 0;
					 }
					  ProximityReady = xePR_State_Empty;
					  xeWearState = xeWS_WearInitial;
				  }
				  if(calibration_check == 1) {
					  calibration_away_task();
					  calibration_check++;
					  for(int i = 0; i < TIM2_CNT_NUM; i++) {
						  save_TIM2[i] = 0;
					  }
					  ProximityReady = xePR_State_Empty;
					  xeWearState = xeWS_WearInitial;
				  }
				  polling1ms = HAL_GetTick();
				  polling5ms = 0;
				  polling10ms = 0;
				  polling50ms = 0;
				  polling100ms = 0;
				  polling500ms = 0;
				  polling1000ms = 0;
				  polling2000ms = 0;
				  polling5000ms = 0;
			  }
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_TIM2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_PLLCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart2.Init.BaudRate = 38400;
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
  HAL_TIM_Base_Start(&htim2);
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
float ffnPostAvg_0d05(float u32Input) {
	static float fsaveFreq = 0.0;
		   float fcurrFreq = 0.0;

	const float postAVGweight = (0.05);

	fcurrFreq = u32Input;
	fsaveFreq = fsaveFreq*(1-postAVGweight) + fcurrFreq*(postAVGweight);

	return fsaveFreq;
}

float ffnPostAvg_0d25(float u32Input) {
	static float fsaveFreq = 0.0;
		   float fcurrFreq = 0.0;

	const float postAVGweight = (0.25);

	fcurrFreq = u32Input;
	fsaveFreq = fsaveFreq*(1-postAVGweight) + fcurrFreq*(postAVGweight);

	return fsaveFreq;
}

float present_freq(uint32_t u32Input) {
	static float fsaveFreq = 0.0;
		   float fcurrFreq = 0.0;

	const int weight = 100;

	fcurrFreq = u32Input;
	fsaveFreq =  fcurrFreq*weight;

	return fsaveFreq;
}

uint32_t avg_freq(uint32_t u32Input) {
	static float fsaveFreq = 0.0;
		   float fcurrFreq = 0.0;

	const float postAVGweight = (0.05);

//	fcurrFreq = u32Input;
//	fsaveFreq = fsaveFreq*(1-postAVGweight) + fcurrFreq*(postAVGweight);

	return u32Input;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
