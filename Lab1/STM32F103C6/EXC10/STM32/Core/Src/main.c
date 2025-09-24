/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
GPIO_TypeDef* LED_PORT = GPIOA;

uint16_t LED_PINS[12] = {
    led0_Pin,
    led1_Pin,
    led2_Pin,
    led3_Pin,
    led4_Pin,
    led5_Pin,
    led6_Pin,
    led7_Pin,
    led8_Pin,
    led9_Pin,
    led10_Pin,
    led11_Pin
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void clearAllClock();
void setNumberOnClock(int num);
void clearNumberOnClock(int num);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void clearAllClock(){
	HAL_GPIO_WritePin(led0_GPIO_Port, led0_Pin, SET);
	HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, SET);
	HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, SET);
	HAL_GPIO_WritePin(led3_GPIO_Port, led3_Pin, SET);
	HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, SET);
	HAL_GPIO_WritePin(led5_GPIO_Port, led5_Pin, SET);
	HAL_GPIO_WritePin(led6_GPIO_Port, led6_Pin, SET);
	HAL_GPIO_WritePin(led7_GPIO_Port, led7_Pin, SET);
	HAL_GPIO_WritePin(led8_GPIO_Port, led8_Pin, SET);
	HAL_GPIO_WritePin(led9_GPIO_Port, led9_Pin, SET);
	HAL_GPIO_WritePin(led10_GPIO_Port, led10_Pin, SET);
	HAL_GPIO_WritePin(led11_GPIO_Port, led11_Pin, SET);

 }
void setNumberOnClock(int num) {
    if (num >= 0 && num < 12) {
        HAL_GPIO_WritePin(LED_PORT, LED_PINS[num], RESET);
    }
}
void clearNumberOnClock(int num) {
    if (num >= 0 && num < 12) {
        HAL_GPIO_WritePin(LED_PORT, LED_PINS[num], SET);
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
  /* USER CODE BEGIN 2 */
  int sec_cnt = 0;
  int min_cnt = 50*60;
  uint32_t hour_cnt = 6*3600;
  clearAllClock();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  ++sec_cnt;
	  ++min_cnt;
	  ++hour_cnt;
	  if(sec_cnt>59) sec_cnt =0;
	  if(min_cnt>3599) min_cnt =0;
	  if(hour_cnt>43199) hour_cnt=0;
	  clearAllClock();
	  setNumberOnClock(sec_cnt/5);
	  setNumberOnClock(min_cnt/300);
	  setNumberOnClock(hour_cnt/3600);
	  HAL_Delay(200);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, led0_Pin|led1_Pin|led2_Pin|led3_Pin
                          |led4_Pin|led5_Pin|led6_Pin|led7_Pin
                          |led8_Pin|led9_Pin|led10_Pin|led11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : led0_Pin led1_Pin led2_Pin led3_Pin
                           led4_Pin led5_Pin led6_Pin led7_Pin
                           led8_Pin led9_Pin led10_Pin led11_Pin */
  GPIO_InitStruct.Pin = led0_Pin|led1_Pin|led2_Pin|led3_Pin
                          |led4_Pin|led5_Pin|led6_Pin|led7_Pin
                          |led8_Pin|led9_Pin|led10_Pin|led11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
#ifdef USE_FULL_ASSERT
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
