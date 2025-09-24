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
#include "stdint.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIMER_SCALE 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t led7_table[10][7] =    {{0,0,0,0,0,0,1},
								{1,0,0,1,1,1,1},
								{0,0,1,0,0,1,0},
								{0,0,0,0,1,1,0},
								{1,0,0,1,1,0,0},
								{0,1,0,0,1,0,0},
								{0,1,0,0,0,0,0},
								{0,0,0,1,1,1,1},
								{0,0,0,0,0,0,0},
								{0,0,0,0,1,0,0}};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void display7SEG_1(int num);
void display7SEG_2(int num);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void display7SEG_1(int num){
	HAL_GPIO_WritePin(seg1_a_GPIO_Port, seg1_a_Pin, led7_table[num][0]);
	HAL_GPIO_WritePin(seg1_b_GPIO_Port, seg1_b_Pin, led7_table[num][1]);
	HAL_GPIO_WritePin(seg1_c_GPIO_Port, seg1_c_Pin, led7_table[num][2]);
	HAL_GPIO_WritePin(seg1_g_GPIO_Port, seg1_g_Pin, led7_table[num][3]);
	HAL_GPIO_WritePin(seg1_f_GPIO_Port, seg1_f_Pin, led7_table[num][4]);
	HAL_GPIO_WritePin(seg1_e_GPIO_Port, seg1_e_Pin, led7_table[num][5]);
	HAL_GPIO_WritePin(seg1_d_GPIO_Port, seg1_d_Pin, led7_table[num][6]);
}
void display7SEG_2(int num){
	HAL_GPIO_WritePin(seg2_a_GPIO_Port, seg2_a_Pin, led7_table[num][0]);
	HAL_GPIO_WritePin(seg2_b_GPIO_Port, seg2_b_Pin, led7_table[num][1]);
	HAL_GPIO_WritePin(seg2_c_GPIO_Port, seg2_c_Pin, led7_table[num][2]);
	HAL_GPIO_WritePin(seg2_g_GPIO_Port, seg2_d_Pin, led7_table[num][3]);
	HAL_GPIO_WritePin(seg2_f_GPIO_Port, seg2_e_Pin, led7_table[num][4]);
	HAL_GPIO_WritePin(seg2_e_GPIO_Port, seg2_f_Pin, led7_table[num][5]);
	HAL_GPIO_WritePin(seg2_d_GPIO_Port, seg2_g_Pin, led7_table[num][6]);
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
  uint16_t led_cnt = 0;
  uint16_t Led7_seg1_cnt = 290;
  uint16_t Led7_seg2_cnt = 116;
  HAL_GPIO_WritePin(LED_Red_1_GPIO_Port, LED_Red_1_Pin, RESET);
  HAL_GPIO_WritePin(LED_Yellow_1_GPIO_Port, LED_Yellow_1_Pin, SET);
  HAL_GPIO_WritePin(LED_Green_1_GPIO_Port, LED_Green_1_Pin, SET);
  HAL_GPIO_WritePin(LED_Red_2_GPIO_Port, LED_Red_2_Pin, SET);
  HAL_GPIO_WritePin(LED_Yellow_2_GPIO_Port, LED_Yellow_2_Pin, SET);
  HAL_GPIO_WritePin(LED_Green_2_GPIO_Port, LED_Green_2_Pin, RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  /*LED BEHAVIOR CODE BEGIN*/
      // LED 1
	  if(led_cnt==290){
		  HAL_GPIO_WritePin(LED_Red_1_GPIO_Port, LED_Red_1_Pin, SET);
		  HAL_GPIO_WritePin(LED_Yellow_1_GPIO_Port, LED_Yellow_1_Pin, SET);
		  HAL_GPIO_WritePin(LED_Green_1_GPIO_Port, LED_Green_1_Pin, RESET);
		  Led7_seg1_cnt = 116;
	  }
	  else if(led_cnt==406){
		  HAL_GPIO_WritePin(LED_Red_1_GPIO_Port, LED_Red_1_Pin, SET);
		  HAL_GPIO_WritePin(LED_Yellow_1_GPIO_Port, LED_Yellow_1_Pin, RESET);
		  HAL_GPIO_WritePin(LED_Green_1_GPIO_Port, LED_Green_1_Pin, SET);
		  Led7_seg1_cnt = 174;
	  }
	  else if(led_cnt==580){
		  HAL_GPIO_WritePin(LED_Red_1_GPIO_Port, LED_Red_1_Pin, RESET);
		  HAL_GPIO_WritePin(LED_Yellow_1_GPIO_Port, LED_Yellow_1_Pin, SET);
		  HAL_GPIO_WritePin(LED_Green_1_GPIO_Port, LED_Green_1_Pin, SET);
		  Led7_seg1_cnt = 290;

	  }
	  //LED 2
	  if(led_cnt==116){

		  HAL_GPIO_WritePin(LED_Red_2_GPIO_Port, LED_Red_2_Pin, SET);
		  HAL_GPIO_WritePin(LED_Yellow_2_GPIO_Port, LED_Yellow_2_Pin, RESET);
		  HAL_GPIO_WritePin(LED_Green_2_GPIO_Port, LED_Green_2_Pin, SET);
		  Led7_seg2_cnt = 174;
	  }
	  else if(led_cnt==290){
		  HAL_GPIO_WritePin(LED_Red_2_GPIO_Port, LED_Red_2_Pin, RESET);
		  HAL_GPIO_WritePin(LED_Yellow_2_GPIO_Port, LED_Yellow_2_Pin, SET);
		  HAL_GPIO_WritePin(LED_Green_2_GPIO_Port, LED_Green_2_Pin, SET);
		  Led7_seg2_cnt = 290;
	  }
	  else if(led_cnt==580){
		  HAL_GPIO_WritePin(LED_Red_2_GPIO_Port, LED_Red_2_Pin, SET);
		  HAL_GPIO_WritePin(LED_Yellow_2_GPIO_Port, LED_Yellow_2_Pin, SET);
		  HAL_GPIO_WritePin(LED_Green_2_GPIO_Port, LED_Green_2_Pin, RESET);
		  Led7_seg2_cnt = 116;

	  }
	  if(led_cnt>=580) led_cnt = 0;

	  display7SEG_1((Led7_seg1_cnt/58)+1);
	  display7SEG_2((Led7_seg2_cnt/58)+1);


	  ++led_cnt;
	  --Led7_seg1_cnt;
	  --Led7_seg2_cnt;
	  /*LED BEHAVIOR CODE END*/

	  HAL_Delay(10);
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Red_2_Pin|LED_Yellow_2_Pin|LED_Green_2_Pin|LED_Red_1_Pin
                          |LED_Yellow_1_Pin|LED_Green_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, seg1_a_Pin|seg1_b_Pin|seg1_c_Pin|seg2_d_Pin
                          |seg2_e_Pin|seg2_f_Pin|seg2_g_Pin|seg1_g_Pin
                          |seg1_f_Pin|seg1_e_Pin|seg1_d_Pin|seg2_a_Pin
                          |seg2_b_Pin|seg2_c_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Red_2_Pin LED_Yellow_2_Pin LED_Green_2_Pin LED_Red_1_Pin
                           LED_Yellow_1_Pin LED_Green_1_Pin */
  GPIO_InitStruct.Pin = LED_Red_2_Pin|LED_Yellow_2_Pin|LED_Green_2_Pin|LED_Red_1_Pin
                          |LED_Yellow_1_Pin|LED_Green_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : seg1_a_Pin seg1_b_Pin seg1_c_Pin seg2_d_Pin
                           seg2_e_Pin seg2_f_Pin seg2_g_Pin seg1_g_Pin
                           seg1_f_Pin seg1_e_Pin seg1_d_Pin seg2_a_Pin
                           seg2_b_Pin seg2_c_Pin */
  GPIO_InitStruct.Pin = seg1_a_Pin|seg1_b_Pin|seg1_c_Pin|seg2_d_Pin
                          |seg2_e_Pin|seg2_f_Pin|seg2_g_Pin|seg1_g_Pin
                          |seg1_f_Pin|seg1_e_Pin|seg1_d_Pin|seg2_a_Pin
                          |seg2_b_Pin|seg2_c_Pin;
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
