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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "global.h"
#include "inout_driver.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, L1_RED_Pin| L1_YELLOW_Pin| L1_GREEN_Pin | L2_RED_Pin| L2_YELLOW_Pin| L2_GREEN_Pin, SET);
  fsm_state = FSM_INIT;
  set_timer(RED_LED_TIM, 1000);
  set_timer(FSM_TIM, FSM_period);
  set_timer(TRAFIC_LIGHT_TIM,130);
  set_timer(SEG7_1_TIM, 250);
  set_timer(SEG7_2_TIM, 250);
  set_timer(BUTTON_TIM, 10);
  set_timer(LONG_PRESS_BUT1_TIM, 11);
  set_timer(LONG_PRESS_BUT2_TIM, 12);
  set_timer(LONG_PRESS_BUT3_TIM, 13);
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	Check_timer();
	if(TimerExpired(RED_LED_TIM)){
		set_timer(RED_LED_TIM, 1000);

		HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
		if((fsm_state == FSM_NORMAL || fsm_state == FSM_MANUAL_YR)&&(seg1_val>1)){
			--seg1_val;
		}
		if((fsm_state == FSM_NORMAL || fsm_state == FSM_MANUAL_RY)&&(seg2_val>1)){
			--seg2_val;
		}
	}
	if(TimerExpired(FSM_TIM)){
		main_fsm();
		set_timer(FSM_TIM, FSM_period);
	}
	if(TimerExpired(TRAFIC_LIGHT_TIM)){
		led_fsm();
	}
	if(TimerExpired(SEG7_1_TIM)){
		  set_timer(SEG7_1_TIM, 123);
		  display_seg1();
	}
	if(TimerExpired(SEG7_2_TIM)){
		  set_timer(SEG7_2_TIM, 123);
		  display_seg2();
	}
	if(TimerExpired(BUTTON_TIM)){
		set_timer(BUTTON_TIM, 10);
		button_reading();
	}
	if(TimerExpired(LONG_PRESS_BUT1_TIM)){
		long_pressed_buf[0] = is_button_pressed_500ms(0);
		if(long_pressed_buf[0]){
			  set_timer(LONG_PRESS_BUT1_TIM, 250);
		}
		else set_timer(LONG_PRESS_BUT1_TIM, 10);
	}
	if(TimerExpired(LONG_PRESS_BUT2_TIM)){
		long_pressed_buf[1] = is_button_pressed_500ms(1);
		if(long_pressed_buf[1]){
			  set_timer(LONG_PRESS_BUT2_TIM, 250);
		}
		else set_timer(LONG_PRESS_BUT2_TIM, 10);
	}
	if(TimerExpired(LONG_PRESS_BUT3_TIM)){
		long_pressed_buf[2] = is_button_pressed_500ms(2);
		if(long_pressed_buf[2]){
			  set_timer(LONG_PRESS_BUT3_TIM, 250);
		}
		else set_timer(LONG_PRESS_BUT3_TIM, 10);
	}
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
