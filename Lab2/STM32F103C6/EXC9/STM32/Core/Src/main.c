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
#include "software_timer.h"
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
int hour = 23, minute = 59, second = 55;
int led_buffer[4] = {0, 0, 0, 0};
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
uint16_t EN_PIN[4] = {
    EN0_Pin,
    EN1_Pin,
    EN2_Pin,
    EN3_Pin,
};
void display7SEG(int num,int led){
	HAL_GPIO_WritePin(EN0_GPIO_Port, EN_PIN[0], SET);
	HAL_GPIO_WritePin(EN0_GPIO_Port, EN_PIN[1], SET);
	HAL_GPIO_WritePin(EN0_GPIO_Port, EN_PIN[2], SET);
	HAL_GPIO_WritePin(EN0_GPIO_Port, EN_PIN[3], SET);


	HAL_GPIO_WritePin(seg_a_GPIO_Port, seg_a_Pin, led7_table[num][0]);
	HAL_GPIO_WritePin(seg_b_GPIO_Port, seg_b_Pin, led7_table[num][1]);
	HAL_GPIO_WritePin(seg_c_GPIO_Port, seg_c_Pin, led7_table[num][2]);
	HAL_GPIO_WritePin(seg_g_GPIO_Port, seg_d_Pin, led7_table[num][3]);
	HAL_GPIO_WritePin(seg_f_GPIO_Port, seg_e_Pin, led7_table[num][4]);
	HAL_GPIO_WritePin(seg_e_GPIO_Port, seg_f_Pin, led7_table[num][5]);
	HAL_GPIO_WritePin(seg_d_GPIO_Port, seg_g_Pin, led7_table[num][6]);
	HAL_GPIO_WritePin(EN0_GPIO_Port, EN_PIN[led], RESET);
}



int timer0_counter = 0;
int timer0_flag = 0;
int TIMER_CYCLE = 10;

void updateClockBuffer(){
	led_buffer[0] = hour/10;
	led_buffer[1] = hour - led_buffer[0]*10;
	led_buffer[2] = minute/10;
	led_buffer[3] = minute - led_buffer[2]*10;

};
void update7SEG(int index){
    switch (index){
        case 0:
        	display7SEG(led_buffer[0],0);
            break;
        case 1:
        	display7SEG(led_buffer[1],1);
            break;
        case 2:
        	display7SEG(led_buffer[2],2);
            break;
        case 3:
        	display7SEG(led_buffer[3],3);
            break;
        default:
            break;
    }
}
uint8_t matrix_buffer[8] = {0x00,0xFC,0x0A,0x09,0x09,0x0A,0xFC,0x00}; //A
//uint8_t matrix_buffer[8] = {0x00,0x18,0x18,0x18,0x7E,0x3C,0x18,0x00};



void updateLEDMatrix(int index){
	HAL_GPIO_WritePin(GPIOA, ENM0_Pin|ENM1_Pin|ENM2_Pin|ENM3_Pin|ENM4_Pin|ENM5_Pin|ENM6_Pin|ENM7_Pin, SET);
	uint16_t matrix_pin_for_buffer0 = matrix_buffer[index];
	uint16_t matrix_pin_for_buffer1 = ~matrix_buffer[index];
	matrix_pin_for_buffer0 = matrix_pin_for_buffer0<<8;
	matrix_pin_for_buffer1 = matrix_pin_for_buffer1<<8;
	HAL_GPIO_WritePin(ROW0_GPIO_Port, matrix_pin_for_buffer0, RESET);
	HAL_GPIO_WritePin(ROW0_GPIO_Port, matrix_pin_for_buffer1, SET);

	 switch (index){
		 case 0:
			HAL_GPIO_WritePin(ENM0_GPIO_Port, ENM0_Pin, RESET);
		 break;
		 case 1:
			 HAL_GPIO_WritePin(ENM1_GPIO_Port, ENM1_Pin, RESET);
		 break;
		 case 2:
			 HAL_GPIO_WritePin(ENM2_GPIO_Port, ENM2_Pin, RESET);
		break;
		 case 3:
			 HAL_GPIO_WritePin(ENM3_GPIO_Port, ENM3_Pin, RESET);
		 break;
		 case 4:
			 HAL_GPIO_WritePin(ENM4_GPIO_Port, ENM4_Pin, RESET);
		 break;
		 case 5:
			 HAL_GPIO_WritePin(ENM5_GPIO_Port, ENM5_Pin, RESET);
		 break;
		 case 6:
			 HAL_GPIO_WritePin(ENM6_GPIO_Port, ENM6_Pin, RESET);
		 break;
		 case 7:
			 HAL_GPIO_WritePin(ENM7_GPIO_Port, ENM7_Pin, RESET);
		 break;
		  default:
		  break;
	  }
  }
void mat_tran(){
	uint8_t tmp = matrix_buffer[7];
	matrix_buffer[7] = matrix_buffer[6];
	matrix_buffer[6] = matrix_buffer[5];
	matrix_buffer[5] = matrix_buffer[4];
	matrix_buffer[4] = matrix_buffer[3];
	matrix_buffer[3] = matrix_buffer[2];
	matrix_buffer[2] = matrix_buffer[1];
	matrix_buffer[1] = matrix_buffer[0];
	matrix_buffer[0] = tmp;

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
  /* USER CODE BEGIN 2 */
  int mat_idx = 0;
  int seg_idx = 0;
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, RESET);
  HAL_GPIO_WritePin(DOT_GPIO_Port, DOT_Pin, RESET);
  updateClockBuffer();
  update7SEG(seg_idx++);
  set_timer_init(0, 100);
  set_timer_init(1,50);
  set_timer_init(2, 25);
  set_timer_init(3,10);
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Check_timer();
	  if(TimerExpired(0)){
		  set_timer(0, 100);
		  HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
		  second++;
		  if (second >= 60){
		            second = 0;
		            minute++;
		        }
	      if(minute >= 60){
		            minute = 0;
		            hour++;
		        }
	      if(hour >=24){
		            hour = 0;
		        }
		  updateClockBuffer();
	  }
	  if(TimerExpired(1)){
		  set_timer(1, 50);
		  HAL_GPIO_TogglePin(DOT_GPIO_Port, DOT_Pin);
	  }
	  if(TimerExpired(2)){
		  set_timer(2, 25);
		  update7SEG(seg_idx++);
		  if(seg_idx > 3) seg_idx = 0;

	  }
	  if(TimerExpired(3)){
		  int ms = 1;

		  updateLEDMatrix(mat_idx++);
		  if(mat_idx > 8) {
			  mat_idx = 0;
			  mat_tran();
			  ms = 25;
		  }
		  set_timer(3, ms);

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
