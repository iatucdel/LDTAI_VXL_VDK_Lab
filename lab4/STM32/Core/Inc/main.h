/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define L1_RED_Pin GPIO_PIN_1
#define L1_RED_GPIO_Port GPIOA
#define L1_YELLOW_Pin GPIO_PIN_2
#define L1_YELLOW_GPIO_Port GPIOA
#define L1_GREEN_Pin GPIO_PIN_3
#define L1_GREEN_GPIO_Port GPIOA
#define BTN0_Pin GPIO_PIN_4
#define BTN0_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_5
#define LED_RED_GPIO_Port GPIOA
#define L2_RED_Pin GPIO_PIN_6
#define L2_RED_GPIO_Port GPIOA
#define L2_YELLOW_Pin GPIO_PIN_7
#define L2_YELLOW_GPIO_Port GPIOA
#define L1_A_Pin GPIO_PIN_0
#define L1_A_GPIO_Port GPIOB
#define L1_B_Pin GPIO_PIN_1
#define L1_B_GPIO_Port GPIOB
#define L1_C_Pin GPIO_PIN_2
#define L1_C_GPIO_Port GPIOB
#define L2_B_Pin GPIO_PIN_10
#define L2_B_GPIO_Port GPIOB
#define L2_C_Pin GPIO_PIN_11
#define L2_C_GPIO_Port GPIOB
#define L2_D_Pin GPIO_PIN_12
#define L2_D_GPIO_Port GPIOB
#define L2_E_Pin GPIO_PIN_13
#define L2_E_GPIO_Port GPIOB
#define L2_F_Pin GPIO_PIN_14
#define L2_F_GPIO_Port GPIOB
#define L2_G_Pin GPIO_PIN_15
#define L2_G_GPIO_Port GPIOB
#define L2_GREEN_Pin GPIO_PIN_8
#define L2_GREEN_GPIO_Port GPIOA
#define L2_CON1_Pin GPIO_PIN_9
#define L2_CON1_GPIO_Port GPIOA
#define L2_CON2_Pin GPIO_PIN_10
#define L2_CON2_GPIO_Port GPIOA
#define BTN1_Pin GPIO_PIN_12
#define BTN1_GPIO_Port GPIOA
#define BTN2_Pin GPIO_PIN_15
#define BTN2_GPIO_Port GPIOA
#define L1_D_Pin GPIO_PIN_3
#define L1_D_GPIO_Port GPIOB
#define L1_E_Pin GPIO_PIN_4
#define L1_E_GPIO_Port GPIOB
#define L1_F_Pin GPIO_PIN_5
#define L1_F_GPIO_Port GPIOB
#define L1_G_Pin GPIO_PIN_6
#define L1_G_GPIO_Port GPIOB
#define L1_CON1_Pin GPIO_PIN_7
#define L1_CON1_GPIO_Port GPIOB
#define L1_CON2_Pin GPIO_PIN_8
#define L1_CON2_GPIO_Port GPIOB
#define L2_A_Pin GPIO_PIN_9
#define L2_A_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
