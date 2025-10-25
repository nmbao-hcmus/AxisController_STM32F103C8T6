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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define JoyX_Pin GPIO_PIN_0
#define JoyX_GPIO_Port GPIOA
#define JoyY_Pin GPIO_PIN_1
#define JoyY_GPIO_Port GPIOA
#define HOME_BUTTON_Pin GPIO_PIN_2
#define HOME_BUTTON_GPIO_Port GPIOA
#define SWITCH_BUTTON_Pin GPIO_PIN_3
#define SWITCH_BUTTON_GPIO_Port GPIOA
#define L_BUTTON_Pin GPIO_PIN_4
#define L_BUTTON_GPIO_Port GPIOA
#define R_BUTTON_Pin GPIO_PIN_5
#define R_BUTTON_GPIO_Port GPIOA
#define ZDOWN_Pin GPIO_PIN_6
#define ZDOWN_GPIO_Port GPIOA
#define ZUP_Pin GPIO_PIN_7
#define ZUP_GPIO_Port GPIOA
#define D_BUTTON_Pin GPIO_PIN_0
#define D_BUTTON_GPIO_Port GPIOB
#define C_BUTTON_Pin GPIO_PIN_1
#define C_BUTTON_GPIO_Port GPIOB
#define B_BUTTON_Pin GPIO_PIN_10
#define B_BUTTON_GPIO_Port GPIOB
#define A_BUTTON_Pin GPIO_PIN_11
#define A_BUTTON_GPIO_Port GPIOB
#define EndY_Pin GPIO_PIN_11
#define EndY_GPIO_Port GPIOA
#define EndX_Pin GPIO_PIN_12
#define EndX_GPIO_Port GPIOA
#define TIM_Y_Pin GPIO_PIN_15
#define TIM_Y_GPIO_Port GPIOA
#define TIM_X_Pin GPIO_PIN_3
#define TIM_X_GPIO_Port GPIOB
#define TIM_Z_Pin GPIO_PIN_4
#define TIM_Z_GPIO_Port GPIOB
#define DirX_Pin GPIO_PIN_5
#define DirX_GPIO_Port GPIOB
#define DirY_Pin GPIO_PIN_6
#define DirY_GPIO_Port GPIOB
#define DirZ_Pin GPIO_PIN_7
#define DirZ_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
