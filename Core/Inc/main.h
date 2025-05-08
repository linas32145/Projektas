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
#include "stm32l0xx_hal.h"

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
#define G_seg_Pin GPIO_PIN_0
#define G_seg_GPIO_Port GPIOC
#define D_seg_Pin GPIO_PIN_1
#define D_seg_GPIO_Port GPIOC
#define B_seg_Pin GPIO_PIN_4
#define B_seg_GPIO_Port GPIOC
#define F_seg_Pin GPIO_PIN_5
#define F_seg_GPIO_Port GPIOC
#define Button1_Pin GPIO_PIN_0
#define Button1_GPIO_Port GPIOB
#define Button1_EXTI_IRQn EXTI0_1_IRQn
#define A_seg_Pin GPIO_PIN_6
#define A_seg_GPIO_Port GPIOC
#define DP_seg_Pin GPIO_PIN_7
#define DP_seg_GPIO_Port GPIOC
#define COM4_Pin GPIO_PIN_8
#define COM4_GPIO_Port GPIOC
#define COM3_Pin GPIO_PIN_9
#define COM3_GPIO_Port GPIOC
#define COM2_Pin GPIO_PIN_10
#define COM2_GPIO_Port GPIOC
#define COM1_Pin GPIO_PIN_11
#define COM1_GPIO_Port GPIOC
#define Button2_Pin GPIO_PIN_5
#define Button2_GPIO_Port GPIOB
#define Button2_EXTI_IRQn EXTI4_15_IRQn
#define LD4_Pin GPIO_PIN_6
#define LD4_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_7
#define LD3_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_8
#define LD2_GPIO_Port GPIOB
#define LD1_Pin GPIO_PIN_9
#define LD1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
