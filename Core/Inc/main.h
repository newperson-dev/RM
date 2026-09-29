/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define Motor1GPIOA_Pin GPIO_PIN_0
#define Motor1GPIOA_GPIO_Port GPIOC
#define Motor1GPIOB_Pin GPIO_PIN_1
#define Motor1GPIOB_GPIO_Port GPIOC
#define Motor2GPIOA_Pin GPIO_PIN_2
#define Motor2GPIOA_GPIO_Port GPIOC
#define Motor2GPIOB_Pin GPIO_PIN_3
#define Motor2GPIOB_GPIO_Port GPIOC
#define Motor4EncoderA_Pin GPIO_PIN_0
#define Motor4EncoderA_GPIO_Port GPIOA
#define Motor4EncoderB_Pin GPIO_PIN_1
#define Motor4EncoderB_GPIO_Port GPIOA
#define Motor2EncoderA_Pin GPIO_PIN_6
#define Motor2EncoderA_GPIO_Port GPIOA
#define Motor2EncoderB_Pin GPIO_PIN_7
#define Motor2EncoderB_GPIO_Port GPIOA
#define Motor4GPIOB_Pin GPIO_PIN_12
#define Motor4GPIOB_GPIO_Port GPIOB
#define Motor4GPIOA_Pin GPIO_PIN_13
#define Motor4GPIOA_GPIO_Port GPIOB
#define Motor3GPIOB_Pin GPIO_PIN_14
#define Motor3GPIOB_GPIO_Port GPIOB
#define Motor3GPIOA_Pin GPIO_PIN_15
#define Motor3GPIOA_GPIO_Port GPIOB
#define Arm4PWM_Pin GPIO_PIN_6
#define Arm4PWM_GPIO_Port GPIOC
#define Arm3PWM_Pin GPIO_PIN_7
#define Arm3PWM_GPIO_Port GPIOC
#define Arm2PWM_Pin GPIO_PIN_8
#define Arm2PWM_GPIO_Port GPIOC
#define Arm1PWM_Pin GPIO_PIN_9
#define Arm1PWM_GPIO_Port GPIOC
#define Motor1PWM_Pin GPIO_PIN_8
#define Motor1PWM_GPIO_Port GPIOA
#define Motor2PWM_Pin GPIO_PIN_9
#define Motor2PWM_GPIO_Port GPIOA
#define Motor3PWM_Pin GPIO_PIN_10
#define Motor3PWM_GPIO_Port GPIOA
#define Motor4PWM_Pin GPIO_PIN_11
#define Motor4PWM_GPIO_Port GPIOA
#define Motor1EncoderA_Pin GPIO_PIN_15
#define Motor1EncoderA_GPIO_Port GPIOA
#define Motor1EncoderB_Pin GPIO_PIN_3
#define Motor1EncoderB_GPIO_Port GPIOB
#define Motor3EncoderA_Pin GPIO_PIN_6
#define Motor3EncoderA_GPIO_Port GPIOB
#define Motor3EncoderB_Pin GPIO_PIN_7
#define Motor3EncoderB_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
