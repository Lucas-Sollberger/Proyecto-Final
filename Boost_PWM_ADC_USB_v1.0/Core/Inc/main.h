/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

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
#define Rele_1_Pin GPIO_PIN_4
#define Rele_1_GPIO_Port GPIOE
#define Rele_2_Pin GPIO_PIN_5
#define Rele_2_GPIO_Port GPIOE
#define Rele_3_Pin GPIO_PIN_6
#define Rele_3_GPIO_Port GPIOE
#define I_OUT_Pin GPIO_PIN_1
#define I_OUT_GPIO_Port GPIOA
#define V_LEM_Pin GPIO_PIN_2
#define V_LEM_GPIO_Port GPIOA
#define V_i30_Pin GPIO_PIN_3
#define V_i30_GPIO_Port GPIOA
#define V_OUT_Pin GPIO_PIN_4
#define V_OUT_GPIO_Port GPIOA
#define I_IN_Pin GPIO_PIN_5
#define I_IN_GPIO_Port GPIOA
#define V_EXT_Pin GPIO_PIN_6
#define V_EXT_GPIO_Port GPIOA
#define Pote_Pin GPIO_PIN_7
#define Pote_GPIO_Port GPIOA
#define V_IN_Pin GPIO_PIN_4
#define V_IN_GPIO_Port GPIOC
#define LED_VERDE_Pin GPIO_PIN_12
#define LED_VERDE_GPIO_Port GPIOD
#define LED_ROJO_Pin GPIO_PIN_14
#define LED_ROJO_GPIO_Port GPIOD
#define LED_AZUL_Pin GPIO_PIN_15
#define LED_AZUL_GPIO_Port GPIOD
#define TRGO_Pin GPIO_PIN_8
#define TRGO_GPIO_Port GPIOC
#define PWM_Pin GPIO_PIN_6
#define PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
