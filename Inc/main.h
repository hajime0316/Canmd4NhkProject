/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

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
#define PWM_PERIOD 240
#define PWM_DUTY_ZERO 120
#define PWM_DUTY_MAX 96
#define MAX_ENCODER_COUNT 2000
#define SW_ENC1_Pin GPIO_PIN_1
#define SW_ENC1_GPIO_Port GPIOF
#define SW_ENC1_EXTI_IRQn EXTI1_IRQn
#define LED_ENC1_Pin GPIO_PIN_2
#define LED_ENC1_GPIO_Port GPIOA
#define DIP_SW_1_Pin GPIO_PIN_8
#define DIP_SW_1_GPIO_Port GPIOE
#define DIP_SW_2_Pin GPIO_PIN_9
#define DIP_SW_2_GPIO_Port GPIOE
#define DIP_SW_3_Pin GPIO_PIN_14
#define DIP_SW_3_GPIO_Port GPIOB
#define DIP_SW_4_Pin GPIO_PIN_15
#define DIP_SW_4_GPIO_Port GPIOB
#define LED_ENC2_Pin GPIO_PIN_8
#define LED_ENC2_GPIO_Port GPIOD
#define SW_ENC2_Pin GPIO_PIN_8
#define SW_ENC2_GPIO_Port GPIOA
#define SW_ENC2_EXTI_IRQn EXTI9_5_IRQn
#define LED_GP_Pin GPIO_PIN_7
#define LED_GP_GPIO_Port GPIOF
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
