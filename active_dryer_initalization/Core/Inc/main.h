/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#define Red_LED_PWM_OUT_Pin GPIO_PIN_6
#define Red_LED_PWM_OUT_GPIO_Port GPIOF
#define Green_LED_PWM_OUT_Pin GPIO_PIN_7
#define Green_LED_PWM_OUT_GPIO_Port GPIOF
#define Blue_LED_PWM_OUT_Pin GPIO_PIN_8
#define Blue_LED_PWM_OUT_GPIO_Port GPIOF
#define sysINIT_Analog_IN_Pin GPIO_PIN_6
#define sysINIT_Analog_IN_GPIO_Port GPIOA
#define Suction_FAN_PWM_OUT_Pin GPIO_PIN_0
#define Suction_FAN_PWM_OUT_GPIO_Port GPIOB
#define Exhaust_FAN_PWM_OUT_Pin GPIO_PIN_1
#define Exhaust_FAN_PWM_OUT_GPIO_Port GPIOB
#define powerLED_ENable_OUT_Pin GPIO_PIN_12
#define powerLED_ENable_OUT_GPIO_Port GPIOB
#define RELAY_ENable_OUT_Pin GPIO_PIN_13
#define RELAY_ENable_OUT_GPIO_Port GPIOB
#define Container_ENable_IN_Pin GPIO_PIN_13
#define Container_ENable_IN_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
