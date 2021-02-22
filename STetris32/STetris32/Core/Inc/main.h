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
#define Test 255
#define LED_BUILTIN_Pin GPIO_PIN_13
#define LED_BUILTIN_GPIO_Port GPIOC
#define BTN_RST_Pin GPIO_PIN_15
#define BTN_RST_GPIO_Port GPIOC
#define BTN_RST_EXTI_IRQn EXTI15_10_IRQn
#define BTN_UP_Pin GPIO_PIN_0
#define BTN_UP_GPIO_Port GPIOA
#define BTN_UP_EXTI_IRQn EXTI0_IRQn
#define BTN_DOWN_Pin GPIO_PIN_1
#define BTN_DOWN_GPIO_Port GPIOA
#define BTN_DOWN_EXTI_IRQn EXTI1_IRQn
#define BTN_LEFT_Pin GPIO_PIN_2
#define BTN_LEFT_GPIO_Port GPIOA
#define BTN_LEFT_EXTI_IRQn EXTI2_IRQn
#define BTN_RIGHT_Pin GPIO_PIN_3
#define BTN_RIGHT_GPIO_Port GPIOA
#define BTN_RIGHT_EXTI_IRQn EXTI3_IRQn
#define BTN_CENTER_Pin GPIO_PIN_4
#define BTN_CENTER_GPIO_Port GPIOA
#define BTN_CENTER_EXTI_IRQn EXTI4_IRQn
#define SPI1_CS_Pin GPIO_PIN_6
#define SPI1_CS_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
