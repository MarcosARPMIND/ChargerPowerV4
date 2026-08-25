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
#include "stm32c0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "app_config.h"

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
extern volatile uint8_t RCD_Fault;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define general_LED_Pin GPIO_PIN_2
#define general_LED_GPIO_Port GPIOB
#define cable_lock_Pin GPIO_PIN_10
#define cable_lock_GPIO_Port GPIOB
#define button_2_ext_Pin GPIO_PIN_11
#define button_2_ext_GPIO_Port GPIOB
#define button_1_ext_Pin GPIO_PIN_15
#define button_1_ext_GPIO_Port GPIOB
#define h_bridge_1_Pin GPIO_PIN_6
#define h_bridge_1_GPIO_Port GPIOC
#define h_bridge_2_Pin GPIO_PIN_12
#define h_bridge_2_GPIO_Port GPIOA
#define os_temp_Pin GPIO_PIN_1
#define os_temp_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
