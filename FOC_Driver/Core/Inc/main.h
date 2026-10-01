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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main_cpp.hpp"
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
#define DEAD_TIME 0
#define FULL_DUTY 13600
#define ENCODER_SCALER 17000
#define VELOCITY_CONTROL 34000
#define Button_Pin GPIO_PIN_13
#define Button_GPIO_Port GPIOC
#define PULSE_Pin GPIO_PIN_0
#define PULSE_GPIO_Port GPIOA
#define COMM_ERROR_Pin GPIO_PIN_1
#define COMM_ERROR_GPIO_Port GPIOA
#define DRV_ERROR_Pin GPIO_PIN_2
#define DRV_ERROR_GPIO_Port GPIOA
#define USER_LED_Pin GPIO_PIN_3
#define USER_LED_GPIO_Port GPIOA
#define DRV8316_NSS_Pin GPIO_PIN_4
#define DRV8316_NSS_GPIO_Port GPIOC
#define DRVOFF_Pin GPIO_PIN_7
#define DRVOFF_GPIO_Port GPIOB
#define nFault_Pin GPIO_PIN_9
#define nFault_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
