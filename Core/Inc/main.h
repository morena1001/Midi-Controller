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
#define GB_Pin GPIO_PIN_7
#define GB_GPIO_Port GPIOA
#define CB_Pin GPIO_PIN_0
#define CB_GPIO_Port GPIOB
#define CSB_Pin GPIO_PIN_1
#define CSB_GPIO_Port GPIOB
#define DB_Pin GPIO_PIN_2
#define DB_GPIO_Port GPIOB
#define ASB_Pin GPIO_PIN_10
#define ASB_GPIO_Port GPIOB
#define BB_Pin GPIO_PIN_11
#define BB_GPIO_Port GPIOB
#define SPB_Pin GPIO_PIN_12
#define SPB_GPIO_Port GPIOB
#define DSB_Pin GPIO_PIN_3
#define DSB_GPIO_Port GPIOB
#define EB_Pin GPIO_PIN_4
#define EB_GPIO_Port GPIOB
#define FB_Pin GPIO_PIN_5
#define FB_GPIO_Port GPIOB
#define FSB_Pin GPIO_PIN_6
#define FSB_GPIO_Port GPIOB
#define GSB_Pin GPIO_PIN_8
#define GSB_GPIO_Port GPIOB
#define AB_Pin GPIO_PIN_9
#define AB_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
