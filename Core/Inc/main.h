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
#define MIDI_IN_PORTS_NUM   0x01 // Specify input ports number of your device
#define MIDI_OUT_PORTS_NUM  0x01 // Specify output ports number of your device
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define C_B_Pin GPIO_PIN_2
#define C_B_GPIO_Port GPIOA
#define CS_B_Pin GPIO_PIN_3
#define CS_B_GPIO_Port GPIOA
#define D_B_Pin GPIO_PIN_4
#define D_B_GPIO_Port GPIOA
#define DS_B_Pin GPIO_PIN_5
#define DS_B_GPIO_Port GPIOA
#define E_B_Pin GPIO_PIN_6
#define E_B_GPIO_Port GPIOA
#define F_B_Pin GPIO_PIN_7
#define F_B_GPIO_Port GPIOA
#define D_V_Pin GPIO_PIN_0
#define D_V_GPIO_Port GPIOB
#define P_V_Pin GPIO_PIN_1
#define P_V_GPIO_Port GPIOB
#define FS_B_Pin GPIO_PIN_15
#define FS_B_GPIO_Port GPIOA
#define G_B_Pin GPIO_PIN_3
#define G_B_GPIO_Port GPIOB
#define GS_B_Pin GPIO_PIN_4
#define GS_B_GPIO_Port GPIOB
#define A_B_Pin GPIO_PIN_5
#define A_B_GPIO_Port GPIOB
#define AS_B_Pin GPIO_PIN_6
#define AS_B_GPIO_Port GPIOB
#define B_B_Pin GPIO_PIN_7
#define B_B_GPIO_Port GPIOB
#define SP_B_Pin GPIO_PIN_8
#define SP_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
