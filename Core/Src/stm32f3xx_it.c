/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "liquidcrystal_i2c.h"
#include <stdbool.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define C		(0x24)
#define CS  	(0x25)
#define D		(0x26)
#define DS  	(0x27)
#define E		(0x28)
#define F   	(0x29)
#define FS  	(0x2A)
#define G		(0x2B)
#define GS  	(0x2C)
#define A		(0x2D)
#define AS  	(0x2E)
#define B   	(0x2F)
#define STOP	(0x00)
#define PLAY	(0x7F)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t pin_num = 0;
bool play_toggled = false;
bool pressed = false;
char msg[10];
uint8_t message[6] = { 0x19, 0x91, C, 0x40, C, 0x00 };
uint8_t on_msg[4] = { 0x19, 0x91, C, 0x40};
uint8_t off_msg[4] = { 0x19, 0x91, C, 0x00 };
uint8_t PS_message[4] = { 0x1B, 0xB1, 0x15, STOP };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
/* USER CODE BEGIN EV */
extern UART_HandleTypeDef huart2;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	if (!HAL_GPIO_ReadPin (CB_GPIO_Port, CB_Pin)) {
		if (!pressed) {
			pressed = true;
			message[2] = C;
			message[4] = C;
			HAL_UART_Transmit (&huart2, (uint8_t *) message, 6, 100);
		}
	} else if (!HAL_GPIO_ReadPin (CSB_GPIO_Port, CSB_Pin)) {
		if (!pressed) {
			pressed = true;
			message[2] = CS;
			message[4] = CS;
			HAL_UART_Transmit (&huart2, (uint8_t *) message, 6, 100);
		}
	} else if (!HAL_GPIO_ReadPin (DB_GPIO_Port, DB_Pin)) {
		if (!pressed) {
			pressed = true;
			message[2] = D;
			message[4] = D;
			HAL_UART_Transmit (&huart2, (uint8_t *) message, 6, 100);
		}
	} else if (!HAL_GPIO_ReadPin (DSB_GPIO_Port, DSB_Pin)) {
		if (!pressed) {
			pressed = true;
			message[2] = DS;
			message[4] = DS;
			HAL_UART_Transmit (&huart2, (uint8_t *) message, 6, 100);
		}
	} else if (!HAL_GPIO_ReadPin (EB_GPIO_Port, EB_Pin)) {
		if (!pressed) {
			pressed = true;
			message[2] = E;
			message[4] = E;
			HAL_UART_Transmit (&huart2, (uint8_t *) message, 6, 100);
		}
	} else if (!HAL_GPIO_ReadPin (FB_GPIO_Port, FB_Pin)) {
		if (!pressed) {
			pressed = true;
			message[2] = F;
			message[4] = F;
			HAL_UART_Transmit (&huart2, (uint8_t *) message, 6, 100);
		}
	} else if (!HAL_GPIO_ReadPin (FSB_GPIO_Port, FSB_Pin)) {
		if (!pressed) {
			pressed = true;
			message[2] = FS;
			message[4] = FS;
			HAL_UART_Transmit (&huart2, (uint8_t *) message, 6, 100);
		}
	} else if (!HAL_GPIO_ReadPin (GB_GPIO_Port, GB_Pin)) {
		if (!pressed) {
			pressed = true;
			message[2] = G;
			message[4] = G;
			HAL_UART_Transmit (&huart2, (uint8_t *) message, 6, 100);
		}
	} else if (!HAL_GPIO_ReadPin (GSB_GPIO_Port, GSB_Pin)) {
		if (!pressed) {
			pressed = true;
			message[2] = GS;
			message[4] = GS;
			HAL_UART_Transmit (&huart2, (uint8_t *) message, 6, 100);
		}
	} else if (!HAL_GPIO_ReadPin (AB_GPIO_Port, AB_Pin)) {
		if (!pressed) {
			pressed = true;
			message[2] = A;
			message[4] = A;
			HAL_UART_Transmit (&huart2, (uint8_t *) message, 6, 100);
		}
	} else if (!HAL_GPIO_ReadPin (ASB_GPIO_Port, ASB_Pin)) {
		if (!pressed) {
			pressed = true;
			message[2] = AS;
			message[4] = AS;
			HAL_UART_Transmit (&huart2, (uint8_t *) message, 6, 100);
		}
	} else if (!HAL_GPIO_ReadPin (BB_GPIO_Port, BB_Pin)) {
		if (!pressed) {
			pressed = true;
			message[2] = B;
			message[4] = B;
			HAL_UART_Transmit (&huart2, (uint8_t *) message, 6, 100);
		}
	} else if (!HAL_GPIO_ReadPin (SPB_GPIO_Port, SPB_Pin)) {
		if (!pressed) {
			pressed = true;
			play_toggled = !play_toggled;
			PS_message[3] = play_toggled ? PLAY : STOP;
			HAL_UART_Transmit (&huart2, (uint8_t *) PS_message, 4, 100);
//			sprintf (msg, "%s\r\n", play_toggled ? "PLAY" : "STOP");
//			HAL_UART_Transmit (&huart2, (uint8_t *) msg, 6, 100);
		}
	}
	else {
		pressed = false;
	}
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
