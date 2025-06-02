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
#include "usbd_midi.h"
#include <stdbool.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define C		(0x30)
#define CS  	(0x31)
#define D		(0x32)
#define DS  	(0x33)
#define E		(0x34)
#define F   	(0x35)
#define FS  	(0x36)
#define G		(0x37)
#define GS  	(0x38)
#define A		(0x39)
#define AS  	(0x3A)
#define B   	(0x3B)
#define STOP	(0x00)
#define PLAY	(0x7F)

#define END 	(0x00)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
//uint8_t pin_num = 0;
//bool play_toggled = false;
//bool pressed = false;
//char msg[10];
//uint8_t message[6] = { 0x19, 0x91, C, 0x40, C, 0x00 };
//uint8_t on_msg[4] = { 0x19, 0x91, C, 0x40};
//uint8_t off_msg[4] = { 0x19, 0x91, C, 0x00 };
//uint8_t PS_message[4] = { 0x1B, 0xB1, 0x15, STOP };

uint8_t note_message [6] = { 0x09, 0x90, 0x00, 0x40, 0x00, 0x00 };
uint8_t on_message[4] = { 0x09, 0x90, 0x00, 0x40 };
uint8_t off_message[4] = { 0x08, 0x80, 0x00, 0x40 };
uint8_t PS_message[4] = { 0x0B, 0xB0, 0x15, STOP };
bool control_toggled = false;
bool play_toggled = false;
bool pressed = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void Note_Change (uint8_t value);
void Toggle_Control ();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;
extern TIM_HandleTypeDef htim2;
/* USER CODE BEGIN EV */
extern USBD_HandleTypeDef hUsbDeviceFS;
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
  * @brief This function handles CAN RX0 and USB low priority interrupts.
  */
void USB_LP_CAN_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN_RX0_IRQn 0 */

  /* USER CODE END USB_LP_CAN_RX0_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_LP_CAN_RX0_IRQn 1 */

  /* USER CODE END USB_LP_CAN_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	if (!HAL_GPIO_ReadPin (CB_GPIO_Port, CB_Pin)) {
		if (!pressed) {
			pressed = true;

			Note_Change (C);

//			on_message [2] = C;
//			off_message [2] = C;
//			while (USBD_MIDI_GetState (&hUsbDeviceFS) != MIDI_IDLE) {}
//			USBD_MIDI_SendPackets (&hUsbDeviceFS, on_message, 4);
		}
	} else if (!HAL_GPIO_ReadPin (CSB_GPIO_Port, CSB_Pin)) {
		if (!pressed) {
			pressed = true;

			Note_Change (CS);

//			on_message [2] = CS;
//			off_message [2] = CS;
//			while (USBD_MIDI_GetState (&hUsbDeviceFS) != MIDI_IDLE) {}
//			USBD_MIDI_SendPackets (&hUsbDeviceFS, on_message, 4);
		}
	} else if (!HAL_GPIO_ReadPin (DB_GPIO_Port, DB_Pin)) {
		if (!pressed) {
			pressed = true;

			Note_Change (D);

//			on_message [2] = D;
//			off_message [2] = D;
//			while (USBD_MIDI_GetState (&hUsbDeviceFS) != MIDI_IDLE) {}
//			USBD_MIDI_SendPackets (&hUsbDeviceFS, on_message, 4);
		}
	} else if (!HAL_GPIO_ReadPin (DSB_GPIO_Port, DSB_Pin)) {
		if (!pressed) {
			pressed = true;

			Note_Change (DS);

//			on_message [2] = DS;
//			off_message [2] = DS;
//			while (USBD_MIDI_GetState (&hUsbDeviceFS) != MIDI_IDLE) {}
//			USBD_MIDI_SendPackets (&hUsbDeviceFS, on_message, 4);
		}
	} else if (!HAL_GPIO_ReadPin (EB_GPIO_Port, EB_Pin)) {
		if (!pressed) {
			pressed = true;

			Note_Change (E);

//			on_message [2] = E;
//			off_message [2] = E;
//			while (USBD_MIDI_GetState (&hUsbDeviceFS) != MIDI_IDLE) {}
//			USBD_MIDI_SendPackets (&hUsbDeviceFS, on_message, 4);
		}
	} else if (!HAL_GPIO_ReadPin (FB_GPIO_Port, FB_Pin)) {
		if (!pressed) {
			pressed = true;

			Note_Change (F);

//			on_message [2] = F;
//			off_message [2] = F;
//			while (USBD_MIDI_GetState (&hUsbDeviceFS) != MIDI_IDLE) {}
//			USBD_MIDI_SendPackets (&hUsbDeviceFS, on_message, 4);
		}
	} else if (!HAL_GPIO_ReadPin (FSB_GPIO_Port, FSB_Pin)) {
		if (!pressed) {
			pressed = true;

//			on_message [2] = FS;
//			off_message [2] = FS;
//			while (USBD_MIDI_GetState (&hUsbDeviceFS) != MIDI_IDLE) {}
//			USBD_MIDI_SendPackets (&hUsbDeviceFS, on_message, 4);
		}
	} else if (!HAL_GPIO_ReadPin (GB_GPIO_Port, GB_Pin)) {
		if (!pressed) {
			pressed = true;

			Note_Change (G);

//			on_message [2] = G;
//			off_message [2] = G;
//			while (USBD_MIDI_GetState (&hUsbDeviceFS) != MIDI_IDLE) {}
//			USBD_MIDI_SendPackets (&hUsbDeviceFS, on_message, 4);
		}
	} else if (!HAL_GPIO_ReadPin (GSB_GPIO_Port, GSB_Pin)) {
		if (!pressed) {
			pressed = true;

			Note_Change (GS);

//			on_message [2] = GS;
//			off_message [2] = GS;
//			while (USBD_MIDI_GetState (&hUsbDeviceFS) != MIDI_IDLE) {}
//			USBD_MIDI_SendPackets (&hUsbDeviceFS, on_message, 4);
		}
	} else if (!HAL_GPIO_ReadPin (AB_GPIO_Port, AB_Pin)) {
		if (!pressed) {
			pressed = true;

			Note_Change (A);

//			on_message [2] = A;
//			off_message [2] = A;
//			while (USBD_MIDI_GetState (&hUsbDeviceFS) != MIDI_IDLE) {}
//			USBD_MIDI_SendPackets (&hUsbDeviceFS, on_message, 4);
		}
	} else if (!HAL_GPIO_ReadPin (ASB_GPIO_Port, ASB_Pin)) {
		if (!pressed) {
			pressed = true;

			Note_Change (AS);

//			on_message [2] = AS;
//			off_message [2] = AS;
//			while (USBD_MIDI_GetState (&hUsbDeviceFS) != MIDI_IDLE) {}
//			USBD_MIDI_SendPackets (&hUsbDeviceFS, on_message, 4);
		}
	} else if (!HAL_GPIO_ReadPin (BB_GPIO_Port, BB_Pin)) {
		if (!pressed) {
			pressed = true;

			Note_Change (B);

//			on_message [2] = B;
//			off_message [2] = B;
//			while (USBD_MIDI_GetState (&hUsbDeviceFS) != MIDI_IDLE) {}
//			USBD_MIDI_SendPackets (&hUsbDeviceFS, on_message, 4);
		}
	} else if (!HAL_GPIO_ReadPin (SPB_GPIO_Port, SPB_Pin)) {
		if (!pressed) {
			pressed = true;
			control_toggled = true;

			Toggle_Control ();

//			play_toggled = !play_toggled;
//			PS_message[3] = play_toggled ? PLAY : STOP;
//			while (USBD_MIDI_GetState (&hUsbDeviceFS) != MIDI_IDLE) {}
//			USBD_MIDI_SendPackets (&hUsbDeviceFS, PS_message, 4);
		}
	} else {
		if (pressed && !control_toggled) {
			Note_Change (END);
//			while (USBD_MIDI_GetState (&hUsbDeviceFS) != MIDI_IDLE) {}
//			USBD_MIDI_SendPackets (&hUsbDeviceFS, off_message, 4);
		}

		pressed = false;
		control_toggled = false;
	}
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void Note_Change (uint8_t value) {
	if (value == END) {
		while (USBD_MIDI_GetState (&hUsbDeviceFS) != MIDI_IDLE) {}
		USBD_MIDI_SendPackets (&hUsbDeviceFS, off_message, 4);
	} else {
		on_message [2] = value;
		off_message [2] = value;

		while (USBD_MIDI_GetState (&hUsbDeviceFS) != MIDI_IDLE) {}
		USBD_MIDI_SendPackets (&hUsbDeviceFS, on_message, 4);
	}
}

void Toggle_Control () {
	play_toggled = !play_toggled;
	PS_message[3] = play_toggled ? PLAY : STOP;

	while (USBD_MIDI_GetState (&hUsbDeviceFS) != MIDI_IDLE) {}
	USBD_MIDI_SendPackets (&hUsbDeviceFS, PS_message, 4);
}
/* USER CODE END 1 */
