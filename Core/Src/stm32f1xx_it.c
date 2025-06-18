/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_midi.h"
#include <stdbool.h>
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
uint8_t on_message[4] = { 0x09, 0x90, 0x00, 0x40 };
uint8_t off_message[4] = { 0x08, 0x80, 0x00, 0x40 };

uint8_t PS_message[4] = { 0x0B, 0xB0, 0x15, STOP };

uint8_t D_vol_message [4] = { 0x0B, 0xB0, 0x07, 0x00 };
uint8_t P_vol_message [4] = { 0x0B, 0xB0, 0x07, 0x00 };

bool control_toggled = false;
bool play_toggled = false;
bool pressed = false;

uint16_t D_previous = 0;
uint16_t D_current = 0;
uint16_t D_sum = 0;

uint16_t P_previous = 0;
uint16_t P_current = 0;
uint16_t P_sum = 0;

uint8_t elapsed_times = 1;

bool locked = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void Note_Change (uint8_t value);
void Toggle_Control ();
void Update_Volume ();

uint16_t ADC_Convert_Rank1 (void);
uint16_t ADC_Convert_Rank2 (void);

//uint8_t ratio (long value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;
extern TIM_HandleTypeDef htim2;
/* USER CODE BEGIN EV */
extern USBD_HandleTypeDef hUsbDeviceFS;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
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
  * @brief This function handles Prefetch fault, memory access fault.
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
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	if (!HAL_GPIO_ReadPin (C_B_GPIO_Port, C_B_Pin)) {
		if (!pressed) {
			pressed = true;

			Note_Change (C);
		}
	} else if (!HAL_GPIO_ReadPin (CS_B_GPIO_Port, CS_B_Pin)) {
		if (!pressed) {
			pressed = true;

			Note_Change (CS);
		}
	} else if (!HAL_GPIO_ReadPin (D_B_GPIO_Port, D_B_Pin)) {
		if (!pressed) {
			pressed = true;

			Note_Change (D);
		}
	} else if (!HAL_GPIO_ReadPin (DS_B_GPIO_Port, DS_B_Pin)) {
		if (!pressed) {
			pressed = true;

			Note_Change (DS);
		}
	} else if (!HAL_GPIO_ReadPin (E_B_GPIO_Port, E_B_Pin)) {
		if (!pressed) {
			pressed = true;

			Note_Change (E);
		}
	} else if (!HAL_GPIO_ReadPin (F_B_GPIO_Port, F_B_Pin)) {
		if (!pressed) {
			pressed = true;

			Note_Change (F);
		}
	} else if (!HAL_GPIO_ReadPin (FS_B_GPIO_Port, FS_B_Pin)) {
		if (!pressed) {
			pressed = true;
		}
	} else if (!HAL_GPIO_ReadPin (G_B_GPIO_Port, G_B_Pin)) {
		if (!pressed) {
			pressed = true;

			Note_Change (G);
		}
	} else if (!HAL_GPIO_ReadPin (GS_B_GPIO_Port, GS_B_Pin)) {
		if (!pressed) {
			pressed = true;

			Note_Change (GS);
		}
	} else if (!HAL_GPIO_ReadPin (A_B_GPIO_Port, A_B_Pin)) {
		if (!pressed) {
			pressed = true;

			Note_Change (A);
		}
	} else if (!HAL_GPIO_ReadPin (AS_B_GPIO_Port, AS_B_Pin)) {
		if (!pressed) {
			pressed = true;

			Note_Change (AS);
		}
	} else if (!HAL_GPIO_ReadPin (B_B_GPIO_Port, B_B_Pin)) {
		if (!pressed) {
			pressed = true;

			Note_Change (B);
		}
	} else if (!HAL_GPIO_ReadPin (SP_B_GPIO_Port, SP_B_Pin)) {
		if (!pressed) {
			pressed = true;
			control_toggled = true;

			Toggle_Control ();
		}
	} else {
		if (pressed && !control_toggled) {
			Note_Change (END);
		}

		pressed = false;
		control_toggled = false;
	}




	if (elapsed_times > 4) {

		for (uint8_t i = 0; i < 16; i++) {
	        D_sum += ADC_Convert_Rank1 ();
		}

		D_current = ((D_sum >> 4) * 127) / 4095;
		D_sum = 0;

		if (D_current < D_previous - 3 || D_current > D_previous + 3) {
			D_previous = D_current;
			D_vol_message[3] = D_current;

			while (USBD_MIDI_GetState (&hUsbDeviceFS) != MIDI_IDLE) {}
			while (locked) {}
			locked = true;
			USBD_MIDI_SendPackets (&hUsbDeviceFS, D_vol_message, 4);
			locked = false;
		}

		for (uint8_t i = 0; i < 16; i++) {
			P_sum += ADC_Convert_Rank2 ();
		}

	  	P_current = ((P_sum >> 4) * 127) / 4095;
	  	P_sum = 0;

	  	if (P_current < P_previous - 3 || P_current > P_previous + 3) {
			P_previous = P_current;
			P_vol_message[3] = P_current;

			while (USBD_MIDI_GetState (&hUsbDeviceFS) != MIDI_IDLE) {}
			USBD_MIDI_SendPackets (&hUsbDeviceFS, P_vol_message, 4);
		}

		elapsed_times = 0;
	}

	elapsed_times++;


//		for (uint8_t i = 0; i < 16; i++) {
//			HAL_ADC_Start (&hadc1);
//			HAL_ADC_PollForConversion (&hadc1, 100);
//			D_sum += HAL_ADC_GetValue (&hadc1);
//
//			HAL_ADC_Start (&hadc1);
//			HAL_ADC_PollForConversion (&hadc1, 100);
//			P_sum += HAL_ADC_GetValue (&hadc1);
//
//			HAL_ADC_Stop (&hadc1);
//			HAL_Delay (100);
//		}
//
//		D_current = ((D_sum >> 4) * 127) / 4095;
//		P_current = ((P_sum >> 4) * 127) / 4095;
//		D_sum = 0;
//		P_sum = 0;
//
//		if (D_current < D_previous - 3 || D_current > D_previous + 3) {
//			D_previous = D_current;
//			D_vol_message[3] = D_current;
//
//			while (USBD_MIDI_GetState (&hUsbDeviceFS) != MIDI_IDLE) {}
//			USBD_MIDI_SendPackets (&hUsbDeviceFS, D_vol_message, 4);
//		}
//
//		if (P_current < P_previous - 3 || P_current > P_previous + 3) {
//			P_previous = P_current;
//			P_vol_message[3] = P_current;
//
//			while (USBD_MIDI_GetState (&hUsbDeviceFS) != MIDI_IDLE) {}
//			USBD_MIDI_SendPackets (&hUsbDeviceFS, P_vol_message, 4);
//		}

//		elapsed_times = 0;


//		HAL_ADC_Start (&hadc1);
//		HAL_ADC_PollForConversion (&hadc1, 100);
//		D_ADC_val[0] = HAL_ADC_GetValue (&hadc1);
//
//		HAL_ADC_Start (&hadc1);
//		HAL_ADC_PollForConversion (&hadc1, 100);
//		P_ADC_val[0] = HAL_ADC_GetValue (&hadc1);
//
//		HAL_ADC_Start (&hadc1);
//		HAL_ADC_PollForConversion (&hadc1, 100);
//		D_ADC_val[1] = HAL_ADC_GetValue (&hadc1);
//
//		HAL_ADC_Start (&hadc1);
//		HAL_ADC_PollForConversion (&hadc1, 100);
//		P_ADC_val[1] = HAL_ADC_GetValue (&hadc1);

//		D_current = (alpha * D_ADC_val[1]) + ((1 - alpha) * D_ADC_val[0]);
//		D_current = ratio (D_current);
//
//		P_current = (alpha * P_ADC_val[1]) + ((1 - alpha) * P_ADC_val[0]);
//		P_current = ratio (P_current);
//		P_previous = P_current;
//		P_vol_message [3] = P_current;
//
//	    if (D_current < D_previous - 3 || D_current > D_previous + 3) {
//	    	D_previous = D_current;
//	    	D_vol_message [3] = D_current;
//
//	    	while (USBD_MIDI_GetState (&hUsbDeviceFS) != MIDI_IDLE) {}
//	    	USBD_MIDI_SendPackets (&hUsbDeviceFS, D_vol_message, 4);
//	    }
//
//	    if (P_current < P_previous - 3 || P_current > P_previous + 3) {
//			P_previous = P_current;
//			P_vol_message [3] = P_current;
//
//			while (USBD_MIDI_GetState (&hUsbDeviceFS) != MIDI_IDLE) {}
//			USBD_MIDI_SendPackets (&hUsbDeviceFS, P_vol_message, 4);
//		}


//		elapsed_times = 0;
//	}

//	elapsed_times++;

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void Note_Change (uint8_t value) {
	if (value == END) {
		while (USBD_MIDI_GetState (&hUsbDeviceFS) != MIDI_IDLE) {}
		while (locked) {}
		locked = true;
		USBD_MIDI_SendPackets (&hUsbDeviceFS, off_message, 4);
		locked = false;
	} else {
		on_message [2] = value;
		off_message [2] = value;

		while (USBD_MIDI_GetState (&hUsbDeviceFS) != MIDI_IDLE) {}
		while (locked) {}
		locked = true;
		USBD_MIDI_SendPackets (&hUsbDeviceFS, on_message, 4);
		locked = false;
	}
}

void Toggle_Control () {
	play_toggled = !play_toggled;
	PS_message[3] = play_toggled ? PLAY : STOP;

	while (USBD_MIDI_GetState (&hUsbDeviceFS) != MIDI_IDLE) {}\
	while (locked) {}
	locked = true;
	USBD_MIDI_SendPackets (&hUsbDeviceFS, PS_message, 4);
	locked = false;
}
/* USER CODE END 1 */
