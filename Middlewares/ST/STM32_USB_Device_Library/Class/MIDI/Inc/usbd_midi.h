/**
  ******************************************************************************
  * @file    usbd_midi.h
  * @author  Illia Pikin, MCD Application Team
  * @brief   header file for the usbd_midi.c file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2025 Illia Pikin</center></h2>
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  * Licensed under BSD 2-Clause License
  *
  * Copyright (c) 2025, Illia Pikin a.k.a Hypnotriod
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice, this
  *    list of conditions and the following disclaimer.
  *
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#ifndef __USB_MIDI_H
#define __USB_MIDI_H

#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_MIDI_Exported_Defines
  * @{
  */
#define MIDI_EPIN_ADDR                 0x81
#define MIDI_EPIN_SIZE                 0x40

#define MIDI_EPOUT_ADDR                0x01
#define MIDI_EPOUT_SIZE                0x40

#define USB_MIDI_CLASS_DESC_SHIFT      18
#define USB_MIDI_DESC_SIZE             7
#define USB_MIDI_INTERFACE_DESC_SIZE   (MIDI_IN_PORTS_NUM * 16 + MIDI_OUT_PORTS_NUM * 16 + 33)
#define USB_MIDI_CONFIG_DESC_SIZE      (USB_MIDI_INTERFACE_DESC_SIZE + USB_MIDI_CLASS_DESC_SHIFT)

#define MIDI_DESCRIPTOR_TYPE           0x21

#define MIDI_REQ_SET_PROTOCOL          0x0B
#define MIDI_REQ_GET_PROTOCOL          0x03

#define MIDI_REQ_SET_IDLE              0x0A
#define MIDI_REQ_GET_IDLE              0x02

#define MIDI_JACK_1    0x01
#define MIDI_JACK_2    0x02
#define MIDI_JACK_3    0x03
#define MIDI_JACK_4    0x04
#define MIDI_JACK_5    0x05
#define MIDI_JACK_6    0x06
#define MIDI_JACK_7    0x07
#define MIDI_JACK_8    0x08
#define MIDI_JACK_9    0x09
#define MIDI_JACK_10   0x0a
#define MIDI_JACK_11   0x0b
#define MIDI_JACK_12   0x0c
#define MIDI_JACK_13   0x0d
#define MIDI_JACK_14   0x0e
#define MIDI_JACK_15   0x0f
#define MIDI_JACK_16   0x10
#define MIDI_JACK_17   (MIDI_IN_PORTS_NUM * 2 + 0x01)
#define MIDI_JACK_18   (MIDI_IN_PORTS_NUM * 2 + 0x02)
#define MIDI_JACK_19   (MIDI_IN_PORTS_NUM * 2 + 0x03)
#define MIDI_JACK_20   (MIDI_IN_PORTS_NUM * 2 + 0x04)
#define MIDI_JACK_21   (MIDI_IN_PORTS_NUM * 2 + 0x05)
#define MIDI_JACK_22   (MIDI_IN_PORTS_NUM * 2 + 0x06)
#define MIDI_JACK_23   (MIDI_IN_PORTS_NUM * 2 + 0x07)
#define MIDI_JACK_24   (MIDI_IN_PORTS_NUM * 2 + 0x08)
#define MIDI_JACK_25   (MIDI_IN_PORTS_NUM * 2 + 0x09)
#define MIDI_JACK_26   (MIDI_IN_PORTS_NUM * 2 + 0x0a)
#define MIDI_JACK_27   (MIDI_IN_PORTS_NUM * 2 + 0x0b)
#define MIDI_JACK_28   (MIDI_IN_PORTS_NUM * 2 + 0x0c)
#define MIDI_JACK_29   (MIDI_IN_PORTS_NUM * 2 + 0x0d)
#define MIDI_JACK_30   (MIDI_IN_PORTS_NUM * 2 + 0x0e)
#define MIDI_JACK_31   (MIDI_IN_PORTS_NUM * 2 + 0x0f)
#define MIDI_JACK_32   (MIDI_IN_PORTS_NUM * 2 + 0x10)

/**
  * @}
  */


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */
typedef enum
{
  MIDI_IDLE = 0,
  MIDI_BUSY,
}
MIDI_StateTypeDef;


typedef struct
{
  uint32_t             Protocol;
  uint32_t             IdleState;
  uint32_t             AltSetting;
  MIDI_StateTypeDef     state;
}
USBD_MIDI_HandleTypeDef;
/**
  * @}
  */



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */

extern USBD_ClassTypeDef  USBD_MIDI;
extern void USBD_MIDI_OnPacketsReceived(uint8_t *data, uint8_t len);
extern void USBD_MIDI_OnPacketsSent(void);

/**
  * @}
  */

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
uint8_t USBD_MIDI_SendPackets(USBD_HandleTypeDef *pdev, uint8_t *data, uint16_t len);
uint8_t *USBD_MIDI_DeviceQualifierDescriptor (uint16_t *length);
uint8_t USBD_MIDI_GetState(USBD_HandleTypeDef  *pdev);
uint8_t USBD_MIDI_GetDeviceState(USBD_HandleTypeDef  *pdev);

/**
  * @}
  */

#endif  // __USB_MIDI_H
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
