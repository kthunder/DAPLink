/**
 * @file    IO_Config.h
 * @brief
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2016, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __IO_CONFIG_H__
#define __IO_CONFIG_H__

#include "stm32f1xx.h"
#include "compiler.h"
#include "daplink.h"

COMPILER_ASSERT(DAPLINK_HIC_ID == DAPLINK_HIC_ID_STM32F103XB);

// Enable JTAG for DAPLink, which uses RTS as TDI and DTR as TDO
#define ENABLE_JTAG 1

//USB control pin
#define USB_CONNECT_PORT_ENABLE()    __HAL_RCC_GPIOA_CLK_ENABLE()
#define USB_CONNECT_PORT_DISABLE()   __HAL_RCC_GPIOA_CLK_DISABLE()
#define USB_CONNECT_PORT             GPIOA
#define USB_CONNECT_PIN              GPIO_PIN_15
#define USB_CONNECT_ON()             (USB_CONNECT_PORT->BSRR = USB_CONNECT_PIN)
#define USB_CONNECT_OFF()            (USB_CONNECT_PORT->BRR  = USB_CONNECT_PIN)

//Connected LED
#define CONNECTED_LED_PORT           GPIOB
#define CONNECTED_LED_PIN            GPIO_PIN_11
#define CONNECTED_LED_PIN_Bit        11

//When bootloader, disable the target port(not used)
#define POWER_EN_PIN_PORT            GPIOB
#define POWER_EN_PIN                 GPIO_PIN_15
#define POWER_EN_Bit                 15

// nRESET OUT Pin
#define nRESET_PIN_PORT              GPIOB
#define nRESET_PIN                   GPIO_PIN_0
#define nRESET_PIN_Bit               0

//SWD
#define SWCLK_TCK_PIN_PORT           GPIOB
#define SWCLK_TCK_PIN                GPIO_PIN_14
#define SWCLK_TCK_PIN_Bit            14

#define SWDIO_TMS_PIN_PORT           GPIOB
#define SWDIO_TMS_PIN                GPIO_PIN_13
#define SWDIO_TMS_PIN_Bit            13

// #define SWDIO_OUT_PIN_PORT           GPIOB
// #define SWDIO_OUT_PIN                GPIO_PIN_14
// #define SWDIO_OUT_PIN_Bit            14

// #define SWDIO_IN_PIN_PORT            GPIOB
// #define SWDIO_IN_PIN                 GPIO_PIN_12
// #define SWDIO_IN_PIN_Bit             12

// cJAG
#define cJTAG_TCKC_PIN_PORT       SWCLK_TCK_PIN_PORT
#define cJTAG_TCKC_PIN            SWCLK_TCK_PIN
#define cJTAG_TCKC_PIN_Bit        SWCLK_TCK_PIN_Bit

#define cJTAG_TMSC_PIN_PORT       SWDIO_TMS_PIN_PORT
#define cJTAG_TMSC_PIN            SWDIO_TMS_PIN
#define cJTAG_TMSC_PIN_Bit        SWDIO_TMS_PIN_Bit

// JTAG
#define JTAG_TCK_PIN_PORT            SWCLK_TCK_PIN_PORT
#define JTAG_TCK_PIN                 SWCLK_TCK_PIN
#define JTAG_TCK_PIN_Bit             SWCLK_TCK_PIN_Bit

#define JTAG_TMS_PIN_PORT            SWDIO_TMS_PIN_PORT
#define JTAG_TMS_PIN                 SWDIO_TMS_PIN
#define JTAG_TMS_PIN_Bit             SWDIO_TMS_PIN_Bit

// useing UART_DTR
#define JTAG_TDI_PIN_PORT            GPIOA
#define JTAG_TDI_PIN                 GPIO_PIN_10
#define JTAG_TDI_PIN_Bit             10
// useing UART_RTS
#define JTAG_TDO_PIN_PORT            GPIOA
#define JTAG_TDO_PIN                 GPIO_PIN_9
#define JTAG_TDO_PIN_Bit             9

//LEDs
//USB status LED
#define RUNNING_LED_PORT             GPIOB
#define RUNNING_LED_PIN              GPIO_PIN_2
#define RUNNING_LED_Bit              2

#define PIN_HID_LED_PORT             GPIOB
#define PIN_HID_LED                  GPIO_PIN_6
#define PIN_HID_LED_Bit              6

#define PIN_CDC_LED_PORT             GPIOB
#define PIN_CDC_LED                  GPIO_PIN_10
#define PIN_CDC_LED_Bit              10

#define PIN_MSC_LED_PORT             GPIOA
#define PIN_MSC_LED                  GPIO_PIN_0
#define PIN_MSC_LED_Bit              0


#endif
