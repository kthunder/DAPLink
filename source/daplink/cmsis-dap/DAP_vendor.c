/*
 * Copyright (c) 2013-2020 Arm Limited. All rights reserved.
 * Copyright 2019, Cypress Semiconductor Corporation
 * or a subsidiary of Cypress Semiconductor Corporation.
 * Copyright (c) 2021 Chris Reed
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * ----------------------------------------------------------------------
 *
 * $Date:        20. May 2015
 * $Revision:    V1.10
 *
 * Project:      CMSIS-DAP Source
 * Title:        DAP_vendor.c CMSIS-DAP Vendor Commands
 *
 *---------------------------------------------------------------------------*/

#include "rl_usb.h"
#include "DAP_config.h"
#include "DAP.h"
#include "info.h"
#include "daplink.h"
#include DAPLINK_MAIN_HEADER
#include "uart.h"
#include "settings.h"
#include "target_family.h"
#include "flash_manager.h"
#include "util.h"
#include <string.h>
#include "daplink_vendor_commands.h"

#ifdef DRAG_N_DROP_SUPPORT
#include "file_stream.h"

// Reusing the MSC sector buffer from vfs_manager.c to save memory
// as using both at the same time will break anyway
extern uint32_t usb_buffer[VFS_SECTOR_SIZE / sizeof(uint32_t)];
static uint8_t *file_stream_buffer = (uint8_t *)usb_buffer;
static const uint32_t file_stream_buffer_size = sizeof(usb_buffer);
static uint16_t file_stream_buffer_pos = 0;
#endif

//**************************************************************************************************
/**
\defgroup DAP_Vendor_Adapt_gr Adapt Vendor Commands
\ingroup DAP_Vendor_gr
@{

The file DAP_vendor.c provides template source code for extension of a Debug Unit with
Vendor Commands. Copy this file to the project folder of the Debug Unit and add the
file to the MDK-ARM project under the file group Configuration.
*/

/** Process DAP Vendor Command and prepare Response Data
\param request   pointer to request data
\param response  pointer to response data
\return          number of bytes in response (lower 16 bits)
                 number of bytes in request (upper 16 bits)
*/
uint32_t DAP_ProcessVendorCommand(const uint8_t *request, uint8_t *response) {
  uint32_t num = (1U << 16) | 1U;

  *response++ = *request;        // copy Command ID

  switch (*request++) {          // first byte in request is Command ID
    case ID_DAP_GetUniqueID: {
        const char *id_str = info_get_unique_id();
        uint8_t len = strlen(id_str);
        *response++ = len;
        memcpy(response, id_str, len);
        num += (len + 1); // increment response count by ID length + length byte
        break;
    }
    case ID_DAP_UART_GetLineCoding: {
        // get line coding
        int32_t read_len = sizeof(CDC_LINE_CODING);
        CDC_LINE_CODING cdc_line_coding;
        USBD_CDC_ACM_PortGetLineCoding(&cdc_line_coding);
        memcpy(response, &cdc_line_coding, read_len);
        num += (read_len + 1);
        break;
    }
    case ID_DAP_UART_SetConfiguration: {
        // set uart configuration
        CDC_LINE_CODING cdc_line_coding;
        USBD_CDC_ACM_PortGetLineCoding(&cdc_line_coding);
        //set BaudRate
        uint32_t baud_rate = 0;
        memcpy(&baud_rate, request, sizeof(uint32_t));
        cdc_line_coding.dwDTERate = baud_rate;
        USBD_CDC_ACM_PortSetLineCoding(&cdc_line_coding);
        USBD_CDC_ACM_SendBreak(0);
        *response = 1;
        num += (sizeof(uint32_t) << 16) | 1;
        break;
    }
    case ID_DAP_UART_Read:  {
        // uart read
        int32_t read_len = 62;
        read_len = uart_read_data(response + 1, read_len);
        if (read_len) {
            main_blink_cdc_led(MAIN_LED_FLASH);
        }
        response[0] = read_len;
        // increment request and response count
        num += (read_len + 1);
        break;
    }
    case ID_DAP_UART_Write:  {
        // uart write
        int32_t write_len = *request;
        request++;
        uart_write_data((uint8_t *)request, write_len);
        main_blink_cdc_led(MAIN_LED_FLASH);
        *response = 1;
        num += ((write_len + 1) << 16) | 1;
        break;
    }
    case ID_DAP_Vendor5:  break;
    case ID_DAP_Vendor6:  break;
    case ID_DAP_Vendor7:  break;
    case ID_DAP_SetUSBTestMode: {
        *response = 1;
        if (0 == *request) {
            main_usb_set_test_mode(false);
        } else if (1 == *request) {
            main_usb_set_test_mode(true);
        } else {
            *response = 0;
        }
        num += (1U << 16) | 1U; // increment request and response count each by 1
        break;
    }
    case ID_DAP_ResetTargetIfNoAutoReset: {
        // reset target
        *response = 1;
        if (!config_get_auto_rst()) {
            target_set_state(RESET_RUN);
        }
        num += 1;
        break;
    }
#ifdef DRAG_N_DROP_SUPPORT
    case ID_DAP_MSD_Open: {
        // open mass storage device stream
        *response = stream_open((stream_type_t)(*request));
        num += (1 << 16) | 1;
        file_stream_buffer_pos = 0;
        break;
    }
    case ID_DAP_MSD_Close: {
        // write the remaining data in the buffer
         if (file_stream_buffer_pos) {
            *response = stream_write(file_stream_buffer, file_stream_buffer_pos);
            file_stream_buffer_pos = 0;
            if (ERROR_SUCCESS != *response &&
                    ERROR_SUCCESS_DONE != *response &&
                    ERROR_SUCCESS_DONE_OR_CONTINUE != *response) {
                num += 1;
                break;
            }
        }
        // close mass storage device stream
        *response = stream_close();
        num += 1;
        break;
    }
    case ID_DAP_MSD_Write: {
        // write to mass storage device in blocks of length == vfs sector size
        uint32_t write_len = *request;
        request++;
        main_blink_msc_led(MAIN_LED_FLASH);
        if (file_stream_buffer_pos || (write_len % VFS_SECTOR_SIZE)) {
            uint32_t write_len_left = write_len;
            while (write_len_left > 0) {
                uint16_t copy_len = MIN(VFS_SECTOR_SIZE - file_stream_buffer_pos, write_len_left);
                memcpy(file_stream_buffer + file_stream_buffer_pos, request, copy_len);
                file_stream_buffer_pos += copy_len;
                write_len_left -= copy_len;
                request += copy_len;
                if (file_stream_buffer_pos >= VFS_SECTOR_SIZE) {
                    *response = stream_write(file_stream_buffer, VFS_SECTOR_SIZE);
                    file_stream_buffer_pos = 0;
                } else {
                    *response = ERROR_SUCCESS;
                }
            }
        } else {
            *response = stream_write((uint8_t *)request, write_len);
        }
        num += ((write_len + 1) << 16) | 1;
        break;
    }
#endif
    case ID_DAP_SelectEraseMode: {
        // switching between chip erase and page erase
        //              COMMAND(OUT Packet)
        //              BYTE 0 1000 1110 0x8D
        //              BYTE 1 Desired Mode:
        //                                              0x00 - Chip Erase
        //                                              nonzero - Page Erase
        //              RESPONSE(IN Packet)
        //              BYTE 0
        //                                              0x00 - OK
        *response = DAP_OK;
        if (0x00U == *request) {
            flash_manager_set_page_erase(false);
        } else {
            flash_manager_set_page_erase(true);
        }
        num += (1U << 16) | 1U; // increment request and response count each by 1
        break;
    }
    case ID_DAP_Vendor14: break;
    case ID_DAP_Vendor15: break;
    case ID_DAP_Vendor16: break;
    case ID_DAP_Vendor17: break;
    case ID_DAP_Vendor18: break;
    case ID_DAP_Vendor19: break;
    case ID_DAP_Vendor20: break;
    case ID_DAP_Vendor21: break;
    case ID_DAP_Vendor22: break;
    case ID_DAP_Vendor23: break;
    case ID_DAP_Vendor24: break;
    case ID_DAP_Vendor25: break;
    case ID_DAP_Vendor26: break;
    case ID_DAP_Vendor27: break;
    case ID_DAP_Vendor28: break;
    case ID_DAP_Vendor29: break;
    case ID_DAP_Vendor30: break;
    case ID_DAP_Vendor31: {
        num += 1U << 16;           // increment request count
        if (*request == 1U) {      // when first command data byte is 1
    extern void cJtag_active(void);
    extern uint8_t cJtag_enabled;
            cJtag_enabled = 1;       // enable cJTAG
            cJtag_active(); 
            *response++ = DAP_OK;
            num++;
        }
        break;
    }
    default: break;
  }

  return (num);
}

///@}
