
//Portions of this software are provided under the original MIT license

/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

//Other portions of this software are provided under the original BSD-3-Clause license:

/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-breakme-Identifier: BSD-3-Clause
 */

// Any changes or additions are available under a GPLv3 license.

// Source: https://github.com/raspberrypi/pico-bootrom/blob/master/usb_device_tiny/virtual_disk.h
// source: https://github.com/raspberrypi/pico-bootrom/blob/master/bootrom/virtual_disk.c
// Source: https://github.com/raspberrypi/tinyusb/blob/pico/examples/device/cdc_msc/src/usb_descriptors.c

#ifndef TINYUSB_MULTITOOL_USBCONFIG_H
#define TINYUSB_MULTITOOL_USBCONFIG_H

#include "tusb.h"
#include "tinyusb_multitool_debug.h"

#define USBD_VID (0x2E8A) // Raspberry Pi
#define USBD_PID (0xF00b) // TODO: DO NOT USE


#define USBD_DESC_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_CDC_DESC_LEN + TUD_CDC_DESC_LEN + TUD_MSC_DESC_LEN)
#define USBD_MAX_POWER_MA (250)

#define USBD_ITF_CDC_STDIO      (0) // needs 2 interfaces - rp2040 stdio output
#define USBD_ITF_CDC_STDIO_DATA	(1)
#define USBD_ITF_CDC_UART0	(2) // needs 2 interfaces - rp2040 uart0 bridge
#define USBD_ITF_CDC_UART0_DATA (3) 
#define USBD_ITF_CDC_UART1	(4) // needs 2 interfaces - rp2040 uart1 bridge
#define USBD_ITF_CDC_UART1_DATA (5)
#define USBD_ITF_MSC		(6) // needs 1 interface
#define USBD_ITF_MAX       	(7)

#define CDCD_ITF_STDIO		(0) //Not sure where these numbers come from, guessing it's the descriptor order?
#define CDCD_ITF_UART0		(1)
#define CDCD_ITF_UART1		(2)

#define USBD_CDC_CMD_MAX_SIZE    (8)
#define USBD_CDC_IN_OUT_MAX_SIZE (64)
#define USBD_MSC_IN_OUT_MAX_SIZE (64)

#define USBD_CDC_STDIO_EP_CMD   (0x81) //EP_CMD and EP_IN are all IN endpoints
#define USBD_CDC_STDIO_EP_IN    (0x82)
#define USBD_CDC_UART0_EP_CMD   (0x83)
#define USBD_CDC_UART0_EP_IN    (0x84)
#define USBD_CDC_UART1_EP_CMD   (0x85)
#define USBD_CDC_UART1_EP_IN    (0x86)
#define USBD_MSC_EP_IN          (0x87)


#define USBD_CDC_STDIO_EP_OUT	(0x02) // STDIO UART
#define USBD_CDC_UART0_EP_OUT   (0x04) // UART0 Bridge
#define USBD_CDC_UART1_EP_OUT   (0x06) // UART1 Bridge
#define USBD_MSC_EP_OUT         (0x07) // USB Mass Storage

#define USBD_STR_0		(0x00)
#define USBD_STR_MANUF		(0x01)
#define USBD_STR_PRODUCT	(0x02)
#define USBD_STR_SERIAL		(0x03)
#define USBD_STR_MSC		(0x04)
#define USBD_STR_CDC_STDIO	(0x05)
#define USBD_STR_CDC_UART0	(0x06)
#define USBD_STR_CDC_UART1	(0x07)


// Fri, 05 Sep 2008 16:20:51
#define RASPBERRY_PI_TIME_FRAC 100
#define RASPBERRY_PI_TIME ((16u << 11u) | (20u << 5u) | (51u >> 1u))
#define RASPBERRY_PI_DATE ((28u << 9u) | (9u << 5u) | (5u))

#define SECTOR_SIZE 512u
#define SECTOR_COUNT (VOLUME_SIZE / SECTOR_SIZE)

#define CLUSTER_UP_SHIFT 0u
#define CLUSTER_UP_MUL (1u << CLUSTER_UP_SHIFT)
#define VOLUME_SIZE (CLUSTER_UP_MUL * 128u * 1024u * 1024u)

#define CLUSTER_SIZE (4096u * CLUSTER_UP_MUL)
#define CLUSTER_SHIFT (3u + CLUSTER_UP_SHIFT)
static_assert(CLUSTER_SIZE == SECTOR_SIZE << CLUSTER_SHIFT, "");

#define CLUSTER_COUNT (VOLUME_SIZE / CLUSTER_SIZE)

static_assert(CLUSTER_COUNT <= 65526, "FAT16 limit");

#define VOLUME_SECTOR_COUNT (SECTOR_COUNT-1)

#define FAT_COUNT 2u
#define MAX_ROOT_DIRECTORY_ENTRIES 512
#define ROOT_DIRECTORY_SECTORS (MAX_ROOT_DIRECTORY_ENTRIES * 32u / SECTOR_SIZE)

#define lsb_hword(x) (((uint)(x)) & 0xffu), ((((uint)(x))>>8u)&0xffu)
#define lsb_word(x) (((uint)(x)) & 0xffu), ((((uint)(x))>>8u)&0xffu),  ((((uint)(x))>>16u)&0xffu),  ((((uint)(x))>>24u)&0xffu)
#define SECTORS_PER_FAT (2 * (CLUSTER_COUNT + SECTOR_SIZE - 1) / SECTOR_SIZE)
static_assert(SECTORS_PER_FAT < 65536, "");

static_assert(VOLUME_SIZE >= 16 * 1024 * 1024, "volume too small for fat16");

// we are a hard drive - SCSI inquiry defines removability
#define IS_REMOVABLE_MEDIA false
#define MEDIA_TYPE (IS_REMOVABLE_MEDIA ? 0xf0u : 0xf8u)

#define MAX_RAM_UF2_BLOCKS 1280
static_assert(MAX_RAM_UF2_BLOCKS >= ((SRAM_END - SRAM_BASE) + (XIP_SRAM_END - XIP_SRAM_BASE)) / 256, "");

#define BOOT_OFFSET_SERIAL_NUMBER 0x27
#define BOOT_OFFSET_LABEL 0x2b

#define ATTR_READONLY       0x01u
#define ATTR_HIDDEN         0x02u
#define ATTR_SYSTEM         0x04u
#define ATTR_VOLUME_LABEL   0x08u
#define ATTR_DIR            0x10u
#define ATTR_ARCHIVE        0x20u

#define MBR_OFFSET_SERIAL_NUMBER 0x1b8


static struct uf2_info {
    uint32_t *valid_blocks;
    uint32_t max_valid_blocks;
    uint32_t *cleared_pages;
    uint32_t max_cleared_pages;
    uint32_t num_blocks;
    uint32_t token;
    uint32_t valid_block_count;
    uint32_t lowest_addr;
    uint32_t block_no;
    bool ram;
} _uf2_info;






#define WELCOME_HTML 		"<html><head><meta http-equiv='refresh' content=\"0;URL='https://raspberrypi.com/device/RP2'\"/></head><body>Redirecting to <a href='https://raspberrypi.com/device/RP2'>raspberrypi.com</a></body></html>"
#define WELCOME_HTML_LEN 	strlen(WELCOME_HTML)

#define INFO_UF2_TXT_MODEL 	"LED Matrix MCU 16x16"
#define INFO_UF2_TXT_BOARD_ID 	"LEDMCUMZ16163-A"
#define INFO_UF2_TXT 		"TinyUSB Multitool UF2 loader v1.0\r\nModel: " INFO_UF2_TXT_MODEL "\r\nBoard-ID: " INFO_UF2_TXT_BOARD_ID  "\r\n"
#define INFO_UF2_TXT_LEN 	strlen(INFO_UF2_TXT)

#define README_TXT		"2bn 16x16 LED Matrix MCU Flash Interface\r\n----------------------------------------\r\n\r\nThis virtual USB thumb drive can be used to flash a new firmware onto your\r\n2bn 16x16 LED Matrix. It will only show up while your device is plugged into USB.\r\nAll you need to do is drag and drop a new firmware into this drive\r\n\r\nWARNING: This flash interface is created WITHIN the original software provided\r\nwith your device, if you flash a custom firmware that does not include the tinyusb_multitool library\r\nthe flash interface will no longer exist.\r\n"
#define README_TXT_LEN		strlen(README_TXT)



//!< Defines where flash should be written to initially, flash is rewritten from here to 0x0 after the first write. This is requisite because USB will become non-functional as soon as we start erasing flash.
#define FLASH_OFFSET 1*1024*1024
#endif
