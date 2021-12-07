
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

#ifndef TINYUSB_MULTITOOL_CONFIG_H
#define TINYUSB_MULTITOOL_CONFIG_H


#include "tusb.h"
#include "tinyusb_multitool_debug.h"
#include "boot/uf2.h"
#include "hardware/watchdog.h"
#include "hardware/flash.h"
#include "pico/unique_id.h"
//#include "unique_id_modified.h"
//#include "program_flash_generic.h"

#define USBD_VID (0x2E8A) // Raspberry Pi
#define USBD_PID (0x100b) // TODO: DO NOT USE

#define USBD_DESC_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_MSC_DESC_LEN)
#define USBD_MAX_POWER_MA (250)

#define USBD_ITF_CDC       	(0) // needs 2 interfaces
#define USBD_ITF_CDC_DATA	(1)
#define USBD_ITF_MSC		(2) // needs 1 interface
#define USBD_ITF_MAX       	(3)

#define USBD_CDC_EP_CMD 	(0x81)
#define USBD_CDC_EP_OUT		(0x02)
#define USBD_CDC_EP_IN 		(0x82)
#define USBD_CDC_CMD_MAX_SIZE 	(8)
#define USBD_CDC_IN_OUT_MAX_SIZE (64)

#define USBD_MSC_EP_OUT		(0x03)
#define USBD_MSC_EP_IN		(0x83)
#define USBD_MSC_IN_OUT_MAX_SIZE (64)

#define USBD_STR_0 		(0x00)
#define USBD_STR_MANUF 		(0x01)
#define USBD_STR_PRODUCT 	(0x02)
#define USBD_STR_SERIAL 	(0x03)
#define USBD_STR_CDC 		(0x04)
#define USBD_STR_MSC		(0x05)

// Note: descriptors returned from callbacks must exist long enough for transfer to complete

static const tusb_desc_device_t usbd_desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor = USBD_VID,
    .idProduct = USBD_PID,
    .bcdDevice = 0x0100,

    .iManufacturer = USBD_STR_MANUF,
    .iProduct = USBD_STR_PRODUCT,
    .iSerialNumber = USBD_STR_SERIAL,
    .bNumConfigurations = 1,
};

static const uint8_t usbd_desc_cfg[USBD_DESC_LEN] = {
	TUD_CONFIG_DESCRIPTOR(1, USBD_ITF_MAX, USBD_STR_0, USBD_DESC_LEN, 0, USBD_MAX_POWER_MA),

	TUD_CDC_DESCRIPTOR(USBD_ITF_CDC, USBD_STR_CDC, USBD_CDC_EP_CMD, USBD_CDC_CMD_MAX_SIZE, 	USBD_CDC_EP_OUT, USBD_CDC_EP_IN, USBD_CDC_IN_OUT_MAX_SIZE),
	TUD_MSC_DESCRIPTOR(USBD_ITF_MSC, USBD_STR_MSC, 						USBD_MSC_EP_OUT, USBD_MSC_EP_IN, USBD_MSC_IN_OUT_MAX_SIZE),

};

static char usbd_serial_str[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1];

static const char *const usbd_desc_str[] = {
    [USBD_STR_MANUF] = "2bn Development",
    [USBD_STR_PRODUCT] = "TinyUSB Multitool",
    [USBD_STR_SERIAL] = usbd_serial_str,
    [USBD_STR_CDC] = "TinyUSB Multitool CDC",
    [USBD_STR_MSC] = "TinyUSB Multitool MSC",

};

const uint8_t *tud_descriptor_device_cb(void) {	
	_DBG("tud_descriptor_device_cb()");
	return (const uint8_t *)&usbd_desc_device;
}

const uint8_t *tud_descriptor_configuration_cb(uint8_t index) {
	_DBG("tud_descriptor_configuration_cb(%d)", index);
	return usbd_desc_cfg;
}




const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
	_DBG("tud_descriptor_string_cb(%d, %d)", index, langid);
    #define DESC_STR_MAX (20)
    static uint16_t desc_str[DESC_STR_MAX];

    // Assign the SN using the unique flash id
    if (!usbd_serial_str[0]) {
        pico_get_unique_board_id_string(usbd_serial_str, sizeof(usbd_serial_str));
    }

    uint8_t len;
    if (index == 0) {
        desc_str[1] = 0x0409; // supported language is English
        len = 1;
    } else {
        if (index >= sizeof(usbd_desc_str) / sizeof(usbd_desc_str[0])) {
            return NULL;
        }
        const char *str = usbd_desc_str[index];
        for (len = 0; len < DESC_STR_MAX - 1 && str[len]; ++len) {
            desc_str[1 + len] = str[len];
        }
    }

    // first byte is length (including header), second byte is string type
    desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 * len + 2));

    return desc_str;
}





#define __not_in_flash(group) __attribute__((section(".time_critical." group)))
#define __not_in_flash_func(func_name) __not_in_flash(__STRING(func_name)) func_name
#define __no_inline_not_in_flash_func(func_name) __noinline __not_in_flash_func(func_name)

// give us ourselves 64M which should strictly be the minimum for FAT16

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

static __attribute__((aligned(4))) uint32_t uf2_valid_ram_blocks[(MAX_RAM_UF2_BLOCKS + 31) / 32];

enum partition_type {
    PT_FAT12 = 1,
    PT_FAT16 = 4,
    PT_FAT16_LBA = 0xe,
};

static const uint8_t boot_sector[] = {
        // 00 here should mean not bootable (according to spec) -- still windows unhappy without it
        0xeb, 0x3c, 0x90,
        // 03 id
        'M', 'S', 'W', 'I', 'N', '4', '.', '1',
        // 0b bytes per sector
        lsb_hword(512),
        // 0d sectors per cluster
        (CLUSTER_SIZE / SECTOR_SIZE),
        // 0e reserved sectors
        lsb_hword(1),
        // 10 fat count
        FAT_COUNT,
        // 11 max number root entries
        lsb_hword(MAX_ROOT_DIRECTORY_ENTRIES),
        // 13 number of sectors, if < 32768
#if VOLUME_SECTOR_COUNT < 65536
        lsb_hword(VOLUME_SECTOR_COUNT),
#else
        lsb_hword(0),
#endif
        // 15 media descriptor
        MEDIA_TYPE,
        // 16 sectors per FAT
        lsb_hword(SECTORS_PER_FAT),
        // 18 sectors per track (non LBA)
        lsb_hword(1),
        // 1a heads (non LBA)
        lsb_hword(1),
        // 1c hidden sectors 1 for MBR
        lsb_word(SECTOR_COUNT - VOLUME_SECTOR_COUNT),
// 20 sectors if >32K
#if VOLUME_SECTOR_COUNT >= 65536
        lsb_word(VOLUME_SECTOR_COUNT),
#else
        lsb_word(0),
#endif
        // 24 drive number
        0,
        // 25 reserved (seems to be chkdsk flag for clean unmount - linux writes 1)
        0,
        // 26 extended boot sig
        0x29,
        // 27 serial number
        0, 0, 0, 0,
        // 2b label
        '2', 'b', 'n', '-', 'L', 'E', 'D', '1', '6', '1', '6',
        'F', 'A', 'T', '1', '6', ' ', ' ', ' ',
        0xeb, 0xfe // while(1);
};

static_assert(sizeof(boot_sector) == 0x40, "");

#define BOOT_OFFSET_SERIAL_NUMBER 0x27
#define BOOT_OFFSET_LABEL 0x2b

#define ATTR_READONLY       0x01u
#define ATTR_HIDDEN         0x02u
#define ATTR_SYSTEM         0x04u
#define ATTR_VOLUME_LABEL   0x08u
#define ATTR_DIR            0x10u
#define ATTR_ARCHIVE        0x20u

#define MBR_OFFSET_SERIAL_NUMBER 0x1b8


enum picoboot_status {
    PICOBOOT_OK = 0,
    PICOBOOT_UNKNOWN_CMD = 1,
    PICOBOOT_INVALID_CMD_LENGTH = 2,
    PICOBOOT_INVALID_TRANSFER_LENGTH = 3,
    PICOBOOT_INVALID_ADDRESS = 4,
    PICOBOOT_BAD_ALIGNMENT = 5,
    PICOBOOT_INTERLEAVED_WRITE = 6,
    PICOBOOT_REBOOTING = 7,
    PICOBOOT_UNKNOWN_ERROR = 8,
};

enum task_source {
	TASK_SOURCE_VIRTUAL_DISK = 1,
	TASK_SOURCE_PICOBOOT,
};

struct dir_entry {
    uint8_t name[11];
    uint8_t attr;
    uint8_t reserved;
    uint8_t creation_time_frac;
    uint16_t creation_time;
    uint16_t creation_date;
    uint16_t last_access_date;
    uint16_t cluster_hi;
    uint16_t last_modified_time;
    uint16_t last_modified_date;
    uint16_t cluster_lo;
    uint32_t size;
};

static_assert(sizeof(struct dir_entry) == 32, "");

struct async_task;
typedef void (*async_task_callback)(struct async_task *task);



// copy by value task definition
struct async_task {
    uint32_t token;
    uint32_t result;
    async_task_callback callback;

    // we only have one  task type now, so inlining
    uint32_t transfer_addr;
    uint32_t erase_addr;
    uint32_t erase_size;
    uint8_t *data;
    uint32_t data_length;
    uint32_t picoboot_user_token;
    uint8_t type;
    uint8_t exclusive_param;
    // an identifier for the logical source of the task
    uint8_t source;
    // if true, fail the task if the source isn't the same as the last source that did a mutation
    bool check_last_mutation_source;
};

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
    struct async_task next_task;
    bool ram;
} _uf2_info;


// note caller must pass SECTOR_SIZE buffer
void init_dir_entry(struct dir_entry *entry, const char *fn, uint cluster, uint len) {
    entry->creation_time_frac = RASPBERRY_PI_TIME_FRAC;
    entry->creation_time = RASPBERRY_PI_TIME;
    entry->creation_date = RASPBERRY_PI_DATE;
    entry->last_modified_time = RASPBERRY_PI_TIME;
    entry->last_modified_date = RASPBERRY_PI_DATE;
    memcpy(entry->name, fn, 11);
    entry->attr = ATTR_READONLY | ATTR_ARCHIVE;
    entry->cluster_lo = cluster;
    entry->size = len;
}



#ifndef USB_BOOT_EXPANDED_RUNTIME
#define FLASH_VALID_BLOCKS_BASE XIP_SRAM_BASE
#else
#define FLASH_VALID_BLOCKS_BASE (SRAM_BASE + 96 * 1024)
#endif
#define FLASH_BITMAPS_SIZE (XIP_SRAM_END - XIP_SRAM_BASE)

#define AT_EXIT_XIP         0x01u
#define AT_FLASH_ERASE      0x02u
#define AT_READ             0x04u
#define AT_WRITE            0x08u
#define AT_EXCLUSIVE        0x10u
#define AT_ENTER_CMD_XIP    0x20u
#define AT_EXEC             0x40u
#define AT_VECTORIZE_FLASH  0x80u


#define FLASH_PAGE_MASK (FLASH_PAGE_SIZE - 1u)
#define FLASH_SECTOR_ERASE_SIZE 4096u



#define FLASH_MAX_VALID_BLOCKS ((FLASH_BITMAPS_SIZE * 8LL * FLASH_SECTOR_ERASE_SIZE / (FLASH_PAGE_SIZE + FLASH_SECTOR_ERASE_SIZE)) & ~31u)
#define FLASH_CLEARED_PAGES_BASE (FLASH_VALID_BLOCKS_BASE + FLASH_MAX_VALID_BLOCKS / 8)

static_assert(!(FLASH_CLEARED_PAGES_BASE & 0x3), "");

#define FLASH_MAX_CLEARED_PAGES (FLASH_MAX_VALID_BLOCKS * FLASH_PAGE_SIZE / FLASH_SECTOR_ERASE_SIZE)

static_assert(FLASH_CLEARED_PAGES_BASE + (FLASH_MAX_CLEARED_PAGES / 32 - FLASH_VALID_BLOCKS_BASE <= FLASH_BITMAPS_SIZE),"");




#define PIN_DBG1 19

// note these two macros may only be used once per complilation unit
#define CU_REGISTER_DEBUG_PINS(p...) enum __unused DEBUG_PIN_TYPE { _none = 0, p }; static enum DEBUG_PIN_TYPE __selected_debug_pins;
#define CU_SELECT_DEBUG_PINS(x) static enum DEBUG_PIN_TYPE __selected_debug_pins = (x);

// Drive high every GPIO appearing in mask
static inline void gpio_set_mask(uint32_t mask) {
    *(volatile uint32_t *) (SIO_BASE + SIO_GPIO_OUT_SET_OFFSET) = mask;
}

// Drive low every GPIO appearing in mask
static inline void gpio_clr_mask(uint32_t mask) {
    *(volatile uint32_t *) (SIO_BASE + SIO_GPIO_OUT_CLR_OFFSET) = mask;
}

// Toggle every GPIO appearing in mask
static inline void gpio_xor_mask(uint32_t mask) {
    *(volatile uint32_t *) (SIO_BASE + SIO_GPIO_OUT_XOR_OFFSET) = mask;
}

#define DEBUG_PINS_ENABLED(p) (__selected_debug_pins == (p))
#define DEBUG_PINS_SET(p, v) if (DEBUG_PINS_ENABLED(p)) gpio_set_mask((unsigned)(v)<<PIN_DBG1)
#define DEBUG_PINS_CLR(p, v) if (DEBUG_PINS_ENABLED(p)) gpio_clr_mask((unsigned)(v)<<PIN_DBG1)
#define DEBUG_PINS_XOR(p, v) if (DEBUG_PINS_ENABLED(p)) gpio_xor_mask((unsigned)(v)<<PIN_DBG1)



static struct {
    uint32_t serial_number32;
    bool serial_number_valid;
} boot_device_state;

uint32_t msc_get_serial_number32() {
    if (!boot_device_state.serial_number_valid) {
        boot_device_state.serial_number32 = to_ms_since_boot(get_absolute_time());
        boot_device_state.serial_number_valid = true;
    }
    return boot_device_state.serial_number32;
}


void __no_inline_not_in_flash_func(safe_reboot)(uint32_t addr, uint32_t sp, uint32_t delay_ms) {
	    watchdog_reboot(addr, sp, delay_ms);
}

static void __no_inline_not_in_flash_func(_write_uf2_page_complete)(struct async_task *task) {
        if (!task->result && _uf2_info.valid_block_count == _uf2_info.num_blocks) {
            safe_reboot(_uf2_info.ram ? _uf2_info.lowest_addr : 0, SRAM_END, 1000); //300); // reboot in 300 ms
        }
}



// note these are inclusive to save - 1 checks... we always test the start and end of a range, so the range would have to be zero length which we don't use
static inline bool is_address_ram(uint32_t addr) {
    // todo allow access to parts of USB ram?
    return (addr >= SRAM_BASE && addr <= SRAM_END) ||
           (addr >= XIP_SRAM_BASE && addr <= XIP_SRAM_END);
}

static inline bool is_address_flash(uint32_t addr) {
    // todo maybe smaller?
    return (addr >= XIP_MAIN_BASE && addr <= SRAM_BASE);
}

static inline bool is_address_rom(uint32_t addr) {
    return addr < 8192;
}


void memset0(void *dest, uint n) {
	memset(dest, 0, n);
}

CU_REGISTER_DEBUG_PINS(flash);



static uint32_t __no_inline_not_in_flash_func(_do_flash_enter_cmd_xip)();
static uint32_t __no_inline_not_in_flash_func(_do_flash_exit_xip)();
static uint32_t __no_inline_not_in_flash_func(_do_flash_erase_sector)(uint32_t addr);
static uint32_t __no_inline_not_in_flash_func(_do_flash_erase_range)(uint32_t addr, uint32_t len);
static uint32_t __no_inline_not_in_flash_func(_do_flash_page_program)(uint32_t addr, uint8_t *data);
static uint32_t __no_inline_not_in_flash_func(_do_flash_page_read)(uint32_t addr, uint8_t *data);

// keep table of flash function pointers in case RPI user wants to redirect them
static const struct flash_funcs {
    uint32_t size;
    uint32_t (*do_flash_enter_cmd_xip)();
    uint32_t (*do_flash_exit_xip)();
    uint32_t (*do_flash_erase_sector)();
    uint32_t (*do_flash_erase_range)(uint32_t addr, uint32_t size);
    uint32_t (*do_flash_page_program)(uint32_t addr, uint8_t *data);
    uint32_t (*do_flash_page_read)(uint32_t addr, uint8_t *data);
} default_flash_funcs = {
        .size = sizeof(struct flash_funcs),
        _do_flash_enter_cmd_xip,
        _do_flash_exit_xip,
        _do_flash_erase_sector,
        _do_flash_erase_range,
        _do_flash_page_program,
        _do_flash_page_read,
};

const struct flash_funcs *flash_funcs;



static uint32_t __no_inline_not_in_flash_func(_do_flash_enter_cmd_xip)() {
    _DBG("flash enter cmd XIP\n");
    //flash_enter_cmd_xip();
    return 0;
}

static uint32_t __no_inline_not_in_flash_func(_do_flash_exit_xip)() {
    _DBG("flash exit XIP\n");
    //DEBUG_PINS_SET(flash, 2);
    //connect_internal_flash();
    //DEBUG_PINS_SET(flash, 4);
    //flash_exit_xip();
    //DEBUG_PINS_CLR(flash, 6);
    return 0;
}

static uint32_t __no_inline_not_in_flash_func(_do_flash_erase_sector)(uint32_t addr) {
    _DBG("erasing flash sector @%08x\n", (uint) addr);
    //DEBUG_PINS_SET(flash, 2);
    //flash_sector_erase(addr - XIP_MAIN_BASE);
    flash_range_erase(addr, 4096);
    //DEBUG_PINS_CLR(flash, 2);
    return 0;
}

static uint32_t __no_inline_not_in_flash_func(_do_flash_erase_range)(uint32_t addr, uint32_t len) {
    uint32_t end = addr + len;
    uint32_t ret = PICOBOOT_OK;
    while (addr < end && !ret) {
        ret = flash_funcs->do_flash_erase_sector(addr);
        addr += FLASH_SECTOR_ERASE_SIZE;
    }
    return ret;
}

static uint32_t __no_inline_not_in_flash_func(_do_flash_page_program)(uint32_t addr, uint8_t *data) {
    _DBG("writing flash page @%08x\n", (uint) addr);
    //DEBUG_PINS_SET(flash, 4);
    //flash_page_program(addr - XIP_MAIN_BASE, data);
    flash_range_program(addr, data, 256);
    //DEBUG_PINS_CLR(flash, 4);
    // todo set error result
    return 0;
}

static uint32_t __no_inline_not_in_flash_func(_do_flash_page_read)(uint32_t addr, uint8_t *data) {
    DEBUG_PINS_SET(flash, 4);
    _DBG("reading flash page @%08x\n", (uint) addr);
    //flash_read_data(addr - XIP_MAIN_BASE, data, FLASH_PAGE_SIZE);
    //DEBUG_PINS_CLR(flash, 4);
    // todo set error result
    return 0;
}

static uint8_t _last_mutation_source;


static uint32_t __no_inline_not_in_flash_func(_execute_task)(struct async_task *task) {
    uint32_t ret;
    uint type = task->type;
    /*
    if (type & AT_VECTORIZE_FLASH) {
        if (task->transfer_addr & 1u) {
            return PICOBOOT_BAD_ALIGNMENT;
        }
        if (_is_address_safe_for_vectoring(task->transfer_addr) &&
            _is_address_safe_for_vectoring(task->transfer_addr + sizeof(struct flash_funcs))) {
            memcpy((void *) task->transfer_addr, &default_flash_funcs, sizeof(struct flash_funcs));
            flash_funcs = (struct flash_funcs *) task->transfer_addr;
        } else {
            return PICOBOOT_INVALID_ADDRESS;
        }
    }*/
    if (type & AT_EXCLUSIVE) {
        // we do this in executex task, so we know we aren't executing and virtual_disk_queue tasks at this moment
        _DBG("SET EXCLUSIVE ACCESS %d\n", task->exclusive_param);
    }
    if (type & AT_EXIT_XIP) {
        ret = flash_funcs->do_flash_exit_xip();
        if (ret) return ret;
    }
    if (type & AT_EXEC) {
        _DBG("exec %08x\n", (uint) task->transfer_addr);
        // scary but true; note callee must not overflow our stack (note also we reuse existing field task->transfer_addr to save code/data space)
        (((void (*)()) (task->transfer_addr | 1u)))();
    }
    if (type & (AT_WRITE | AT_FLASH_ERASE)) {
        if (task->check_last_mutation_source && _last_mutation_source != task->source) {
            return PICOBOOT_INTERLEAVED_WRITE;
        }
        _last_mutation_source = task->source;
    }
    if (type & AT_FLASH_ERASE) {
        _DBG("request flash erase at %08x+%08x\n", (uint) task->erase_addr, (uint) task->erase_size);
        // todo maybe remove to save space
        if (task->erase_addr & (FLASH_SECTOR_ERASE_SIZE - 1)) return PICOBOOT_BAD_ALIGNMENT;
        if (task->erase_size & (FLASH_SECTOR_ERASE_SIZE - 1)) return PICOBOOT_BAD_ALIGNMENT;
        if (!(is_address_flash(task->erase_addr) && is_address_flash(task->erase_addr + task->erase_size))) {
            return PICOBOOT_INVALID_ADDRESS;
        }
        ret = flash_funcs->do_flash_erase_range(task->erase_addr, task->erase_size);
        if (ret) return ret;
    }
    bool direct_access = false;
    if (type & (AT_WRITE | AT_READ)) {
        if ((is_address_ram(task->transfer_addr) && is_address_ram(task->transfer_addr + task->data_length))
            #ifndef NO_ROM_READ
            || (!(type & AT_WRITE) && is_address_rom(task->transfer_addr) &&
                is_address_rom(task->transfer_addr + task->data_length))
#endif
                ) {
            direct_access = true;
        } else if ((is_address_flash(task->transfer_addr) &&
                    is_address_flash(task->transfer_addr + task->data_length))) {
            // flash
            if (task->transfer_addr & (FLASH_PAGE_SIZE - 1)) return PICOBOOT_BAD_ALIGNMENT;
        } else {
            // bad address
            return PICOBOOT_INVALID_ADDRESS;
        }
        if (type & AT_WRITE) {
            if (direct_access) {
                _DBG("writing %08x +%04x\n", (uint) task->transfer_addr, (uint) task->data_length);
                uint32_t ff = (uintptr_t) flash_funcs;
                if (MAX(ff, task->transfer_addr) <
                    MIN(ff + sizeof(struct flash_funcs), task->transfer_addr + task->data_length)) {
                    _DBG("RAM write overlaps vectors, reverting them to ROM\n");
                    flash_funcs = &default_flash_funcs;
                }
                memcpy((void *) task->transfer_addr, task->data, task->data_length);
            } else {
                assert(task->data_length <= FLASH_PAGE_SIZE);
                ret = flash_funcs->do_flash_page_program(task->transfer_addr, task->data);
                if (ret) return ret;
            }
        }
        if (type & AT_READ) {
            if (direct_access) {
                _DBG("reading %08x +%04x\n", (uint) task->transfer_addr, (uint) task->data_length);
                memcpy(task->data, (void *) task->transfer_addr, task->data_length);
            } else {
                assert(task->data_length <= FLASH_PAGE_SIZE);
                ret = flash_funcs->do_flash_page_read(task->transfer_addr, task->data);
                if (ret) return ret;
            }
        }
        if (type & AT_ENTER_CMD_XIP) {
            ret = flash_funcs->do_flash_enter_cmd_xip();
            if (ret) return ret;
        }
    }
    return PICOBOOT_OK;
}

static void _task_copy(struct async_task *to, struct async_task *from) {
	memcpy(to, from, sizeof(struct async_task));
}


void __no_inline_not_in_flash_func(sync_execute_task)(struct async_task *task, async_task_callback callback) {
	task->callback = callback;

	assert(!task->result); // convention is that it is zero, so try to catch missing reset
	task->result = _execute_task(task);

	if (task->callback) task->callback(task);
}



// return true for async
static bool __no_inline_not_in_flash_func(_write_uf2_page)() {
    // If we need to write a page (i.e. it hasn't been written before, then we queue a task to do that asynchronously
    //
    // Note that in an ideal world, given that we aren't synchronizing with the task in any way from here on,
    // we'd hand that task an immutable work item so that we don't step on the task's toes later.
    //
    // In the constrained bootrom (no RAM use) environment we don't have space to do that, so instead we pass
    // it a work item which is immutable except for the data buffer to be written.
    //
    // Note that we also pre-update all _uf2_info state in anticipation of the write being completed. This saves us
    // doing some extra figuring in _write_uf2_page_complete later, and there are only two cases we care about
    //
    // 1) that the task fails, in which case we'll notice in _write_uf2_page_complete anyway, and we can reset.
    // 2) that we superseded what the task was doing with a new UF2 download, in which case the old state is irrelevant.
    //
    // So basically the rule is, that this method (and _write_uf2_page_complete) which are both called under our
    // pseudo-lock (i.e. during IRQ or with IRQs disabled) are the onlu things that touch UF2 tracking state...
    // the task just takes an immutable command (with possibly mutable data), and takes care of writing that data to FLASH or RAM
    // along with erase etc.
    _DBG("_write_uf2_page tok %d block %d / %d\n", (int) _uf2_info.token, _uf2_info.block_no, (int) _uf2_info.num_blocks);
    uint block_offset = _uf2_info.block_no / 32;
    uint32_t block_mask = 1u << (_uf2_info.block_no & 31u);
    if (!(_uf2_info.valid_blocks[block_offset] & block_mask)) {
        // note we don't want to pick XIP_CACHE over RAM even though it has a lower address
        bool xip_cache_next = _uf2_info.next_task.transfer_addr < SRAM_BASE;
        bool xip_cache_lowest = _uf2_info.lowest_addr < SRAM_BASE;
        if ((_uf2_info.next_task.transfer_addr < _uf2_info.lowest_addr && xip_cache_next == xip_cache_lowest) || (xip_cache_lowest && !xip_cache_next)) {
            _uf2_info.lowest_addr = _uf2_info.next_task.transfer_addr;
        }
        if (_uf2_info.ram) {
            assert(_uf2_info.next_task.transfer_addr);
        } else {
            uint page_no = _uf2_info.block_no * 256 / FLASH_SECTOR_ERASE_SIZE;
            assert(_uf2_info.cleared_pages);
            assert(page_no < _uf2_info.max_cleared_pages);
            uint page_offset = page_no / 32;
            uint32_t page_mask = 1u << (page_no & 31u);
            assert(page_offset <= _uf2_info.max_cleared_pages);
            if (!(_uf2_info.cleared_pages[page_offset] & page_mask)) {
                _uf2_info.next_task.erase_addr = _uf2_info.next_task.transfer_addr & ~(FLASH_SECTOR_ERASE_SIZE - 1u);
                _uf2_info.next_task.erase_size = FLASH_SECTOR_ERASE_SIZE; // always erase a single sector
                _DBG("Setting erase addr %08x\n", (uint) _uf2_info.next_task.erase_addr);
                _uf2_info.cleared_pages[page_offset] |= page_mask;
                _uf2_info.next_task.type |= AT_FLASH_ERASE;
            }
            _DBG("Have flash destined page %08x (%08x %08x)\n", (uint) _uf2_info.next_task.transfer_addr,
                      (uint) *(uint32_t *) _uf2_info.next_task.data,
                      (uint) *(uint32_t *) (_uf2_info.next_task.data + 4));
            assert(!(_uf2_info.next_task.transfer_addr & 0xffu));
        }
        _uf2_info.valid_block_count++;
        _uf2_info.valid_blocks[block_offset] |= block_mask;
        _DBG("Queuing 0x%08x->0x%08x valid %d/%d checked %d/%d\n", (uint)
                (uint) _uf2_info.next_task.transfer_addr, (uint) (_uf2_info.next_task.transfer_addr + FLASH_PAGE_SIZE),
                 (uint) _uf2_info.block_no + 1u, (uint) _uf2_info.num_blocks, (uint) _uf2_info.valid_block_count,
                 (uint) _uf2_info.num_blocks);

        sync_execute_task(&_uf2_info.next_task, _write_uf2_page_complete);
	
	
        // after the first write (i.e. next time, we want to check the source)
        _uf2_info.next_task.check_last_mutation_source = true;
        // note that queue_task may actually be handled sychronously based on #define, however that is OK
        // because it still calls _write_uf2_page_complete which still calls vd_async_complete which is allowed even in non async.
        return true;
    } else {
        assert(_uf2_info.next_task.type); // we should not have had any valid blocks after reset... we must take the above path so that the task gets executed
        _DBG("Ignore duplicate write to 0x%08x->0x%08x\n", (uint) _uf2_info.next_task.transfer_addr, (uint) (_uf2_info.next_task.transfer_addr + FLASH_PAGE_SIZE));
    }
    return false; // not async
}

static void _clear_bitset(uint32_t *mask, uint32_t count) {
	    memset0(mask, count / 8);
}

void reset_task(struct async_task *task) {
	memset0(task, sizeof(struct async_task));
}
void __no_inline_not_in_flash_func(tumt_periodic_task)(void);

static bool __no_inline_not_in_flash_func(_update_current_uf2_info)(struct uf2_block *uf2) {
	bool ram = is_address_ram(uf2->target_addr) && is_address_ram(uf2->target_addr + (FLASH_PAGE_MASK));

	bool flash = is_address_flash(uf2->target_addr) && is_address_flash(uf2->target_addr + (FLASH_PAGE_MASK));

	if (!(uf2->num_blocks && (ram || flash)) || (flash && (uf2->target_addr & (FLASH_PAGE_MASK)))) {
		_DBG("Resetting active UF2 transfer because received garbage\n");
	} else if (!0 /*virtual_disk_queue.disable*/) {
		uint8_t type = AT_WRITE; // we always write
		if (_uf2_info.num_blocks != uf2->num_blocks) {
			_DBG("Resetting active UF2 transfer because have new binary size %d->%d\n", (int) _uf2_info.num_blocks, (int) uf2->num_blocks);
			memset(&_uf2_info, 0x0, sizeof(_uf2_info));
			_uf2_info.ram = ram;
			_uf2_info.valid_blocks = ram ? uf2_valid_ram_blocks : (uint32_t *) FLASH_VALID_BLOCKS_BASE;
			_uf2_info.max_valid_blocks = ram ? count_of(uf2_valid_ram_blocks) * 32 : FLASH_MAX_VALID_BLOCKS;
			_DBG("  ram %d, so valid_blocks (max %d) %p->%p for %dK\n", ram, (int) _uf2_info.max_valid_blocks, _uf2_info.valid_blocks, _uf2_info.valid_blocks + ((_uf2_info.max_valid_blocks + 31) / 32), (uint) _uf2_info.max_valid_blocks / 4);

			//_clear_bitset(_uf2_info.valid_blocks, _uf2_info.max_valid_blocks);
			_DBG(" post _clear_bitset()");
			if (flash) {
				_uf2_info.cleared_pages = (uint32_t *) FLASH_CLEARED_PAGES_BASE;
				_uf2_info.max_cleared_pages = FLASH_MAX_CLEARED_PAGES;
				_DBG("    cleared_pages %p->%p\n", _uf2_info.cleared_pages,
				_uf2_info.cleared_pages + ((_uf2_info.max_cleared_pages + 31) / 32));
				//_clear_bitset(_uf2_info.cleared_pages, _uf2_info.max_cleared_pages);
			}
	
			if (uf2->num_blocks > _uf2_info.max_valid_blocks) {
				_DBG("Oops image requires %d blocks and won't fit", (uint) uf2->num_blocks);
				return false;
			}
			_DBG("New UF2 transfer\n");
			_uf2_info.num_blocks = uf2->num_blocks;
			_uf2_info.valid_block_count = 0;
			_uf2_info.lowest_addr = 0xffffffff;
		}

		if (ram != _uf2_info.ram) {
			_DBG("Ignoring write to out of range address 0x%08x->0x%08x\n", (uint) uf2->target_addr, (uint) (uf2->target_addr + uf2->payload_size));
	        } else {
			assert(uf2->num_blocks <= _uf2_info.max_valid_blocks);
			if (uf2->block_no <= uf2->num_blocks) {
				// set up next task state (also serves as a holder for state scoped to this block write to avoid copying data around)
				reset_task(&_uf2_info.next_task);
				_uf2_info.block_no = uf2->block_no;
				//_uf2_info.token = _uf2_info.next_task.token = token;
				_uf2_info.next_task.transfer_addr = uf2->target_addr;
				_uf2_info.next_task.type = type;
				_uf2_info.next_task.data = uf2->data;
				_uf2_info.next_task.callback = _write_uf2_page_complete;
				_uf2_info.next_task.data_length = FLASH_PAGE_SIZE; // always a full page
				_uf2_info.next_task.source = TASK_SOURCE_VIRTUAL_DISK;
				return true;
			} else {
				_DBG("Ignoring write to out of range block %d >= %d\n", (int) uf2->block_no, (int) uf2->num_blocks);
			}
		}
	}

	_uf2_info.num_blocks = 0; // invalid
	return false;
}




// Invoked when received SCSI_CMD_INQUIRY
// Application fill vendor id, product id and revision with string up to 8, 16, 4 characters respectively
void __no_inline_not_in_flash_func(tud_msc_inquiry_cb)(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4]){
	(void) lun;

	const char vid[] = "TinyUSB Multitool";
	const char pid[] = "Mass Storage";
	const char rev[] = "1.0";

	memcpy(vendor_id  , vid, strlen(vid));
	memcpy(product_id , pid, strlen(pid));
	memcpy(product_rev, rev, strlen(rev));
}

// Invoked when received Test Unit Ready command.
// return true allowing host to read/write this LUN e.g SD card inserted
bool __no_inline_not_in_flash_func(tud_msc_test_unit_ready_cb)(uint8_t lun){
	(void) lun;
	return true; // RAM disk is always ready
}

// Invoked when received SCSI_CMD_READ_CAPACITY_10 and SCSI_CMD_READ_FORMAT_CAPACITY to determine the disk size
// Application update block count and block size
void __no_inline_not_in_flash_func(tud_msc_capacity_cb)(uint8_t lun, uint32_t* block_count, uint16_t* block_size){
	(void) lun;

	*block_count = SECTOR_COUNT;
	*block_size  = SECTOR_SIZE;
}

// Invoked when received Start Stop Unit command
// - Start = 0 : stopped power mode, if load_eject = 1 : unload disk storage
// - Start = 1 : active mode, if load_eject = 1 : load disk storage
bool __no_inline_not_in_flash_func(tud_msc_start_stop_cb)(uint8_t lun, uint8_t power_condition, bool start, bool load_eject){
	(void) lun;
	(void) power_condition;

	if ( load_eject ){
		if (start){
		      // load disk storage
			_DBG("tud_msc_start_stop_cb() load");
		    }else{
		      // unload disk storage
			_DBG("tud_msc_start_stop_cb() eject");
		}
	}

	return true;
}



#define WELCOME_HTML "<html><head><meta http-equiv='refresh' content=\"0;URL='https://raspberrypi.com/device/RP2'\"/></head><body>Redirecting to <a href='https://raspberrypi.com/device/RP2'>raspberrypi.com</a></body></html>"
#define WELCOME_HTML_LEN strlen(WELCOME_HTML)

#define INFO_UF2_TXT_MODEL "LEDMCUMZ16163-A"
#define INFO_UF2_TXT "TinyUSB Multitool UF2 loader v1.0\r\nModel:" INFO_UF2_TXT_MODEL " \r\nBoard-ID: RPI-RP2\r\n"

#define INFO_UF2_TXT_LEN strlen(INFO_UF2_TXT)


// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and return number of copied bytes.
int32_t __no_inline_not_in_flash_func(tud_msc_read10_cb)(uint8_t lun, uint32_t lba, uint32_t offset, void* buf, uint32_t bufsize)
{
  	(void) lun;
	assert(buf_size >= SECTOR_SIZE);
    	memset(buf, 0x0, SECTOR_SIZE);

	if (!lba) {
		uint8_t *ptable = buf + SECTOR_SIZE - 2 - 64;
		static_assert(!((SECTOR_COUNT - 1u) >> 24), "");
	        static const uint8_t _ptable_data4[] = {
	                PT_FAT16_LBA, 0, 0, 0,
	                lsb_word(1), // sector 1
	                // sector count, but we know the MS byte is zero
	                (SECTOR_COUNT - 1u) & 0xffu,
	                ((SECTOR_COUNT - 1u) >> 8u) & 0xffu,
                	((SECTOR_COUNT - 1u) >> 16u) & 0xffu,
        	};
		memcpy(ptable + 4, _ptable_data4, sizeof(_ptable_data4));

		ptable[64] = 0x55;
	        ptable[65] = 0xaa;

	        uint32_t sn = msc_get_serial_number32();
	        memcpy(buf + MBR_OFFSET_SERIAL_NUMBER, &sn, 4);
	        return bufsize;
	}
	lba--;
	if (!lba) {
	        uint32_t sn = msc_get_serial_number32();
	        memcpy(buf, boot_sector, sizeof(boot_sector));
	        memcpy(buf + BOOT_OFFSET_SERIAL_NUMBER, &sn, 4);
	} else {
        	lba--;
		if (lba < SECTORS_PER_FAT * FAT_COUNT) {
			// mirror
			while (lba >= SECTORS_PER_FAT) lba -= SECTORS_PER_FAT;
			if (!lba) {
			uint16_t *p = (uint16_t *) buf;
				p[0] = 0xff00u | MEDIA_TYPE;
				p[1] = 0xffff;
				p[2] = 0xffff; // cluster2 is index.htm
				p[3] = 0xffff; // cluster3 is info_uf2.txt
			}
		}else{
			lba -= SECTORS_PER_FAT * FAT_COUNT;
			if (lba < ROOT_DIRECTORY_SECTORS) {
				// we don't support that many directory entries actually
		                if (!lba) {
					// root directory
		        		struct dir_entry *entries = (struct dir_entry *) buf;
					memcpy(entries[0].name, (boot_sector + BOOT_OFFSET_LABEL), 11);
					entries[0].attr = ATTR_VOLUME_LABEL | ATTR_ARCHIVE;
					init_dir_entry(++entries, "INDEX   HTM", 2, WELCOME_HTML_LEN);
					init_dir_entry(++entries, "INFO_UF2TXT", 3, INFO_UF2_TXT_LEN);
				}
			}else{
				lba -= ROOT_DIRECTORY_SECTORS;
				uint cluster = lba >> CLUSTER_SHIFT;
				uint cluster_offset = lba - (cluster << CLUSTER_SHIFT);
				if (!cluster_offset) {
					if (cluster == 0) {
						memcpy(buf, WELCOME_HTML, WELCOME_HTML_LEN);
					}else if( cluster == 1){
						memcpy(buf, INFO_UF2_TXT, INFO_UF2_TXT_LEN);
					}
				}
			}
		}
	}
				
	return bufsize;
}

void __no_inline_not_in_flash_func(tumt_periodic_task)(void);

#define FLASH_OFFSET 1*1024*1024
// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and return number of written bytes
int32_t __no_inline_not_in_flash_func(tud_msc_write10_cb)(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t* buf, uint32_t bufsize){
	(void) lun;
	struct uf2_block *uf2 = (struct uf2_block *) buf;
	if (uf2->magic_start0 == UF2_MAGIC_START0 && uf2->magic_start1 == UF2_MAGIC_START1 && uf2->magic_end == UF2_MAGIC_END) {
		if (uf2->flags & UF2_FLAG_FAMILY_ID_PRESENT && uf2->file_size == RP2040_FAMILY_ID && !(uf2->flags & UF2_FLAG_NOT_MAIN_FLASH) && uf2->payload_size == 256) {
			if (_update_current_uf2_info(uf2)) {
				// if we have a valid uf2 page, write it
				tumt_periodic_task();

				_DBG("Write: %d, %d (%d/%d)", uf2->target_addr - XIP_MAIN_BASE, uf2->payload_size, uf2->block_no, uf2->num_blocks);
				uint32_t save = save_and_disable_interrupts();
				if((uf2->target_addr - XIP_MAIN_BASE+FLASH_OFFSET) % 4096 == 0){
					_DBG("Flash erase");
					flash_range_erase(uf2->target_addr - XIP_MAIN_BASE+FLASH_OFFSET, 4096);
				}
				_DBG("flash_range_program(%lu, [], %d)",(uf2->target_addr - XIP_MAIN_BASE+FLASH_OFFSET), uf2->payload_size);
				flash_range_program(uf2->target_addr - XIP_MAIN_BASE+FLASH_OFFSET, uf2->data, uf2->payload_size);
				restore_interrupts(save);

				tumt_periodic_task();

				
				//if(uf2->block_no == uf2->num_blocks){
				//	safe_reboot(_uf2_info.ram ? _uf2_info.lowest_addr : 0, SRAM_END, 1000);
				///}
				return bufsize;

			}
		} else {
			_DBG("Sector %d: ignoring write of non Mu UF2 sector\n", (uint) lba);
		}
	} else {
		/*
		 * struct uf2_block {
		 *     // 32 byte header
		 *     uint32_t magic_start0;
		 *     uint32_t magic_start1;
		 *     uint32_t flags;
		 *     uint32_t target_addr;
		 *     uint32_t payload_size;
		 *     uint32_t block_no;
		 *     uint32_t num_blocks;
		 *     uint32_t file_size; // or familyID;
		 *     uint8_t  data[476];
		 *     uint32_t magic_end;
		 *    };
		 */
		_DBG("Sector %d: ignoring write of non UF2 sector [0x%02x, 0x%02x, 0x%02x, 0x%02x, %d, %d, %d, %d, 0x%02x]\n", (uint) lba, uf2->magic_start0, uf2->magic_start1, uf2->flags, uf2->target_addr, uf2->payload_size, uf2->block_no, uf2->num_blocks, uf2->file_size, uf2->magic_end);
	}

	return bufsize;
}

// Callback invoked when received an SCSI command not in built-in list below
// - READ_CAPACITY10, READ_FORMAT_CAPACITY, INQUIRY, MODE_SENSE6, REQUEST_SENSE
// - READ10 and WRITE10 has their own callbacks
int32_t __no_inline_not_in_flash_func(tud_msc_scsi_cb)(uint8_t lun, uint8_t const scsi_cmd[16], void* buffer, uint16_t bufsize)
{
  // read10 & write10 has their own callback and MUST not be handled here

  void const* response = NULL;
  uint16_t resplen = 0;

  // most scsi handled is input
  bool in_xfer = true;

  switch (scsi_cmd[0])
  {
    case SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
      // Host is about to read/write etc ... better not to disconnect disk
      resplen = 0;
    break;

    default:
      // Set Sense = Invalid Command Operation
      tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);

      // negative means error -> tinyusb could stall and/or response with failed status
      resplen = -1;
    break;
  }

  // return resplen must not larger than bufsize
  if ( resplen > bufsize ) resplen = bufsize;

  if ( response && (resplen > 0) )
  {
    if(in_xfer)
    {
      memcpy(buffer, response, resplen);
    }else
    {
      // SCSI output
    }
  }

  return resplen;
}

#endif
