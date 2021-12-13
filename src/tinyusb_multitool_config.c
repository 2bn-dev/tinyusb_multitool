
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

#define PARAM_ASSERTIONS_ENABLED_FLASH 1

#include "tusb.h"
#include "tinyusb_multitool_debug.h"
#include "boot/uf2.h"
#include "hardware/watchdog.h"
#include <stdlib.h>
#include "pico/bootrom.h"
#include "flash.h" // Modified hardware/flash.h library, may not be necessary, further testing.

#include "unique_id_modified.h"
//#include "pico/unique_id.h" // We use our modified unique_id library because the standard one includes hardware/flash.h
#include "tumt_usb_config.h"
#include "pico/multicore.h"
#include "pico/time.h"
#include "tinyusb_multitool.h"

void __no_inline_not_in_flash_func(flash_erase_and_move_core1_entry)();
int64_t __no_inline_not_in_flash_func(flash_erase_and_move)(void *user_data);


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

	TUD_CDC_DESCRIPTOR(USBD_ITF_CDC_STDIO, USBD_STR_CDC_STDIO	, USBD_CDC_STDIO_EP_CMD, USBD_CDC_CMD_MAX_SIZE,	USBD_CDC_STDIO_EP_OUT, USBD_CDC_STDIO_EP_IN, USBD_CDC_IN_OUT_MAX_SIZE),
	TUD_CDC_DESCRIPTOR(USBD_ITF_CDC_UART0, USBD_STR_CDC_UART0	, USBD_CDC_UART0_EP_CMD, USBD_CDC_CMD_MAX_SIZE, USBD_CDC_UART0_EP_OUT, USBD_CDC_UART0_EP_IN, USBD_CDC_IN_OUT_MAX_SIZE),
	TUD_CDC_DESCRIPTOR(USBD_ITF_CDC_UART1, USBD_STR_CDC_UART1	, USBD_CDC_UART1_EP_CMD, USBD_CDC_CMD_MAX_SIZE, USBD_CDC_UART1_EP_OUT, USBD_CDC_UART1_EP_IN, USBD_CDC_IN_OUT_MAX_SIZE),
	TUD_MSC_DESCRIPTOR(USBD_ITF_MSC	     , USBD_STR_MSC		, 						USBD_MSC_EP_OUT	     , USBD_MSC_EP_IN      , USBD_MSC_IN_OUT_MAX_SIZE),

};

static char usbd_serial_str[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1];

static const char *const usbd_desc_str[] = {
    [USBD_STR_MANUF] = "2bn Development",
    [USBD_STR_PRODUCT] = "TinyUSB Multitool",
    [USBD_STR_SERIAL] = usbd_serial_str,
    [USBD_STR_MSC] = "TinyUSB Multitool MSC",
    [USBD_STR_CDC_STDIO] = "TinyUSB Multitool CDC STDIO",
    [USBD_STR_CDC_UART0] = "TinyUSB Multitool CDC uart0 Bridge",
    [USBD_STR_CDC_UART1] = "TinyUSB Multitool CDC uart1 Bridge",

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




#ifndef USB_BOOT_EXPANDED_RUNTIME
#define FLASH_VALID_BLOCKS_BASE XIP_SRAM_BASE
#else
#define FLASH_VALID_BLOCKS_BASE (SRAM_BASE + 96 * 1024)
#endif
#define FLASH_BITMAPS_SIZE (XIP_SRAM_END - XIP_SRAM_BASE)

#define FLASH_PAGE_MASK (FLASH_PAGE_SIZE - 1u)
#define FLASH_SECTOR_ERASE_SIZE 4096u

#define FLASH_MAX_VALID_BLOCKS ((FLASH_BITMAPS_SIZE * 8LL * FLASH_SECTOR_ERASE_SIZE / (FLASH_PAGE_SIZE + FLASH_SECTOR_ERASE_SIZE)) & ~31u)
#define FLASH_CLEARED_PAGES_BASE (FLASH_VALID_BLOCKS_BASE + FLASH_MAX_VALID_BLOCKS / 8)

static_assert(!(FLASH_CLEARED_PAGES_BASE & 0x3), "");

#define FLASH_MAX_CLEARED_PAGES (FLASH_MAX_VALID_BLOCKS * FLASH_PAGE_SIZE / FLASH_SECTOR_ERASE_SIZE)

static_assert(FLASH_CLEARED_PAGES_BASE + (FLASH_MAX_CLEARED_PAGES / 32 - FLASH_VALID_BLOCKS_BASE <= FLASH_BITMAPS_SIZE),"");


#define __not_in_flash(group) __attribute__((section(".time_critical." group)))
#define __not_in_flash_func(func_name) __not_in_flash(__STRING(func_name)) func_name
#define __no_inline_not_in_flash_func(func_name) __noinline __not_in_flash_func(func_name)

// give us ourselves 64M which should strictly be the minimum for FAT16

#define lsb_hword(x) (((uint)(x)) & 0xffu), ((((uint)(x))>>8u)&0xffu)
#define lsb_word(x) (((uint)(x)) & 0xffu), ((((uint)(x))>>8u)&0xffu),  ((((uint)(x))>>16u)&0xffu),  ((((uint)(x))>>24u)&0xffu)


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
	return 	(addr >= SRAM_BASE && addr <= SRAM_END) ||
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


static void _clear_bitset(uint32_t *mask, uint32_t count) {
	memset0(mask, count / 8);
}

void reset_task(struct async_task *task) {
	memset0(task, sizeof(struct async_task));
}
static bool __no_inline_not_in_flash_func(_update_current_uf2_info)(struct uf2_block *uf2) {
	bool ram = is_address_ram(uf2->target_addr) && is_address_ram(uf2->target_addr + (FLASH_PAGE_MASK));

	bool flash = is_address_flash(uf2->target_addr) && is_address_flash(uf2->target_addr + (FLASH_PAGE_MASK));

	if (!(uf2->num_blocks && (ram || flash)) || (flash && (uf2->target_addr & (FLASH_PAGE_MASK)))) {
		_DBG("Resetting active UF2 transfer because received garbage\n");
	} else if (!0 /*virtual_disk_queue.disable*/) {
		if (_uf2_info.num_blocks != uf2->num_blocks) {
			_DBG("Resetting active UF2 transfer because have new binary size %d->%d\n", (int) _uf2_info.num_blocks, (int) uf2->num_blocks);
			_DBG("flash_pending_cb()");
			flash_pending_cb();
			_DBG("flash_pending_cb() complete");
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



// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and return number of copied bytes.
int32_t __no_inline_not_in_flash_func(tud_msc_read10_cb)(uint8_t lun, uint32_t lba, uint32_t offset, void* buf, uint32_t bufsize){
  	(void) lun;
	assert(bufsize >= SECTOR_SIZE);
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
				p[4] = 0xffff; // cluster4 is readme.txt
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
					init_dir_entry(++entries, "README  TXT", 4, README_TXT_LEN);
				}
			}else{
				lba -= ROOT_DIRECTORY_SECTORS;
				uint cluster = lba >> CLUSTER_SHIFT;
				uint cluster_offset = lba - (cluster << CLUSTER_SHIFT);
				if (!cluster_offset) {
					if (cluster == 0) {
						memcpy(buf, WELCOME_HTML, WELCOME_HTML_LEN);
					}else if(cluster == 1){
						memcpy(buf, INFO_UF2_TXT, INFO_UF2_TXT_LEN);
					}else if(cluster == 2){
						memcpy(buf, README_TXT, README_TXT_LEN);
					}
				}
			}
		}
	}
				
	return bufsize;
}

void tumt_periodic_task();


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

				//_DBG("Write: %d, %d (%d/%d)", uf2->target_addr - XIP_MAIN_BASE, uf2->payload_size, uf2->block_no, uf2->num_blocks);
				uint32_t save = save_and_disable_interrupts();
				if((uf2->target_addr - XIP_MAIN_BASE+FLASH_OFFSET) % 4096 == 0){
					//_DBG("Flash erase");
					flash_range_erase(uf2->target_addr - XIP_MAIN_BASE+FLASH_OFFSET, 4096, true);
				}
				//_DBG("flash_range_program(%lu, [], %d)",(uf2->target_addr - XIP_MAIN_BASE+FLASH_OFFSET), uf2->payload_size);
				flash_range_program(uf2->target_addr - XIP_MAIN_BASE+FLASH_OFFSET, uf2->data, uf2->payload_size, true);
				restore_interrupts(save);

				tumt_periodic_task();

				
				if(uf2->block_no == uf2->num_blocks-1){ //block_no is 0 indexed
					uint32_t * arguments = (void *) malloc(sizeof(uint32_t)*3);
					memset(arguments, 0x0, sizeof(uint32_t)*3);
					arguments[0] = 0x0;
					arguments[1] = 0x0+FLASH_OFFSET;
					arguments[2] = ((uf2->target_addr - XIP_MAIN_BASE)+uf2->payload_size);
					//bool rc = add_alarm_in_ms(100, flash_erase_and_move,  arguments, true);
					flash_erase_and_move(arguments);
				} 
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

uint32_t * __no_inline_not_in_flash_func(__memcpy_ram)(uint8_t * address_to, const uint8_t * address_from, uint32_t length){
	rom_memcpy_fn rom_memcpy = (rom_memcpy_fn)rom_func_lookup_inline(ROM_FUNC_MEMCPY);
	assert(rom_memcpy);
	rom_memcpy(address_to, address_from, length);
}

uint32_t * __no_inline_not_in_flash_func(__memset_ram)(uint8_t * address, uint8_t value, uint32_t length){
	rom_memset_fn rom_memset = (rom_memset_fn)rom_func_lookup_inline(ROM_FUNC_MEMSET);
	assert(rom_memset);
	rom_memset(address, value, length);
}

int64_t __no_inline_not_in_flash_func(flash_erase_and_move)(void *user_data){
	multicore_reset_core1();
	multicore_launch_core1(flash_erase_and_move_core1_entry);
	multicore_fifo_push_blocking((uint32_t)user_data);
	return 0;
}

void __no_inline_not_in_flash_func(flash_erase_and_move_core1_entry)(){
	uint32_t *user_data = (uint32_t *) multicore_fifo_pop_blocking();
	multicore_lockout_start_timeout_us(10*1000*1000); 
	uint32_t save = save_and_disable_interrupts();
	busy_wait_ms(1000);

	uint32_t * user_data2 = (uint32_t *) user_data;

	uint32_t flash1_start = user_data2[0];
	uint32_t flash2_start = user_data2[1];
	uint32_t flash2_len = user_data2[2];
	free(user_data);
	
	uint8_t buf[4096];



	_DBG("user_data: %lu, %lu, %lu", flash1_start, flash2_start, flash2_len);
	_DBG("No further logging beyond this point, flash rewrite beginning, if successful device will reboot");
	//Logging is not possible, because we're rewriting flash, and the memory addresses of functions and strings will be erased and replaced.
	//I've tried a few ways to work around this, and I think basically the only way would be to have a ram only image that gets loaded to handle this process.

	//Theoretically copy flash2 to flash1 in stage2 bootloader mode-ish using hardware/flash.h
	uint32_t save = save_and_disable_interrupts();

	for(uint32_t i = flash1_start; i < flash1_start+flash2_len; i+=256){
		if(i%4096 == 0){
			flash_range_erase(i, 4096, true);
		}


		__memset_ram(buf, 0x0, 4096);
		uint8_t *ptr_src = (void *) (i-flash1_start+flash2_start+XIP_NOCACHE_NOALLOC_BASE);
		__memcpy_ram(buf, ptr_src, 256);
		
		flash_range_program(i, buf, 256, true);

	}
	//restore_interrupts(save);
	safe_reboot(0, SRAM_END, 1000);
	return;
}

// Callback invoked when received an SCSI command not in built-in list below
// - READ_CAPACITY10, READ_FORMAT_CAPACITY, INQUIRY, MODE_SENSE6, REQUEST_SENSE
// - READ10 and WRITE10 has their own callbacks
int32_t __no_inline_not_in_flash_func(tud_msc_scsi_cb)(uint8_t lun, uint8_t const scsi_cmd[16], void* buffer, uint16_t bufsize){
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

  if ( response && (resplen > 0) ){
    if(in_xfer){
      memcpy(buffer, response, resplen);
    }else{
      // SCSI output
    }
  }

  return resplen;
}

