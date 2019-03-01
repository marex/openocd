/*
 * RTL8196E SPI NOR driver
 * Copyright (C) 2019 Marek Vasut <marek.vasut@gmail.com>
 *
 * Based on ATH79 SPI NOR driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the
 * Free Software Foundation, Inc.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "spi.h"
#include <jtag/jtag.h>
#include <helper/time_support.h>
#include <helper/types.h>
#include <target/mips32.h>
#include <target/mips32_pracc.h>
#include <target/target.h>

#define BITS_PER_BYTE 8

#define RTL8196_REG_FS     0
#define RTL8196_REG_CLOCK  4
#define RTL8196_REG_WRITE  8
#define RTL8196_REG_DATA  12

#define RTL8196_SPI_CS_ALLHI 0x70000
#define RTL8196_SPI_CS0_HI   0x10000
#define RTL8196_SPI_CS1_HI   0x20000
#define RTL8196_SPI_CE_HI    0x00100
#define RTL8196_SPI_DO_HI    0x00001

#define RTL8196_XFER_FINAL   0x00000001
#define RTL8196_XFER_PARTIAL 0x00000000

/* Timeout in ms */
#define RTL8196_MAX_TIMEOUT  (3000)

struct rtl8196_spi_ctx {
	uint8_t *page_buf;
	int pre_deselect;
	int post_deselect;
};

struct rtl8196_flash_bank {
	int probed;
	int chipselect;
	uint32_t io_base;
	const struct flash_device *dev;
	struct rtl8196_spi_ctx spi;
};

struct rtl8196_target {
	char *name;
	uint32_t tap_idcode;
	uint32_t io_base;
};

static const struct rtl8196_target target_devices[] = {
	/* name,	tap_idcode, io_base */
	{ "RTL8196",	0x1438000d, 0xb8001200 },
	{ NULL,		0,          0 }
};

FLASH_BANK_COMMAND_HANDLER(rtl8196_flash_bank_command)
{
	struct rtl8196_flash_bank *rtl8196_info;
	int chipselect = 0;

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC < 6 || CMD_ARGC > 7)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (CMD_ARGC == 7) {
		if (strcmp(CMD_ARGV[6], "cs0") == 0)
			chipselect = 0;  /* default */
		else if (strcmp(CMD_ARGV[6], "cs1") == 0)
			chipselect = 1;
		else {
			LOG_ERROR("Unknown arg: %s", CMD_ARGV[6]);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}

	rtl8196_info = calloc(1, sizeof(*rtl8196_info));
	if (!rtl8196_info) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	rtl8196_info->chipselect = chipselect;
	bank->driver_priv = rtl8196_info;

	return ERROR_OK;
}

#define SFCR			0x00
#define SFCR_SPI_CLK_DIV(val)	((val) << 29)
#define SFCR_RBO(val)		((val) << 28)
#define SFCR_WBO(val)		((val) << 27)
#define SFCR_SPI_TCS(val)	((val) << 22)

#define SFCR2			0x04
#define SFCR2_SFCMD(val)	((val) << 24)
#define SFCR2_SFSIZE(val)	((val) << 21)
#define SFCR2_RD_OPT(val)	((val) << 20)
#define SFCR2_CMD_IO(val)	((val) << 18)
#define SFCR2_ADDR_IO(val)	((val) << 16)
#define SFCR2_DUMMY_CYCLE(val)	((val) << 13)
#define SFCR2_DATA_IO(val)	((val) << 11)
#define SFCR2_HOLD_TILL_SFDR2(val)	((val) << 10)

#define SFCSR			0x08
#define SFCSR_SPI_CSB0(val)	((val) << 31)
#define SFCSR_SPI_CSB1(val)	((val) << 30)
#define SFCSR_LEN(val)		((val) << 28)
#define SFCSR_SPI_RDY(val)	((val) << 27)
#define SFCSR_IO_WIDTH(val)	((val) << 25)
#define SFCSR_CHIP_SEL(val)	((val) << 24)
#define SFCSR_CMD_BYTE(val)	((val) << 16)

#define SFCSR_SPI_CSB(val)	((val) << 30)

#define SFDR			0x0c

#define SFDR2			0x10

#define ___swab32(x) \
	((uint32_t)( \
		(((uint32_t)(x) & (uint32_t)0x000000ffUL) << 24) | \
		(((uint32_t)(x) & (uint32_t)0x0000ff00UL) <<  8) | \
		(((uint32_t)(x) & (uint32_t)0x00ff0000UL) >>  8) | \
		(((uint32_t)(x) & (uint32_t)0xff000000UL) >> 24) ))

static int rtl8196_write_page_data(struct flash_bank *bank,
				const uint32_t *buf32, int count)
{
	struct target *target = bank->target;
	struct rtl8196_flash_bank *rtl8196_info = bank->driver_priv;
	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	struct pracc_queue_info ctx = {.ejtag_info = ejtag_info};
	uint32_t addr = rtl8196_info->io_base + SFDR;
	uint32_t data;

	pracc_queue_init(&ctx);

	while (count) {
		ctx.code_count = 0;
		ctx.store_count = 0;

		int this_round_count = (count > 128) ? 128 : count;
		uint32_t last_upper_base_addr = UPPER16(addr) | 0x8000;
			      /* load $15 with memory base address */
		pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 15, last_upper_base_addr));

		for (int i = 0; i != this_round_count; i++) {
			data = ___swab32(*buf32);
			pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 8, UPPER16(data)));	/* load upper and lower */
			pracc_add(&ctx, 0, MIPS32_ORI(ctx.isa, 8, 8, LOWER16(data)));
			pracc_add(&ctx, 0, MIPS32_SW(ctx.isa, 8, LOWER16(addr), 15)); /* store word to mem */
			pracc_add(&ctx, 0, MIPS32_NOP);					/* nop in delay slot */
			buf32++;
		}

		pracc_add_li32(&ctx, 8, ejtag_info->reg8, 0);				/* restore $8 */
		pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 8, UPPER16(ejtag_info->reg8)));	/* load upper and lower */
		pracc_add(&ctx, 0, MIPS32_ORI(ctx.isa, 8, 8, LOWER16(ejtag_info->reg8)));

		pracc_add(&ctx, 0, MIPS32_B(ctx.isa, NEG16((ctx.code_count + 1) << ctx.isa)));	/* jump to start */
		pracc_add(&ctx, 0, MIPS32_MFC0(ctx.isa, 15, 18, 0));			/* restore $15 from DeSave */

		ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL, 1);
		if (ctx.retval != ERROR_OK)
			goto exit;
		count -= this_round_count;
	}
exit:
	pracc_queue_free(&ctx);
	return ctx.retval;
}

static int sfcsr_cs(struct flash_bank *bank, unsigned int low, unsigned int len)
{
	struct target *target = bank->target;
	struct rtl8196_flash_bank *rtl8196_info = bank->driver_priv;
	uint32_t value;
	int ret;

	do {
		ret = target_read_u32(target, rtl8196_info->io_base + SFCSR,
					&value);
		if (ret != ERROR_OK)
			return ret;

		if (value & SFCSR_SPI_RDY(1))
			break;
	} while (1);

	if (low)
		value = (rtl8196_info->chipselect + 1) & 0x3;
	else
		value = 3;

	ret = target_write_u32(target, rtl8196_info->io_base + SFCSR,
				SFCSR_SPI_CSB(value) | SFCSR_LEN(len) |
				SFCSR_SPI_RDY(1) | SFCSR_IO_WIDTH(0) |
				SFCSR_CHIP_SEL(0) | SFCSR_CMD_BYTE(5));

	return ERROR_OK;
}

static int sfcsr_reset(struct flash_bank *bank)
{
	int i, ret;

	for (i = 0; i <= 3; i++) {
		ret = sfcsr_cs(bank, !(i & 1), 0);
		if (ret != ERROR_OK)
			return ret;
	}

	return ERROR_OK;
}

/* Send "write enable" command to SPI flash chip. */
static int rtl8196_write_enable(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct rtl8196_flash_bank *rtl8196_info = bank->driver_priv;
	int ret;

	ret = sfcsr_cs(bank, 1, 0);
	if (ret != ERROR_OK)
		return ret;

	/* Write Enable Command */
	ret = target_write_u32(target, rtl8196_info->io_base + SFDR,
				SPIFLASH_WRITE_ENABLE << 24);
	if (ret != ERROR_OK)
		return ret;

	ret = sfcsr_cs(bank, 0, 0);
	if (ret != ERROR_OK)
		return ret;

	return ERROR_OK;
}

/* Read the status register of the external SPI flash chip. */
static int read_status_reg(struct flash_bank *bank, uint32_t *status)
{
	struct target *target = bank->target;
	struct rtl8196_flash_bank *rtl8196_info = bank->driver_priv;
	int ret;

	ret = sfcsr_cs(bank, 1, 0);
	if (ret != ERROR_OK)
		return ret;

	/* Read Status Command */
	ret = target_write_u32(target, rtl8196_info->io_base + SFDR,
				SPIFLASH_READ_STATUS << 24);
	if (ret != ERROR_OK)
		return ret;

	ret = sfcsr_cs(bank, 1, 0);
	if (ret != ERROR_OK)
		return ret;

	/* Read Value */
	ret = target_read_u32(target, rtl8196_info->io_base + SFCSR,
				status);
	if (ret != ERROR_OK)
		return ret;

	*status = (*status >> 24) & 0xff;

	ret = sfcsr_cs(bank, 0, 0);
	if (ret != ERROR_OK)
		return ret;

	return ERROR_OK;
}

/* check for WIP (write in progress) bit in status register */
/* timeout in ms */
static int wait_till_ready(struct flash_bank *bank, int timeout)
{
	long long endtime;
	uint32_t status;
	int ret;

	endtime = timeval_ms() + timeout;
	do {
		/* read flash status register */
		ret = read_status_reg(bank, &status);
		if (ret != ERROR_OK)
			return ret;

		if ((status & SPIFLASH_BSY_BIT) == 0)
			return ERROR_OK;
		alive_sleep(1);
	} while (timeval_ms() < endtime);

	LOG_ERROR("timeout");
	return ERROR_FAIL;
}

static int rtl8196_erase_sector(struct flash_bank *bank, int sector)
{
	struct target *target = bank->target;
	struct rtl8196_flash_bank *rtl8196_info = bank->driver_priv;
	int ret;

	/*
	 * Write Enable
	 */
	ret = rtl8196_write_enable(bank);
	if (ret != ERROR_OK)
		return ret;

	/*
	 * Erase
	 */
	ret = sfcsr_cs(bank, 1, 0);
	if (ret != ERROR_OK)
		return ret;

	/* Sector Erase Command */
	ret = target_write_u32(target, rtl8196_info->io_base + SFDR,
				rtl8196_info->dev->erase_cmd << 24);
	if (ret != ERROR_OK)
		return ret;

	ret = sfcsr_cs(bank, 1, 2);
	if (ret != ERROR_OK)
		return ret;

	/* Sector Erase Address */
	ret = target_write_u32(target, rtl8196_info->io_base + SFDR,
				sector << 8);
	if (ret != ERROR_OK)
		return ret;

	ret = sfcsr_cs(bank, 0, 0);
	if (ret != ERROR_OK)
		return ret;

	/*
	 * Poll status register
	 */
	ret = wait_till_ready(bank, 3000);
	if (ret != ERROR_OK)
		return ret;

	return ERROR_OK;
}

static int rtl8196_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	struct rtl8196_flash_bank *rtl8196_info = bank->driver_priv;
	int retval = ERROR_OK;
	int sector;

	LOG_DEBUG("%s: from sector %d to sector %d", __func__, first, last);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((first < 0) || (last < first) || (last >= bank->num_sectors)) {
		LOG_ERROR("Flash sector invalid");
		return ERROR_FLASH_SECTOR_INVALID;
	}

	if (!rtl8196_info->probed) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	if (rtl8196_info->dev->erase_cmd == 0x00)
		return ERROR_FLASH_OPER_UNSUPPORTED;

	for (sector = first; sector <= last; sector++) {
		if (bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %d protected", sector);
			return ERROR_FAIL;
		}
	}

	for (sector = first; sector <= last; sector++) {
		retval = rtl8196_erase_sector(bank, sector);
		if (retval != ERROR_OK)
			break;
		keep_alive();
	}

	return retval;
}

static int rtl8196_write_page(struct flash_bank *bank, const uint8_t *buffer,
			    uint32_t address, uint32_t len)
{
	struct target *target = bank->target;
	struct rtl8196_flash_bank *rtl8196_info = bank->driver_priv;
	uint32_t i, pagesize;
	int ret;

	/* if no write pagesize, use reasonable default */
	pagesize = rtl8196_info->dev->pagesize ?
		rtl8196_info->dev->pagesize : SPIFLASH_DEF_PAGESIZE;

	if (address & 0xff) {
		LOG_ERROR("rtl8196_write_page: unaligned write address: %08x",
			  address);
		return ERROR_FAIL;
	}
	if (!rtl8196_info->spi.page_buf) {
		LOG_ERROR("rtl8196_write_page: page buffer not initialized");
		return ERROR_FAIL;
	}
	if (len > rtl8196_info->dev->pagesize) {
		LOG_ERROR("rtl8196_write_page: len bigger than page size %d: %d",
			pagesize, len);
		return ERROR_FAIL;
	}

	for (i = 0; i < len; i++) {
		if (buffer[i] != 0xff)
			break;
	}
	if (i == len)  /* all 0xff, no need to program. */
		return ERROR_OK;

	LOG_INFO("writing %d bytes to flash page @0x%08x", len, address);

	memcpy(rtl8196_info->spi.page_buf, buffer, len);

	/*
	 * Write Enable
	 */
	ret = rtl8196_write_enable(bank);
	if (ret != ERROR_OK)
		return ret;

	/*
	 * Program
	 */
	ret = sfcsr_cs(bank, 1, 0);
	if (ret != ERROR_OK)
		return ret;

	/* Command */
	ret = target_write_u32(target, rtl8196_info->io_base + SFDR,
				SPIFLASH_PAGE_PROGRAM << 24);
	if (ret != ERROR_OK)
		return ret;

	ret = sfcsr_cs(bank, 1, 0);
	if (ret != ERROR_OK)
		return ret;

	/* Address */
	ret = target_write_u32(target, rtl8196_info->io_base + SFDR,
				address << 8);
	if (ret != ERROR_OK)
		return ret;

	ret = target_write_u32(target, rtl8196_info->io_base + SFDR,
				address << 16);
	if (ret != ERROR_OK)
		return ret;

	ret = target_write_u32(target, rtl8196_info->io_base + SFDR,
				address << 24);
	if (ret != ERROR_OK)
		return ret;

	ret = sfcsr_cs(bank, 1, 3);
	if (ret != ERROR_OK)
		return ret;

	/* Write data */
	ret = rtl8196_write_page_data(bank, (uint32_t *)buffer, len / 4);
	if (ret != ERROR_OK)
		return ret;

	ret = sfcsr_cs(bank, 0, 0);
	if (ret != ERROR_OK)
		return ret;

	/*
	 * Poll status register
	 */
	ret = wait_till_ready(bank, 3000);
	if (ret != ERROR_OK)
		return ret;

	return ERROR_OK;
}

static int rtl8196_write_buffer(struct flash_bank *bank, const uint8_t *buffer,
			      uint32_t address, uint32_t len)
{
	struct rtl8196_flash_bank *rtl8196_info = bank->driver_priv;
	uint32_t page_size;
	int retval;

	LOG_DEBUG("%s: address=0x%08" PRIx32 " len=0x%08" PRIx32,
		  __func__, address, len);

	/* if no valid page_size, use reasonable default */
	page_size = rtl8196_info->dev->pagesize ?
		rtl8196_info->dev->pagesize : SPIFLASH_DEF_PAGESIZE;

	while (len > 0) {
		int page_len = len > page_size ? page_size : len;

		retval = rtl8196_write_page(
			bank, buffer, address, page_len);
		if (retval != ERROR_OK)
			return retval;

		buffer += page_size;
		address += page_size;
		len -= page_len;
	}

	return ERROR_OK;
}

static int rtl8196_write(struct flash_bank *bank, const uint8_t *buffer,
		       uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	int sector;

	LOG_DEBUG("%s: offset=0x%08" PRIx32 " count=0x%08" PRIx32,
		  __func__, offset, count);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset + count > bank->size) {
		LOG_WARNING("Write pasts end of flash. Extra data discarded.");
		count = bank->size - offset;
	}

	/* Check sector protection */
	for (sector = 0; sector < bank->num_sectors; sector++) {
		/* Start offset in or before this sector? */
		/* End offset in or behind this sector? */
		struct flash_sector *bs = &bank->sectors[sector];

		if ((offset < (bs->offset + bs->size)) &&
		    ((offset + count - 1) >= bs->offset) &&
		    bs->is_protected) {
			LOG_ERROR("Flash sector %d protected", sector);
			return ERROR_FAIL;
		}
	}

	return rtl8196_write_buffer(bank, buffer, offset, count);
}


static int rtl8196_protect(struct flash_bank *bank, int set,
			 int first, int last)
{
	int sector;

	for (sector = first; sector <= last; sector++)
		bank->sectors[sector].is_protected = set;
	return ERROR_OK;
}


static int rtl8196_read(struct flash_bank *bank, uint8_t *buffer,
		      uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct rtl8196_flash_bank *rtl8196_info = bank->driver_priv;
	uint32_t i, value;
	int ret;

	LOG_DEBUG("%s: offset=0x%08" PRIx32 " count=0x%08" PRIx32,
		  __func__, offset, count);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset + count > bank->size) {
		LOG_WARNING("Reads past end of flash. Extra data discarded.");
		count = bank->size - offset;
	}

	ret = sfcsr_cs(bank, 1, 0);
	if (ret != ERROR_OK)
		return ret;

	/* Command */
	ret = target_write_u32(target, rtl8196_info->io_base + SFDR,
				SPIFLASH_FAST_READ << 24);
	if (ret != ERROR_OK)
		return ret;

	ret = sfcsr_cs(bank, 1, 0);
	if (ret != ERROR_OK)
		return ret;

	/* Address */
	ret = target_write_u32(target, rtl8196_info->io_base + SFDR,
				offset << 8);
	if (ret != ERROR_OK)
		return ret;

	ret = target_write_u32(target, rtl8196_info->io_base + SFDR,
				offset << 16);
	if (ret != ERROR_OK)
		return ret;

	ret = target_write_u32(target, rtl8196_info->io_base + SFDR,
				offset << 24);
	if (ret != ERROR_OK)
		return ret;

	/* One dummy cycle */
	ret = target_write_u32(target, rtl8196_info->io_base + SFDR, 0);
	if (ret != ERROR_OK)
		return ret;

	ret = sfcsr_cs(bank, 1, 3);
	if (ret != ERROR_OK)
		return ret;

	for (i = 0; i < count; i += 4) {
		ret = target_read_u32(target, rtl8196_info->io_base + SFDR,
					&value);
		if (ret)
			return ret;

		value = ___swab32(value);
		memcpy(buffer, &value, 4);
		buffer += 4;
	}

	ret = sfcsr_cs(bank, 0, 0);
	if (ret != ERROR_OK)
		return ret;

	return ERROR_OK;
}

/* Return ID of flash device */
static int read_flash_id(struct flash_bank *bank, uint32_t *id)
{
	struct target *target = bank->target;
	struct rtl8196_flash_bank *rtl8196_info = bank->driver_priv;
	uint32_t value;
	int ret;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	ret = target_write_u32(target, rtl8196_info->io_base + SFCR,
				SFCR_SPI_CLK_DIV(7) | SFCR_RBO(1) |
				SFCR_WBO(1) | SFCR_SPI_TCS(31));
	if (ret != ERROR_OK)
		return ret;

	ret = sfcsr_reset(bank);
	if (ret != ERROR_OK)
		return ret;

	ret = sfcsr_cs(bank, 1, 0);
	if (ret != ERROR_OK)
		return ret;

	ret = target_write_u32(target, rtl8196_info->io_base + SFDR,
				SPIFLASH_READ_ID << 24);
	if (ret != ERROR_OK)
		return ret;

	ret = sfcsr_cs(bank, 1, 3);
	if (ret != ERROR_OK)
		return ret;

	ret = target_read_u32(target, rtl8196_info->io_base + SFDR,
				&value);
	if (ret)
		return ret;

	ret = sfcsr_cs(bank, 0, 0);
	if (ret != ERROR_OK)
		return ret;

	*id = ___swab32(value) & 0xffffff;

	if (*id == 0xffffff) {
		LOG_ERROR("No SPI flash found");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int rtl8196_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct rtl8196_flash_bank *rtl8196_info = bank->driver_priv;
	struct flash_sector *sectors;
	uint32_t id = 0; /* silence uninitialized warning */
	uint32_t pagesize, sectorsize;
	const struct rtl8196_target *target_device;
	int retval;

	if (rtl8196_info->probed) {
		free(bank->sectors);
		free(rtl8196_info->spi.page_buf);
	}
	rtl8196_info->probed = 0;

	for (target_device = target_devices; target_device->name;
		++target_device) {
		if (target_device->tap_idcode == target->tap->idcode)
			break;
	}
	if (!target_device->name) {
		LOG_ERROR("Device ID 0x%" PRIx32 " is not known",
			  target->tap->idcode);
		return ERROR_FAIL;
	}

	rtl8196_info->io_base = target_device->io_base;

	LOG_DEBUG("Found device %s at address 0x%" PRIx32,
		  target_device->name, bank->base);

	retval = read_flash_id(bank, &id);
	if (retval != ERROR_OK)
		return retval;

	rtl8196_info->dev = NULL;
	for (const struct flash_device *p = flash_devices; p->name; p++)
		if (p->device_id == id) {
			rtl8196_info->dev = p;
			break;
		}

	if (!rtl8196_info->dev) {
		LOG_ERROR("Unknown flash device (ID 0x%08" PRIx32 ")", id);
		return ERROR_FAIL;
	}

	LOG_INFO("Found flash device \'%s\' (ID 0x%08" PRIx32 ")",
		 rtl8196_info->dev->name, rtl8196_info->dev->device_id);

	/* Set correct size value */
	bank->size = rtl8196_info->dev->size_in_bytes;
	if (bank->size <= (1UL << 16))
		LOG_WARNING("device needs 2-byte addresses - not implemented");
	if (bank->size > (1UL << 24))
		LOG_WARNING("device needs paging or 4-byte addresses - not implemented");

	/* if no sectors, treat whole bank as single sector */
	sectorsize = rtl8196_info->dev->sectorsize ?
		rtl8196_info->dev->sectorsize : rtl8196_info->dev->size_in_bytes;

	/* create and fill sectors array */
	bank->num_sectors = rtl8196_info->dev->size_in_bytes / sectorsize;
	sectors = calloc(1, sizeof(struct flash_sector) * bank->num_sectors);
	if (!sectors) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	/* if no write pagesize, use reasonable default */
	pagesize = rtl8196_info->dev->pagesize ? rtl8196_info->dev->pagesize : SPIFLASH_DEF_PAGESIZE;

	rtl8196_info->spi.page_buf = malloc(pagesize);
	if (!rtl8196_info->spi.page_buf) {
		LOG_ERROR("not enough memory");
		free(sectors);
		return ERROR_FAIL;
	}

	for (int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * sectorsize;
		sectors[sector].size = sectorsize;
		sectors[sector].is_erased = 0;
		sectors[sector].is_protected = 0;
	}

	bank->sectors = sectors;
	rtl8196_info->probed = 1;
	return ERROR_OK;
}

static int rtl8196_auto_probe(struct flash_bank *bank)
{
	struct rtl8196_flash_bank *rtl8196_info = bank->driver_priv;

	if (rtl8196_info->probed)
		return ERROR_OK;
	return rtl8196_probe(bank);
}

static int rtl8196_flash_blank_check(struct flash_bank *bank)
{
	/* Not implemented */
	return ERROR_OK;
}

static int rtl8196_protect_check(struct flash_bank *bank)
{
	/* Not implemented */
	return ERROR_OK;
}

static int get_rtl8196_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct rtl8196_flash_bank *rtl8196_info = bank->driver_priv;

	if (!rtl8196_info->probed) {
		snprintf(buf, buf_size,
			 "\nRTL8196 flash bank not probed yet\n");
		return ERROR_OK;
	}

	snprintf(buf, buf_size, "\nRTL8196 flash information:\n"
		"  Device \'%s\' (ID 0x%08" PRIx32 ")\n",
		rtl8196_info->dev->name, rtl8196_info->dev->device_id);

	return ERROR_OK;
}

struct flash_driver rtl8196_flash = {
	.name = "rtl8196",
	.flash_bank_command = rtl8196_flash_bank_command,
	.erase = rtl8196_erase,
	.protect = rtl8196_protect,
	.write = rtl8196_write,
	.read = rtl8196_read,
	.probe = rtl8196_probe,
	.auto_probe = rtl8196_auto_probe,
	.erase_check = rtl8196_flash_blank_check,
	.protect_check = rtl8196_protect_check,
	.info = get_rtl8196_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
