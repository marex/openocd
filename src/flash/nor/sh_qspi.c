// SPDX-License-Identifier: GPL-2.0
/*
 * SH QSPI (Quad SPI) driver
 * Copyright (C) 2019 Marek Vasut <marek.vasut@gmail.com>
 *
 * Based on U-Boot SH QSPI driver
 * Copyright (C) 2013 Renesas Electronics Corporation
 * Copyright (C) 2013 Nobuhiro Iwamatsu <nobuhiro.iwamatsu.yj@renesas.com>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "spi.h"
#include <helper/binarybuffer.h>
#include <helper/time_support.h>
#include <helper/types.h>
#include <jtag/jtag.h>
#include <target/algorithm.h>
#include <target/arm.h>
#include <target/arm_opcodes.h>
#include <target/target.h>

#define BIT(n)		(1UL << (n))
/* SH QSPI register bit masks <REG>_<BIT> */
#define SPCR_MSTR	0x08
#define SPCR_SPE	0x40
#define SPSR_SPRFF	0x80
#define SPSR_SPTEF	0x20
#define SPPCR_IO3FV	0x04
#define SPPCR_IO2FV	0x02
#define SPPCR_IO1FV	0x01
#define SPBDCR_RXBC0	BIT(0)
#define SPCMD_SCKDEN	BIT(15)
#define SPCMD_SLNDEN	BIT(14)
#define SPCMD_SPNDEN	BIT(13)
#define SPCMD_SSLKP	BIT(7)
#define SPCMD_BRDV0	BIT(2)
#define SPCMD_INIT1	SPCMD_SCKDEN | SPCMD_SLNDEN | \
			SPCMD_SPNDEN | SPCMD_SSLKP | \
			SPCMD_BRDV0
#define SPCMD_INIT2	SPCMD_SPNDEN | SPCMD_SSLKP | \
			SPCMD_BRDV0
#define SPBFCR_TXRST	BIT(7)
#define SPBFCR_RXRST	BIT(6)
#define SPBFCR_TXTRG	0x30
#define SPBFCR_RXTRG	0x07

/* SH QSPI register set */
#define SH_QSPI_SPCR		0x00
#define SH_QSPI_SSLP		0x01
#define SH_QSPI_SPPCR		0x02
#define SH_QSPI_SPSR		0x03
#define SH_QSPI_SPDR		0x04
#define SH_QSPI_SPSCR		0x08
#define SH_QSPI_SPSSR		0x09
#define SH_QSPI_SPBR		0x0a
#define SH_QSPI_SPDCR		0x0b
#define SH_QSPI_SPCKD		0x0c
#define SH_QSPI_SSLND		0x0d
#define SH_QSPI_SPND		0x0e
#define SH_QSPI_DUMMY0		0x0f
#define SH_QSPI_SPCMD0		0x10
#define SH_QSPI_SPCMD1		0x12
#define SH_QSPI_SPCMD2		0x14
#define SH_QSPI_SPCMD3		0x16
#define SH_QSPI_SPBFCR		0x18
#define SH_QSPI_DUMMY1		0x19
#define SH_QSPI_SPBDCR		0x1a
#define SH_QSPI_SPBMUL0		0x1c
#define SH_QSPI_SPBMUL1		0x20
#define SH_QSPI_SPBMUL2		0x24
#define SH_QSPI_SPBMUL3		0x28

struct sh_qspi_flash_bank {
	const struct flash_device *dev;
	uint32_t		io_base;
	int			probed;
	struct working_area	*io_algorithm;
	struct working_area	*source;
	uint8_t			*page_buf;
	unsigned int		buffer_size;
};

struct sh_qspi_target {
	char		*name;
	uint32_t	tap_idcode;
	uint32_t	io_base;
};

static const struct sh_qspi_target target_devices[] = {
	/* name,	tap_idcode,	io_base */
	{ "SH QSPI",	0x4ba00477,	0xe6b10000 },
	{ NULL,		0,		0 }
};

static int sh_qspi_init(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct sh_qspi_flash_bank *info = bank->driver_priv;
	uint8_t val;
	int ret;

	/* QSPI initialize */
	/* Set master mode only */
	ret = target_write_u8(target, info->io_base + SH_QSPI_SPCR, SPCR_MSTR);
	if (ret)
		return ret;

	/* Set SSL signal level */
	ret = target_write_u8(target, info->io_base + SH_QSPI_SSLP, 0x00);
	if (ret)
		return ret;

	/* Set MOSI signal value when transfer is in idle state */
	ret = target_write_u8(target, info->io_base + SH_QSPI_SPPCR,
			      SPPCR_IO3FV | SPPCR_IO2FV);
	if (ret)
		return ret;

	/* Set bit rate. See 58.3.8 Quad Serial Peripheral Interface */
	ret = target_write_u8(target, info->io_base + SH_QSPI_SPBR, 0x01);
	if (ret)
		return ret;

	/* Disable Dummy Data Transmission */
	ret = target_write_u8(target, info->io_base + SH_QSPI_SPDCR, 0x00);
	if (ret)
		return ret;

	/* Set clock delay value */
	ret = target_write_u8(target, info->io_base + SH_QSPI_SPCKD, 0x00);
	if (ret)
		return ret;

	/* Set SSL negation delay value */
	ret = target_write_u8(target, info->io_base + SH_QSPI_SSLND, 0x00);
	if (ret)
		return ret;

	/* Set next-access delay value */
	ret = target_write_u8(target, info->io_base + SH_QSPI_SPND, 0x00);
	if (ret)
		return ret;

	/* Set equence command */
	ret = target_write_u16(target, info->io_base + SH_QSPI_SPCMD0,
			       SPCMD_INIT2);
	if (ret)
		return ret;

	/* Reset transfer and receive Buffer */
	ret = target_read_u8(target, info->io_base + SH_QSPI_SPBFCR, &val);
	if (ret)
		return ret;

	val |= SPBFCR_TXRST | SPBFCR_RXRST;

	ret = target_write_u8(target, info->io_base + SH_QSPI_SPBFCR, val);
	if (ret)
		return ret;

	/* Clear transfer and receive Buffer control bit */
	ret = target_read_u8(target, info->io_base + SH_QSPI_SPBFCR, &val);
	if (ret)
		return ret;

	val &= ~(SPBFCR_TXRST|SPBFCR_RXRST);

	ret = target_write_u8(target, info->io_base + SH_QSPI_SPBFCR, val);
	if (ret)
		return ret;

	/* Set equence control method. Use equence0 only */
	ret = target_write_u8(target, info->io_base + SH_QSPI_SPSCR, 0x00);
	if (ret)
		return ret;

	/* Enable SPI function */
	ret = target_read_u8(target, info->io_base + SH_QSPI_SPCR, &val);
	if (ret)
		return ret;

	val |= SPCR_SPE;

	ret = target_write_u8(target, info->io_base + SH_QSPI_SPCR, val);
	if (ret)
		return ret;

	return ERROR_OK;
}

static int sh_qspi_cs_activate(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct sh_qspi_flash_bank *info = bank->driver_priv;
	uint8_t val;
	int ret;

	/* Set master mode only */
	ret = target_write_u8(target, info->io_base + SH_QSPI_SPCR, SPCR_MSTR);
	if (ret)
		return ret;

	/* Set command */
	ret = target_write_u16(target, info->io_base + SH_QSPI_SPCMD0,
			       SPCMD_INIT1);
	if (ret)
		return ret;

	/* Reset transfer and receive Buffer */
	ret = target_read_u8(target, info->io_base + SH_QSPI_SPBFCR, &val);
	if (ret)
		return ret;

	val |= SPBFCR_TXRST|SPBFCR_RXRST;

	ret = target_write_u8(target, info->io_base + SH_QSPI_SPBFCR, val);
	if (ret)
		return ret;

	/* Clear transfer and receive Buffer control bit */
	ret = target_read_u8(target, info->io_base + SH_QSPI_SPBFCR, &val);
	if (ret)
		return ret;

	val &= ~(SPBFCR_TXRST|SPBFCR_RXRST);

	ret = target_write_u8(target, info->io_base + SH_QSPI_SPBFCR, val);
	if (ret)
		return ret;

	/* Set equence control method. Use equence0 only */
	ret = target_write_u8(target, info->io_base + SH_QSPI_SPSCR, 0x00);
	if (ret)
		return ret;

	/* Enable SPI function */
	ret = target_read_u8(target, info->io_base + SH_QSPI_SPCR, &val);
	if (ret)
		return ret;

	val |= SPCR_SPE;

	ret = target_write_u8(target, info->io_base + SH_QSPI_SPCR, val);
	if (ret)
		return ret;

	return ERROR_OK;
}

static int sh_qspi_cs_deactivate(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct sh_qspi_flash_bank *info = bank->driver_priv;
	uint8_t val;
	int ret;

	/* Disable SPI Function */
	ret = target_read_u8(target, info->io_base + SH_QSPI_SPCR, &val);
	if (ret)
		return ret;

	val &= ~SPCR_SPE;

	ret = target_write_u8(target, info->io_base + SH_QSPI_SPCR, val);
	if (ret)
		return ret;

	return ERROR_OK;
}

static int sh_qspi_wait_for_bit(struct flash_bank *bank, uint8_t reg,
				uint32_t mask, bool set,
				unsigned long timeout)
{
	struct target *target = bank->target;
	struct sh_qspi_flash_bank *info = bank->driver_priv;
	long long endtime;
	uint8_t val;
	int ret;

	endtime = timeval_ms() + timeout;
	do {
		ret = target_read_u8(target, info->io_base + reg, &val);
		if (ret)
			return ERROR_FAIL;

		if (!set)
			val = ~val;

		if ((val & mask) == mask)
			return ERROR_OK;

		alive_sleep(1);
	} while (timeval_ms() < endtime);

	LOG_ERROR("timeout");
	return ERROR_FAIL;
}

static int sh_qspi_xfer_common(struct flash_bank *bank,
			       const uint8_t *dout, unsigned int outlen,
			       uint8_t *din, unsigned int inlen,
			       bool xfer_start, bool xfer_end)
{
	struct target *target = bank->target;
	struct sh_qspi_flash_bank *info = bank->driver_priv;
	uint8_t tdata, rdata;
	uint8_t val;
	unsigned int nbyte = outlen + inlen;
	int ret = 0;

	if (xfer_start) {
		ret = sh_qspi_cs_activate(bank);
		if (ret)
			return ret;

		ret = target_write_u32(target, info->io_base + SH_QSPI_SPBMUL0,
				       nbyte);
		if (ret)
			return ret;

		ret = target_read_u8(target, info->io_base + SH_QSPI_SPBFCR,
				     &val);
		if (ret)
			return ret;

		val &= ~(SPBFCR_TXTRG | SPBFCR_RXTRG);

		ret = target_write_u8(target, info->io_base + SH_QSPI_SPBFCR,
				      val);
		if (ret)
			return ret;
	}

	while (nbyte > 0) {
		ret = sh_qspi_wait_for_bit(bank, SH_QSPI_SPSR, SPSR_SPTEF,
						true, 1000);
		if (ret)
			return ret;

		tdata = outlen ? *dout++ : 0;
		ret = target_write_u8(target, info->io_base + SH_QSPI_SPDR,
				      tdata);
		if (ret)
			return ret;

		ret = sh_qspi_wait_for_bit(bank, SH_QSPI_SPSR, SPSR_SPRFF,
						true, 1000);
		if (ret)
			return ret;

		ret = target_read_u8(target, info->io_base + SH_QSPI_SPDR,
				     &rdata);
		if (ret)
			return ret;
		if (!outlen && inlen) {
			*din++ = rdata;
			inlen--;
		}

		if (outlen)
			outlen--;

		nbyte--;
	}

	if (xfer_end)
		return sh_qspi_cs_deactivate(bank);
	else
		return ERROR_OK;
}

static int sh_qspi_xfer_fast(struct flash_bank *bank,
			     const uint8_t *dout, unsigned int outlen,
			     uint8_t *din, unsigned int inlen,
			     bool xfer_start, bool xfer_end)
{
	struct target *target = bank->target;
	struct sh_qspi_flash_bank *info = bank->driver_priv;
	struct reg_param reg_params[4];
	struct arm_algorithm arm_algo;
	unsigned int nbyte = outlen + inlen;
	uint32_t io_base = (uint32_t)(info->io_base);
	uint32_t src_base = (uint32_t)(info->source->address);
	uint32_t chunk;
	int ret = ERROR_OK;

	if (xfer_start) {
		ret = sh_qspi_cs_activate(bank);
		if (ret)
			return ret;

		ret = target_write_u32(target,
				       info->io_base + SH_QSPI_SPBMUL0,
				       nbyte);
		if (ret)
			return ret;
	}

	arm_algo.common_magic = ARM_COMMON_MAGIC;
	arm_algo.core_mode = ARM_MODE_SVC;
	arm_algo.core_state = ARM_STATE_ARM;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);

	while (outlen > 0) {
		chunk = (outlen > info->buffer_size) ?
			info->buffer_size : outlen;

		target_write_buffer(target, info->source->address,
				    chunk, dout);

		buf_set_u32(reg_params[0].value, 0, 32, io_base);
		buf_set_u32(reg_params[1].value, 0, 32, src_base);
		buf_set_u32(reg_params[2].value, 0, 32, 0);
		buf_set_u32(reg_params[3].value, 0, 32, chunk);

		ret = target_run_algorithm(target, 0, NULL, 4, reg_params,
				info->io_algorithm->address,
				0, 10000, &arm_algo);
		if (ret != ERROR_OK) {
			LOG_ERROR("error executing SH QSPI flash IO algorithm");
			ret = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		dout += chunk;
		outlen -= chunk;
	}

	while (inlen > 0) {
		chunk = (inlen > info->buffer_size) ?
			info->buffer_size : inlen;

		buf_set_u32(reg_params[0].value, 0, 32, io_base);
		buf_set_u32(reg_params[1].value, 0, 32, 0);
		buf_set_u32(reg_params[2].value, 0, 32, src_base);
		buf_set_u32(reg_params[3].value, 0, 32, chunk);

		ret = target_run_algorithm(target, 0, NULL, 4, reg_params,
				info->io_algorithm->address,
				0, 10000, &arm_algo);
		if (ret != ERROR_OK) {
			LOG_ERROR("error executing SH QSPI flash IO algorithm");
			ret = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		target_read_buffer(target, info->source->address,
				   chunk, din);

		din += chunk;
		inlen -= chunk;
	}

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);

	if (ret)
		return ret;

	if (xfer_end)
		return sh_qspi_cs_deactivate(bank);
	else
		return ERROR_OK;
}

/* Send "write enable" command to SPI flash chip. */
static int sh_qspi_write_enable(struct flash_bank *bank)
{
	uint8_t dout = SPIFLASH_WRITE_ENABLE;
	int ret;

	ret = sh_qspi_xfer_common(bank, &dout, 1, NULL, 0, 1, 1);
	if (ret != ERROR_OK)
		return ret;

	return ERROR_OK;
}

/* Read the status register of the external SPI flash chip. */
static int read_status_reg(struct flash_bank *bank, uint32_t *status)
{
	uint8_t dout = SPIFLASH_READ_STATUS;
	uint8_t din;
	int ret;

	ret = sh_qspi_xfer_common(bank, &dout, 1, &din, 1, 1, 1);
	if (ret != ERROR_OK)
		return ret;

	*status = din & 0xff;

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

static int sh_qspi_erase_sector(struct flash_bank *bank, int sector)
{
	struct sh_qspi_flash_bank *info = bank->driver_priv;
	bool addr4b = info->dev->size_in_bytes > (1UL << 24);
	uint32_t address = (sector * info->dev->sectorsize) <<
			   (addr4b ? 0 : 8);
	uint8_t dout[5] = {
		info->dev->erase_cmd,
		(address >> 24) & 0xff, (address >> 16) & 0xff,
		(address >> 8) & 0xff, (address >> 0) & 0xff
	};
	unsigned int doutlen = addr4b ? 5 : 4;
	int ret;

	/* Write Enable */
	ret = sh_qspi_write_enable(bank);
	if (ret != ERROR_OK)
		return ret;

	/* Erase */
	ret = sh_qspi_xfer_common(bank, dout, doutlen, NULL, 0, 1, 1);
	if (ret != ERROR_OK)
		return ret;

	/* Poll status register */
	ret = wait_till_ready(bank, 3000);
	if (ret != ERROR_OK)
		return ret;

	return ERROR_OK;
}

static int sh_qspi_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	struct sh_qspi_flash_bank *info = bank->driver_priv;
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

	if (!info->probed) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	if (info->dev->erase_cmd == 0x00)
		return ERROR_FLASH_OPER_UNSUPPORTED;

	for (sector = first; sector <= last; sector++) {
		if (bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %d protected", sector);
			return ERROR_FAIL;
		}
	}

	for (sector = first; sector <= last; sector++) {
		retval = sh_qspi_erase_sector(bank, sector);
		if (retval != ERROR_OK)
			break;
		keep_alive();
	}

	return retval;
}

static int sh_qspi_write_page(struct flash_bank *bank, const uint8_t *buffer,
			      uint32_t addr, uint32_t len)
{
	struct sh_qspi_flash_bank *info = bank->driver_priv;
	bool addr4b = info->dev->size_in_bytes > (1UL << 24);
	uint32_t address = addr << (addr4b ? 0 : 8);
	uint8_t dout[5] = {
		info->dev->pprog_cmd,
		(address >> 24) & 0xff, (address >> 16) & 0xff,
		(address >> 8) & 0xff, (address >> 0) & 0xff
	};
	unsigned int doutlen = addr4b ? 5 : 4;
	uint32_t i, pagesize;
	int ret;

	/* if no write pagesize, use reasonable default */
	pagesize = info->dev->pagesize ?
		info->dev->pagesize : SPIFLASH_DEF_PAGESIZE;

	if (address & 0xff) {
		LOG_ERROR("sh_qspi_write_page: unaligned write address: %08x",
			  address);
		return ERROR_FAIL;
	}
	if (!info->page_buf) {
		LOG_ERROR("sh_qspi_write_page: page buffer not initialized");
		return ERROR_FAIL;
	}
	if (len > info->dev->pagesize) {
		LOG_ERROR("sh_qspi_write_page: len bigger than page size %d: %d",
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

	memcpy(info->page_buf, buffer, len);

	/* Write Enable */
	ret = sh_qspi_write_enable(bank);
	if (ret != ERROR_OK)
		return ret;

	/* Program */
	ret = sh_qspi_xfer_common(bank, dout, doutlen, NULL, 0, 1, 0);
	if (ret != ERROR_OK)
		return ret;

	ret = sh_qspi_xfer_fast(bank, buffer, len, NULL, 0, 0, 1);
	if (ret != ERROR_OK)
		return ret;

	/* Poll status register */
	ret = wait_till_ready(bank, 3000);
	if (ret != ERROR_OK)
		return ret;

	return ERROR_OK;
}

static int sh_qspi_write_buffer(struct flash_bank *bank, const uint8_t *buffer,
			      uint32_t address, uint32_t len)
{
	struct sh_qspi_flash_bank *info = bank->driver_priv;
	uint32_t page_size;
	int retval;

	LOG_DEBUG("%s: address=0x%08" PRIx32 " len=0x%08" PRIx32,
		  __func__, address, len);

	/* if no valid page_size, use reasonable default */
	page_size = info->dev->pagesize ?
		info->dev->pagesize : SPIFLASH_DEF_PAGESIZE;

	while (len > 0) {
		int page_len = len > page_size ? page_size : len;

		retval = sh_qspi_write_page(
			bank, buffer, address, page_len);
		if (retval != ERROR_OK)
			return retval;

		buffer += page_size;
		address += page_size;
		len -= page_len;
	}

	return ERROR_OK;
}

static int sh_qspi_write(struct flash_bank *bank, const uint8_t *buffer,
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

	return sh_qspi_write_buffer(bank, buffer, offset, count);
}

static int sh_qspi_read(struct flash_bank *bank, uint8_t *buffer,
		      uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint8_t dout[5] = {
		SPIFLASH_FAST_READ, (offset >> 16) & 0xff,
		(offset >> 8) & 0xff, (offset >> 0) & 0xff, 0
	};
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

	ret = sh_qspi_xfer_fast(bank, dout, sizeof(dout), buffer, count, 1, 1);
	if (ret != ERROR_OK)
		return ret;

	return ERROR_OK;
}

/* Return ID of flash device */
static int read_flash_id(struct flash_bank *bank, uint32_t *id)
{
	struct target *target = bank->target;
	uint8_t dout = SPIFLASH_READ_ID;
	uint8_t din[3] = { 0, 0, 0 };
	int ret;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	ret = sh_qspi_xfer_common(bank, &dout, 1, din, 3, 1, 1);
	if (ret != ERROR_OK)
		return ret;

	*id = (din[0] << 0) | (din[1] << 8) | (din[2] << 16);

	if (*id == 0xffffff) {
		LOG_ERROR("No SPI flash found");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int sh_qspi_protect(struct flash_bank *bank, int set,
			 int first, int last)
{
	int sector;

	for (sector = first; sector <= last; sector++)
		bank->sectors[sector].is_protected = set;

	return ERROR_OK;
}

static int sh_qspi_upload_helper(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct sh_qspi_flash_bank *info = bank->driver_priv;

	/* see contrib/loaders/flash/sh_qspi.s for src */

	static const uint32_t sh_qspi_io_code[] = {
		0xe5904018,
		0xe3844037,
		0xe3a05020,
		0xe3530020,
		0xb3c44037,
		0xb3a05001,
		0xe5804018,
		0xe5d04003,
		0xe3140020,
		0x0afffffc,
		0xe1a06005,
		0xe3a04000,
		0xe3510000,
		0x0a000004,
		0xe4d14001,
		0xe5c04004,
		0xe2566001,
		0x1afffffb,
		0xea000002,
		0xe5c04004,
		0xe2566001,
		0x1afffffc,
		0xe5d04003,
		0xe3140080,
		0x0afffffc,
		0xe1a06005,
		0xe3520000,
		0x0a000004,
		0xe5d04004,
		0xe4c24001,
		0xe2566001,
		0x1afffffb,
		0xea000002,
		0xe5d04004,
		0xe2566001,
		0x1afffffc,
		0xe0533005,
		0x1affffd9,
		0xe1200070,
	};
	uint8_t code[sizeof(sh_qspi_io_code)];
	int ret;

	if (info->source)
		target_free_working_area(target, info->source);
	if (info->io_algorithm)
		target_free_working_area(target, info->io_algorithm);

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(sh_qspi_io_code),
			&info->io_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	target_buffer_set_u32_array(target, code, ARRAY_SIZE(sh_qspi_io_code),
				    sh_qspi_io_code);
	target_write_buffer(target, info->io_algorithm->address,
			    sizeof(code), code);

	/* memory buffer */
	info->buffer_size = 32768;
	while (true) {
		ret = target_alloc_working_area_try(target, info->buffer_size,
						    &info->source);
		if (ret == ERROR_OK)
			break;

		info->buffer_size /= 2;
		if (info->buffer_size <= 256) {
			target_free_working_area(target, info->io_algorithm);

			LOG_WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	return ERROR_OK;
}

static int sh_qspi_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct sh_qspi_flash_bank *info = bank->driver_priv;
	struct flash_sector *sectors;
	uint32_t id = 0; /* silence uninitialized warning */
	uint32_t pagesize, sectorsize;
	const struct sh_qspi_target *target_device;
	int ret;

	if (info->probed) {
		free(bank->sectors);
		free(info->page_buf);
	}
	info->probed = 0;

	for (target_device = target_devices; target_device->name;
		++target_device)
		if (target_device->tap_idcode == target->tap->idcode)
			break;
	if (!target_device->name) {
		LOG_ERROR("Device ID 0x%" PRIx32 " is not known",
			  target->tap->idcode);
		return ERROR_FAIL;
	}

	info->io_base = target_device->io_base;

	LOG_DEBUG("Found device %s at address " TARGET_ADDR_FMT,
		  target_device->name, bank->base);

	ret = sh_qspi_upload_helper(bank);
	if (ret != ERROR_OK)
		return ret;

	ret = sh_qspi_init(bank);
	if (ret != ERROR_OK)
		return ret;

	ret = read_flash_id(bank, &id);
	if (ret != ERROR_OK)
		return ret;

	info->dev = NULL;
	for (const struct flash_device *p = flash_devices; p->name; p++)
		if (p->device_id == id) {
			info->dev = p;
			break;
		}

	if (!info->dev) {
		LOG_ERROR("Unknown flash device (ID 0x%08" PRIx32 ")", id);
		return ERROR_FAIL;
	}

	LOG_INFO("Found flash device \'%s\' (ID 0x%08" PRIx32 ")",
		 info->dev->name, info->dev->device_id);

	/* Set correct size value */
	bank->size = info->dev->size_in_bytes;
	if (bank->size <= (1UL << 16))
		LOG_WARNING("device needs 2-byte addresses - not implemented");

	/* if no sectors, treat whole bank as single sector */
	sectorsize = info->dev->sectorsize ?
		     info->dev->sectorsize :
		     info->dev->size_in_bytes;

	/* create and fill sectors array */
	bank->num_sectors = info->dev->size_in_bytes / sectorsize;
	sectors = calloc(1, sizeof(*sectors) * bank->num_sectors);
	if (!sectors) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	/* if no write pagesize, use reasonable default */
	pagesize = info->dev->pagesize ?
		   info->dev->pagesize : SPIFLASH_DEF_PAGESIZE;

	info->page_buf = malloc(pagesize);
	if (!info->page_buf) {
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
	info->probed = 1;
	return ERROR_OK;
}

static int sh_qspi_auto_probe(struct flash_bank *bank)
{
	struct sh_qspi_flash_bank *info = bank->driver_priv;

	if (info->probed)
		return ERROR_OK;

	return sh_qspi_probe(bank);
}

static int sh_qspi_flash_blank_check(struct flash_bank *bank)
{
	/* Not implemented */
	return ERROR_OK;
}

static int sh_qspi_protect_check(struct flash_bank *bank)
{
	/* Not implemented */
	return ERROR_OK;
}

static int sh_qspi_get_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct sh_qspi_flash_bank *info = bank->driver_priv;

	if (!info->probed) {
		snprintf(buf, buf_size,
			 "\nSH QSPI flash bank not probed yet\n");
		return ERROR_OK;
	}

	snprintf(buf, buf_size, "\nSH QSPI flash information:\n"
		"  Device \'%s\' (ID 0x%08" PRIx32 ")\n",
		info->dev->name, info->dev->device_id);

	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(sh_qspi_flash_bank_command)
{
	struct sh_qspi_flash_bank *info;

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC < 6 || CMD_ARGC > 7)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if ((CMD_ARGC == 7) && strcmp(CMD_ARGV[6], "cs0")) {
		LOG_ERROR("Unknown arg: %s", CMD_ARGV[6]);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	info = calloc(1, sizeof(struct sh_qspi_flash_bank));
	if (!info) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	bank->driver_priv = info;

	return ERROR_OK;
}

const struct flash_driver sh_qspi_flash = {
	.name			= "sh_qspi",
	.flash_bank_command	= sh_qspi_flash_bank_command,
	.erase			= sh_qspi_erase,
	.protect		= sh_qspi_protect,
	.write			= sh_qspi_write,
	.read			= sh_qspi_read,
	.probe			= sh_qspi_probe,
	.auto_probe		= sh_qspi_auto_probe,
	.erase_check		= sh_qspi_flash_blank_check,
	.protect_check		= sh_qspi_protect_check,
	.info			= sh_qspi_get_info,
	.free_driver_priv	= default_flash_free_driver_priv,
};
