/*-
 * Copyright (c) 2024 Jason R. Thorpe.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *      
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */      

/*
 * SPI-connected SD card driver for RP2040.
 */

/* Pico SDK headers */
#include "pico/stdlib.h"
#include "pico/stdio_uart.h"
#include "pico/printf.h"
#include "pico/multicore.h"
#include "hardware/spi.h"
#include "hardware/timer.h"

/* Standard headers */
#include <stdbool.h>
#include <stdio.h>

/* Local headers */
#include "sd.h"

static spi_inst_t *sd_spi;
static int sd_cs_gpio;
static int sd_cd_gpio;
static int sd_sticky_status;
static uint32_t sd_block_count;
static uint32_t sd_eblock_size;

#define	SD_SPI_CLKSPEED	(25 * 1000000)

#define	SD_SPI_CMDLEN	6

/* R1 response format */
#define	R1_IDLE_STATE	(1U << 0)
#define	R1_ERASE_RESET	(1U << 1)
#define	R1_ILL_CMD	(1U << 2)
#define	R1_CMD_CRC_ERR	(1U << 3)
#define	R1_ERASE_SEQ_ERR (1U << 4)
#define	R1_ADDR_ERR	(1U << 5)
#define	R1_PARAM_ERR	(1U << 6)
			/* MSB MBZ */

#define	RES_ISERROR(x)	((x) > R1_IDLE_STATE)

#define	SD_R3_R7_LEN	5

/* SEND_IF_COND / R7 voltage suppled / accepted values */
#define	R7_VA_2v7_3v6	(1U << 0)
#define	R7_VA_LV	(1U << 1)

/* R7 response word */
#define	R7_ECHOBACK_MASK  0xff
#define	R7_ECHOBACK_SHIFT 0
#define	R7_VA_MASK	  0x0f
#define	R7_VA_SHIFT	  8
#define	R7_CMD_VERS_MASK  0x0f
#define	R7_CMD_VERS_SHIFT 28

/* OCR register */
#define	OCR_2v7_2v8	(1U << 15)
#define	OCR_2v8_2v9	(1U << 16)
#define	OCR_2v9_3v0	(1U << 17)
#define	OCR_3v0_3v1	(1U << 18)
#define	OCR_3v1_3v2	(1U << 19)
#define	OCR_3v2_3v3	(1U << 20)
#define	OCR_3v3_3v4	(1U << 21)
#define	OCR_3v4_3v5	(1U << 22)
#define	OCR_3v5_3v6	(1U << 23)
#define	OCR_S18A	(1U << 24)
#define	OCR_UHS_II	(1U << 29)
#define	OCR_CCS		(1U << 30)	/* valid only when OCR_PU_nBUSY */
#define	OCR_PU_nBUSY	(1U << 31)	/* LOW when power-up finished */

static bool
sd_timed_out(uint64_t *curp, uint64_t timo_ms)
{
	uint64_t now = to_us_since_boot(get_absolute_time());

	if (*curp == 0) {
		*curp = now;
	} else {
		/* Allow a 1ms fudge factor. */
		if ((now - *curp) > ((timo_ms + 1) * 1000)) {
			return true;
		}
		*curp = now;
	}
	return false;
}

static void
sd_cd_callback(unsigned int gpio, uint32_t events)
{
	/*
	 * CD GPIO has transitioned from low to high, meaning the
	 * card has been removed from the slot.
	 */
	sd_sticky_status = SD_ENODEV;
}

static inline int
sd_set_status(int err)
{
	return (sd_sticky_status = err);
}

static inline void
sd_cs_assert(void)
{
	gpio_set_dir(sd_cs_gpio, true);
}

static inline void
sd_cs_deassert(void)
{
	gpio_set_dir(sd_cs_gpio, false);
}

static inline bool
sd_card_present_p(void)
{
	return !gpio_get(sd_cd_gpio);
}

static inline void
sd_send(const void *vbuf, size_t len)
{
	spi_write_blocking(sd_spi, vbuf, len);
}

static inline void
sd_send_byte(uint8_t val)
{
	spi_write_blocking(sd_spi, &val, 1);
}

static inline void
sd_recv(void *vbuf, size_t len)
{
	spi_read_blocking(sd_spi, 0xff, vbuf, len);
}

static inline uint8_t
sd_recv_byte(void)
{
	uint8_t buf;

	spi_read_blocking(sd_spi, 0xff, &buf, 1);
	return buf;
}

static void
sd_powerup(void)
{
	/* Ensure CS is de-asserted. */
	sd_cs_deassert();

	/* Give card time to power up. */
	busy_wait_ms(1);

	/* Send 80 clock cycles. */
	for (int i = 0; i < 10; i++) {
		sd_send_byte(0xff);
	}

	/* De-select card. */
	sd_cs_deassert();
	sd_send_byte(0xff);
}

static void
sd_begin(void)
{
	sd_send_byte(0xff);
	sd_cs_assert();
	sd_send_byte(0xff);
}

static void
sd_end(void)
{
	sd_send_byte(0xff);
	sd_cs_deassert();
	sd_send_byte(0xff);
}

static void
sd_command(uint8_t cmd, uint32_t arg, uint8_t crc)
{
	uint8_t buf[SD_SPI_CMDLEN];

	buf[0] = cmd | 0x40;
	buf[1] = (uint8_t)(arg >> 24);
	buf[2] = (uint8_t)(arg >> 16);
	buf[3] = (uint8_t)(arg >> 8);
	buf[4] = (uint8_t)(arg);
	buf[5] = (crc << 1) | 0x01;

	sd_send(buf, sizeof(buf));
}

static uint8_t
sd_recv_r1(void)
{
	uint8_t val;

	for (int i = 8; i > 0; i--) {
		val = sd_recv_byte();
		if (val != 0xff) {
			break;
		}
	}
	return val;
}

static uint8_t
sd_recv_r3_r7(uint32_t *valp)
{
	uint8_t buf[SD_R3_R7_LEN];
	*valp = 0;
	buf[0] = sd_recv_r1();
	if (! RES_ISERROR(buf[0])) {
		sd_recv(&buf[1], SD_R3_R7_LEN - 1);
		*valp = (buf[1] << 24) |
		        (buf[2] << 16) |
			(buf[3] <<  8) |
			 buf[4];
	}
	return buf[0];
}

static uint8_t
sd_GO_IDLE_STATE(void)
{
	uint8_t rv;

	sd_begin();
	sd_command(0, 0, 0x4a);
	rv = sd_recv_r1();
	sd_end();

	return rv;
}

static uint8_t
sd_SEND_IF_COND(uint32_t *valp)
{
	uint8_t rv;

	sd_begin();
	sd_command(0, (R7_VA_2v7_3v6 << 8) | 0xaa, 0x43);
	rv = sd_recv_r3_r7(valp);
	sd_end();

	return rv;
}

static uint8_t
sd_APP_CMD(void)
{
	uint8_t rv;

	sd_begin();
	sd_command(55, 0, 0);
	rv = sd_recv_r1();
	sd_end();

	return rv;
}

static uint8_t
sd_READ_OCR(uint32_t *valp)
{
	uint8_t rv;

	sd_begin();
	sd_command(58, 0, 0);
	rv = sd_recv_r3_r7(valp);
	sd_end();

	return rv;
}

static uint8_t
sd_SD_SEND_OP_COND(void)
{
	uint8_t rv;

	rv = sd_APP_CMD();
	if (! RES_ISERROR(rv)) {
		sd_begin();
		sd_command(41, 0x40000000, 0);
		rv = sd_recv_r1();
		sd_end();
	}

	return rv;
}

static int
sd_rdblk_CMD(uint8_t cmd, uint32_t blk, void *vbuf, size_t len)
{
	uint64_t timo;
	uint8_t res, token;
	int error;

	sd_begin();

	sd_command(cmd, blk, 0);
	res = sd_recv_r1();
	if (res != 0xff) {
		/* Wait for response token. */
		for (timo = 0, token = 0xff; token != 0xff;) {
			token = sd_recv_byte();
			if (sd_timed_out(&timo, 100)) {
				error = SD_ETIMEDOUT;
				goto out;
			}
		}
		if (token == 0xfe) {
			/* Got data token, receive data. */
			sd_recv(vbuf, len);

			/* Receive (and discard) 16-bit CRC. */
			sd_recv_byte();
			sd_recv_byte();
		} else if ((token & 0xf0) == 0) {
			/* Got error token. */
			if (token & (1U << 3)) {
				/* Out Of Range error */
				error = SD_EINVAL;
			} else {
				error = SD_EIO;
			}
		} else if (res & R1_PARAM_ERR) {
			/* Fall back on the R1 response. */
			error = SD_EINVAL;
		} else {
			/* &shrug; */
			error = SD_EIO;
		}
	}

 out:
	sd_end();
	return error;
}

#define	CSD_V1		0
#define	CSD_V2		1
#define	CSD_V3		2

static int
sd_SEND_CSD(int *versp, uint32_t *blkcntp, uint32_t *eblksizep)
{
	uint8_t buf[16];
	int error;

	error = sd_rdblk_CMD(9, 0, buf, sizeof(buf));
	if (error) {
		return error;
	}

	/*
	 * This assumes CSD 2.0, which is valid for SDHC and SDXC.
	 */
	uint8_t csd_vers = buf[15] >> 6;
	*versp = csd_vers;

	if (csd_vers < CSD_V2) {
		return SD_ENOTSUP;
	}

	uint32_t c_size =   buf[6]                |
			  ( buf[7]         <<  8) |
			  ((buf[8] & 0x3f) << 16);

	/* Capacity is C_SIZE * 512KB. */
	uint64_t cap64 = (uint64_t)c_size * (512 * 1024);
	uint64_t blkcnt64 = cap64 / SD_SECSIZE;

	/* Saturate the block count. */
	if (blkcnt64 > UINT32_MAX) {
		blkcnt64 = UINT32_MAX;
	}
	*blkcntp = (uint32_t)blkcnt64;

	/*
	 * Erase block size is fixed as 64KB in CSD v2.  The
	 * real erase size should be determined using AU_SIZE
	 * in the SD STATUS register.
	 */
	*eblksizep = (64 * 1024);

	return SD_NOERR;
}

#if 0
static int
sd_SEND_CID(struct sd_cid_info *info)
{
	uint8_t buf[16];
	int error;

	error = sd_rdblk_CMD(10, 0, buf, sizeof(buf));
	if (error) {
		return error;
	}

	return SD_NOERR;
}
#endif

void
sd_init(spi_inst_t *spi, int cs_gpio, int cd_gpio)
{
	/*
	 * We arrive here assuming that the SPI block has been
	 * assigned to a set of pins, but no more than that.
	 *
	 * Because we need more precise control over the CS line
	 * for SD cards, we will re-configure the cs_gpio as a
	 * generic GPIO.  We will also configure it as an input,
	 * pulled high.  When we wish to assert CS, we will switch
	 * direction and drive as a low output.
	 */
	spi_init(spi, SD_SPI_CLKSPEED);
	spi_set_format(spi,
		       8,		/* data bits */
		       SPI_CPOL_0,	/* mode 0 */
		       SPI_CPHA_0,
		       SPI_MSB_FIRST);

	/*
	 * CS GPIO: Input, pull-up.  Output value set to low.
	 * When we want to drive the output low, we change the
	 * direction to an output, thus simulating an open-drain
	 * type output.
	 */
	gpio_init(cs_gpio);
	gpio_set_dir(cs_gpio, false);
	gpio_set_pulls(cs_gpio, true, false);
	gpio_put(cs_gpio, false);

	/*
	 * CD GPIO: Input, pull-up.  When a card is inserted into the
	 * slot, the CD pin is grounded.  We take an interrupt on the
	 * rising edge of this GPIO to set the sticky status to ENODEV.
	 */
	gpio_init(cd_gpio);
	gpio_set_dir(cd_gpio, false);
	gpio_set_pulls(cs_gpio, true, false);
	gpio_set_irq_enabled_with_callback(cd_gpio, GPIO_IRQ_EDGE_RISE,
	    true, sd_cd_callback);

	/*
	 * Sticky status defaults to ENODEV or ENOTINIT, depending on
	 * current value of CD pin.
	 */
	sd_sticky_status = sd_card_present_p() ? SD_ENOTINIT : SD_ENODEV;

	sd_spi = spi;
	sd_cs_gpio = cs_gpio;
	sd_cd_gpio = cd_gpio;
}

int
sd_stat(void)
{
	if (! sd_card_present_p()) {
		return sd_set_status(SD_ENODEV);
	}
	return sd_sticky_status;
}

int
sd_blkcnt(uint32_t *valp)
{
	int error;

	if ((error = sd_stat()) != SD_NOERR) {
		return error;
	}
	*valp = sd_block_count;
	return SD_NOERR;
}

int
sd_eblksz(uint32_t *valp)
{
	int error;

	if ((error = sd_stat()) != SD_NOERR) {
		return error;
	}
	*valp = sd_eblock_size;
	return SD_NOERR;
}

int
sd_mount(void)
{
	uint32_t val32;
	uint8_t res;
	int error, cnt;

	if (! sd_card_present_p()) {
		return sd_set_status(SD_ENODEV);
	}

	/* Power up the card */
	sd_powerup();

	/* Put card in idle state. */
	for (cnt = 0;; cnt++) {
		if (cnt > 10) {
			return sd_set_status(SD_ETIMEDOUT);
		}
		res = sd_GO_IDLE_STATE();
		if (res == R1_IDLE_STATE) {
			break;
		}
	}

	/*
	 * Send interface conditions.  This is mandatory for SDHC / SDXC
	 * cards.
	 */
	for (cnt = 0;; cnt++) {
		if (cnt > 10) {
			return sd_set_status(SD_ENOTSUP);
		}
		res = sd_SEND_IF_COND(&val32);
		if (res == R1_IDLE_STATE) {
			break;
		}
	}

	/* Validate check pattern. */
	if (((val32 >> R7_ECHOBACK_SHIFT) & R7_ECHOBACK_MASK) != 0xaa) {
		return SD_EIO;
	}

	/* Initialize the card. */
	for (cnt = 0;; cnt++) {
		if (cnt > 100) {
			return sd_set_status(SD_ETIMEDOUT);
		}
		res = sd_SD_SEND_OP_COND();
		if (res == 0) {
			break;
		}
		busy_wait_ms(10);
	}

	/* Get OCR value. */
	res = sd_READ_OCR(&val32);
	if (res != 0) {
		return sd_set_status(SD_ENOTSUP);
	}
	if ((val32 & OCR_PU_nBUSY) == 0) {
		/* Card failed to initialize. */
		return sd_set_status(SD_ETIMEDOUT);
	}
	if ((val32 & OCR_CCS) == 0) {
		/* Not HC / XC card; reject these types. */
		return sd_set_status(SD_ENOTSUP);
	}

	int csd_vers;
	uint32_t block_count, eblock_size;

	error = sd_SEND_CSD(&csd_vers, &block_count, &eblock_size);
	if (error != SD_NOERR) {
		return sd_set_status(error);
	}
	sd_block_count = block_count;
	sd_eblock_size = eblock_size;

	if (csd_vers >= CSD_V2) {
		/*
		 * This isn't the real erase block size.  We need to
		 * get the AU_SIZE from the SD STATUS register.
		 */
		/* XXX */
	}

	return sd_set_status(SD_NOERR);
}

int
sd_rdblk(uint32_t blk, void *vbuf)
{
	int error;

	if ((error = sd_stat()) != SD_NOERR) {
		return error;
	}
	return sd_rdblk_CMD(17, blk, vbuf, SD_SECSIZE);
}

int
sd_wrblk(uint32_t blk, const void *vbuf)
{
	uint64_t timo;
	uint8_t res, token = 0xff;
	int error;

	if ((error = sd_stat()) != SD_NOERR) {
		return error;
	}

	sd_begin();

	sd_command(24, blk, 0);
	res = sd_recv_r1();
	if (res == 0) {
		/* Send data token. */
		sd_send_byte(0xfe);

		/* Send the data */
		sd_send(vbuf, SD_SECSIZE);

		/* Send dummy CRC (it's ignored). */
		sd_send_byte(0xff);
		sd_send_byte(0xff);

		/* Wait for response. */
		for (timo = 0, token = 0xff; token != 0xff;) {
			token = sd_recv_byte();
			if (sd_timed_out(&timo, 250)) {
				error = SD_ETIMEDOUT;
				goto out;
			}
		}
		if ((token & 0x1f) == 0x05) {
			/* Data accepted; wait for the write to complete. */
			for (timo = 0; sd_recv_byte() == 0;) {
				if (sd_timed_out(&timo, 250)) {
					error = SD_ETIMEDOUT;
					goto out;
				}
			}
		} else {
			/* Everything else maps to EIO. */
			error = SD_EIO;
		}
	}

 out:
	sd_end();
	return error;
}
