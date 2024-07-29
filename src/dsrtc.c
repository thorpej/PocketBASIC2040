/*
 * Copyright (c) 2003 Wasabi Systems, Inc.
 * All rights reserved.
 *
 * Written by Steve C. Woodford and Jason R. Thorpe for Wasabi Systems, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed for the NetBSD Project by
 *      Wasabi Systems, Inc.
 * 4. The name of Wasabi Systems, Inc. may not be used to endorse
 *    or promote products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY WASABI SYSTEMS, INC. ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL WASABI SYSTEMS, INC
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Dallas DS3231/DS3232 I2C Real Time Clock driver for RP2040.
 *
 * This is essentially a heavily-slimmed-down copy of the driver from
 * NetBSD, and only supports the RTC functionality.
 */

/* Pico SDK headers */
#include "pico/stdlib.h"
#include "hardware/i2c.h"

/* Standard headers */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* Local headers */
#include "dsrtc.h"

#define	DSXXXX_ADDR		0x68	/* fixed i2c device address */

#define	DSXXXX_SECONDS		0x00
#define	DSXXXX_MINUTES		0x01
#define	DSXXXX_HOURS		0x02
#define	DSXXXX_DAY		0x03
#define	DSXXXX_DATE		0x04
#define	DSXXXX_MONTH		0x05
#define	DSXXXX_YEAR		0x06
#define	DSXXXX_RTC_SIZE		7

#define	DS3232_CONTROL		0x0e
#define	DS3232_CSR		0x0f
#define	DS3232_RTC_START	0
#define	DS3232_RTC_SIZE		DSXXXX_RTC_SIZE
#define	DS3232_TEMP_MSB		0x11
#define	DS3232_TEMP_LSB		0x12
#define	DS3232_NVRAM_START	0x14
#define	DS3232_NVRAM_SIZE	0xec

/*
 * Bit definitions.
 */
#define	DSXXXX_SECONDS_MASK	0x7f
#define	DSXXXX_MINUTES_MASK	0x7f
#define	DSXXXX_HOURS_12HRS_MODE	(1u << 6)	/* Set for 12 hour mode */
#define	DSXXXX_HOURS_12HRS_PM	(1u << 5)	/* If 12 hr mode, set = PM */
#define	DSXXXX_HOURS_12MASK	0x1f
#define	DSXXXX_HOURS_24MASK	0x3f
#define	DSXXXX_DAY_MASK		0x07
#define	DSXXXX_DATE_MASK	0x3f
#define	DSXXXX_MONTH_MASK	0x1f
#define	DSXXXX_MONTH_CENTURY	0x80

#define	POSIX_BASE_YEAR		1970

static i2c_inst_t *dsrtc_i2c;

static inline unsigned int
bcdtobin(unsigned int bcd)
{
	return ((bcd >> 4) & 0x0f) * 10 + (bcd & 0x0f);
}

static inline unsigned int
bintobcd(unsigned int bin)
{
	return (((bin / 10) << 4) & 0xf0) | (bin % 10);
}

static uint8_t
dsrtc_reg_read(uint8_t reg, bool nostop)
{
	int rv = 0;
	uint8_t val;

	rv = i2c_write_blocking(dsrtc_i2c, DSXXXX_ADDR, &reg, 1, true);
	if (rv == 1) {
		i2c_read_blocking(dsrtc_i2c, DSXXXX_ADDR, &val, 1, nostop);
	}
	return val;
}

static void
dsrtc_reg_write(uint8_t reg, uint8_t val, bool nostop)
{
	int rv = 0;
	uint8_t cmdbuf[2] = { reg, val };

	i2c_write_blocking(dsrtc_i2c, DSXXXX_ADDR, cmdbuf, 2, nostop);
}

static void
dsrtc_clock_read(datetime_t *t)
{
	uint8_t bcd[DSXXXX_RTC_SIZE];

	memset(bcd, 0, sizeof(bcd));

	for (uint i = 0; i < DS3232_RTC_SIZE; i++) {
		bcd[i] = dsrtc_reg_read(DS3232_RTC_START + i, false);
	}

	t->sec = bcdtobin(bcd[DSXXXX_SECONDS] & DSXXXX_SECONDS_MASK);
	t->min = bcdtobin(bcd[DSXXXX_MINUTES] & DSXXXX_MINUTES_MASK);

	if ((bcd[DSXXXX_HOURS] & DSXXXX_HOURS_12HRS_MODE) != 0) {
		t->hour = bcdtobin(bcd[DSXXXX_HOURS] &
		    DSXXXX_HOURS_12MASK) % 12; /* 12AM -> 0, 12PM -> 12 */
		if (bcd[DSXXXX_HOURS] & DSXXXX_HOURS_12HRS_PM) {
			t->hour += 12;
		}
	} else {
		t->hour = bcdtobin(bcd[DSXXXX_HOURS] & DSXXXX_HOURS_24MASK);
	}

	t->day = bcdtobin(bcd[DSXXXX_DATE] & DSXXXX_DATE_MASK);
	t->dotw = bcdtobin(bcd[DSXXXX_DAY] & DSXXXX_DAY_MASK);
	t->month = bcdtobin(bcd[DSXXXX_MONTH] & DSXXXX_MONTH_MASK);

	/* Color me a traditionalist :-). */
	t->year = bcdtobin(bcd[DSXXXX_YEAR]) + POSIX_BASE_YEAR;
	if (bcd[DSXXXX_MONTH] & DSXXXX_MONTH_CENTURY) {
		t->year += 100;
	}
}

static void
dsrtc_clock_write(const datetime_t *t)
{
	uint8_t bcd[DSXXXX_RTC_SIZE];

	bcd[DSXXXX_SECONDS] = bintobcd(t->sec);
	bcd[DSXXXX_MINUTES] = bintobcd(t->min);
	bcd[DSXXXX_HOURS] = bintobcd(t->hour); /* DSXXXX_HOURS_12HRS_MODE=0 */
	bcd[DSXXXX_DATE] = bintobcd(t->day);
	bcd[DSXXXX_DAY] = bintobcd(t->dotw);
	bcd[DSXXXX_MONTH] = bintobcd(t->month);
	bcd[DSXXXX_YEAR] = bintobcd((t->year - POSIX_BASE_YEAR) % 100);
	if (t->year - POSIX_BASE_YEAR >= 100) {
		bcd[DSXXXX_MONTH] |= DSXXXX_MONTH_CENTURY;
	}

	/* Stop the clock. */
	dsrtc_reg_write(DS3232_RTC_START + DSXXXX_SECONDS, 0, true);

	/*
	 * Write the registers in the reverse order.  The last write
	 * (to the seconds register) will undo the clock hold.
	 */
	for (int i = DS3232_RTC_SIZE - 1; i >= 0; i--) {
		dsrtc_reg_write(DS3232_RTC_START + i, bcd[i],
		    i == DSXXXX_SECONDS ? false : true);
	}
}

void
dsrtc_init(i2c_inst_t *i2c)
{
	dsrtc_i2c = i2c;
}

void
dsrtc_gettime(datetime_t *t)
{
	datetime_t check;

	memset(t, 0, sizeof(*t));
	memset(&check, 0, sizeof(check));

	/*
	 * We're not using Burst Read, so read the clock twice until we get
	 * two consecutive identical results.
	 */
	int retries = 5;
	do {
		dsrtc_clock_read(t);
		dsrtc_clock_read(&check);
	} while (memcmp(t, &check, sizeof(check)) != 0 && --retries);

	return 0;
}

void
dsrtc_settime(const datetime_t *t)
{
	dsrtc_clock_write(t);
}
