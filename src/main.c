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
 * PocketBASIC2040 -- A Raspberry Pi RP2040-based BASIC computer running
 * Jason's Tiny-ish BASIC.
 */

/* Pico SDK headers */
#include "pico/stdlib.h"
#include "pico/stdio_uart.h"
#include "pico/printf.h"
#include "pico/multicore.h"

/* Standard headers */
#include <stdio.h>

/* Local headers */
#include "pico9918-glue.h"
#include "dsrtc.h"
#include "sd.h"
#include "tbvm.h"
#include "vdp.h"

#include "ff.h"

/*
 * We need the following GPIOs for hardware functions.  The PIO VGA interface
 * needs 2 contiguous GPIOs for sync signals and 12 contiguous GPIOs for the
 * 3 4-bit RGB signals.  XXX These are currently hard-coded as GP0-GP13 in the
 * VGA submodule, will will have to be changed.
 *
 *   RP2040 GPIO        FUNCTION                Purpose              Pico pin
 * ============================================================================
 *	GP0		UART0 TX		Serial console		1
 *	GP1		UART1 RX					2
 *
 *	GP2		PIO0 SM1		VGA RGB data		4
 *	GP3		PIO0 SM1					5
 *	GP4		PIO0 SM1					6
 *	GP5		PIO0 SM1					7
 *	GP6		PIO0 SM1					9
 *	GP7		PIO0 SM1					10
 *	GP8		PIO0 SM1					11
 *	GP9		PIO0 SM1					12
 *	GP10		PIO0 SM1					14
 *	GP11		PIO0 SM1					15
 *	GP12		PIO0 SM1					16
 *	GP13		PIO0 SM1					17
 *
 *	GP14		PIO0 SM0		VGA SYNC data		19
 *	GP15		PIO0 SM0					20
 *
 *	GP16		SPI0 MISO		SD card interface	21
 *	GP17		SPI0 CSn					22
 *	GP18		SPI0 SCK					24
 *	GP19		SPI0 MOSI					25
 *
 *	GP20		I2C0 SDA		RTC, etc.		26
 *	GP21		I2C0 SCL					27
 *
 *	GP22		SIO			SD Card Detect		29
 */

#define	PIN_UART0_TX	0
#define	PIN_UART0_RX	1

#define	PIN_PIO0_FIRST	2
#define	PIN_PIO0_LAST	15

#define	PIN_SPI0_MISO	16
#define	PIN_SPI0_CSn	17
#define	PIN_SPI0_SCK	18
#define	PIN_SPI0_MOSI	19

#define	PIN_I2C0_SDA	20
#define	PIN_I2C0_SCL	21

#define	PIN_SD_CDn	22

static const struct gpio_pin_config {
	unsigned int       first;
	unsigned int       last;
	enum gpio_function func;
} gpio_pin_config[] = {
	{ PIN_UART0_TX,		PIN_UART0_RX,		GPIO_FUNC_UART },

	{ PIN_PIO0_FIRST,	PIN_PIO0_LAST,		GPIO_FUNC_PIO0 },

	/*
	 * SPI is special here, because we need more direct control
	 * over the CS line.
	 */
	{ PIN_SPI0_MISO,	PIN_SPI0_MISO,		GPIO_FUNC_SPI },
	{ PIN_SPI0_CSn,		PIN_SPI0_CSn,		GPIO_FUNC_SIO },
	{ PIN_SPI0_SCK,		PIN_SPI0_MOSI,		GPIO_FUNC_SPI },

	{ PIN_I2C0_SDA,		PIN_I2C0_SCL,		GPIO_FUNC_I2C },

	{ PIN_SD_CDn,		PIN_SD_CDn,		GPIO_FUNC_SIO },
};
static const size_t ngpio_pin_config =
    sizeof(gpio_pin_config) / sizeof(gpio_pin_config[0]);

static void
config_gpio(void)
{
	for (size_t x = 0; x < ngpio_pin_config; x++) {
		const struct gpio_pin_config *c = &gpio_pin_config[x];
		for (unsigned int i = c->first; i <= c->last; i++) {
			gpio_set_function(i, c->func);
		}
	}
}

/*****************************************************************************
 * FatFs glue
 *****************************************************************************/

static FATFS fat0;

DWORD
get_fattime(void)
{
	datetime_t t;

	dsrtc_gettime(&t);

	return (((DWORD)t.year - 1980) << 25) |
	        ((DWORD)t.month        << 21) |
		((DWORD)t.day          << 16) |
		((DWORD)t.hour         << 11) |
		((DWORD)t.min          <<  5) |
		((DWORD)t.sec               );
}

/* This implements a simple buffered I/O layer for FatFs. */

#define	FATFS_FILEBUF_SIZE	1024
#define	FATFS_FILEBUF_UNK	0
#define	FATFS_FILEBUF_READ	1
#define	FATFS_FILEBUF_WRITE	2
#define	FATFS_FILEBUF_EOF	3

struct fatfs_file {
	FIL	f_fil;
	int	f_mode;
	char	f_buf[FATFS_FILEBUF_SIZE];
	uint	f_bufsize;	/* only used for READ */
	uint	f_bufidx;
	FSIZE_t	f_bufoff;
	int	f_bufdir;
};

static inline bool
fatfs_file_dirty_p(struct fatfs_file *f)
{
	return ((f->f_mode & FA_WRITE) != 0 &&
	        f->f_bufdir == FATFS_FILEBUF_WRITE &&
		f->f_bufidx != 0);
}

static void
fatfs_file_clean(struct fatfs_file *f)
{
	if (fatfs_file_dirty_p(f)) {
		UINT actual;
		FRESULT res;

		res = f_write(&f->f_fil, f->f_buf, f->f_bufidx, &actual);
		(void)res;	/* XXX report error */
	}
	f->f_bufsize = 0;
	f->f_bufidx = 0;
	f->f_bufoff = f_tell(&f->f_fil);
	f->f_bufdir = FATFS_FILEBUF_UNK;
}

static void
fatfs_file_load(struct fatfs_file *f)
{
	UINT actual;
	FRESULT res;

	fatfs_file_clean(f);
	actual = 0;
	res = f_read(&f->f_fil, f->f_buf, FATFS_FILEBUF_SIZE, &actual);
	(void)res;		/* XXX report error */
	f->f_bufsize = actual;
	f->f_bufidx = 0;
	f->f_bufdir = actual != 0 ? FATFS_FILEBUF_READ : FATFS_FILEBUF_EOF;
}

static int
fatfs_file_getc(struct fatfs_file *f)
{
	int rv;

	if ((f->f_mode & FA_READ) == 0) {
		return EOF;
	}

	switch (f->f_bufdir) {
	case FATFS_FILEBUF_WRITE:
	case FATFS_FILEBUF_UNK:
		fatfs_file_load(f);
		break;

	default:
		break;
	}

	if (f->f_bufdir == FATFS_FILEBUF_EOF) {
		return EOF;
	}

	rv = f->f_buf[f->f_bufidx++];
	if (f->f_bufidx == f->f_bufsize) {
		fatfs_file_load(f);
	}
	return rv;
}

static void
fatfs_file_putc(struct fatfs_file *f, int ch)
{
	if ((f->f_mode & FA_WRITE) == 0) {
		/* XXX report error */
		return;
	}

	switch (f->f_bufdir) {
	case FATFS_FILEBUF_UNK:
		f->f_bufdir = FATFS_FILEBUF_WRITE;
		/* FALLTHROUGH */

	case FATFS_FILEBUF_WRITE:
		break;

	default:
		fatfs_file_clean(f);
		f->f_bufdir = FATFS_FILEBUF_WRITE;
		break;
	}

	f->f_buf[f->f_bufidx++] = (char)ch;
	if (f->f_bufidx == FATFS_FILEBUF_SIZE) {
		fatfs_file_clean(f);
	}
}

/*****************************************************************************
 * Tiny-ish BASIC interfaces.
 *****************************************************************************/

static tbvm	*vm;

static int
mode2fatfs(const char *mode)
{
	bool in_p = false;
	bool out_p = false;
	const char *cp;

	for (cp = mode; *cp != '\0'; cp++) {
		switch (*cp) {
		case 'i':
		case 'I':
			in_p = true;
			break;

		case 'o':
		case 'O':
			out_p = true;
			break;

		default:
			break;
		}
	}

	if (in_p && out_p) {
		return FA_READ | FA_WRITE;
	} else if (in_p) {
		return FA_READ;
	} else if (out_p) {
		return FA_CREATE_ALWAYS | FA_WRITE;
	} else {
		return -1;
	}
}

static void *
jttb_openfile(void *vctx, const char *fname, const char *mode)
{
	BYTE fmode = mode2fatfs(mode);
	if (fmode == -1) {
		return NULL;
	}

	struct fatfs_file *f = calloc(1, sizeof(struct fatfs_file));
	if (f == NULL) {
		return NULL;
	}

	FRESULT res = f_open(&f->f_fil, fname, fmode);
	if (res != FR_OK) {
		free(f);
		f = NULL;
	}
	f->f_mode = fmode;
	return f;
}

static void
jttb_closefile(void *vctx, void *vf)
{
	if (vf != TBVM_FILE_CONSOLE) {
		struct fatfs_file *f = vf;
		fatfs_file_clean(f);
		f_close(&f->f_fil);
		free(f);
	}
}

static int
jttb_getchar(void *vctx, void *vf)
{
	int rv;

	if (vf == TBVM_FILE_CONSOLE) {
 again:
		rv = getchar();
		switch (rv) {
		case 0x03:	/* CTRL-C -> BREAK */
			rv = TBVM_BREAK;
			break;

		case 0x08:	/* pass CTRL-H (backspace) */
		case 0x09:	/* pass CTRL-I (horizontal tab) */
		case 0x0d:	/* pass CTRL-M (carraige return) */
			break;

		default:
			/* space -> ~ are OK, discard everything else. */
			if (rv < 0x20 || rv > 0x7e) {
				goto again;
			}
			break;
		}
	} else {
		struct fatfs_file *f = vf;
		rv = fatfs_file_getc(f);
	}
	return rv;
}

static void
jttb_putchar(void *vctx, void *vf, int ch)
{
	if (vf == TBVM_FILE_CONSOLE) {
		/* Out to the VDP TTY. */
		switch (ch) {
		case '\b':	/* CTRL-H (backspace) */
			vdp_tty_putc('\b');
			vdp_tty_putc(' ');
			vdp_tty_putc('\b');
			break;

		case '\n':
			vdp_tty_putc('\r');
			vdp_tty_putc('\n');
			break;

		default:
			vdp_tty_putc(ch);
			break;
		}

		/* ...and out to the Pico's UART. */
		putchar(ch);
	} else {
		struct fatfs_file *f = vf;
		fatfs_file_putc(f, ch);
	}
}

static bool
jttb_check_break(void *vctx, void *vf)
{
	bool rv = false;

	if (vf == TBVM_FILE_CONSOLE) {
		/* XXX */
	}

	return rv;
}

static const struct tbvm_file_io jttb_file_io = {
	.io_openfile = jttb_openfile,
	.io_closefile = jttb_closefile,
	.io_getchar = jttb_getchar,
	.io_putchar = jttb_putchar,
	.io_check_break = jttb_check_break,
};

/*****************************************************************************
 * VSYNC interrupt handler.
 *****************************************************************************/

static void
__time_critical_func(core0_fifo_irq_handler)(void)
{
	uint32_t vdp_status;

	/* Get the most recent value. */
	while (multicore_fifo_rvalid()) {
		vdp_status = multicore_fifo_pop_blocking();
	}
	multicore_fifo_clear_irq();

	/*
	 * We know STATUS_INT is set.  Eventually we could check
	 * the sprite-related bits, too.
	 */

	vdp_intr((uint8_t)vdp_status);
}

/*****************************************************************************
 * Main entry point.
 *****************************************************************************/

static const char version_string[] = "0.1";

int
main(void)
{
	config_gpio();

	stdio_uart_init();

	printf("PocketBASIC2040, version %s\n", version_string);
	printf("%s, version %s\n", tbvm_name(), tbvm_version());

	/* Register an interrupt handler for the FIFO and enable it. */
	irq_set_exclusive_handler(SIO_IRQ_PROC0, core0_fifo_irq_handler);
	irq_set_enabled(SIO_IRQ_PROC0, true);

	/* Initialize the virtual 9918 VDP. */
	pico9918_init();

	/* Initialize the VDP TTY. */
	vdp_tty_init(40);

	/* Initialize I2C and peripherals. */
	i2c_init(i2c0, 100000);		/* 100KHz */
	dsrtc_init(i2c0);

	/* Initialize the SD card interface. */
	sd_init(spi0, PIN_SPI0_CSn, PIN_SD_CDn);

	/* Mount the file system (lazily). */
	f_mount(&fat0, "", 0);

	for (;;) {
		vm = tbvm_alloc(NULL);
		tbvm_set_file_io(vm, &jttb_file_io);
		tbvm_exec(vm);
		tbvm_free(vm);
	}
}
