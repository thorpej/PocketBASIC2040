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
#include "tbvm.h"
#include "vdp.h"

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
 *	GP20		I2C0 SDA		"future expansion"	26
 *	GP21		I2C0 SCL					27
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
 * Tiny-ish BASIC interfaces.
 *****************************************************************************/

static tbvm	*vm;

static void *
jttb_openfile(void *vctx, const char *fname, const char *mode)
{
	return NULL;
}

static void
jttb_closefile(void *vctx, void *vf)
{
	if (vf != TBVM_FILE_CONSOLE) {
		/* XXX */
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
		return rv;
	}
	return EOF;
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

	for (;;) {
		vm = tbvm_alloc(NULL);
		tbvm_set_file_io(vm, &jttb_file_io);
		tbvm_exec(vm);
		tbvm_free(vm);
	}
}
