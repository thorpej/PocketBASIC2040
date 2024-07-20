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

/* Standard headers */
#include <stdio.h>

/* Local headers */
#include "pico9918-glue.h"
#include "tbvm.h"

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
	if (vf == TBVM_FILE_CONSOLE) {
		return getchar();
	}
	return EOF;
}

static void
jttb_putchar(void *vctx, void *vf, int ch)
{
	if (vf == TBVM_FILE_CONSOLE) {
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
 * Main entry point.
 *****************************************************************************/

int
main(void)
{

	stdio_uart_init();
	pico9918_glue_init();

	for (;;) {
		printf("%s, version %s\n", tbvm_name(), tbvm_version());

		vm = tbvm_alloc(NULL);
		tbvm_set_file_io(vm, &jttb_file_io);
		tbvm_exec(vm);
		tbvm_free(vm);
	}
}
