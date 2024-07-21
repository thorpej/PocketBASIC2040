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
 * TMS9918 VDP display routines.  These are largely lifted from my
 * 6809 Playground OS and translated from 6809 asm into C.
 */

/* Pico SDK headers */
#include "pico/stdlib.h"

/* Standard headers */
#include <stdio.h>
#include <string.h>

/* Local headers */
#include "pico9918-glue.h"
#include "vdp.h"

/*****************************************************************************
 * VDP registers
 *****************************************************************************/

#define	VDP_NREGS		8

#define	VDP_R0_EXTIN_EN		0x01
#define	VDP_R0_M3		0x02
#define	VDP_R0_MODEMASK		VDP_R0_M3

#define	VDP_R1_SPRITE_MAG	0x01
#define	VDP_R1_SPRITE_SIZE	0x02
/*				0x04		MBZ */
#define	VDP_R1_M2		0x08
#define	VDP_R1_M1		0x10
#define	VDP_R1_IE		0x20
#define	VDP_R1_SCREEN		0x40
#define	VDP_R1_16K		0x80
#define	VDP_R1_MODEMASK		(VDP_R1_SPRITE_MAG | VDP_R1_SPRITE_SIZE | \
				 VDP_R1_M2 | VDP_R1_M1)

/*
 * VIDEO MODES
 *
 *	M1	M2	M3
 *	0	0	0	Graphics I
 *	0	0	1	Graphics II
 *	0	1	0	Multicolor
 *	1	0	0	Text
 */

#define	VDP_VRAM_READ		0x00
#define	VDP_VRAM_WRITE		0x40

#define	VDP_REG_WRITE		0x80

/*
 * Name Table Base Address -- 4 MSB MBZ!
 *
 *  |  REGISTER 2   |
 *   0 0 0 0 x x x x 0 0 0 0 0 0 0 0 0 0
 *          |   14-bit VDP address      |
 */
#define	VDP_R2_NTBA_SHIFT	10

/*
 * Color Table Base Address
 *
 *  |  REGISTER 3   |
 *   x x x x x x x x 0 0 0 0 0 0
 *  |    14-bit VDP address     |
 */
#define	VDP_R3_CTBA_SHIFT	6

/*
 * Pattern Generator Base Address - 5 MSB MBZ!
 *
 *  |  REGISTER 4   |
 *   0 0 0 0 0 x x x 0 0 0 0 0 0 0 0 0 0 0
 *            |    14-bit VDP address     |
 */
#define	VDP_R4_PGBA_SHIFT	11

/*
 * Sprite Attribute Table Base Address - 1 MSB MBZ!
 *
 *  |  REGISTER 5   |
 *   0 x x x x x x x 0 0 0 0 0 0 0
 *    |    14-bit VDP address     |
 */
#define	VDP_R5_SATBA_SHIFT	7

/*
 * Sprite Pattern Generator Base Address - 5 MSB MBZ!
 *
 *  |  REGISTER 6   |
 *   0 0 0 0 0 x x x 0 0 0 0 0 0 0 0 0 0 0
 *            |    14-bit VDP address     |
 */
#define	VDP_R6_SPGBA_SHIFT	11

/*
 * Text Color
 * 4 MSB -- text color1
 * 4 LSB -- text color0 / backdrop color
 */
#define	VDP_R7_FGCOL_SHIFT	4

/*
 * Status register:
 */
#define	VDP_STS_5S_NUM		0x1f
#define	VDP_STS_C		0x20
#define	VDP_STS_5S		0x40
#define	VDP_STS_F		0x80

/*
 * Color codes:
 */
#define	VDP_COLOR_TRANS		0
#define	VDP_COLOR_BLACK		1
#define	VDP_COLOR_MED_GREEN	2
#define	VDP_COLOR_LT_GREEN	3
#define	VDP_COLOR_DK_BLUE	4
#define	VDP_COLOR_LT_BLUE	5
#define	VDP_COLOR_DK_RED	6
#define	VDP_COLOR_CYAN		7
#define	VDP_COLOR_MED_RED	8
#define	VDP_COLOR_LT_RED	9
#define	VDP_COLOR_DK_YELLOW	10
#define	VDP_COLOR_LT_YELLOW	11
#define	VDP_COLOR_DK_GREEN	12
#define	VDP_COLOR_MAGENTA	13
#define	VDP_COLOR_GRAY		14
#define	VDP_COLOR_WHITE		15

/*
 * Default definitions for Graphics 1 mode.  These come from Figure 6-2
 * of the 1984 "Video Display Processors Programmer's Guide".
 */
#define	VDP_G1_NTBA_DEFAULT	0x1400
#define	VDP_G1_CTBA_DEFAULT	0x2000
#define	VDP_G1_PGBA_DEFAULT	0x0800
#define	VDP_G1_SATBA_DEFAULT	0x1000
#define	VDP_G1_SPGBA_DEFAULT	0x0000
#define	VDP_G1_R0		0
#define	VDP_G1_R1		0

/*
 * Default definitions for Graphics 2 mode.  These come from Figure 6-3
 * of the 1984 "Video Display Processors Programmer's Guide".
 */
#define	VDP_G2_NTBA_DEFAULT	0x3800
#define	VDP_G2_CTBA_DEFAULT	0x2000
#define	VDP_G2_PGBA_DEFAULT	0x0000
#define	VDP_G2_SATBA_DEFAULT	0x3b00
#define	VDP_G2_SPGBA_DEFAULT	0x1800
#define	VDP_G2_R0		VDP_R0_M3
#define	VDP_G2_R1		0
#define	VDP_G2_R3_DEFAULT	0xff	/* RTFM */
#define	VDP_G2_R4_DEFAULT	0x03	/* RTFM */

/*
 * Default definitions for Multicolor mode.  These come from Figure 6-4
 * of the 1984 "Video Display Processors Programmer's Guide".
 */
#define	VDP_MC_NTBA_DEFAULT	0x1400
#define	VDP_MC_CTBA_DEFAULT	0x0000	/* don't care in MC mode */
#define	VDP_MC_PGBA_DEFAULT	0x0800
#define	VDP_MC_SATBA_DEFAULT	0x1000
#define	VDP_MC_SPGBA_DEFAULT	0x0000
#define	VDP_MC_R0		0
#define	VDP_MC_R1		VDP_R1_M2

/*
 * Default definitions for Text mode.  These come from Figure 6-5
 * of the 1984 "Video Display Processors Programmer's Guide".
 */
#define	VDP_TXT_NTBA_DEFAULT	0x0800
#define	VDP_TXT_CTBA_DEFAULT	0x0000	/* don't care in Text mode */
#define	VDP_TXT_PGBA_DEFAULT	0x0000
#define	VDP_TXT_SATBA_DEFAULT	0x0000	/* don't care in Text mode */
#define	VDP_TXT_SPGBA_DEFAULT	0x0000	/* don't care in Text mode */
#define	VDP_TXT_R0		0
#define	VDP_TXT_R1		VDP_R1_M1

/*****************************************************************************
 * VDP software descriptors
 *****************************************************************************/

struct vdp_modedesc {
	uint8_t		vdpm_regs[VDP_NREGS];
	uint32_t	vdpm_ntba;
	uint32_t	vdpm_ctba;
	uint32_t	vdpm_pgba;
	uint32_t	vdpm_satba;
	uint32_t	vdpm_spgba;
	void		(*vdpm_intr)(uint8_t);
};

struct vdp_softc {
	uint8_t		regs[VDP_NREGS];
	const struct vdp_modedesc *mode;
};

static struct vdp_softc vdp_softc_store;
static struct vdp_softc *vdp = &vdp_softc_store;

/*****************************************************************************/

static inline void
vdp_setreg(int reg, uint8_t val)
{
	pico9918_write_reg(val);
	pico9918_write_reg(reg & (VDP_NREGS - 1));
}

static void
vdp_set_mode(const struct vdp_modedesc *mode)
{
	/* Make sure the screen is off while we change modes. */
	vdp_setreg(1, 0);

	/* Copy the prototype regs into the cached copy. */
	for (int i = 0; i < VDP_NREGS; i++) {
		vdp->regs[i] = mode->vdpm_regs[i];
	}

	/* Always set 16K RAM. */
	vdp->regs[1] |= VDP_R1_16K;

	/* Always start with screen off an intrs disabled. */
	vdp->regs[1] &= (VDP_R1_SCREEN | VDP_R1_IE);

	vdp->mode = mode;

	/* Now program all of the registers. */
	for (int i = 0; i < VDP_NREGS; i++) {
		vdp_setreg(i, vdp->regs[i]);
	}
}

static inline void
vdp_set_rX_enab(int r, uint8_t bit, bool enab)
{
	if (enab) {
		vdp->regs[r] |= bit;
	} else {
		vdp->regs[r] &= ~bit;
	}
	vdp_setreg(r, vdp->regs[r]);
}

static inline void
vdp_set_screen(bool enab)
{
	vdp_set_rX_enab(1, VDP_R1_SCREEN, enab);
}

static inline void
vdp_set_intr(bool enab)
{
	vdp_set_rX_enab(1, VDP_R1_IE, enab);
}

static void
vdp_load_tiles(uint32_t ptb, const uint8_t *tiles, size_t ntiles)
{
	for (; ntiles > 0; ntiles--) {
		int tile = *tiles++;
		pico9918_copyin(ptb + (tile * 8), tiles, 8);
		tiles += 8;
	}
}

/*****************************************************************************/

static void	vdp_tty_intr(uint8_t);

const struct vdp_modedesc vdp_mode_text = {
	.vdpm_regs = {
		[0]	= VDP_TXT_R0,
		[1]	= VDP_TXT_R1,
		[2]	= VDP_TXT_NTBA_DEFAULT >> VDP_R2_NTBA_SHIFT,
		[3]	= VDP_TXT_CTBA_DEFAULT >> VDP_R3_CTBA_SHIFT,
		[4]	= VDP_TXT_PGBA_DEFAULT >> VDP_R4_PGBA_SHIFT,
		[5]	= VDP_TXT_SATBA_DEFAULT >> VDP_R5_SATBA_SHIFT,
		[6]	= VDP_TXT_SPGBA_DEFAULT >> VDP_R6_SPGBA_SHIFT,
		[7]	= (VDP_COLOR_WHITE << VDP_R7_FGCOL_SHIFT) |
			  VDP_COLOR_LT_BLUE,
	},
	.vdpm_ntba	= VDP_TXT_NTBA_DEFAULT,
	.vdpm_ctba	= VDP_TXT_CTBA_DEFAULT,
	.vdpm_pgba	= VDP_TXT_PGBA_DEFAULT,
	.vdpm_satba	= VDP_TXT_SATBA_DEFAULT,
	.vdpm_spgba	= VDP_TXT_SPGBA_DEFAULT,
	.vdpm_intr	= vdp_tty_intr,
};

#define	VDP_TTY_ROWS	24
#define	VDP_TTY_COLS	40

/*
 * The "phantom" row is the row that does not exist just below the
 * bottom of the screen.  We allow the row pointer to advance there
 * so we have a place to "park" when we fill the final character cell
 * at the bottom right -- we don't actually want to scroll in that
 * case until we try to put another character into the phantom row.
 */
#define	VDP_TTY_LASTROW	(VDP_TTY_ROWS - 1)
#define	VDP_TTY_LASTCOL	(VDP_TTY_COLS - 1)

#define	VDP_TTY_ROWOFF(r)	((r) * VDP_TTY_COLS)

#define	VDP_TTY_CURSOR_TICKS	30	/* 1/2 second cursor timer */

static bool vdp_tty_buf_locked;
static bool vdp_tty_buf_dirty;

static uint8_t vdp_tty_buf[VDP_TTY_ROWS * VDP_TTY_COLS];
static uint8_t *vdp_tty_pos;
static int vdp_tty_row;
static int vdp_tty_col;

static uint8_t vdp_tty_cursor_savechar;
static uint8_t vdp_tty_cursor_curchar;
static uint8_t vdp_tty_cursor_timer;

static inline void
vdp_tty_setpos(void)
{
	static uint8_t phantom_row = ' ';

	if (vdp_tty_row > VDP_TTY_LASTROW) {
		vdp_tty_pos = &phantom_row;
	} else {
		vdp_tty_pos =
		    &vdp_tty_buf[VDP_TTY_ROWOFF(vdp_tty_row) + vdp_tty_col];
	}

	/*
	 * Cache the character at this position now in case the
	 * cursor is drawn in this cell.
	 */
	vdp_tty_cursor_savechar = *vdp_tty_pos;

	/*
	 * If the cursor is enabled, make sure it's visible in the new
	 * location.
	 */
	if (vdp_tty_cursor_timer) {
		*vdp_tty_pos = 0;
	}
}

#define	VDP_TTY_LOCK()							\
	do {								\
		vdp_tty_buf_locked = true;				\
		__compiler_memory_barrier();				\
	} while (/*CONSTCOND*/0)

#define	VDP_TTY_UNLOCK_REPAINT()					\
	do {								\
		vdp_tty_buf_dirty = true;				\
		__compiler_memory_barrier();				\
		vdp_tty_buf_locked = false;				\
	} while (/*CONSTCOND*/0)

static void
vdp_tty_cursor_enable(void)
{
	if (vdp_tty_cursor_timer == 0) {
		vdp_tty_cursor_timer = VDP_TTY_CURSOR_TICKS;
	}
}

static inline void
vdp_tty_cursor_erase(void)
{
	*vdp_tty_pos = vdp_tty_cursor_savechar;
}

static void
vdp_tty_cursor_disable(void)
{
	vdp_tty_cursor_timer = 0;
	__compiler_memory_barrier();
	vdp_tty_cursor_erase();
}

void
vdp_tty_init(void)
{
	/* Set Text mode. */
	vdp_set_mode(&vdp_mode_text);

	/* Initialize the shadow buffer with spaces. */
	memset(vdp_tty_buf, ' ', sizeof(vdp_tty_buf));

	/* Ensure the cursor is disabled initially. */
	vdp_tty_cursor_disable();

	/* Paint the initial screen. */
	pico9918_copyin(vdp_mode_text.vdpm_ntba, vdp_tty_buf,
	    sizeof(vdp_tty_buf));

	/* Zero out the pattern table and load our sparse font. */
	pico9918_memset(vdp_mode_text.vdpm_pgba, 0, 256);
	vdp_load_tiles(vdp_mode_text.vdpm_pgba,
	    vdp_tty_font_tiles, vdp_tty_font_ntiles);

	/* Reset the current row/col. */
	vdp_tty_row = vdp_tty_col = 0;
	vdp_tty_setpos();

	/* Initialize the cursor. */
	vdp_tty_cursor_curchar = vdp_tty_cursor_savechar;
	vdp_tty_cursor_enable();

	/* Enable VSYNC interrupts and the screen. */
	vdp_set_intr(true);
	vdp_set_screen(true);
}

static void
__time_critical_func(vdp_tty_intr)(uint8_t sts)
{
	/*
	 * First, check to see if the frame buffer is locked due to
	 * being in-flux.  If so, we just get out now and check again
	 * on the next VSYNC interrupt.
	 */
	if (vdp_tty_buf_locked) {
		return;
	}

	bool repaint = vdp_tty_buf_dirty;

	/*
	 * Next, process the blinking cursor.  When the counter reaches
	 * 1, it's time to switch state.
	 */
	if (vdp_tty_cursor_timer == 1) {
		vdp_tty_cursor_timer = VDP_TTY_CURSOR_TICKS;
		if (vdp_tty_cursor_curchar == 0) {
			/* Cursor currently visible, switch to cell. */
			vdp_tty_cursor_curchar = vdp_tty_cursor_savechar;
		} else {
			/* Cell currently visible, switch to cursor. */
			vdp_tty_cursor_curchar = 0;
		}
		*vdp_tty_pos = vdp_tty_cursor_curchar;
		repaint = true;
	} else if (vdp_tty_cursor_timer) {
		vdp_tty_cursor_timer--;
	}

	if (repaint) {
		pico9918_copyin(vdp_mode_text.vdpm_ntba, vdp_tty_buf,
		    sizeof(vdp_tty_buf));
		vdp_tty_buf_dirty = false;
	}
}

static void
vdp_tty_scroll(void)
{
	/* Copy rows 1-lastRow up one row. */
	memcpy(vdp_tty_buf, &vdp_tty_buf[VDP_TTY_ROWOFF(1)],
	    sizeof(vdp_tty_buf) - VDP_TTY_COLS);

	/* Clear lastRow. */
	memset(&vdp_tty_buf[VDP_TTY_ROWOFF(VDP_TTY_LASTROW)], ' ',
	    VDP_TTY_COLS);
}

void
vdp_tty_putc(int ch)
{
	ch &= 0xff;

	if (ch >= ' ') {
		goto printable_ascii;
	}

	switch (ch) {
	case 7:		/* BEL (CTRL-G) */
		/* XXX Nothing yet. */
		return;

	case 8:		/* BS (CTRL-H) */
		/*
		 * Backspace is pretty straightforward, with just a couple
		 * of checks we need to make:
		 *
		 * -> If we're at a column > 0, we just decrement the
		 *    column and we're out.
		 *
		 * -> If we're at column 0, we may need to underflow.  If
		 *    we're already on row 0, do nothing.  Otherwise, we
		 *    underflow to lastCol and decrement our row.
		 */
		VDP_TTY_LOCK();
		vdp_tty_cursor_erase();
		if (vdp_tty_col > 0) {
			vdp_tty_col--;
			vdp_tty_setpos();
		} else {
			if (vdp_tty_row > 0) {
				vdp_tty_col = VDP_TTY_LASTCOL;
				vdp_tty_row--;
				vdp_tty_setpos();
			}
		}
		goto done;

	case 10:	/* LF (CTRL-J) */
		/*
		 * Line Feed is pretty straight forward, with just one
		 * special case we need to check for:
		 *
		 * If we're in the last row -or- the phantom row, we need
		 * to scroll and set ourselves to the last row.  Otherwise,
		 * we simply increment the row.
		 */
		VDP_TTY_LOCK();
		vdp_tty_cursor_erase();
		if (vdp_tty_row >= VDP_TTY_LASTROW) {
			vdp_tty_scroll();
			vdp_tty_row = VDP_TTY_LASTROW;
		} else {
			vdp_tty_row++;
		}
		vdp_tty_setpos();
		goto done;

	case 13:	/* CR (CTRL-M) */
		/*
		 * Carraige Return is really easy ... we just reset the
		 * column pointer to 0.  But we do need to check if we're
		 * on the phantom row; if we are, we go back to lastRow.
		 */
		VDP_TTY_LOCK();
		vdp_tty_cursor_erase();
		if (vdp_tty_row > VDP_TTY_LASTROW) {
			vdp_tty_row = VDP_TTY_LASTROW;
		}
		vdp_tty_col = 0;
		vdp_tty_setpos();
		goto done;

	default:
		return;
	}

 printable_ascii:
	VDP_TTY_LOCK();
	vdp_tty_cursor_erase();
	/*
	 * First we need to check if we're in the phantom row.  If so,
	 * we need to scroll the display.
	 */
	if (vdp_tty_row > VDP_TTY_LASTROW) {
		vdp_tty_scroll();
		vdp_tty_row = VDP_TTY_LASTROW;
		vdp_tty_setpos();
	}

	/*
	 * Store the character at the current position and advance the
	 * cursor.
	 */
	*vdp_tty_pos = (uint8_t)ch;
	if (vdp_tty_col < VDP_TTY_LASTCOL) {
		vdp_tty_col++;
	} else {
		vdp_tty_col = 0;
		vdp_tty_row++;		/* might now be in phantom row */
	}
	vdp_tty_setpos();

 done:
	VDP_TTY_UNLOCK_REPAINT();
}
