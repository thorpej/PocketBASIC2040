/*
 * Derived from: pico9918
 *
 * Copyright (c) 2024 Troy Schrapel
 * Copyright (c) 2024 Jason R. Thorpe
 *
 * This code is licensed under the MIT license
 *
 * https://github.com/visrealm/pico9918
 */

/*
 * This code is lifted from the pico9918 main.c and tweaked to provide
 * pico9918 glue to pocketbasic2040.
 */

#include <stdbool.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/printf.h"
#include "pico/multicore.h"

#include "pico9918-glue.h"

#include "vga.h"
#include "vga-modes.h"

#include "palette.h"

#include "vrEmuTms9918Priv.h"

/*
 * This differs from what is used in the pico9918 project, although
 * the resulting frequency is the same.  I used:
 *
 * dhcp-194:thorpej$ rp2_common/hardware_clocks/scripts/vcocalc.py 252
 * Requested: 252.0 MHz
 * Achieved: 252.0 MHz
 * REFDIV: 1
 * FBDIV: 126 (VCO = 1512.0 MHz)
 * PD1: 6
 * PD2: 1
 * dhcp-194:thorpej$
 */
#define	PICO_CLOCK_PLL_VCO  1512000000
#define	PICO_CLOCK_PLL_DIV1 6
#define	PICO_CLOCK_PLL_DIV2 1
#define	PICO_CLOCK_HZ       (PICO_CLOCK_PLL_VCO / PICO_CLOCK_PLL_DIV1 / PICO_CLOCK_PLL_DIV2)

  /* file globals */
static uint8_t nextValue = 0;     /* TMS9918A read-ahead value */
static uint8_t currentStatus = 0; /* current status register value */

static uint8_t __aligned(4) tmsScanlineBuffer[TMS9918_PIXELS_X];

static inline void
pico9918_post_interrupt(void)
{
  /*
   * Push the currentStatus through the FIFO to Core 0, and treat it
   * like this was a read of the STATUS register (i.e. clears the
   * register).
   */
  uint32_t val = currentStatus;
  currentStatus = 0;
  vrEmuTms9918SetStatusImpl(currentStatus);
  multicore_fifo_push_timeout_us(val, 0);
}

static inline void
pico9918_block_fifo_irq(void)
{
  __dmb();
  *((io_rw_32 *) (PPB_BASE + M0PLUS_NVIC_ICPR_OFFSET)) = 1U << SIO_IRQ_PROC1;
  *((io_rw_32 *) (PPB_BASE + M0PLUS_NVIC_ISER_OFFSET)) = 1U << SIO_IRQ_PROC1;
}

static inline void
pico9918_unblock_fifo_irq(void)
{
  *((io_rw_32 *) (PPB_BASE + M0PLUS_NVIC_ICER_OFFSET)) = 1U << SIO_IRQ_PROC1;
  __dmb();
}

#define	REG_WRITE	(1U << 31)	/* this is a register write */
#define	REG_MODE	(1U << 8)	/* 1 = reg/addr, 0 = data */
#define	REG_VAL		0xff		/* register value on write */

static void
__time_critical_func(core1_fifo_irq_handler)(void)
{
  uint32_t fdata;

  while (multicore_fifo_rvalid()) {
    fdata = multicore_fifo_pop_blocking();
    if (fdata & REG_WRITE) {
      if (fdata & REG_MODE) {
        /* reg/addr was written. */
        vrEmuTms9918WriteAddrImpl(fdata & REG_VAL);
        /*
         * Don't bother posting an interrupte to Core 0 here if it were to
         * now be enabled.  We will notice at the next end-of-frame and post
         * it then.
         */
      } else {
        /* DATA was written. */
        vrEmuTms9918WriteDataImpl(fdata & REG_VAL);
        nextValue = vrEmuTms9918ReadDataNoIncImpl();
      }
    } else {
      if (fdata & REG_MODE) {
        /* STATUS register was read. */
        currentStatus = 0;
        vrEmuTms9918SetStatusImpl(currentStatus);
      } else {
        /* DATA was read. */
        vrEmuTms9918ReadDataImpl();
        nextValue = vrEmuTms9918ReadDataNoIncImpl();
      }
    }
  }
  multicore_fifo_clear_irq();
}

/*
 * Generate a single VGA scanline (called by vgaLoop(), runs on proc1).
 */
static void
__time_critical_func(tmsScanline)(uint16_t y, VgaParams *params,
				  uint16_t* pixels)
{
#if 1
	// better compile-time optimizations if we hard-code these
#define	VIRTUAL_PIXELS_X	640
#define	VIRTUAL_PIXELS_Y	240
#else
#define	VIRTUAL_PIXELS_X	params->hVirtualPixels
#define	VIRTUAL_PIXELS_Y	params->vVirtualPixels
#endif

  const uint32_t vBorder = (VIRTUAL_PIXELS_Y - TMS9918_PIXELS_Y) / 2;
  const uint32_t hBorder = (VIRTUAL_PIXELS_X - TMS9918_PIXELS_X * 2) / 2;

  uint32_t bg = tms9918PaletteBGR12[vrEmuTms9918RegValue(TMS_REG_FG_BG_COLOR) & 0x0f];

  /*** top and bottom borders ***/
  if (y < vBorder || y >= (vBorder + TMS9918_PIXELS_Y)) {
    for (int x = 0; x < VIRTUAL_PIXELS_X; ++x) {
      pixels[x] = bg;
    }
    return;
  }

  y -= vBorder;

  /*** left border ***/
  for (int x = 0; x < hBorder; ++x) {
    pixels[x] = bg;
  }

  /*** main display region ***/

  /* generate the scanline */
  uint8_t tempStatus = vrEmuTms9918ScanLine(y, tmsScanlineBuffer);

  /*** interrupt signal? ***/
  if (y == TMS9918_PIXELS_Y - 1) {
    tempStatus |= STATUS_INT;
  }

  /* convert from palette to bgr12 */
  int tmsX = 0;
  if (tmsScanlineBuffer[0] & 0xf0) {
    for (int x = hBorder; x < hBorder + TMS9918_PIXELS_X * 2; x += 2, ++tmsX) {
      pixels[x] = tms9918PaletteBGR12[(tmsScanlineBuffer[tmsX] & 0xf0) >> 4];
      pixels[x + 1] = tms9918PaletteBGR12[tmsScanlineBuffer[tmsX] & 0x0f];
    }
  } else {
    for (int x = hBorder; x < hBorder + TMS9918_PIXELS_X * 2; x += 2, ++tmsX) {
      pixels[x] = tms9918PaletteBGR12[tmsScanlineBuffer[tmsX]];
      pixels[x + 1] = pixels[x];
    }
  }

  /*** right border ***/
  for (int x = hBorder + TMS9918_PIXELS_X * 2; x < VIRTUAL_PIXELS_X; ++x) {
    pixels[x] = bg;
  }

  pico9918_block_fifo_irq();
  if ((currentStatus & STATUS_INT) == 0) {
    if ((currentStatus & STATUS_5S) != 0) {
      currentStatus |= tempStatus & 0xe0;
    } else {
      currentStatus |= tempStatus;
    }

    vrEmuTms9918SetStatusImpl(currentStatus);
    if (vrEmuTms9918InterruptStatusImpl()) {
      pico9918_post_interrupt();
    }
  }
  pico9918_unblock_fifo_irq();
}

/*
 * 2nd CPU core (proc1) entry
 */
static void
proc1Entry(void)
{
  /* Wait untill everything is ready on Core 0. */
  multicore_fifo_pop_blocking();

  /* Now register an interrupt handler for the FIFO and enable it. */
  irq_set_exclusive_handler(SIO_IRQ_PROC1, core1_fifo_irq_handler);
  irq_set_enabled(SIO_IRQ_PROC1, true);

  /* Now run the VGA loop. */
  vgaLoop();
}

void
pico9918_init(void)
{
  /*
   * Curently, VGA hard-coded to 640x480@60Hz.  We want a high clock frequency
   * that comes close to being divisible by 25.175MHz. 252.0 is close enough.
   *
   * Troy has code which sets the best clock based on the chosen VGA mode,
   * but this'll do for now, and any faster than this, we'd probably have to
   * start fiddling with running at a higher core voltage.
   *
   * (See near top of file where how the PLL configuration values are
   * derived.)
   */
  set_sys_clock_pll(PICO_CLOCK_PLL_VCO, PICO_CLOCK_PLL_DIV1,
                    PICO_CLOCK_PLL_DIV2);
  printf("Initialized clock to %d MHz (VCO=%d MHz, PD1=%d, PD2=%d)\n",
         PICO_CLOCK_HZ, PICO_CLOCK_PLL_VCO, PICO_CLOCK_PLL_DIV1,
	 PICO_CLOCK_PLL_DIV2);

  /* we need one of these. it's the main guy */
  vrEmuTms9918Init();

  /* launch core 1 which handles rendering scanlines */
  printf("Launching VDP process on Core 1.\n");
  multicore_reset_core1();
  multicore_launch_core1(proc1Entry);

  /* then set up VGA output */
  VgaInitParams params = { 0 };
  params.params = vgaGetParams(VGA_640_480_60HZ);

  /* virtual size will be 640 x 240 to accomodate 80-column mode */
  setVgaParamsScaleY(&params.params, 2);

  /* set vga scanline callback to generate tms9918 scanlines */
  params.scanlineFn = tmsScanline;

  vgaInit(params);

  /* signal proc1 that we're ready to start the display */
  printf("Starting VDP processing on Core 1.\n");
  multicore_fifo_push_timeout_us(0, 0);

  /*
   * Everything on the video side is handled with interrupts.  Return
   * to the caller who will run the main application loop.
   */
}

uint8_t
pico9918_read_status(void)
{
  /*
   * We don't expect this to be used too often because we're pushing
   * the STATUS register value directly to the end-of-frame interrupt
   * on Core 0.  Note the status register will be cleared in Core 1's
   * interrupt handler.
   */
  uint8_t rv = currentStatus;
  multicore_fifo_push_blocking(REG_MODE);
}

/*
 * There is absolutely no reason to provide a pico9918_read_data()
 * function.
 */

void
pico9918_write_reg(uint8_t val)
{
  multicore_fifo_push_blocking(REG_WRITE | REG_MODE | val);
}

void
pico9918_write_data(uint8_t val)
{
  /*
   * We expect this to not be used very often, instead favoring
   * direct access to the virtual 9918's VRAM.
   */
  multicore_fifo_push_blocking(REG_WRITE | val);
}

void
pico9918_copyin(uint32_t vram_addr, const void *buf, size_t len)
{
  vram_addr &= VRAM_MASK;
  if (len > VRAM_SIZE - vram_addr) {
    len = VRAM_SIZE - vram_addr;
  }
  memcpy(&tms9918->vram[vram_addr], buf, len);
}

void
pico9918_memset(uint32_t vram_addr, uint8_t val, size_t len)
{
  vram_addr &= VRAM_MASK;
  if (len > VRAM_SIZE - vram_addr) {
    len = VRAM_SIZE - vram_addr;
  }
  memset(&tms9918->vram[vram_addr], val, len);
}
