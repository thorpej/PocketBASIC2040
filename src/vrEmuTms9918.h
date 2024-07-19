/*
 * Troy's TMS9918 Emulator - Core interface
 *
 * Copyright (c) 2021 Troy Schrapel
 *
 * This code is licensed under the MIT license
 *
 * https://github.com/visrealm/vrEmuTms9918
 *
 */

#ifndef _VR_EMU_TMS9918_H_
#define _VR_EMU_TMS9918_H_

/* ------------------------------------------------------------------
 * LINKAGE MODES:
 * 
 * Default (nothing defined):    When your executable is using vrEmuTms9918 as a DLL
 * VR_6502_EMU_COMPILING_DLL:    When compiling vrEmuTms9918 as a DLL
 * VR_6502_EMU_STATIC:           When linking vrEmu6502 statically in your executable
 */

#if __EMSCRIPTEN__
#include <emscripten.h>
  #ifdef __cplusplus
  #define VR_EMU_TMS9918_DLLEXPORT EMSCRIPTEN_KEEPALIVE extern "C"
  #define VR_EMU_TMS9918_DLLEXPORT_CONST extern "C"
#else
  #define VR_EMU_TMS9918_DLLEXPORT EMSCRIPTEN_KEEPALIVE extern
  #define VR_EMU_TMS9918_DLLEXPORT_CONST extern
#endif
#elif VR_TMS9918_EMU_COMPILING_DLL
#define VR_EMU_TMS9918_DLLEXPORT __declspec(dllexport)
#elif defined WIN32 && !defined VR_EMU_TMS9918_STATIC
#define VR_EMU_TMS9918_DLLEXPORT __declspec(dllimport)
#else
#ifdef __cplusplus
#define VR_EMU_TMS9918_DLLEXPORT extern "C"
#else
#define VR_EMU_TMS9918_DLLEXPORT extern
#endif
#endif

#ifndef VR_EMU_TMS9918_DLLEXPORT_CONST
#define VR_EMU_TMS9918_DLLEXPORT_CONST VR_EMU_TMS9918_DLLEXPORT
#endif

#if VR_EMU_TMS9918_SINGLE_INSTANCE
#define VR_EMU_INST_ARG
#define VR_EMU_INST_ONLY_ARG
#define VR_EMU_INST
#define VR_EMU_INST_ONLY
#else
#define VR_EMU_INST_ARG VrEmuTms9918* tms9918,
#define VR_EMU_INST_ONLY_ARG VrEmuTms9918* tms9918
#define VR_EMU_INST tms9918,
#define VR_EMU_INST_ONLY tms9918
#endif


#include <stdint.h>
#include <stdbool.h>

/* PRIVATE DATA STRUCTURE
 * ---------------------------------------- */
struct vrEmuTMS9918_s;
typedef struct vrEmuTMS9918_s VrEmuTms9918;

typedef enum
{
  TMS_MODE_GRAPHICS_I,
  TMS_MODE_GRAPHICS_II,
  TMS_MODE_TEXT,
  TMS_MODE_MULTICOLOR,
  TMS_MODE_TEXT80,
} vrEmuTms9918Mode;

typedef enum
{
  TMS_TRANSPARENT = 0,
  TMS_BLACK,
  TMS_MED_GREEN,
  TMS_LT_GREEN,
  TMS_DK_BLUE,
  TMS_LT_BLUE,
  TMS_DK_RED,
  TMS_CYAN,
  TMS_MED_RED,
  TMS_LT_RED,
  TMS_DK_YELLOW,
  TMS_LT_YELLOW,
  TMS_DK_GREEN,
  TMS_MAGENTA,
  TMS_GREY,
  TMS_WHITE,
} vrEmuTms9918Color;

typedef enum
{
  TMS_REG_0 = 0,
  TMS_REG_1,
  TMS_REG_2,
  TMS_REG_3,
  TMS_REG_4,
  TMS_REG_5,
  TMS_REG_6,
  TMS_REG_7,
  TMS_NUM_REGISTERS,
  TMS_REG_NAME_TABLE        = TMS_REG_2,
  TMS_REG_COLOR_TABLE       = TMS_REG_3,
  TMS_REG_PATTERN_TABLE     = TMS_REG_4,
  TMS_REG_SPRITE_ATTR_TABLE = TMS_REG_5,
  TMS_REG_SPRITE_PATT_TABLE = TMS_REG_6,
  TMS_REG_FG_BG_COLOR       = TMS_REG_7,
} vrEmuTms9918Register;

#define TMS9918_PIXELS_X 256
#define TMS9918_PIXELS_Y 192


/* PUBLIC INTERFACE
 * ---------------------------------------- */

#if VR_EMU_TMS9918_SINGLE_INSTANCE

 /* Function:  vrEmuTms9918Init
  * --------------------
  * initialize the TMS9918 library in single-instance mode
  */
VR_EMU_TMS9918_DLLEXPORT
void vrEmuTms9918Init();

#else

 /* Function:  vrEmuTms9918New
  * --------------------
  * create a new TMS9918
  */
VR_EMU_TMS9918_DLLEXPORT
VrEmuTms9918* vrEmuTms9918New();

#endif

/* Function:  vrEmuTms9918Reset
  * --------------------
  * reset the new TMS9918
  */
VR_EMU_TMS9918_DLLEXPORT
void vrEmuTms9918Reset(VR_EMU_INST_ONLY_ARG);

/* Function:  vrEmuTms9918Destroy
 * --------------------
 * destroy a TMS9918
 *
 * tms9918: tms9918 object to destroy / clean up
 */
VR_EMU_TMS9918_DLLEXPORT
void vrEmuTms9918Destroy(VR_EMU_INST_ONLY_ARG);

/* Function:  vrEmuTms9918WriteAddr
 * --------------------
 * write an address (mode = 1) to the tms9918
 *
 * uint8_t: the data (DB0 -> DB7) to send
 */
VR_EMU_TMS9918_DLLEXPORT
void vrEmuTms9918WriteAddr(VR_EMU_INST_ARG uint8_t data);

/* Function:  vrEmuTms9918WriteData
 * --------------------
 * write data (mode = 0) to the tms9918
 *
 * uint8_t: the data (DB0 -> DB7) to send
 */
VR_EMU_TMS9918_DLLEXPORT
void vrEmuTms9918WriteData(VR_EMU_INST_ARG uint8_t data);

/* Function:  vrEmuTms9918ReadStatus
 * --------------------
 * read from the status register
 */
VR_EMU_TMS9918_DLLEXPORT
uint8_t vrEmuTms9918ReadStatus(VR_EMU_INST_ONLY_ARG);

/* Function:  vrEmuTms9918PeekStatus
 * --------------------
 * read from the status register without resetting it
 */
VR_EMU_TMS9918_DLLEXPORT
uint8_t vrEmuTms9918PeekStatus(VR_EMU_INST_ONLY_ARG);

/* Function:  vrEmuTms9918ReadData
 * --------------------
 * read data (mode = 0) from the tms9918
 */
VR_EMU_TMS9918_DLLEXPORT
uint8_t vrEmuTms9918ReadData(VR_EMU_INST_ONLY_ARG);

/* Function:  vrEmuTms9918ReadDataNoInc
 * --------------------
 * read data (mode = 0) from the tms9918
 * don't increment the address pointer
 */
VR_EMU_TMS9918_DLLEXPORT
uint8_t vrEmuTms9918ReadDataNoInc(VR_EMU_INST_ONLY_ARG);


/* Function:  vrEmuTms9918InterruptStatus
 * --------------------
 * return true if both INT status and INT control set
 */
VR_EMU_TMS9918_DLLEXPORT
bool vrEmuTms9918InterruptStatus(VR_EMU_INST_ONLY_ARG);

/* Function:  vrEmuTms9918InterruptSet
 * --------------------
 * set interrupt flag
 */
VR_EMU_TMS9918_DLLEXPORT
void vrEmuTms9918InterruptSet(VR_EMU_INST_ONLY_ARG);

/* Function:  vrEmuTms9918SetStatus
 * --------------------
 * set status flag
 */
VR_EMU_TMS9918_DLLEXPORT
void vrEmuTms9918SetStatus(VR_EMU_INST_ARG uint8_t status);

/* Function:  vrEmuTms9918ScanLine
 * ----------------------------------------
 * generate a scanline
 *
 * pixels to be filled with TMS9918 color palette indexes (vrEmuTms9918Color)
 */
VR_EMU_TMS9918_DLLEXPORT
uint8_t vrEmuTms9918ScanLine(VR_EMU_INST_ARG uint8_t y, uint8_t pixels[TMS9918_PIXELS_X]);

/* Function:  vrEmuTms9918RegValue
 * ----------------------------------------
 * return a reigister value
 */
VR_EMU_TMS9918_DLLEXPORT
uint8_t vrEmuTms9918RegValue(VR_EMU_INST_ARG vrEmuTms9918Register reg);

/* Function:  vrEmuTms9918WriteRegValue
 * ----------------------------------------
 * write a reigister value
 */
VR_EMU_TMS9918_DLLEXPORT
void vrEmuTms9918WriteRegValue(VR_EMU_INST_ARG vrEmuTms9918Register reg, uint8_t value);


/* Function:  vrEmuTms9918VramValue
 * ----------------------------------------
 * return a value from vram
 */
VR_EMU_TMS9918_DLLEXPORT
uint8_t vrEmuTms9918VramValue(VR_EMU_INST_ARG uint16_t addr);


/* Function:  vrEmuTms9918DisplayEnabled
  * --------------------
  * check BLANK flag
  */
VR_EMU_TMS9918_DLLEXPORT
bool vrEmuTms9918DisplayEnabled(VR_EMU_INST_ONLY_ARG);


/* Function:  vrEmuTms9918DisplayMode
  * --------------------
  * current display mode
  */
VR_EMU_TMS9918_DLLEXPORT
vrEmuTms9918Mode vrEmuTms9918DisplayMode(VR_EMU_INST_ONLY_ARG);


#endif // _VR_EMU_TMS9918_H_
