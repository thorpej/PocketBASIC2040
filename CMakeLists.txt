cmake_minimum_required(VERSION 3.12)

# Pull in the Pico SDK.  This correctly pulls in TinyUSB for us.
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pocketbasic2040)
pico_sdk_init()

set(BUILD_SHARED_LIBS OFF)

add_definitions(-DPICO_BUILD=1)
add_definitions(-DVR_EMU_TMS9918_SINGLE_INSTANCE=1)

add_subdirectory(submodules/JTTB/tbvm)
add_subdirectory(submodules/pico9918/src/pio-utils)
add_subdirectory(submodules/pico9918/src/vga)
add_subdirectory(src)
