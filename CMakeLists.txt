cmake_minimum_required(VERSION 3.12)

# Pull in the Pico SDK.  This correctly pulls in TinyUSB for us.
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pocketbasic2040)
pico_sdk_init()

add_subdirectory(submodules/JTTB/tbvm)
add_subdirectory(src)
