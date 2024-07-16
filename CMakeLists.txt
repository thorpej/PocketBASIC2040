cmake_minimum_required(VERSION 3.12)

# Pull in the Pico SDK.  This correctly pulls in TinyUSB for us.
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pocketbasic2040)
pico_sdk_init()

add_executable(pocketbasic2040
	main.c
	)

target_link_libraries(pocketbasic2040
	pico_stdlib
	)

# create map/bin/hex file etc.
pico_add_extra_outputs(pocketbasic2040)

add_compile_options(-Wall
	-Wno-unused-function
	)

target_include_directories(pocketbasic2040 PUBLIC
	${CMAKE_CURRENT_LIST_DIR}
	)
