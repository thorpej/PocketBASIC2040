cmake_minimum_required(VERSION 3.12)

set(PICO9918_DIR ${CMAKE_CURRENT_LIST_DIR}/../submodules/pico9918)
set(PICO9918_SOURCE_DIR ${PICO9918_DIR}/src)

add_executable(pocketbasic2040)

target_sources(pocketbasic2040 PRIVATE
	main.c
	pico9918-glue.c
	${PICO9918_SOURCE_DIR}/palette.c
	vrEmuTms9918.c
	dsrtc.c
	sd.c
	vdp.c
	vdp-font-onascii.c
	diskio.c
	ff15/ff.c
	ff15/ffunicode.c
	ps2_io.pio.h
	)

pico_generate_pio_header(pocketbasic2040 ${CMAKE_CURRENT_LIST_DIR}/ps2_io.pio)

# create map/bin/hex file etc.
pico_add_extra_outputs(pocketbasic2040)

add_compile_options(-Wall
	-Wno-unused-function
	)

target_include_directories(pocketbasic2040 PRIVATE
	${PICO9918_SOURCE_DIR}
	${CMAKE_CURRENT_LIST_DIR}/ff15
	${CMAKE_CURRENT_LIST_DIR}
	)

target_link_libraries(pocketbasic2040
	tbvm
	pico_stdlib
	pico_multicore
	hardware_i2c
	hardware_pio
	hardware_spi
	pico9918-vga
	)
