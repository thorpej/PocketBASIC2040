cmake_minimum_required(VERSION 3.12)

set(PICO9918_DIR ${CMAKE_CURRENT_LIST_DIR}/../submodules/pico9918)
set(PICO9918_SOURCE_DIR ${PICO9918_DIR}/src)

add_executable(pocketbasic2040
	main.c
	pico9918-glue.c
	${PICO9918_SOURCE_DIR}/palette.c
	vrEmuTms9918.c
	vdp.c
	vdp-font-onascii.c
	)

# create map/bin/hex file etc.
pico_add_extra_outputs(pocketbasic2040)

add_compile_options(-Wall
	-Wno-unused-function
	)

target_include_directories(pocketbasic2040 PRIVATE
	${PICO9918_SOURCE_DIR}
	)

target_link_libraries(pocketbasic2040
	tbvm
	pico_stdlib
	pico_multicore
	pico9918-vga
	)
