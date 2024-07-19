cmake_minimum_required(VERSION 3.12)

add_executable(pocketbasic2040
	main.c
	)

# create map/bin/hex file etc.
pico_add_extra_outputs(pocketbasic2040)

add_compile_options(-Wall
	-Wno-unused-function
	)

target_link_libraries(pocketbasic2040
	tbvm
	pico_stdlib
	)
