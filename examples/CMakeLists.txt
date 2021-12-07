cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)
project(test_app)

pico_sdk_init()

add_executable(test_app
    test_app.c
)


# Update this to the correct path to the kxtj3/CMakeList.txt!

include(../CMakeLists.txt)




target_link_libraries(
	test_app
	tinyusb_multitool
	pico_stdlib
	pico_time
	tinyusb_device
	)

pico_enable_stdio_usb(test_app 0)
pico_enable_stdio_uart(test_app 1)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(test_app)



