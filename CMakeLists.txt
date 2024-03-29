cmake_minimum_required(VERSION 3.13)




option(BUILD_DOC "Build documentation" OFF)
project(tinyusb_multitool C ASM)


set(GIT_INFO_H ${CMAKE_CURRENT_LIST_DIR}/src/git_info.h)
add_custom_target(update_git_info DEPENDS ${GIT_INFO_H})

add_custom_command(OUTPUT ${GIT_INFO_H}
        COMMENT "Generating ${GIT_INFO_H}"
        DEPENDS ${CMAKE_CURRENT_LIST_DIR}/.git/index
        COMMAND ${CMAKE_COMMAND} -E echo_append " #define GIT_REV 0x" > ${GIT_INFO_H}
        COMMAND git rev-parse --short=8 HEAD >> ${GIT_INFO_H}
        )

if (PICO_SDK_PATH)
	add_library(esp-serial-flasher INTERFACE)

	target_sources(esp-serial-flasher INTERFACE
		${CMAKE_CURRENT_LIST_DIR}/lib/esp-serial-flasher/src/esp_loader.c
		${CMAKE_CURRENT_LIST_DIR}/lib/esp-serial-flasher/src/esp_targets.c
		${CMAKE_CURRENT_LIST_DIR}/lib/esp-serial-flasher/src/serial_comm.c
		${CMAKE_CURRENT_LIST_DIR}/lib/esp-serial-flasher/src/md5_hash.c
	)

	target_include_directories(esp-serial-flasher INTERFACE 
		${CMAKE_CURRENT_LIST_DIR}/lib/esp-serial-flasher/include 
		${CMAKE_CURRENT_LIST_DIR}/lib/esp-serial-flasher/port 
		${CMAKE_CURRENT_LIST_DIR}/lib/esp-serial-flasher/private_include
	)

	add_library(tinyusb_multitool INTERFACE)

	target_sources(tinyusb_multitool INTERFACE
		${CMAKE_CURRENT_LIST_DIR}/src/tinyusb_multitool.c
		${CMAKE_CURRENT_LIST_DIR}/src/tinyusb_multitool.h
		${CMAKE_CURRENT_LIST_DIR}/src/tumt_usb_config.c
		${CMAKE_CURRENT_LIST_DIR}/src/tumt_usb_config.h
		${CMAKE_CURRENT_LIST_DIR}/src/tumt_usb_stdio.c
		${CMAKE_CURRENT_LIST_DIR}/src/tumt_usb_stdio.h
		${CMAKE_CURRENT_LIST_DIR}/src/tumt_debug.h
		${CMAKE_CURRENT_LIST_DIR}/src/tumt_uart_bridge.c
		${CMAKE_CURRENT_LIST_DIR}/src/tumt_uart_bridge.h
		${CMAKE_CURRENT_LIST_DIR}/src/tumt_flash_esp32.c
		${CMAKE_CURRENT_LIST_DIR}/src/tumt_flash_esp32.h
		${CMAKE_CURRENT_LIST_DIR}/src/flash.c
		${CMAKE_CURRENT_LIST_DIR}/src/flash.h
		${CMAKE_CURRENT_LIST_DIR}/src/unique_id_modified.h
		${CMAKE_CURRENT_LIST_DIR}/src/unique_id_modified.c
		${CMAKE_CURRENT_LIST_DIR}/src/tusb_config.h


		)
	target_include_directories( tinyusb_multitool INTERFACE
		${CMAKE_CURRENT_LIST_DIR}/src
		${CMAKE_CURRENT_LIST_DIR}/lib/esp-serial-flasher/include
	)

	add_dependencies(tinyusb_multitool 
		update_git_info
	)

	target_link_libraries(
		esp-serial-flasher INTERFACE
	)

	target_link_libraries(
		tinyusb_multitool INTERFACE
		pico_stdlib
		pico_time
		tinyusb_device
		pico_util
		boot_uf2_headers
		pico_multicore
		esp-serial-flasher
	)

	add_compile_definitions(PICO_STDOUT_MUTEX=0)

	return()
elseif(BUILD_DOC)
	# check if Doxygen is installed
	find_package(Doxygen)
	if (DOXYGEN_FOUND)
	    # set input and output files
	    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
	    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
	
	    # request to configure the file
	    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
	    message("Doxygen build started")
	
	    # note the option ALL which allows to build the docs together with the application
	    add_custom_target( doc_doxygen ALL
	        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
	        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	        COMMENT "Generating API documentation with Doxygen"
	        VERBATIM )
	else (DOXYGEN_FOUND)
		message("Doxygen can't be found by cmake")
	endif (DOXYGEN_FOUND)
	message("######################################################")
	message("Warning: this is a library, it can't be directly built")
        message("you need to include it into your own project (see examples/)")
	message("")
	message("You are in DOC BUILD MODE! (-DBUILD_DOC=ON) so I'll build the docs now.")
	message("######################################################")
else()
	message("######################################################")
        message("Warning: this is a library, it can't be directly built")
	message("you need to include it into your own project (see examples/)")
	message("")
	message("You can build docs by adding -DBUILD_DOC=ON to your cmake")
        message("######################################################")
        message(FATAL_ERROR "Quitting configuration")

endif()
