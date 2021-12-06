cmake_minimum_required(VERSION 3.13)




option(BUILD_DOC "Build documentation" OFF)


if (PICO_SDK_PATH)
	project(tinyusb_multitool C ASM)
	
	add_library(tinyusb_multitool INTERFACE)

	target_sources(tinyusb_multitool INTERFACE
		${CMAKE_CURRENT_LIST_DIR}/src/tinyusb_multitool.c
		${CMAKE_CURRENT_LIST_DIR}/src/tinyusb_multitool_debug.h
		)
	target_include_directories( tinyusb_multitool INTERFACE
		${CMAKE_CURRENT_LIST_DIR}/src
		)

	return()
elseif(BUILD_DOC)
	project(tinyusb_multitool C ASM)
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
