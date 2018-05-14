#  ___ _____ _   ___ _  _____ ___  ___  ___ ___
## / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
## \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
## |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
## embedded.connectivity.solutions.==============
##
## Authors:  Hamza Meddeb
##


cmake_minimum_required(VERSION 3.6)
project(target VERSION 2.1.0 LANGUAGES C)

#---------------------------------------------------------------------------------------
# General Components
#---------------------------------------------------------------------------------------

add_library(${MAIN_PROJECT_NAME}_${PROJECT_NAME} INTERFACE)

#---------------------------------------------------------------------------------------
# Target Boards
#---------------------------------------------------------------------------------------
if(BOARD STREQUAL cc13x2)
	file( GLOB_RECURSE ${PROJECT_NAME}_SOURCES
		${CMAKE_CURRENT_SOURCE_DIR}/arch/arm/cm3/ti/device/cc13x2/*.c
		${CMAKE_CURRENT_SOURCE_DIR}/bsp/ti_cc13x2/*.c
		${CMAKE_CURRENT_SOURCE_DIR}/if/cc13x2/*.c
		${CMAKE_CURRENT_SOURCE_DIR}/mcu/cc13x2/*.c
	)

	target_include_directories(${MAIN_PROJECT_NAME}_${PROJECT_NAME} INTERFACE
	    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/bsp/ti_cc13x2>
	)
elseif(BOARD STREQUAL cc13x0)
	file( GLOB ${PROJECT_NAME}_SOURCES
		${CMAKE_CURRENT_SOURCE_DIR}/arch/arm/cm3/ti/device/cc13xx/bsp/srf06eb_cc26xx/drivers/source/*.c
		${CMAKE_CURRENT_SOURCE_DIR}/arch/arm/cm3/ti/device/cc13xx/*.c
		${CMAKE_CURRENT_SOURCE_DIR}/arch/arm/cm3/ti/device/cc13xx/rf_noRtos_lib/*.c
		${CMAKE_CURRENT_SOURCE_DIR}/arch/arm/cm3/ti/device/cc13xx/rf_noRtos_lib/driverlib/*.c
		${CMAKE_CURRENT_SOURCE_DIR}/arch/arm/cm3/ti/device/cc13xx/rf_noRtos_lib/startup_files/startup_gcc.c
		${CMAKE_CURRENT_SOURCE_DIR}/arch/arm/cm3/ti/device/cc13xx/rf_noRtos_lib/startup_files/ccfg.c
		${CMAKE_CURRENT_SOURCE_DIR}/arch/arm/cm3/ti/device/cc13xx/rflib/*.c
		${CMAKE_CURRENT_SOURCE_DIR}/bsp/ti_cc13xx/*.c
		${CMAKE_CURRENT_SOURCE_DIR}/bsp/*.c
		${CMAKE_CURRENT_SOURCE_DIR}/if/cc13xx/*.c
		${CMAKE_CURRENT_SOURCE_DIR}/mcu/cc13xx/*.c
	)

	target_include_directories(${MAIN_PROJECT_NAME}_${PROJECT_NAME} INTERFACE
	    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/bsp/ti_cc13x2>
	)
elseif(BOARD STREQUAL native)
	file( GLOB ${PROJECT_NAME}_SOURCES
		${CMAKE_CURRENT_SOURCE_DIR}/bsp/*.c
		${CMAKE_CURRENT_SOURCE_DIR}/bsp/native/*.c
		${CMAKE_CURRENT_SOURCE_DIR}/if/native/*.c
		${CMAKE_CURRENT_SOURCE_DIR}/mcu/native/*.c
	)

	target_include_directories(${MAIN_PROJECT_NAME}_${PROJECT_NAME} INTERFACE
	    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/bsp/native>
	)

endif()

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES})

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

#add_dependencies(${PROJECT_NAME} core)

if(BOARD STREQUAL cc13x2)
	# For inclusing via find_package() from build
	target_include_directories(${PROJECT_NAME}  PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/arch/arm/cm3/ti/device/cc13x2
		${CMAKE_CURRENT_SOURCE_DIR}/bsp/ti_cc13x2
		${CMAKE_CURRENT_SOURCE_DIR}/if/cc13x2
		${CMAKE_CURRENT_SOURCE_DIR}/mcu/cc13x2
	    $<TARGET_PROPERTY:${MAIN_PROJECT_NAME}_core,INTERFACE_INCLUDE_DIRECTORIES>
	    $<TARGET_PROPERTY:${MAIN_PROJECT_NAME}_utils,INTERFACE_INCLUDE_DIRECTORIES>
	)
elseif(BOARD STREQUAL cc13x0)
	# For inclusing via find_package() from build
	target_include_directories(${PROJECT_NAME}  PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/arch/arm/cm3/ti/device/cc13xx
		${CMAKE_CURRENT_SOURCE_DIR}/bsp/ti_cc13xx
		${CMAKE_CURRENT_SOURCE_DIR}/if/cc13xx
		${CMAKE_CURRENT_SOURCE_DIR}/mcu/cc13xx
	    $<TARGET_PROPERTY:${MAIN_PROJECT_NAME}_core,INTERFACE_INCLUDE_DIRECTORIES>
	    $<TARGET_PROPERTY:${MAIN_PROJECT_NAME}_utils,INTERFACE_INCLUDE_DIRECTORIES>
	)
elseif(BOARD STREQUAL native)

	# For inclusing via find_package() from build
	target_include_directories(${PROJECT_NAME}  PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/bsp/native
		${CMAKE_CURRENT_SOURCE_DIR}/bsp
		${CMAKE_CURRENT_SOURCE_DIR}/if/native
		${CMAKE_CURRENT_SOURCE_DIR}/mcu/native
	    $<TARGET_PROPERTY:${MAIN_PROJECT_NAME}_core,INTERFACE_INCLUDE_DIRECTORIES>
	    $<TARGET_PROPERTY:${MAIN_PROJECT_NAME}_utils,INTERFACE_INCLUDE_DIRECTORIES>
	)

	target_compile_definitions( ${PROJECT_NAME} PUBLIC
		${DEFINITION_LIST}
	)

	target_compile_options(${PROJECT_NAME} PUBLIC
		${OPTION_LIST}
	)

	set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS
		 ${LINK_LIST}
	)

	target_link_libraries(${PROJECT_NAME} PUBLIC
		core
		lcm
	)
endif()