#  ___ _____ _   ___ _  _____ ___  ___  ___ ___
## / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
## \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
## |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
## embedded.connectivity.solutions.==============
##
## Authors:  Hamza Meddeb
##


cmake_minimum_required(VERSION 3.6)
project(demo_${APP_CONFIG}  VERSION 2.1.0 LANGUAGES C)

#---------------------------------------------------------------------------------------
# General Components
#---------------------------------------------------------------------------------------


#---------------------------------------------------------------------------------------
# Target Boards
#---------------------------------------------------------------------------------------
if(${APP_CONFIG} STREQUAL udp_socket)
	file( GLOB_RECURSE ${PROJECT_NAME}_SOURCES
		${CMAKE_CURRENT_SOURCE_DIR}/${APP_CONFIG}/*.c
		${CMAKE_CURRENT_SOURCE_DIR}/demo_main.c
	)
endif()

add_executable(${PROJECT_NAME}
	${${PROJECT_NAME}_SOURCES}
)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

add_dependencies(${PROJECT_NAME} target)
add_dependencies(${PROJECT_NAME} utils)
add_dependencies(${PROJECT_NAME} core)

if(${APP_CONFIG} STREQUAL udp_socket)
	# For inclusing via find_package() from build
	target_include_directories(${PROJECT_NAME} PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/${APP_CONFIG}
	    $<BUILD_INTERFACE:$<TARGET_PROPERTY:core,INTERFACE_INCLUDE_DIRECTORIES>>
	    $<BUILD_INTERFACE:$<TARGET_PROPERTY:target,INTERFACE_INCLUDE_DIRECTORIES>>
	    $<BUILD_INTERFACE:$<TARGET_PROPERTY:utils,INTERFACE_INCLUDE_DIRECTORIES>>
	)

	target_link_libraries(${PROJECT_NAME} PUBLIC
		core
		target
		utils
	)
endif()


target_compile_definitions( ${PROJECT_NAME} PUBLIC
	${DEFINITION_LIST}
)

target_compile_options(${PROJECT_NAME} PUBLIC
	${OPTION_LIST}
)

set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS
	 ${LINK_LIST}
)

