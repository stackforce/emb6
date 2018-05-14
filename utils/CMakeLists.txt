#  ___ _____ _   ___ _  _____ ___  ___  ___ ___
## / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
## \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
## |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
## embedded.connectivity.solutions.==============
##
## Authors:  Hamza Meddeb
##


cmake_minimum_required(VERSION 3.6)
project(utils VERSION 2.1.0 LANGUAGES C)

#---------------------------------------------------------------------------------------
# General Components
#---------------------------------------------------------------------------------------

file( GLOB ${PROJECT_NAME}_SOURCES
	${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
	)

add_library(${MAIN_PROJECT_NAME}_${PROJECT_NAME} INTERFACE)

target_include_directories(${MAIN_PROJECT_NAME}_${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
)

# intermediate object library to avoid compiling sources twice for shared and static
add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES})

# For inclusing via find_package() from build
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    $<TARGET_PROPERTY:${MAIN_PROJECT_NAME}_target,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:${MAIN_PROJECT_NAME}_core,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
	core
	target
)

# set the output name to have the same libs names files for .so and .a
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

target_compile_definitions( ${PROJECT_NAME} PUBLIC
	${DEFINITION_LIST}
)

target_compile_options(${PROJECT_NAME} PUBLIC
	${OPTION_LIST}
)

set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS
	${LINK_LIST}
)