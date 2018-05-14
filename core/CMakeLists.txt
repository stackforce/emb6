#  ___ _____ _   ___ _  _____ ___  ___  ___ ___
## / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
## \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
## |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
## embedded.connectivity.solutions.==============
##
## Authors:  Hamza Meddeb
##


cmake_minimum_required(VERSION 3.6)
project(core VERSION 2.1.0 LANGUAGES C)

#---------------------------------------------------------------------------------------
# General Components
#---------------------------------------------------------------------------------------

file( GLOB ${PROJECT_NAME}_SOURCES
	${CMAKE_CURRENT_SOURCE_DIR}/src/dll/dllc/*.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/dll/dllsec/*.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/dll/framer/*.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/dll/mac/*.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/net/ipv6/*.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/net/multicast/roll-tm.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/net/multicast/smrf.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/net/multicast/uip-mcast6-route.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/net/multicast/uip-mcast6-stats.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/net/rpl/*.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/net/sicslowpan/*.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/net/*.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/phy/*.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/sock/*.c
	${CMAKE_CURRENT_SOURCE_DIR}/*.c
	)

file( GLOB_RECURSE ${PROJECT_NAME}_HEADERS
	${CMAKE_CURRENT_SOURCE_DIR}/*.h
	)

set(LIST_HEADERS)

foreach(item ${${PROJECT_NAME}_HEADERS})
	get_filename_component(PATH ${item} PATH)
	list(APPEND LIST_HEADERS ${PATH} )
endforeach(item)

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES})

# set the output name to have the same libs names files for .so and .a
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

# For inclusing via find_package() from build
target_include_directories(${PROJECT_NAME}  PUBLIC
	${LIST_HEADERS}
    $<TARGET_PROPERTY:${MAIN_PROJECT_NAME}_target,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:${MAIN_PROJECT_NAME}_utils,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
	utils
	target
)

target_compile_definitions(${PROJECT_NAME} PUBLIC
	${DEFINITION_LIST}
)

target_compile_options(${PROJECT_NAME} PUBLIC
	${OPTION_LIST}
)

set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS
	 ${LINK_LIST}
)

