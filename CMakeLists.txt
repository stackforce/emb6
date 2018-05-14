##  ___ _____ _   ___ _  _____ ___  ___  ___ ___
## / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
## \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
## |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
## embedded.connectivity.solutions.==============
##
## Authors:  Hamza Meddeb
##


cmake_minimum_required(VERSION 3.6)
project(emb6 VERSION 2.1.0 LANGUAGES C)

set(MAIN_PROJECT_NAME "${PROJECT_NAME}" CACHE STRING "name")
set(MAIN_PROJECT_VERSION "${PROJECT_VERSION}" CACHE STRING "version")

#---------------------------------------------------------------------------------------
# General Components
#---------------------------------------------------------------------------------------

# Allow switching of target platform
set(BOARD_LIST cc13x2 cc13x0 native)
set(BOARD native CACHE STRING "Default target platform is native")
set_property(CACHE BOARD PROPERTY STRINGS ${BOARD_LIST})


# Definitions list
set(DEFINITION_LIST
	NETSTK_CFG_IEEE_802154G_EN=1
	MODULATION=MODULATION_2FSK50
	LOGGER_LEVEL=3
	USE_TI_RTOS=0
	LOGGER_DEMO_UDP_SOCKET=TRUE
	#EMB6_INIT_ROOT=0
	EMB6_INIT_ROOT=1
	NET_USE_RPL=1
	DEMO_USE_UDP_SOCKET=1
	DEMO_UDP_SOCKET_ROLE_SERVER=1
	#DEMO_UDP_SOCKET_ROLE_CLIENT=1
	MAC_ADDR_WORD=0x00bb
	LCM_NETWORK_CONF=\"lcmnetwork.conf\"
    MAIN_WITH_ARGS=1
)

set( OPTION_LIST
    -O0
    -Wall
    -fno-strict-aliasing
    -Werror-implicit-function-declaration
    -ffunction-sections
    -fdata-sections
    -std=c99
    -Wno-format
    -Wno-comment
    -g
)

set(LINK_LIST  "-Wl,-Map=${TARGET.base}.map,--gc-section,--cref" )


# Allow switching of application
set(APP_CONFIG_LIST udp_socket)
set(APP_CONFIG udp_socket CACHE STRING "Default application  is udp_socket")
set_property(CACHE APP_CONFIG PROPERTY STRINGS ${APP_CONFIG_LIST})


add_library(${PROJECT_NAME}_core INTERFACE)

target_include_directories(${PROJECT_NAME}_core INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core/inc/net/ipv6>
)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/target)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/utils)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/core)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/demo)