# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico-flightpanel2 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(pico-flightpanel2 
    pico-flightpanel2.c
    usb_descriptors.c)

pico_set_program_name(pico-flightpanel2 "pico-flightpanel2")
pico_set_program_version(pico-flightpanel2 "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(pico-flightpanel2 0)
pico_enable_stdio_usb(pico-flightpanel2 0)

# Add the standard library to the build
target_link_libraries(pico-flightpanel2
        pico_stdlib 
        pico_unique_id 
        pico_multicore
	    pico_util
        hardware_i2c
        hardware_pwm
        tinyusb_device 
        tinyusb_board
        libdvi)

# Add the standard include files to the build
target_include_directories(pico-flightpanel2 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)
add_subdirectory(modules/PicoDVI/software/libdvi)

set(DVI_DEFAULT_SERIAL_CONFIG "pico_sock_cfg" CACHE STRING
	"Select a default pin configuration from common_dvi_pin_configs.h")

    target_compile_definitions(pico-flightpanel2 PRIVATE
	DVI_DEFAULT_SERIAL_CONFIG=${DVI_DEFAULT_SERIAL_CONFIG}
	DVI_VERTICAL_REPEAT=1
	DVI_N_TMDS_BUFFERS=3
	# DVI_MONOCHROME_TMDS=1
	)

pico_add_extra_outputs(pico-flightpanel2)

