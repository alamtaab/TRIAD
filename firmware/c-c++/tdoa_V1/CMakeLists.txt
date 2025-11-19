# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)

# Initialize Pico SDK
# This file must be present in your project directory.
include(pico_sdk_import.cmake)

# UPDATED: Project name matches your folder "tdoa_V1"
project(tdoa_V1 C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# Define the executable
# UPDATED: This will generate "tdoa_V1.uf2"
add_executable(tdoa_V1
    main.c
)

# Link necessary libraries
# pico_stdlib: Standard GPIO and timing
# hardware_adc: For microphone reading
# hardware_dma: For fast data transfer
# m: Standard math library (for atan2f, sqrtf, etc.)
target_link_libraries(tdoa_V1 
    pico_stdlib 
    hardware_adc 
    hardware_dma
    m
)

# Enable USB Output
# This is critical for "Serial Monitor" to work.
# 1 = Enabled, 0 = Disabled
pico_enable_stdio_usb(tdoa_V1 1)
pico_enable_stdio_uart(tdoa_V1 0)

# Create standard output files (.uf2, .hex, .bin)
pico_add_extra_outputs(tdoa_V1)