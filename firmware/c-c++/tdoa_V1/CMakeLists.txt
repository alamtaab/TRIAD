cmake_minimum_required(VERSION 3.13)

# Initialize Pico SDK
include(pico_sdk_import.cmake)

project(apsc140_tdoa C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

# Define the executable
add_executable(apsc140_tdoa
    main.c
)

# Link libraries
target_link_libraries(apsc140_tdoa 
    pico_stdlib 
    hardware_adc 
    hardware_dma
    m  # Math library
)

# Enable USB Output (Critical for Serial Monitor!)
pico_enable_stdio_usb(apsc140_tdoa 1)
pico_enable_stdio_uart(apsc140_tdoa 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(apsc140_tdoa)
