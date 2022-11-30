# Generated Cmake Pico project file
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "C:/Projects/Pico/pico-sdk")
# set(PICO_SDK_PATH "pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

# Project announcement
project(MagicBroom C CXX ASM)
set(PICO_CXX_ENABLE_EXCEPTIONS 1)
set(PICO_CXX_ENABLE_RTTI 1)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Project definitions
add_executable(MagicBroom
        main.cpp
        PWMDriver.cpp
        SwitchScanner.cpp
        WS2812.cpp
)

pico_generate_pio_header(MagicBroom ${CMAKE_CURRENT_LIST_DIR}/WS2812.pio)

pico_set_program_name(MagicBroom "MagicBroom")
pico_set_program_version(MagicBroom "0.1")

# Via UART or USB?
pico_enable_stdio_uart(MagicBroom 0)
pico_enable_stdio_usb(MagicBroom 1)

# pull in common dependencies
target_link_libraries(MagicBroom pico_stdlib pico_sync)

# pull in hardware support
target_link_libraries(MagicBroom
        hardware_dma
        hardware_timer
        hardware_watchdog
        hardware_clocks
        hardware_gpio
        hardware_pwm
        hardware_irq
        hardware_sync
        hardware_pio
)

# create map/bin/hex file etc.
pico_add_extra_outputs(MagicBroom)
