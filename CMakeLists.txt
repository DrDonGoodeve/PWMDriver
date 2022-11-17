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
project(PWMDriverTest C CXX ASM)
set(PICO_CXX_ENABLE_EXCEPTIONS 1)
set(PICO_CXX_ENABLE_RTTI 1)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Project definitions
add_executable(PWMDriverTest
        main.cpp
        PWMDriver.cpp
)
pico_set_program_name(PWMDriverTest "PWMDriverTest")
pico_set_program_version(PWMDriverTest "0.2")

# Via UART or USB?
pico_enable_stdio_uart(PWMDriverTest 0)
pico_enable_stdio_usb(PWMDriverTest 1)

# pull in common dependencies
target_link_libraries(PWMDriverTest pico_stdlib pico_sync)

# pull in hardware support
target_link_libraries(PWMDriverTest
        hardware_dma
        hardware_timer
        hardware_watchdog
        hardware_clocks
        hardware_gpio
        hardware_pwm
        hardware_irq
        hardware_sync
)

# create map/bin/hex file etc.
pico_add_extra_outputs(PWMDriverTest)
