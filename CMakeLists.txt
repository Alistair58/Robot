# Generated Cmake Pico project file
execute_process(COMMAND 
C:/Users/Alistair/AppData/Local/Microsoft/WindowsApps/python3.12.exe c:/Users/Alistair/.pico-sdk/sdk/2.1.0/lib/btstack/tool/compile_gatt.py controller_ble.gatt controller_ble.h
WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
ECHO_OUTPUT_VARIABLE
ECHO_ERROR_VARIABLE
          )
          
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
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(robot_control C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(robot_control robot_control.c )

pico_set_program_name(robot_control "robot_control")
pico_set_program_version(robot_control "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(robot_control 0)
pico_enable_stdio_usb(robot_control 1)

# Add the standard library to the build
target_link_libraries(robot_control 
        pico_stdlib
        pico_btstack_ble
        pico_btstack_cyw43
        pico_cyw43_arch_none
        hardware_pwm
        hardware_adc
        )
# Add the standard include files to the build
target_include_directories(robot_control PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(robot_control 
        pico_cyw43_arch_none
        )

pico_add_extra_outputs(robot_control)

