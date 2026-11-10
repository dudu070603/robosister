cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME dx_assignment5_freertos)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
option(ARM_FOUR_AXIS_BENCH "Four-axis arm bench: no yaw or chassis control" ON)
option(DM_ID_CONFIG_MODE "Standalone CAN1 DM ID configuration; no arm control" OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
file(GLOB_RECURSE USER_SOURCES CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/*.*
        ${CMAKE_CURRENT_SOURCE_DIR}/User_Driver/*.*
        ${CMAKE_CURRENT_SOURCE_DIR}/Task/*.*
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        ${USER_SOURCES}
        Core/Src/usart.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        Bsp
        User_Driver
        User_Driver/Motor_Driver
        User_Driver/Arm
        User_Driver/Remote
        Task
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ARM_FOUR_AXIS_BENCH=$<BOOL:${ARM_FOUR_AXIS_BENCH}>
    DM_ID_CONFIG_MODE=$<BOOL:${DM_ID_CONFIG_MODE}>
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

if(DM_ID_CONFIG_MODE)
    set(FIRMWARE_NAME dm_id_config)
else()
    set(FIRMWARE_NAME engineer_arm)
endif()
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${FIRMWARE_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${FIRMWARE_NAME}.hex
)
