cmake_minimum_required(VERSION 3.22)
project(low_control3 LANGUAGES C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()
option(LOW_CONTROL_ENABLE_IWDG "Enable independent hardware watchdog" ON)
add_executable(${PROJECT_NAME})
add_subdirectory(cmake/stm32cubemx)
# CubeMX main is a peripheral-reference stub; actual entry is usermain.cpp.
set_source_files_properties(Core/Src/main.c PROPERTIES COMPILE_DEFINITIONS "main=cubemx_main_stub")
target_sources(${PROJECT_NAME} PRIVATE
    userCode/core/Src/ControlProtocol.cpp
    userCode/core/Src/RpmDecoder.cpp
    userCode/core/Src/TelemetryProtocol.cpp
    userCode/control/Src/MotionController.cpp
    userCode/devices/Src/usermain.cpp
    userCode/devices/Src/CommUART1.cpp
    userCode/devices/Src/Safety.cpp
    userCode/devices/Src/TelemetryService.cpp
    userCode/devices/Src/CH9434.cpp
    userCode/devices/Src/VoltageMonitor.cpp
    userCode/devices/Src/Motor.cpp
    userCode/devices/Src/Servo.cpp
    userCode/devices/Src/LED.cpp
    userCode/devices/Src/Buzzer.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
    userCode/core/Inc userCode/control/Inc userCode/devices/Inc)
target_compile_definitions(${PROJECT_NAME} PRIVATE
    LOW_CONTROL_ENABLE_IWDG=$<BOOL:${LOW_CONTROL_ENABLE_IWDG}>)
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)
target_link_libraries(${PROJECT_NAME} stm32cubemx m)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    VERBATIM)
