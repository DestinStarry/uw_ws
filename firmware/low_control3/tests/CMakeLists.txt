cmake_minimum_required(VERSION 3.22)
project(low_control3_host_tests LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
enable_testing()
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
add_library(protocol_core
    ${ROOT}/userCode/core/Src/ControlProtocol.cpp
    ${ROOT}/userCode/core/Src/RpmDecoder.cpp
    ${ROOT}/userCode/core/Src/TelemetryProtocol.cpp)
target_include_directories(protocol_core PUBLIC ${ROOT}/userCode/core/Inc)
target_compile_options(protocol_core PRIVATE -Wall -Wextra -Wpedantic)
add_executable(core_tests core_tests.cpp)
target_link_libraries(core_tests protocol_core)
foreach(suite command framing rpm telemetry)
    add_test(NAME ${suite} COMMAND core_tests ${suite})
endforeach()
add_executable(firmware_tests firmware_tests.cpp fake_hal.cpp
    ${ROOT}/userCode/devices/Src/Motor.cpp
    ${ROOT}/userCode/devices/Src/Servo.cpp
    ${ROOT}/userCode/control/Src/MotionController.cpp
    ${ROOT}/userCode/devices/Src/CommUART1.cpp
    ${ROOT}/userCode/devices/Src/Safety.cpp
    ${ROOT}/userCode/devices/Src/VoltageMonitor.cpp
    ${ROOT}/userCode/devices/Src/TelemetryService.cpp
    ${ROOT}/userCode/devices/Src/CH9434.cpp)
target_include_directories(firmware_tests PRIVATE stubs ${ROOT}/userCode/devices/Inc ${ROOT}/userCode/control/Inc)
target_compile_definitions(firmware_tests PRIVATE LOW_CONTROL_ENABLE_IWDG=0)
target_compile_options(firmware_tests PRIVATE -Wall -Wextra)
target_link_libraries(firmware_tests protocol_core)
add_test(NAME firmware_fault_injection COMMAND firmware_tests)
