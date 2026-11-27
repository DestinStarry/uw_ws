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

# Exercise the production callbacks against HAL's documented RX-complete-before-error order.
set(UART_SOURCE_PATH "${ROOT}/userCode/devices/Src/usermain.cpp")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${UART_SOURCE_PATH}")
file(READ "${UART_SOURCE_PATH}" UART_SOURCE)
string(FIND "${UART_SOURCE}" "extern \"C\" void HAL_UART_RxCpltCallback" CALLBACK_START)
string(FIND "${UART_SOURCE}" "extern \"C\" void HAL_UART_TxCpltCallback" CALLBACK_END)
if(CALLBACK_START LESS 0 OR CALLBACK_END LESS CALLBACK_START)
    message(FATAL_ERROR "Cannot locate production UART RX callbacks")
endif()
math(EXPR CALLBACK_LENGTH "${CALLBACK_END} - ${CALLBACK_START}")
string(SUBSTRING "${UART_SOURCE}" ${CALLBACK_START} ${CALLBACK_LENGTH} UART_CALLBACKS)
configure_file(uart_callback_test.cpp.in uart_callback_test.cpp @ONLY)
add_executable(uart_callback_test "${CMAKE_CURRENT_BINARY_DIR}/uart_callback_test.cpp")
add_test(NAME uart_callback_error_order COMMAND uart_callback_test)

# Verify the low2-proven transport and initialization with a banked UART register model.
add_executable(ch9434_baseline_tests ch9434_baseline_tests.cpp fake_hal.cpp
    ${ROOT}/userCode/devices/Src/CH9434.cpp)
target_include_directories(ch9434_baseline_tests PRIVATE stubs ${ROOT}/userCode/devices/Inc)
target_link_libraries(ch9434_baseline_tests protocol_core)
target_compile_options(ch9434_baseline_tests PRIVATE -Wall -Wextra)
add_test(NAME ch9434_low2_baseline COMMAND ch9434_baseline_tests)
