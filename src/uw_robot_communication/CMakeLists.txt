# uw_robot_communication：两条独立链路，各自由「协议库 + 链路库 + 节点」组成
#
#   stm32_protocol  文本协议编解码（纯函数，可单测）
#   stm32_link      非阻塞串口封装（打开/半帧续写/按行读）
#   hardware_node   业务：wrench→执行器文本、遥测/深度解析
#
#   tcp_protocol    CMD 帧解析与格式化（纯函数）
#   tcp_link        非阻塞单客户端 TCP（收发队列与上限）
#   tcp_bridge_node 业务：手柄文本↔control/cmd、遥测回传
cmake_minimum_required(VERSION 3.16)
project(uw_robot_communication)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(uw_robot_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ---------------------------------------------------------------------------
# STM32 串口链路
# ---------------------------------------------------------------------------
add_library(stm32_protocol STATIC src/stm32/protocol.cpp)
target_compile_features(stm32_protocol PUBLIC cxx_std_17)
target_compile_options(stm32_protocol PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(stm32_protocol PUBLIC include)
target_link_libraries(stm32_protocol Eigen3::Eigen)
ament_target_dependencies(stm32_protocol uw_robot_interfaces)

add_library(stm32_link STATIC src/stm32/serial_port.cpp)
target_compile_features(stm32_link PUBLIC cxx_std_17)
target_compile_options(stm32_link PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(stm32_link PUBLIC include)

add_executable(hardware_node src/stm32/hardware_node.cpp)
target_compile_features(hardware_node PRIVATE cxx_std_17)
target_compile_options(hardware_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(hardware_node rclcpp uw_robot_interfaces sensor_msgs std_msgs)
target_link_libraries(hardware_node stm32_protocol stm32_link Threads::Threads)
install(TARGETS hardware_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(hardware_test test/stm32/hardware_test.cpp)
  target_link_libraries(hardware_test stm32_protocol)
  add_test(NAME hardware_test COMMAND hardware_test)
endif()

# ---------------------------------------------------------------------------
# TCP 手柄/遥测链路
# ---------------------------------------------------------------------------
add_library(tcp_protocol STATIC src/tcp/protocol.cpp)
target_compile_features(tcp_protocol PUBLIC cxx_std_17)
target_compile_options(tcp_protocol PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(tcp_protocol PUBLIC include)
target_link_libraries(tcp_protocol Eigen3::Eigen)
ament_target_dependencies(tcp_protocol uw_robot_interfaces)

add_library(tcp_link STATIC src/tcp/tcp_server.cpp)
target_compile_features(tcp_link PUBLIC cxx_std_17)
target_compile_options(tcp_link PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(tcp_link PUBLIC include)

add_executable(tcp_bridge_node src/tcp/tcp_bridge_node.cpp)
target_compile_features(tcp_bridge_node PRIVATE cxx_std_17)
target_compile_options(tcp_bridge_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(tcp_bridge_node rclcpp uw_robot_interfaces sensor_msgs std_msgs)
target_link_libraries(tcp_bridge_node tcp_protocol tcp_link Threads::Threads)
install(TARGETS tcp_bridge_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(teleop_test test/tcp/teleop_test.cpp)
  target_link_libraries(teleop_test tcp_protocol)
  add_test(NAME teleop_test COMMAND teleop_test)
endif()

ament_package()
