# uw_robot_interfaces：自定义消息 + 共享 C++ 类型。
#
# 两个产物：
#   1) rosidl 消息（ControlCommand / WrenchCommand / ...），供 C++ 与 Python 节点共用；
#   2) 纯头文件的 uw_robot_interfaces_types（Eigen 类型与常量），供算法单元测试直接链接，
#      不需要启动 ROS。
cmake_minimum_required(VERSION 3.16)
project(uw_robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 消息顺序按"输入 → 输出 → 状态"排列，便于阅读。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlCommand.msg"
  "msg/DepthState.msg"
  "msg/ThrusterCommand.msg"
  "msg/WrenchCommand.msg"
  "msg/RobotStatus.msg"
  "msg/AllocationStatus.msg"
  DEPENDENCIES std_msgs
)

# 纯头文件库：控制/分配逻辑的单元测试直接依赖它，避免引入 ROS 运行时。
add_library(uw_robot_interfaces_types INTERFACE)
target_include_directories(uw_robot_interfaces_types INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(uw_robot_interfaces_types INTERFACE Eigen3::Eigen)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS uw_robot_interfaces_types EXPORT export_robot_types)
install(EXPORT export_robot_types DESTINATION share/${PROJECT_NAME}/cmake)

ament_export_dependencies(rosidl_default_runtime)
ament_export_targets(export_robot_types)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3)
ament_package()
