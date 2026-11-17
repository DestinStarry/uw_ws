cmake_minimum_required(VERSION 3.16)
project(uw_robot_interfaces)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlCommand.msg" "msg/DepthState.msg" "msg/ThrusterCommand.msg"
  "msg/RobotStatus.msg" "msg/WrenchCommand.msg" "msg/AllocationStatus.msg" DEPENDENCIES std_msgs)
ament_export_dependencies(rosidl_default_runtime)
install(DIRECTORY include/ DESTINATION include)
add_library(uw_robot_interfaces_types INTERFACE)
target_include_directories(uw_robot_interfaces_types INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(uw_robot_interfaces_types INTERFACE Eigen3::Eigen)
install(TARGETS uw_robot_interfaces_types EXPORT export_robot_types)
install(EXPORT export_robot_types DESTINATION share/${PROJECT_NAME}/cmake)
ament_export_targets(export_robot_types)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3)
ament_package()
