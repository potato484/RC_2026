cmake_minimum_required(VERSION 3.10)
project(rc26_omni_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rc26_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(xhu_motion_follower_node
  src/xhu_motion_follower.cpp
)
target_include_directories(xhu_motion_follower_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(xhu_motion_follower_node
  geometry_msgs
  nav_msgs
  rclcpp
  rc26_interfaces
  std_msgs
  tf2
  tf2_ros
)

install(
  TARGETS xhu_motion_follower_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(
  geometry_msgs
  nav_msgs
  rclcpp
  rc26_interfaces
  std_msgs
  tf2
  tf2_ros
)

ament_package()
