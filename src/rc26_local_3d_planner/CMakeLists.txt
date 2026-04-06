cmake_minimum_required(VERSION 3.10)
project(rc26_local_3d_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rc26_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}_core
  src/planner_core.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_core
  geometry_msgs
  nav_msgs
  rclcpp
  rc26_interfaces
  tf2
  tf2_ros
)
target_link_libraries(${PROJECT_NAME}_core yaml-cpp)

add_executable(local_3d_planner_node
  src/local_3d_planner_node.cpp
)
target_include_directories(local_3d_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(local_3d_planner_node ${PROJECT_NAME}_core yaml-cpp)
ament_target_dependencies(local_3d_planner_node
  geometry_msgs
  nav_msgs
  rclcpp
  rc26_interfaces
  tf2
  tf2_ros
)

install(
  TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  TARGETS local_3d_planner_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs
  nav_msgs
  rclcpp
  rc26_interfaces
  tf2
  tf2_ros
  yaml-cpp
)

ament_package()
