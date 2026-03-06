cmake_minimum_required(VERSION 3.8)
project(rc26_telecontrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(telecontrol_nodes STATIC
  src/telecontrol_nodes.cpp
)
target_compile_features(telecontrol_nodes PUBLIC cxx_std_17)
target_include_directories(telecontrol_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(telecontrol_nodes
  rclcpp
  geometry_msgs
  sensor_msgs
)

# 主控制节点
add_executable(rc26_telecontrol src/wheeltec_joy.cpp)
target_compile_features(rc26_telecontrol PUBLIC cxx_std_17)
target_include_directories(rc26_telecontrol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rc26_telecontrol telecontrol_nodes)
ament_target_dependencies(rc26_telecontrol
  rclcpp
  geometry_msgs
  sensor_msgs
)
add_executable(rc26_telecontrol_dpad src/wheeltec_joy_dpad.cpp)
target_compile_features(rc26_telecontrol_dpad PUBLIC cxx_std_17)
target_include_directories(rc26_telecontrol_dpad PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rc26_telecontrol_dpad telecontrol_nodes)
ament_target_dependencies(rc26_telecontrol_dpad
  rclcpp
  geometry_msgs
  sensor_msgs
)

install(TARGETS
  telecontrol_nodes
  rc26_telecontrol
  rc26_telecontrol_dpad
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
