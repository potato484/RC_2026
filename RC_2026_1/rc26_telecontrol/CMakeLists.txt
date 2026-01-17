cmake_minimum_required(VERSION 3.8)
project(rc26_telecontrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 主控制节点
add_executable(rc26_telecontrol src/wheeltec_joy.cpp)
ament_target_dependencies(rc26_telecontrol
  rclcpp
  geometry_msgs
  sensor_msgs
)
add_executable(rc26_telecontrol_dpad src/wheeltec_joy_dpad.cpp)
ament_target_dependencies(rc26_telecontrol_dpad
  rclcpp
  geometry_msgs
  sensor_msgs
)

install(TARGETS
  rc26_telecontrol
  rc26_telecontrol_dpad
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()