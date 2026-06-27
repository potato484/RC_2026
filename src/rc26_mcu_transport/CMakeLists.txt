cmake_minimum_required(VERSION 3.8)
project(rc26_mcu_transport)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rc26_interfaces REQUIRED)
find_package(rc26_serial REQUIRED)

add_executable(mcu_transport_node
  src/mcu_transport_node.cpp
)

target_include_directories(mcu_transport_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mcu_transport_node
  diagnostic_msgs
  geometry_msgs
  rclcpp
  rc26_interfaces
  rc26_serial
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_chassis_cmd_vel_logic
    test/test_chassis_cmd_vel_logic.cpp
  )
  target_include_directories(test_chassis_cmd_vel_logic PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()

install(
  TARGETS mcu_transport_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
