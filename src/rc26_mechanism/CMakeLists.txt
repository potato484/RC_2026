cmake_minimum_required(VERSION 3.8)
project(rc26_mechanism)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rc26_interfaces REQUIRED)
find_package(rc26_serial REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_component SHARED
  src/serial_mechanism_hal.cpp
  src/mechanism_lifecycle_server.cpp
)
ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  rc26_interfaces
  rc26_serial
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "rc26_mechanism::MechanismLifecycleServer"
  EXECUTABLE mechanism_server_node
)

install(
  TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
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

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_component)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  rc26_interfaces
  rc26_serial
)

ament_package()
