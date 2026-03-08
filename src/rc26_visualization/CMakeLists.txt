cmake_minimum_required(VERSION 3.8)
project(rc26_visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rc26_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(visualization_status_core
  src/visualization_status_core.cpp
)
target_include_directories(visualization_status_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(visualization_status_core
  diagnostic_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rc26_interfaces
  sensor_msgs
  std_msgs
)

add_executable(rc26_visualization_status_node
  src/visualization_status_node.cpp
)
target_link_libraries(rc26_visualization_status_node visualization_status_core)
ament_target_dependencies(rc26_visualization_status_node
  diagnostic_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rc26_interfaces
  sensor_msgs
  std_msgs
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  visualization_status_core
  rc26_visualization_status_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_visualization_status
    test/test_visualization_status.cpp
  )
  target_link_libraries(test_visualization_status visualization_status_core)
  target_include_directories(test_visualization_status PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_visualization_status
    diagnostic_msgs
    nav2_msgs
    nav_msgs
    rclcpp
    rc26_interfaces
    sensor_msgs
    std_msgs
  )
endif()

ament_package()
