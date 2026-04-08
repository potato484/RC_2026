cmake_minimum_required(VERSION 3.8)
project(rc26_xhu_viewer_status)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rc26_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(xhu_viewer_status_core
  src/xhu_viewer_status_core.cpp
)
target_include_directories(xhu_viewer_status_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(xhu_viewer_status_core
  diagnostic_msgs
  grid_map_msgs
  nav_msgs
  rclcpp
  rc26_interfaces
  sensor_msgs
  std_msgs
)

add_executable(rc26_xhu_viewer_status_node
  src/xhu_viewer_status_node.cpp
)
target_link_libraries(rc26_xhu_viewer_status_node xhu_viewer_status_core)
ament_target_dependencies(rc26_xhu_viewer_status_node
  diagnostic_msgs
  grid_map_msgs
  nav_msgs
  rclcpp
  rc26_interfaces
  sensor_msgs
  std_msgs
  std_srvs
)

install(
  TARGETS xhu_viewer_status_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS rc26_xhu_viewer_status_node
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_xhu_viewer_status
    test/test_xhu_viewer_status.cpp
  )
  if(TARGET test_xhu_viewer_status)
    target_link_libraries(test_xhu_viewer_status xhu_viewer_status_core)
    target_include_directories(test_xhu_viewer_status PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    ament_target_dependencies(test_xhu_viewer_status
      diagnostic_msgs
      rc26_interfaces
      std_msgs
    )
  endif()
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  diagnostic_msgs
  grid_map_msgs
  nav_msgs
  rclcpp
  rc26_interfaces
  sensor_msgs
  std_msgs
  std_srvs
)

ament_package()
