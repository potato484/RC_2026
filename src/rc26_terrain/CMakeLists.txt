cmake_minimum_required(VERSION 3.8)
project(rc26_terrain)

cmake_policy(SET CMP0074 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rc26_interfaces REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(yaml-cpp REQUIRED)

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  diagnostic_msgs
  rc26_interfaces
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_component SHARED
  src/terrain_semantic_node.cpp
)
add_library(point_cloud_preprocessor STATIC
  src/point_cloud_preprocessor.cpp
)
target_include_directories(point_cloud_preprocessor PUBLIC include ${PCL_INCLUDE_DIRS})
target_link_libraries(point_cloud_preprocessor ${PCL_LIBRARIES})

add_library(grid_evaluator STATIC
  src/grid_evaluator.cpp
)
target_include_directories(grid_evaluator PUBLIC include)

add_library(safety_guard STATIC
  src/safety_guard.cpp
)
target_include_directories(safety_guard PUBLIC include)

target_link_libraries(${PROJECT_NAME}_component
  point_cloud_preprocessor
  grid_evaluator
  safety_guard
  ${PCL_LIBRARIES}
  yaml-cpp
)
ament_target_dependencies(${PROJECT_NAME}_component ${DEPENDENCIES})

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "rc26_terrain::TerrainSemanticNode"
  EXECUTABLE rc26_terrain_node
)

install(TARGETS ${PROJECT_NAME}_component
  point_cloud_preprocessor
  grid_evaluator
  safety_guard
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch rviz docs
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_grid_evaluator test/test_grid_evaluator.cpp)
  target_link_libraries(test_grid_evaluator grid_evaluator)
endif()

ament_package()
