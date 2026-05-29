cmake_minimum_required(VERSION 3.8)
project(rc26_terrain)

cmake_policy(SET CMP0074 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
find_package(filters REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
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
  filters
  grid_map_core
  grid_map_ros
  grid_map_msgs
  visualization_msgs
  ament_index_cpp
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)
get_target_property(GRID_MAP_CORE_INCLUDE_DIRS grid_map_core::grid_map_core INTERFACE_INCLUDE_DIRECTORIES)
if(GRID_MAP_CORE_INCLUDE_DIRS)
  include_directories(${GRID_MAP_CORE_INCLUDE_DIRS})
endif()

add_library(${PROJECT_NAME}_component SHARED
  src/terrain_semantic_node.cpp
  src/terrain_grid_map_bridge.cpp
  src/terrain_risk_model.cpp
)
add_library(point_cloud_preprocessor STATIC
  src/point_cloud_preprocessor.cpp
)
target_include_directories(point_cloud_preprocessor PUBLIC include ${PCL_INCLUDE_DIRS})
target_link_libraries(point_cloud_preprocessor ${PCL_LIBRARIES})
set_target_properties(point_cloud_preprocessor PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(safety_guard STATIC
  src/safety_guard.cpp
)
target_include_directories(safety_guard PUBLIC include)
set_target_properties(safety_guard PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(tf_chain_validator STATIC
  src/tf_chain_validator.cpp
)
target_include_directories(tf_chain_validator PUBLIC include)
ament_target_dependencies(tf_chain_validator geometry_msgs tf2 tf2_geometry_msgs)
set_target_properties(tf_chain_validator PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_link_libraries(${PROJECT_NAME}_component
  point_cloud_preprocessor
  safety_guard
  tf_chain_validator
  ${PCL_LIBRARIES}
  yaml-cpp
)
ament_target_dependencies(${PROJECT_NAME}_component ${DEPENDENCIES})

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "rc26_terrain::TerrainSemanticNode"
  EXECUTABLE rc26_terrain_node
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "rc26_terrain::TerrainGridMapBridge"
  EXECUTABLE terrain_grid_map_bridge_node
)

install(TARGETS ${PROJECT_NAME}_component
  point_cloud_preprocessor
  safety_guard
  tf_chain_validator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/rviz")
  install(DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}
  )
endif()

install(PROGRAMS
  scripts/export_terrain_training_dataset.py
  scripts/fit_terrain_logistic.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(test_tf_chain test/test_tf_chain.cpp)
  ament_target_dependencies(test_tf_chain tf2 tf2_geometry_msgs geometry_msgs)
  target_link_libraries(test_tf_chain tf_chain_validator)

  ament_add_gtest(test_safety_guard test/test_safety_guard.cpp)
  target_link_libraries(test_safety_guard safety_guard)

  ament_add_gtest(test_terrain_grid_map_bridge test/test_terrain_grid_map_bridge.cpp)
  ament_target_dependencies(test_terrain_grid_map_bridge
    rclcpp
    rc26_interfaces
    nav_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    grid_map_core
    grid_map_ros
    grid_map_msgs
    visualization_msgs
  )
  target_link_libraries(test_terrain_grid_map_bridge ${PROJECT_NAME}_component)

  ament_add_gtest(test_terrain_risk_model test/test_terrain_risk_model.cpp)
  target_link_libraries(test_terrain_risk_model ${PROJECT_NAME}_component)

  add_launch_test(test/test_terrain_grid_map_bridge_launch.py TIMEOUT 90)
endif()

ament_package()
