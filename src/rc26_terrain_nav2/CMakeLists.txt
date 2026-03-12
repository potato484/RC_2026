cmake_minimum_required(VERSION 3.8)
project(rc26_terrain_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)

add_library(terrain_traversability_layer SHARED
  src/terrain_traversability_layer.cpp
)
target_include_directories(terrain_traversability_layer PUBLIC include)
ament_target_dependencies(terrain_traversability_layer
  rclcpp
  nav2_costmap_2d
  pluginlib
  grid_map_core
  grid_map_ros
  grid_map_msgs
)

add_library(terrain_speed_limit_bridge_lib SHARED
  src/terrain_speed_limit_bridge.cpp
)
target_include_directories(terrain_speed_limit_bridge_lib PUBLIC include)
ament_target_dependencies(terrain_speed_limit_bridge_lib
  rclcpp
  std_msgs
  nav2_msgs
  nav2_costmap_2d
)

add_executable(terrain_speed_limit_bridge_node
  src/terrain_speed_limit_bridge_node.cpp
)
target_link_libraries(terrain_speed_limit_bridge_node
  terrain_speed_limit_bridge_lib
)
ament_target_dependencies(terrain_speed_limit_bridge_node
  rclcpp
)

pluginlib_export_plugin_description_file(nav2_costmap_2d rc26_terrain_nav2.xml)

install(TARGETS
  terrain_traversability_layer
  terrain_speed_limit_bridge_lib
  terrain_speed_limit_bridge_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES rc26_terrain_nav2.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_terrain_traversability_layer_mapping
    test/test_terrain_traversability_layer_mapping.cpp
  )
  target_link_libraries(test_terrain_traversability_layer_mapping
    terrain_traversability_layer
  )
  ament_target_dependencies(test_terrain_traversability_layer_mapping
    nav2_costmap_2d
  )

  ament_add_gtest(test_terrain_speed_limit_bridge
    test/test_terrain_speed_limit_bridge.cpp
  )
  target_link_libraries(test_terrain_speed_limit_bridge
    terrain_speed_limit_bridge_lib
  )
  ament_target_dependencies(test_terrain_speed_limit_bridge
    rclcpp
    std_msgs
    nav2_msgs
    nav2_costmap_2d
  )
endif()

ament_package()
