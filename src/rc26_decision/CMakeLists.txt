cmake_minimum_required(VERSION 3.8)
project(rc26_decision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================================
# 依赖
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(rc26_serial REQUIRED)
find_package(rc26_interfaces REQUIRED)
find_package(rc26_vision REQUIRED)
find_package(yaml-cpp REQUIRED)

# ============================================================================
# 头文件目录
# ============================================================================
include_directories(include)

# ============================================================================
# 库：行为树节点
# ============================================================================
add_library(${PROJECT_NAME}_nodes SHARED
  src/mc/mc_area.cpp
  src/mf/mf_area.cpp
  src/mf/merlin_rule_world_model.cpp
  src/combat/battle_grid_state.cpp
  src/combat/combat_area.cpp
  src/navigation/waypoint_manager.cpp
  src/navigation/smart_waypoint_navigator.cpp
  src/navigation/bt_nav_to_smart_point.cpp
  src/navigation/bt_check_localization_health.cpp
  src/navigation/bt_localization_observe_spin.cpp
  src/navigation/bt_localization_anchor_recover.cpp
  src/vision/bt_nodes.cpp
)

ament_target_dependencies(${PROJECT_NAME}_nodes
  rclcpp
  rclcpp_action
  behaviortree_cpp
  geometry_msgs
  nav_msgs
  nav2_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  grid_map_core
  grid_map_ros
  grid_map_msgs
  rc26_serial
  rc26_interfaces
  rc26_vision
)

target_link_libraries(${PROJECT_NAME}_nodes
  yaml-cpp
)

# ============================================================================
# 可执行文件：决策主节点
# ============================================================================
add_executable(decision_node src/decision_node.cpp)

target_link_libraries(decision_node
  ${PROJECT_NAME}_nodes
)

ament_target_dependencies(decision_node
  rclcpp
  rclcpp_action
  behaviortree_cpp
  ament_index_cpp
  geometry_msgs
  nav_msgs
  nav2_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  grid_map_core
  grid_map_ros
  grid_map_msgs
  rc26_serial
  rc26_interfaces
  rc26_vision
)

# ============================================================================
# 安装
# ============================================================================
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}_nodes
    decision_node
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(
  DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ============================================================================
# ament 导出
# ============================================================================
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_nodes)
ament_export_dependencies(rclcpp rclcpp_action behaviortree_cpp geometry_msgs nav_msgs nav2_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs grid_map_core grid_map_ros grid_map_msgs rc26_serial rc26_interfaces rc26_vision)

ament_package()
