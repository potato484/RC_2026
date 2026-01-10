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
find_package(tf2_geometry_msgs REQUIRED)
find_package(rc26_serial REQUIRED)

# ============================================================================
# 头文件目录
# ============================================================================
include_directories(include)

# ============================================================================
# 库：行为树节点
# ============================================================================
add_library(${PROJECT_NAME}_nodes SHARED
  src/mc_area.cpp
  src/mf_area.cpp
  src/combat_area.cpp
  src/navigation/waypoint_catalog.cpp
  src/navigation/waypoint_navigator.cpp
  src/navigation/bt_navigate_waypoint.cpp
)

# ============================================================================
# 库：串口驱动与CAN里程计
# ============================================================================
add_library(${PROJECT_NAME}_serial SHARED
  src/gain_odom/pose_sender.cpp
  src/gain_odom/wheel_odom_receiver.cpp
  src/gain_odom/can_odom.cpp
)

target_link_libraries(${PROJECT_NAME}_serial
  rc26_serial::serial_driver
)

ament_target_dependencies(${PROJECT_NAME}_serial
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  rc26_serial
)

ament_target_dependencies(${PROJECT_NAME}_nodes
  rclcpp
  rclcpp_action
  behaviortree_cpp
  geometry_msgs
  nav2_msgs
  tf2
  tf2_geometry_msgs
  rc26_serial
)

# ============================================================================
# 可执行文件：决策主节点
# ============================================================================
add_executable(decision_node src/decision_node.cpp)

target_link_libraries(decision_node
  ${PROJECT_NAME}_nodes
  ${PROJECT_NAME}_serial
)

ament_target_dependencies(decision_node
  rclcpp
  rclcpp_action
  behaviortree_cpp
  ament_index_cpp
  geometry_msgs
  nav_msgs
  nav2_msgs
  tf2
  tf2_geometry_msgs
  rc26_serial
)

# ============================================================================
# 可执行文件：串口测试节点
# ============================================================================
add_executable(serial_test0_node src/gain_odom/serial_test0_node.cpp)
add_executable(serial_test1_node src/gain_odom/serial_test1_node.cpp)
target_link_libraries(serial_test0_node
  ${PROJECT_NAME}_serial
)
target_link_libraries(serial_test1_node
  ${PROJECT_NAME}_serial
)
ament_target_dependencies(serial_test0_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  rc26_serial
)
ament_target_dependencies(serial_test1_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  rc26_serial
)

# ============================================================================
# 可执行文件：CAN里程计测试节点
# ============================================================================
add_executable(can_odom_test_node src/gain_odom/can_odom_test_node.cpp)
target_link_libraries(can_odom_test_node
  ${PROJECT_NAME}_serial
)
ament_target_dependencies(can_odom_test_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  rc26_serial
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
    ${PROJECT_NAME}_serial
    decision_node
    serial_test0_node
    serial_test1_node
    can_odom_test_node
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
ament_export_libraries(${PROJECT_NAME}_nodes ${PROJECT_NAME}_serial)
ament_export_dependencies(rclcpp rclcpp_action behaviortree_cpp geometry_msgs nav_msgs nav2_msgs tf2 tf2_geometry_msgs rc26_serial)

ament_package()
