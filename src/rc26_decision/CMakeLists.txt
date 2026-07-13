cmake_minimum_required(VERSION 3.8)
project(rc26_decision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ============================================================================
# 依赖
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rc26_serial REQUIRED)
find_package(rc26_interfaces REQUIRED)
find_package(rc26_vision REQUIRED)
find_package(OpenCV REQUIRED)

# ============================================================================
# 头文件目录
# ============================================================================
include_directories(include)

# ============================================================================
# 库：行为树节点
# ============================================================================
add_library(${PROJECT_NAME}_nodes SHARED
  src/mc/mc_area.cpp
  src/mc/visual_servo_grab.cpp
  src/mc/rotate_in_place.cpp
  src/mc/wait_forever.cpp
  src/mc/mc_preselection_repeat_control.cpp
  src/mc/preselection_branch_gate.cpp
  src/mc/wait_for_registration_confirm.cpp
  src/mc/wait_start_signal_and_notify.cpp
  src/stair/stair_action_base.cpp
  src/stair/stair_area.cpp
  src/stair/stair_climb.cpp
  src/stair/stair_descend.cpp
  src/mf/conditions.cpp
  src/mf/grid_center.cpp
  src/mf/grid_heading.cpp
  src/mf/grid_transition.cpp
  src/mf/grid_transition_plan.cpp
  src/mf/mf_area.cpp
  src/mf/merlin_map.cpp
  src/mf/select_next_grid.cpp
  src/mf_preselection/mf_preselection_flow.cpp
  src/second_preselection/second_preselection.cpp
  src/navigation/bt_odom_relative_nav.cpp
)

target_include_directories(${PROJECT_NAME}_nodes PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_nodes
  ${OpenCV_LIBS}
)

ament_target_dependencies(${PROJECT_NAME}_nodes
  ament_index_cpp
  rclcpp
  behaviortree_cpp
  geometry_msgs
  std_msgs
  nav_msgs
  rc26_serial
  rc26_interfaces
  rc26_vision
)

# ============================================================================
# 可执行文件：决策主节点
# ============================================================================
add_executable(decision_node
  src/decision_node.cpp
)
target_link_libraries(decision_node
  ${PROJECT_NAME}_nodes
)

ament_target_dependencies(decision_node
  rclcpp
  behaviortree_cpp
  ament_index_cpp
  geometry_msgs
  std_msgs
  nav_msgs
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

# ============================================================================
# ament 导出
# ============================================================================
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_nodes)
ament_export_dependencies(ament_index_cpp rclcpp behaviortree_cpp geometry_msgs std_msgs nav_msgs rc26_serial rc26_interfaces rc26_vision OpenCV)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_mf_preselection_logic
    test/test_mf_preselection_logic.cpp
  )
  target_link_libraries(test_mf_preselection_logic
    ${PROJECT_NAME}_nodes
  )
  ament_target_dependencies(test_mf_preselection_logic
    behaviortree_cpp
    geometry_msgs
    nav_msgs
    rclcpp
    rc26_interfaces
    rc26_serial
    rc26_vision
  )

  ament_add_gtest(test_second_preselection_logic
    test/test_second_preselection_logic.cpp
  )
  target_link_libraries(test_second_preselection_logic
    ${PROJECT_NAME}_nodes
    ${OpenCV_LIBS}
  )
  target_include_directories(test_second_preselection_logic PRIVATE
    ${OpenCV_INCLUDE_DIRS}
  )
  target_compile_definitions(test_second_preselection_logic PRIVATE
    RC26_DECISION_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )
  ament_target_dependencies(test_second_preselection_logic
    behaviortree_cpp
    geometry_msgs
    nav_msgs
    rclcpp
    rc26_interfaces
    rc26_serial
    rc26_vision
  )

  ament_add_gtest(test_managed_preselection
    test/test_managed_preselection.cpp
  )
  target_link_libraries(test_managed_preselection
    ${PROJECT_NAME}_nodes
    ${OpenCV_LIBS}
  )
  target_include_directories(test_managed_preselection PRIVATE
    ${OpenCV_INCLUDE_DIRS}
  )
  target_compile_definitions(test_managed_preselection PRIVATE
    RC26_DECISION_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )
  ament_target_dependencies(test_managed_preselection
    behaviortree_cpp
    geometry_msgs
    nav_msgs
    rclcpp
    rc26_interfaces
    rc26_serial
    rc26_vision
  )

  ament_add_gtest(test_straight_line_trajectory
    test/test_straight_line_trajectory.cpp
  )

  ament_add_gtest(test_stair_command_tolerance
    test/test_stair_command_tolerance.cpp
  )
  target_link_libraries(test_stair_command_tolerance
    ${PROJECT_NAME}_nodes
  )
  ament_target_dependencies(test_stair_command_tolerance
    behaviortree_cpp
    geometry_msgs
    nav_msgs
    rclcpp
    rc26_interfaces
    rc26_serial
  )
endif()

ament_package()
