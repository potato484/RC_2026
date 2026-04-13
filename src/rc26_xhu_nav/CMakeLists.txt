cmake_minimum_required(VERSION 3.10)
project(rc26_xhu_nav)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(ament_index_cpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rc26_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

set(COMMON_INCLUDE_DIRS
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set(TOPOLOGY_DEPS
  ament_index_cpp
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  rc26_interfaces
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

set(LOCAL_PLANNER_DEPS
  geometry_msgs
  nav_msgs
  rclcpp
  rc26_interfaces
  tf2
  tf2_ros
)

set(MODE_MANAGER_DEPS
  ament_index_cpp
  nav_msgs
  rclcpp
  rc26_interfaces
)

set(RUNTIME_DEPS
  geometry_msgs
  nav_msgs
  rclcpp
  rc26_interfaces
  std_msgs
  tf2
  tf2_ros
)

add_library(${PROJECT_NAME}_body_planner SHARED
  src/body_planner/graph_loader.cpp
  src/body_planner/planner.cpp
)
target_include_directories(${PROJECT_NAME}_body_planner PUBLIC ${COMMON_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_body_planner yaml-cpp)

add_library(${PROJECT_NAME}_local_planner SHARED
  src/local_planner/planner_core.cpp
)
target_include_directories(${PROJECT_NAME}_local_planner PUBLIC ${COMMON_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}_local_planner ${LOCAL_PLANNER_DEPS})
target_link_libraries(${PROJECT_NAME}_local_planner yaml-cpp)

add_library(${PROJECT_NAME}_mode_manager SHARED
  src/mode_manager/profile_loader.cpp
  src/mode_manager/profile_db.cpp
  src/mode_manager/watchdog_timer.cpp
)
target_include_directories(${PROJECT_NAME}_mode_manager PUBLIC ${COMMON_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}_mode_manager ${MODE_MANAGER_DEPS})
target_link_libraries(${PROJECT_NAME}_mode_manager yaml-cpp)

add_library(${PROJECT_NAME}_topology SHARED
  src/topology/graph_loader.cpp
  src/topology/body_planning.cpp
  src/topology/overlay_reducer.cpp
  src/topology/planner.cpp
  src/topology/surface_route.cpp
  src/topology/edge_executor.cpp
  src/topology/diagnostics.cpp
  src/topology/topo_nav_node.cpp
)
target_include_directories(${PROJECT_NAME}_topology PUBLIC ${COMMON_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}_topology ${TOPOLOGY_DEPS})
target_link_libraries(${PROJECT_NAME}_topology
  ${PROJECT_NAME}_body_planner
  yaml-cpp
)

add_executable(topo_nav_node src/topology/main.cpp)
target_link_libraries(topo_nav_node ${PROJECT_NAME}_topology)
ament_target_dependencies(topo_nav_node ${TOPOLOGY_DEPS})

add_executable(planner_trace_cli src/topology/planner_trace_cli.cpp)
target_link_libraries(planner_trace_cli ${PROJECT_NAME}_topology)
ament_target_dependencies(planner_trace_cli ${TOPOLOGY_DEPS})

add_executable(surface_route_cli src/topology/surface_route_cli.cpp)
target_link_libraries(surface_route_cli ${PROJECT_NAME}_topology)
ament_target_dependencies(surface_route_cli ${TOPOLOGY_DEPS})

add_executable(surface_body_planner_cli src/body_planner/surface_body_planner_cli.cpp)
target_include_directories(surface_body_planner_cli PUBLIC ${COMMON_INCLUDE_DIRS})
target_link_libraries(surface_body_planner_cli ${PROJECT_NAME}_body_planner yaml-cpp)

add_executable(local_planner_trace_cli src/local_planner/local_planner_trace_cli.cpp)
target_include_directories(local_planner_trace_cli PUBLIC ${COMMON_INCLUDE_DIRS})
target_link_libraries(local_planner_trace_cli ${PROJECT_NAME}_local_planner yaml-cpp)
ament_target_dependencies(local_planner_trace_cli ${LOCAL_PLANNER_DEPS})

add_executable(xhu_motion_mode_manager_node src/mode_manager/xhu_motion_mode_manager.cpp)
target_include_directories(xhu_motion_mode_manager_node PUBLIC ${COMMON_INCLUDE_DIRS})
target_link_libraries(xhu_motion_mode_manager_node ${PROJECT_NAME}_mode_manager yaml-cpp)
ament_target_dependencies(xhu_motion_mode_manager_node ${MODE_MANAGER_DEPS})

add_executable(xhu_motion_runtime_node src/runtime/xhu_motion_runtime.cpp)
target_include_directories(xhu_motion_runtime_node PUBLIC ${COMMON_INCLUDE_DIRS})
target_link_libraries(xhu_motion_runtime_node ${PROJECT_NAME}_local_planner yaml-cpp)
ament_target_dependencies(xhu_motion_runtime_node ${RUNTIME_DEPS})

install(
  TARGETS
    ${PROJECT_NAME}_body_planner
    ${PROJECT_NAME}_local_planner
    ${PROJECT_NAME}_mode_manager
    ${PROJECT_NAME}_topology
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  TARGETS
    topo_nav_node
    planner_trace_cli
    surface_route_cli
    surface_body_planner_cli
    local_planner_trace_cli
    xhu_motion_mode_manager_node
    xhu_motion_runtime_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/validate_graph.py
    scripts/generate_graph.py
    scripts/generate_surface_graph.py
    scripts/render_graph_html.py
    scripts/render_graph_sim_html.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY scenarios/ DESTINATION share/${PROJECT_NAME}/scenarios)
install(DIRECTORY sim_assets/ DESTINATION share/${PROJECT_NAME}/sim_assets)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_graph_loader test/test_graph_loader.cpp)
  target_link_libraries(test_graph_loader ${PROJECT_NAME}_topology)
  ament_target_dependencies(test_graph_loader ${TOPOLOGY_DEPS})

  ament_add_gtest(test_planner test/test_planner.cpp)
  target_link_libraries(test_planner ${PROJECT_NAME}_topology)
  ament_target_dependencies(test_planner ${TOPOLOGY_DEPS})

  ament_add_gtest(test_overlay_reducer test/test_overlay_reducer.cpp)
  target_link_libraries(test_overlay_reducer ${PROJECT_NAME}_topology)
  ament_target_dependencies(test_overlay_reducer ${TOPOLOGY_DEPS})

  ament_add_gtest(test_surface_body_planner test/test_surface_body_planner.cpp)
  target_link_libraries(test_surface_body_planner ${PROJECT_NAME}_body_planner yaml-cpp)

  ament_add_gtest(test_planner_core test/test_planner_core.cpp)
  target_link_libraries(test_planner_core ${PROJECT_NAME}_local_planner yaml-cpp)
  ament_target_dependencies(test_planner_core ${LOCAL_PLANNER_DEPS})

  ament_add_gtest(test_profile_loader test/test_profile_loader.cpp)
  target_link_libraries(test_profile_loader ${PROJECT_NAME}_mode_manager yaml-cpp)
  ament_target_dependencies(test_profile_loader ${MODE_MANAGER_DEPS})

  add_test(
    NAME test_generate_graph
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_generate_graph.py
  )

  add_test(
    NAME test_generate_surface_graph
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_generate_surface_graph.py
  )

  add_test(
    NAME test_render_graph_html
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_render_graph_html.py
  )

  add_test(
    NAME test_render_graph_sim_html
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_render_graph_sim_html.py
  )

  add_test(
    NAME test_local_planner_trace_cli
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/test/test_local_planner_trace_cli.py
            $<TARGET_FILE:local_planner_trace_cli>
            ${CMAKE_CURRENT_SOURCE_DIR}/scenarios
  )
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  ament_index_cpp
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  rc26_interfaces
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  yaml-cpp
)

ament_package()
