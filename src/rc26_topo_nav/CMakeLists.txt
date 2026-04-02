cmake_minimum_required(VERSION 3.8)
project(rc26_topo_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rc26_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

set(DEPS
  rclcpp
  rclcpp_action
  ament_index_cpp
  nav_msgs
  geometry_msgs
  diagnostic_msgs
  std_msgs
  visualization_msgs
  rc26_interfaces
  tf2_ros
)

include_directories(include)

# Main library
add_library(${PROJECT_NAME}_lib SHARED
  src/graph_loader.cpp
  src/overlay_reducer.cpp
  src/planner.cpp
  src/edge_executor.cpp
  src/diagnostics.cpp
  src/topo_nav_node.cpp
)
target_link_libraries(${PROJECT_NAME}_lib yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}_lib ${DEPS})

# Main executable
add_executable(topo_nav_node src/main.cpp)
target_link_libraries(topo_nav_node ${PROJECT_NAME}_lib)
ament_target_dependencies(topo_nav_node ${DEPS})

add_executable(planner_trace_cli src/planner_trace_cli.cpp)
target_link_libraries(planner_trace_cli ${PROJECT_NAME}_lib)
ament_target_dependencies(planner_trace_cli ${DEPS})

# Graph scripts
install(PROGRAMS
  scripts/validate_graph.py
  scripts/generate_graph.py
  scripts/render_graph_html.py
  scripts/render_graph_sim_html.py
  scripts/topo_sim_server.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/topo_sim_algorithms.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS topo_nav_node planner_trace_cli DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_lib DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_graph_loader test/test_graph_loader.cpp)
  target_link_libraries(test_graph_loader ${PROJECT_NAME}_lib)
  ament_target_dependencies(test_graph_loader ${DEPS})

  ament_add_gtest(test_planner test/test_planner.cpp)
  target_link_libraries(test_planner ${PROJECT_NAME}_lib)
  ament_target_dependencies(test_planner ${DEPS})

  add_test(
    NAME test_generate_graph
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_generate_graph.py
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
    NAME test_topo_sim_server
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_topo_sim_server.py
  )
endif()

ament_package()
