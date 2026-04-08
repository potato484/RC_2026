cmake_minimum_required(VERSION 3.8)
project(rc26_xhu_viewer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(diagnostic_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rc26_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SQLITE3 REQUIRED sqlite3)

# --- Optional Qt5WebEngine for web-embedded panels ---
find_package(Qt5 QUIET COMPONENTS WebEngine WebEngineWidgets WebChannel)
if(Qt5WebEngine_FOUND AND Qt5WebEngineWidgets_FOUND AND Qt5WebChannel_FOUND)
  set(HAS_QT_WEBENGINE TRUE)
  message(STATUS "Qt5WebEngine found - web panels enabled")
else()
  set(HAS_QT_WEBENGINE FALSE)
  message(STATUS "Qt5WebEngine not found - web panels will show fallback placeholder")
endif()

# --- Main executable ---
add_executable(rc26_xhu_viewer
  src/main.cpp
  src/rc26_xhu_viewer_frame.cpp
)
target_include_directories(rc26_xhu_viewer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
ament_target_dependencies(rc26_xhu_viewer
  ament_index_cpp
  rclcpp
  rviz_common
  rviz_rendering
  rviz_ogre_vendor
)
target_link_libraries(rc26_xhu_viewer
  rviz_common::rviz_common
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  Qt5::Widgets
)

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

# --- Plugin library (panels + displays) ---
add_library(rc26_xhu_viewer_plugins SHARED
  src/panels/console_gate_panel.cpp
  src/panels/localization_panel.cpp
  src/panels/registration_panel.cpp
  src/panels/lio_panel.cpp
  src/panels/navigation_panel.cpp
  src/panels/terrain_panel.cpp
  src/panels/bt_area_panel.cpp
  src/panels/bt_tree_panel.cpp
  src/panels/bt_timeline_panel.cpp
  src/panels/mechanism_panel.cpp
  src/panels/vision_panel.cpp
  src/panels/telecontrol_panel.cpp
  src/panels/serial_panel.cpp
  src/panels/operator_status_panel.cpp
  src/panels/history_query_panel.cpp
  src/panels/web_panel_base.cpp
  src/panels/web_bridge.cpp
  src/displays/localization_display.cpp
  src/displays/registration_debug_display.cpp
  src/displays/nav_candidates_display.cpp
  src/displays/dynamic_prediction_display.cpp
  src/displays/terrain_semantic_display.cpp
  src/history/history_store.cpp
  src/history/history_collector.cpp
)
target_include_directories(rc26_xhu_viewer_plugins PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${SQLITE3_INCLUDE_DIRS}
)
ament_target_dependencies(rc26_xhu_viewer_plugins
  ament_index_cpp
  rclcpp
  rviz_common
  pluginlib
)
target_link_libraries(rc26_xhu_viewer_plugins
  rviz_common::rviz_common
  Qt5::Widgets
  ${SQLITE3_LIBRARIES}
)
if(HAS_QT_WEBENGINE)
  target_compile_definitions(rc26_xhu_viewer_plugins PRIVATE HAS_QT_WEBENGINE)
  target_link_libraries(rc26_xhu_viewer_plugins
    Qt5::WebEngine
    Qt5::WebEngineWidgets
    Qt5::WebChannel
  )
endif()
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

# --- Install ---
install(
  TARGETS rc26_xhu_viewer rc26_xhu_viewer_status_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS rc26_xhu_viewer_plugins xhu_viewer_status_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch web resources
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY viewer/
  DESTINATION share/${PROJECT_NAME}/viewer
  PATTERN "node_modules" EXCLUDE
  PATTERN "dist-e2e" EXCLUDE
)

install(
  PROGRAMS
    scripts/validate_viewer_configs.py
    scripts/rc26_xhu_viewer_server.py
    scripts/visualization_algorithms.py
    scripts/render_graph_sim_html.py
  DESTINATION lib/${PROJECT_NAME}
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

  add_test(
    NAME test_xhu_viewer_server
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_xhu_viewer_server.py
  )
endif()

ament_export_include_directories(include)

ament_package()
