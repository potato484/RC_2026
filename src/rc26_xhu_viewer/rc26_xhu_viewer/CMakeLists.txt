cmake_minimum_required(VERSION 3.8)
project(rc26_xhu_viewer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

option(BUILD_XHU_VIEWER_PLUGINS "Build rc26_xhu_viewer RViz panels/displays/history plugins" OFF)
option(BUILD_XHU_VIEWER_WEB_TOOLS "Install rc26_xhu_viewer web adapter and frontend assets" OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

if(BUILD_XHU_VIEWER_PLUGINS)
  find_package(pluginlib REQUIRED)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(SQLITE3 REQUIRED sqlite3)

  # Optional Qt5WebEngine for web-embedded panels.
  find_package(Qt5 QUIET COMPONENTS WebEngine WebEngineWidgets WebChannel)
  if(Qt5WebEngine_FOUND AND Qt5WebEngineWidgets_FOUND AND Qt5WebChannel_FOUND)
    set(HAS_QT_WEBENGINE TRUE)
    message(STATUS "Qt5WebEngine found - web panels enabled")
  else()
    set(HAS_QT_WEBENGINE FALSE)
    message(STATUS "Qt5WebEngine not found - web panels will show fallback placeholder")
  endif()
else()
  message(STATUS "BUILD_XHU_VIEWER_PLUGINS=OFF - skipping rc26_xhu_viewer_plugins, SQLite history, and embedded web panels")
endif()

if(BUILD_XHU_VIEWER_WEB_TOOLS)
  message(STATUS "BUILD_XHU_VIEWER_WEB_TOOLS=ON - installing rc26_xhu_viewer web adapter and frontend assets")
else()
  message(STATUS "BUILD_XHU_VIEWER_WEB_TOOLS=OFF - skipping rc26_xhu_viewer web adapter and frontend asset installation")
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

if(BUILD_XHU_VIEWER_PLUGINS)
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
endif()

# --- Install ---
install(
  TARGETS rc26_xhu_viewer
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_XHU_VIEWER_PLUGINS)
  install(
    TARGETS rc26_xhu_viewer_plugins
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(
  DIRECTORY config launch resources
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_XHU_VIEWER_PLUGINS)
  install(
    DIRECTORY web
    DESTINATION share/${PROJECT_NAME}
  )
endif()

install(
  PROGRAMS
    scripts/validate_viewer_configs.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_XHU_VIEWER_WEB_TOOLS)
  install(
    DIRECTORY viewer/
    DESTINATION share/${PROJECT_NAME}/viewer
    PATTERN "node_modules" EXCLUDE
    PATTERN "dist-e2e" EXCLUDE
  )

  install(
    PROGRAMS
      scripts/rc26_xhu_viewer_server.py
      scripts/visualization_algorithms.py
      scripts/render_graph_sim_html.py
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTING)
  if(BUILD_XHU_VIEWER_WEB_TOOLS)
    add_test(
      NAME test_xhu_viewer_server
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_xhu_viewer_server.py
    )
  endif()
endif()

ament_package()
