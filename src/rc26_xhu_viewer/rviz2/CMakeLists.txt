cmake_minimum_required(VERSION 3.5)

project(rviz2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

option(BUILD_RVIZ2_RC26_PLUGINS "Build RC26 RViz panels/displays/history plugins" OFF)
option(BUILD_RVIZ2_RC26_WEB_TOOLS "Install RC26 web adapter and frontend assets" OFF)

set(RC26_VIEWER_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../rc26_xhu_viewer")
if(NOT EXISTS "${RC26_VIEWER_SOURCE_DIR}/src/rc26_xhu_viewer_frame.cpp")
  message(FATAL_ERROR "Missing RC26 viewer staged sources under ${RC26_VIEWER_SOURCE_DIR}")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

find_package(rviz_common REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets)
# TODO(wjwwood): this block is to setup the windeployqt tool, could be removed later.
if(Qt5_FOUND AND WIN32 AND TARGET Qt5::qmake AND NOT TARGET Qt5::windeployqt)
  get_target_property(_qt5_qmake_location Qt5::qmake IMPORTED_LOCATION)

  execute_process(
    COMMAND "${_qt5_qmake_location}" -query QT_INSTALL_PREFIX
    RESULT_VARIABLE return_code
    OUTPUT_VARIABLE qt5_install_prefix
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )

  set(imported_location "${qt5_install_prefix}/bin/windeployqt.exe")

  if(EXISTS ${imported_location})
    add_executable(Qt5::windeployqt IMPORTED)

    set_target_properties(Qt5::windeployqt PROPERTIES
      IMPORTED_LOCATION ${imported_location}
    )
  endif()
endif()

if(BUILD_RVIZ2_RC26_PLUGINS)
  find_package(pluginlib REQUIRED)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(SQLITE3 REQUIRED sqlite3)

  find_package(Qt5 QUIET COMPONENTS WebEngine WebEngineWidgets WebChannel)
  if(Qt5WebEngine_FOUND AND Qt5WebEngineWidgets_FOUND AND Qt5WebChannel_FOUND)
    set(HAS_QT_WEBENGINE TRUE)
    message(STATUS "Qt5WebEngine found - RC26 web panels enabled")
  else()
    set(HAS_QT_WEBENGINE FALSE)
    message(STATUS "Qt5WebEngine not found - RC26 web panels will show fallback placeholder")
  endif()

  set(RC26_PLUGIN_DESCRIPTION_FILE "${CMAKE_CURRENT_BINARY_DIR}/plugins_description.xml")
  configure_file(
    "${RC26_VIEWER_SOURCE_DIR}/plugins_description.xml"
    "${RC26_PLUGIN_DESCRIPTION_FILE}"
    COPYONLY
  )
else()
  message(STATUS "BUILD_RVIZ2_RC26_PLUGINS=OFF - skipping RC26 plugins, SQLite history, and embedded web panels")
endif()

if(BUILD_RVIZ2_RC26_WEB_TOOLS)
  message(STATUS "BUILD_RVIZ2_RC26_WEB_TOOLS=ON - installing RC26 web adapter and frontend assets")
else()
  message(STATUS "BUILD_RVIZ2_RC26_WEB_TOOLS=OFF - skipping RC26 web adapter and frontend assets")
endif()

add_executable(${PROJECT_NAME}
  src/main.cpp
  ${RC26_VIEWER_SOURCE_DIR}/src/rc26_xhu_viewer_frame.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${RC26_VIEWER_SOURCE_DIR}/src
)
ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  rclcpp
  rviz_common
  rviz_rendering
  rviz_ogre_vendor
)
target_link_libraries(${PROJECT_NAME}
  rviz_common::rviz_common
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  Qt5::Widgets
)

if(BUILD_RVIZ2_RC26_PLUGINS)
  add_library(rc26_xhu_viewer_plugins SHARED
    ${RC26_VIEWER_SOURCE_DIR}/src/panels/console_gate_panel.cpp
    ${RC26_VIEWER_SOURCE_DIR}/src/panels/localization_panel.cpp
    ${RC26_VIEWER_SOURCE_DIR}/src/panels/registration_panel.cpp
    ${RC26_VIEWER_SOURCE_DIR}/src/panels/lio_panel.cpp
    ${RC26_VIEWER_SOURCE_DIR}/src/panels/navigation_panel.cpp
    ${RC26_VIEWER_SOURCE_DIR}/src/panels/terrain_panel.cpp
    ${RC26_VIEWER_SOURCE_DIR}/src/panels/bt_area_panel.cpp
    ${RC26_VIEWER_SOURCE_DIR}/src/panels/bt_tree_panel.cpp
    ${RC26_VIEWER_SOURCE_DIR}/src/panels/bt_timeline_panel.cpp
    ${RC26_VIEWER_SOURCE_DIR}/src/panels/mechanism_panel.cpp
    ${RC26_VIEWER_SOURCE_DIR}/src/panels/vision_panel.cpp
    ${RC26_VIEWER_SOURCE_DIR}/src/panels/telecontrol_panel.cpp
    ${RC26_VIEWER_SOURCE_DIR}/src/panels/serial_panel.cpp
    ${RC26_VIEWER_SOURCE_DIR}/src/panels/operator_status_panel.cpp
    ${RC26_VIEWER_SOURCE_DIR}/src/panels/history_query_panel.cpp
    ${RC26_VIEWER_SOURCE_DIR}/src/panels/web_panel_base.cpp
    ${RC26_VIEWER_SOURCE_DIR}/src/panels/web_bridge.cpp
    ${RC26_VIEWER_SOURCE_DIR}/src/displays/localization_display.cpp
    ${RC26_VIEWER_SOURCE_DIR}/src/displays/registration_debug_display.cpp
    ${RC26_VIEWER_SOURCE_DIR}/src/displays/nav_candidates_display.cpp
    ${RC26_VIEWER_SOURCE_DIR}/src/displays/dynamic_prediction_display.cpp
    ${RC26_VIEWER_SOURCE_DIR}/src/displays/terrain_semantic_display.cpp
    ${RC26_VIEWER_SOURCE_DIR}/src/history/history_store.cpp
    ${RC26_VIEWER_SOURCE_DIR}/src/history/history_collector.cpp
  )
  target_include_directories(rc26_xhu_viewer_plugins PRIVATE
    ${RC26_VIEWER_SOURCE_DIR}/src
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
  pluginlib_export_plugin_description_file(rviz_common "${RC26_PLUGIN_DESCRIPTION_FILE}")
endif()

# TODO(wjwwood): find a way to make this optional or to run without "deploying" the
#                necessary dlls and stuff to the bin folder.
#                see:
# https://stackoverflow.com/questions/41193584/deploy-all-qt-dependencies-when-building#41199492
if(TARGET Qt5::windeployqt)
  # execute windeployqt in a tmp directory after build
  add_custom_command(TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E remove_directory "${CMAKE_CURRENT_BINARY_DIR}/windeployqt"
    COMMAND set PATH=%PATH%$<SEMICOLON>${qt5_install_prefix}/bin
    COMMAND
    Qt5::windeployqt
    --dir "${CMAKE_CURRENT_BINARY_DIR}/windeployqt"
    "$<TARGET_FILE_DIR:${PROJECT_NAME}>/$<TARGET_FILE_NAME:${PROJECT_NAME}>"
  )

  # copy deployment directory during installation
  install(
    DIRECTORY
    "${CMAKE_CURRENT_BINARY_DIR}/windeployqt/"
    DESTINATION bin
  )
  install(
    DIRECTORY
    "${CMAKE_CURRENT_BINARY_DIR}/windeployqt/"
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

if(BUILD_RVIZ2_RC26_PLUGINS)
  install(
    TARGETS rc26_xhu_viewer_plugins
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(
  DIRECTORY
    "${RC26_VIEWER_SOURCE_DIR}/config"
    "${RC26_VIEWER_SOURCE_DIR}/resources"
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_RVIZ2_RC26_PLUGINS)
  install(
    DIRECTORY "${RC26_VIEWER_SOURCE_DIR}/web"
    DESTINATION share/${PROJECT_NAME}
  )
endif()

install(
  PROGRAMS
    "${RC26_VIEWER_SOURCE_DIR}/scripts/validate_viewer_configs.py"
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_RVIZ2_RC26_WEB_TOOLS)
  install(
    DIRECTORY "${RC26_VIEWER_SOURCE_DIR}/viewer/"
    DESTINATION share/${PROJECT_NAME}/viewer
    PATTERN "node_modules" EXCLUDE
    PATTERN "dist-e2e" EXCLUDE
  )

  install(
    PROGRAMS
      "${RC26_VIEWER_SOURCE_DIR}/scripts/rc26_xhu_viewer_server.py"
      "${RC26_VIEWER_SOURCE_DIR}/scripts/visualization_algorithms.py"
      "${RC26_VIEWER_SOURCE_DIR}/scripts/render_graph_sim_html.py"
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTING)
  if(BUILD_RVIZ2_RC26_WEB_TOOLS)
    add_test(
      NAME test_rviz2_rc26_server
      COMMAND ${Python3_EXECUTABLE} ${RC26_VIEWER_SOURCE_DIR}/test/test_xhu_viewer_server.py
    )
  endif()

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

ament_package()
