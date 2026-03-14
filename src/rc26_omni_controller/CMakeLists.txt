cmake_minimum_required(VERSION 3.10)
project(rc26_omni_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(grid_map_core REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/omni_pid_pursuit_controller.cpp
  src/pid.cpp
)

get_target_property(GRID_MAP_CORE_INCLUDE_DIRS grid_map_core::grid_map_core INTERFACE_INCLUDE_DIRECTORIES)
if(GRID_MAP_CORE_INCLUDE_DIRS)
  target_include_directories(${PROJECT_NAME} PRIVATE ${GRID_MAP_CORE_INCLUDE_DIRS})
endif()

pluginlib_export_plugin_description_file(nav2_core rc26_omni_controller.xml)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
