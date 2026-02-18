cmake_minimum_required(VERSION 3.10)
project(rc26_omni_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/omni_pid_pursuit_controller.cpp
  src/pid.cpp
)

pluginlib_export_plugin_description_file(nav2_core rc26_omni_controller.xml)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
