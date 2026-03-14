cmake_minimum_required(VERSION 3.10)
project(rc26_nmpc_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(osqp_vendor REQUIRED)
find_package(osqp REQUIRED CONFIG)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/nmpc_controller.cpp
)

target_compile_definitions(${PROJECT_NAME} PRIVATE RC26_NMPC_HAS_OSQP=1)
target_link_libraries(${PROJECT_NAME} osqp::osqp)
message(STATUS "rc26_nmpc_controller: OSQP backend enabled")

pluginlib_export_plugin_description_file(nav2_core rc26_nmpc_controller.xml)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
