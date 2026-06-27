cmake_minimum_required(VERSION 3.16)
project(rc26_localization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)
set(PCL_FIND_QUIETLY ON)
find_package(PCL REQUIRED COMPONENTS common filters kdtree io features registration search sample_consensus)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

# 直接使用仓库内的 small_gicp，拒绝构建期网络依赖。
set(SMALL_GICP_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../rc26_small_gicp)
if(NOT EXISTS ${SMALL_GICP_SOURCE_DIR}/CMakeLists.txt)
  message(FATAL_ERROR "找不到 small_gicp 源码目录：${SMALL_GICP_SOURCE_DIR}")
endif()
set(BUILD_WITH_PCL ON CACHE BOOL "Enable PCL support for small_gicp" FORCE)
add_subdirectory(${SMALL_GICP_SOURCE_DIR}
                 ${CMAKE_CURRENT_BINARY_DIR}/rc26_small_gicp)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/localization.cpp
)

target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}
  OpenMP::OpenMP_CXX
  small_gicp
  Threads::Threads
  pcl_common
  pcl_filters
  pcl_features
  pcl_kdtree
  pcl_io
  pcl_registration
  pcl_search
  pcl_sample_consensus
)

if(NOT MSVC)
  target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-undefined)
endif()

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rc26_localization::LocalizationNode
  EXECUTABLE ${PROJECT_NAME}_node
)

install(PROGRAMS
  scripts/setup_realtime_aidlux.sh
  scripts/profile_localization_perf.sh
  scripts/run_localization_acceptance.sh
  scripts/publish_synthetic_loc_inputs.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
)
