cmake_minimum_required(VERSION 3.16)
project(rc26_localization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
find_package(OpenMP REQUIRED)
set(PCL_FIND_QUIETLY ON)
find_package(PCL REQUIRED COMPONENTS common filters kdtree)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

# 直接使用仓库内的 small_gicp，拒绝构建期网络依赖
set(SMALL_GICP_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../small_gicp)
if(NOT EXISTS ${SMALL_GICP_SOURCE_DIR}/CMakeLists.txt)
  message(FATAL_ERROR "找不到 small_gicp 源码目录：${SMALL_GICP_SOURCE_DIR}")
endif()
# 启用 small_gicp 的 PCL 支持
set(BUILD_WITH_PCL ON CACHE BOOL "Enable PCL support for small_gicp" FORCE)
add_subdirectory(${SMALL_GICP_SOURCE_DIR}
                 ${CMAKE_CURRENT_BINARY_DIR}/small_gicp)

# 构建库
ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX small_gicp)

# 注册为组件节点
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rc26_localization::LocalizationNode
  EXECUTABLE ${PROJECT_NAME}_node
)

# 安装
ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
  scripts
)
