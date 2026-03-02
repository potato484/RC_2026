cmake_minimum_required(VERSION 3.16)
project(rc26_localization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(CheckCXXCompilerFlag)

find_package(ament_cmake_auto REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)
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

target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX small_gicp Threads::Threads)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|ARM64|arm64")
  add_compile_options(-O3 -ffast-math -funsafe-math-optimizations)
  check_cxx_compiler_flag("-march=armv9-a+simd" RC26_LOCALIZATION_HAS_MARCH_ARMV9_SIMD)
  if(RC26_LOCALIZATION_HAS_MARCH_ARMV9_SIMD)
    add_compile_options(-march=armv9-a+simd)
  else()
    check_cxx_compiler_flag("-march=native" RC26_LOCALIZATION_HAS_MARCH_NATIVE)
    if(RC26_LOCALIZATION_HAS_MARCH_NATIVE)
      add_compile_options(-march=native)
      message(WARNING
        "rc26_localization: compiler does not support -march=armv9-a+simd; "
        "fallback to -march=native")
    else()
      message(WARNING
        "rc26_localization: compiler does not support -march=armv9-a+simd or -march=native; "
        "use generic ARM optimization flags only")
    endif()
  endif()
endif()

# 注册为组件节点
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rc26_localization::LocalizationNode
  EXECUTABLE ${PROJECT_NAME}_node
)

# 安装运维脚本
install(PROGRAMS
  scripts/setup_realtime_aidlux.sh
  scripts/profile_localization_perf.sh
  DESTINATION lib/${PROJECT_NAME}
)

# 安装
ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
)
