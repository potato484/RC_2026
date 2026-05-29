cmake_minimum_required(VERSION 3.8)
project(rc26_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ============================================================================
# 依赖
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# ============================================================================
# 库：串口驱动
# ============================================================================
add_library(serial_driver SHARED
  src/serial_driver.cpp
)

target_include_directories(serial_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(serial_driver
  rclcpp
)

# ============================================================================
# 安装
# ============================================================================
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS serial_driver
  EXPORT serialTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# ============================================================================
# ament 导出
# ============================================================================
ament_export_targets(serialTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(serial_driver)
ament_export_dependencies(rclcpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_ring_parser
    test/test_ring_parser.cpp
  )
  target_include_directories(test_ring_parser PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_add_gtest(test_adaptive_timeout
    test/test_adaptive_timeout.cpp
  )
  target_include_directories(test_adaptive_timeout PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endif()

ament_package()
