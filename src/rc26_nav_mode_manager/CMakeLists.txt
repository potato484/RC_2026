cmake_minimum_required(VERSION 3.8)
project(rc26_nav_mode_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rc26_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  nav_msgs
  nav2_msgs
  sensor_msgs
  rc26_interfaces
  std_msgs
  diagnostic_msgs
  ament_index_cpp
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/nav_mode_manager.cpp
  src/profile_loader.cpp
  src/profile_db.cpp
  src/profile_executor.cpp
  src/watchdog_timer.cpp
  src/terrain_mode_adapter.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})
target_link_libraries(${PROJECT_NAME} yaml-cpp)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "rc26_nav_mode_manager::NavModeManager"
  EXECUTABLE nav_mode_manager_node
  EXECUTOR MultiThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "rc26_nav_mode_manager::TerrainModeAdapter"
  EXECUTABLE terrain_mode_adapter_node
  EXECUTOR SingleThreadedExecutor
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_profile_loader
    test/test_profile_loader.cpp
  )
  target_link_libraries(test_profile_loader
    ${PROJECT_NAME}
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES})

ament_package()
