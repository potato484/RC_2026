cmake_minimum_required(VERSION 3.8)
project(rc26_kfs_keepout)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(composition_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rc26_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  composition_interfaces
  rcl_interfaces
  ament_index_cpp
  nav_msgs
  std_msgs
  diagnostic_msgs
  rc26_interfaces
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/kfs_block_fuser.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})
target_link_libraries(${PROJECT_NAME} yaml-cpp)

add_executable(kfs_keepout_runtime_manager_node
  src/kfs_keepout_runtime_manager.cpp
)
ament_target_dependencies(kfs_keepout_runtime_manager_node ${DEPENDENCIES})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "rc26_kfs_keepout::KfsBlockFuser"
  EXECUTABLE kfs_block_fuser_node
  EXECUTOR SingleThreadedExecutor
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS kfs_keepout_runtime_manager_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES})

ament_package()
