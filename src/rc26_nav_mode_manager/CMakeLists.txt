cmake_minimum_required(VERSION 3.8)
project(rc26_nav_mode_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rc26_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(DEPENDENCIES
  rclcpp
  nav_msgs
  rc26_interfaces
  ament_index_cpp
)

include_directories(include)

add_executable(xhu_motion_mode_manager_node
  src/xhu_motion_mode_manager.cpp
)
ament_target_dependencies(xhu_motion_mode_manager_node ${DEPENDENCIES})
target_link_libraries(xhu_motion_mode_manager_node yaml-cpp)

install(TARGETS xhu_motion_mode_manager_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_profile_loader test/test_profile_loader.cpp)
  target_sources(test_profile_loader PRIVATE
    src/profile_loader.cpp
  )
  target_link_libraries(test_profile_loader yaml-cpp)
  ament_target_dependencies(test_profile_loader ${DEPENDENCIES})
endif()

ament_export_include_directories(include)
ament_export_dependencies(${DEPENDENCIES})

ament_package()
