cmake_minimum_required(VERSION 3.8)
project(rc26_xhu_viewer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_executable(rc26_xhu_viewer
  src/main.cpp
)
ament_target_dependencies(rc26_xhu_viewer
  ament_index_cpp
  rclcpp
  rviz_common
  rviz_ogre_vendor
)
target_link_libraries(rc26_xhu_viewer
  rviz_common::rviz_common
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  Qt5::Widgets
)

install(
  TARGETS rc26_xhu_viewer
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/validate_viewer_configs.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
