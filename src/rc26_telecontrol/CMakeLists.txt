cmake_minimum_required(VERSION 3.8)
project(rc26_telecontrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rc26_interfaces REQUIRED)
find_package(rc26_serial REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(telecontrol_nodes STATIC
  src/telecontrol_nodes.cpp
)
target_compile_features(telecontrol_nodes PUBLIC cxx_std_17)
target_include_directories(telecontrol_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(telecontrol_nodes
  rclcpp
  geometry_msgs
  sensor_msgs
)

# 主控制节点
add_executable(rc26_telecontrol src/wheeltec_joy.cpp)
target_compile_features(rc26_telecontrol PUBLIC cxx_std_17)
target_include_directories(rc26_telecontrol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rc26_telecontrol telecontrol_nodes)
ament_target_dependencies(rc26_telecontrol
  rclcpp
  geometry_msgs
  sensor_msgs
)
add_executable(rc26_telecontrol_dpad src/wheeltec_joy_dpad.cpp)
target_compile_features(rc26_telecontrol_dpad PUBLIC cxx_std_17)
target_include_directories(rc26_telecontrol_dpad PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rc26_telecontrol_dpad telecontrol_nodes)
ament_target_dependencies(rc26_telecontrol_dpad
  rclcpp
  geometry_msgs
  sensor_msgs
)

add_executable(rc26_telecontrol_front_pushrod_buttons src/front_pushrod_button_node.cpp)
target_compile_features(rc26_telecontrol_front_pushrod_buttons PUBLIC cxx_std_17)
target_include_directories(rc26_telecontrol_front_pushrod_buttons PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rc26_telecontrol_front_pushrod_buttons
  rclcpp
  rc26_interfaces
  rc26_serial
  sensor_msgs
)

add_executable(rc26_telecontrol_rear_pushrod_buttons src/rear_pushrod_button_node.cpp)
target_compile_features(rc26_telecontrol_rear_pushrod_buttons PUBLIC cxx_std_17)
target_include_directories(rc26_telecontrol_rear_pushrod_buttons PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rc26_telecontrol_rear_pushrod_buttons
  rclcpp
  rc26_interfaces
  rc26_serial
  sensor_msgs
)

install(TARGETS
  telecontrol_nodes
  rc26_telecontrol
  rc26_telecontrol_dpad
  rc26_telecontrol_front_pushrod_buttons
  rc26_telecontrol_rear_pushrod_buttons
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_front_pushrod_button_logic
    test/test_front_pushrod_button_logic.cpp
  )
  target_include_directories(test_front_pushrod_button_logic PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_front_pushrod_button_logic
    rc26_serial
  )

  ament_add_gtest(test_rear_pushrod_button_logic
    test/test_rear_pushrod_button_logic.cpp
  )
  target_include_directories(test_rear_pushrod_button_logic PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_rear_pushrod_button_logic
    rc26_serial
  )

  ament_add_gtest(test_telecontrol_mapping
    test/test_telecontrol_mapping.cpp
  )
  target_include_directories(test_telecontrol_mapping PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_telecontrol_mapping
    telecontrol_nodes
  )
  ament_target_dependencies(test_telecontrol_mapping
    rclcpp
    geometry_msgs
    sensor_msgs
  )
endif()

ament_package()
