cmake_minimum_required(VERSION 3.8)
project(rc26_merge_odom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rc26_serial REQUIRED)

# CAN里程计库
add_library(can_odom SHARED
  src/can/can_odom.cpp
)
target_include_directories(can_odom PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(can_odom
  rclcpp
  nav_msgs
  tf2
)

# 串口轮式里程计库
add_library(wheel_odom SHARED
  src/wheel/wheel_odom.cpp
)
target_include_directories(wheel_odom PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(wheel_odom
  rc26_serial::serial_driver
)
ament_target_dependencies(wheel_odom
  rclcpp
  nav_msgs
  sensor_msgs
  tf2
  rc26_serial
)

# CAN里程计节点
add_executable(can_odom_node src/can/can_odom_node.cpp)
target_link_libraries(can_odom_node can_odom)
ament_target_dependencies(can_odom_node
  rclcpp
  nav_msgs
)

# 串口轮式里程计节点
add_executable(wheel_odom_node src/wheel/wheel_odom_node.cpp)
target_link_libraries(wheel_odom_node wheel_odom)
ament_target_dependencies(wheel_odom_node
  rclcpp
  nav_msgs
  sensor_msgs
)

# 位姿下传库
add_library(pose_sender SHARED
  src/pose/pose_sender.cpp
)
target_include_directories(pose_sender PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(pose_sender
  rc26_serial::serial_driver
)
ament_target_dependencies(pose_sender
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  rc26_serial
)

# 位姿下传节点
add_executable(pose_sender_node src/pose/pose_sender_node.cpp)
target_link_libraries(pose_sender_node pose_sender)
ament_target_dependencies(pose_sender_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  rc26_serial
)

# 融合里程计主节点(整合CAN/Wheel+位姿下传)
add_executable(merge_odom_node src/merge_odom_node.cpp)
target_link_libraries(merge_odom_node can_odom wheel_odom pose_sender)
ament_target_dependencies(merge_odom_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  rc26_serial
)

# 达妙IMU驱动库
add_library(dm_imu_driver SHARED
  src/imu/dm_imu_driver.cpp
)
target_include_directories(dm_imu_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(dm_imu_driver
  rclcpp
)

# 达妙IMU节点
add_executable(dm_imu_node src/imu/dm_imu_node.cpp)
target_link_libraries(dm_imu_node dm_imu_driver)
ament_target_dependencies(dm_imu_node
  rclcpp
  sensor_msgs
)

# 单轮CAN测试节点
add_executable(single_wheel_test_node src/scripts/single_wheel_test_node.cpp)
ament_target_dependencies(single_wheel_test_node
  rclcpp
)

# 安装C++目标
install(TARGETS
  can_odom
  wheel_odom
  pose_sender
  dm_imu_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  can_odom_node
  wheel_odom_node
  pose_sender_node
  merge_odom_node
  dm_imu_node
  single_wheel_test_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装配置和启动文件
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(can_odom wheel_odom pose_sender dm_imu_driver)
ament_export_dependencies(rclcpp nav_msgs sensor_msgs geometry_msgs tf2 rc26_serial)

ament_package()
