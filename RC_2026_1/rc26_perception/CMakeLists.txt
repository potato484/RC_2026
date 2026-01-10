cmake_minimum_required(VERSION 3.8)
project(rc26_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

# 生成消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BlockDetection.msg"
  "msg/BlockDetections.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# 头文件路径
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# AidLux 平台检测
# 在 AidLux 平台编译时定义 AIDLUX_PLATFORM
option(AIDLUX_PLATFORM "Build for AidLux platform with AidLite SDK" OFF)

if(AIDLUX_PLATFORM)
  add_definitions(-DAIDLUX_PLATFORM)
  # AidLite SDK 路径
  set(AIDLITE_INCLUDE_DIR "/usr/local/include")
  set(AIDLITE_LIB_DIR "/usr/local/lib")
  include_directories(${AIDLITE_INCLUDE_DIR})
  link_directories(${AIDLITE_LIB_DIR})
endif()

# 核心库 (preprocess + postprocess)
add_library(rc26_perception_core SHARED
  src/preprocess.cpp
  src/postprocess.cpp
)
target_include_directories(rc26_perception_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rc26_perception_core
  ${OpenCV_LIBS}
)
ament_target_dependencies(rc26_perception_core
  rclcpp
  sensor_msgs
  geometry_msgs
)

# YOLO 引擎库 (依赖平台)
add_library(rc26_yolo_engine SHARED
  src/yolo_engine.cpp
)
target_include_directories(rc26_yolo_engine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
if(AIDLUX_PLATFORM)
  target_link_libraries(rc26_yolo_engine
    aidlite
  )
endif()

# 感知节点
add_executable(perception_node
  src/perception_node.cpp
)
target_include_directories(perception_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(perception_node
  rc26_perception_core
  rc26_yolo_engine
  ${OpenCV_LIBS}
)
ament_target_dependencies(perception_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 链接生成的消息接口
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(perception_node ${cpp_typesupport_target})

# D455 驱动节点
add_executable(d455_driver_node
  src/d455_driver_node.cpp
)
target_link_libraries(d455_driver_node
  ${OpenCV_LIBS}
  ${realsense2_LIBRARY}
)
ament_target_dependencies(d455_driver_node
  rclcpp
  sensor_msgs
  cv_bridge
)

# 可视化节点 (使用 OpenCV DNN 进行 ONNX 推理)
add_executable(visualization_node
  src/visualization_node.cpp
)
target_include_directories(visualization_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(visualization_node
  rc26_perception_core
  ${OpenCV_LIBS}
)
ament_target_dependencies(visualization_node
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters
)

# 安装
install(TARGETS
  rc26_perception_core
  rc26_yolo_engine
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS perception_node d455_driver_node visualization_node
  DESTINATION lib/${PROJECT_NAME}
)

# Python 脚本安装
install(PROGRAMS
  scripts/visualization_node.py
  scripts/export_onnx.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_export_include_directories(include)
ament_export_libraries(rc26_perception_core rc26_yolo_engine)
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
)

ament_package()
