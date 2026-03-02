cmake_minimum_required(VERSION 3.11)
project(rc26_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================================
# 依赖
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rc26_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# ONNX Runtime
find_library(ONNXRUNTIME_LIB
  NAMES onnxruntime
  HINTS
    /usr/local/lib
    /usr/lib
    /usr/lib/aarch64-linux-gnu
)
find_path(ONNXRUNTIME_INCLUDE
  NAMES onnxruntime_cxx_api.h
  HINTS
    /usr/local/include
    /usr/local/include/onnxruntime
    /usr/include
    /usr/include/onnxruntime
  PATH_SUFFIXES
    onnxruntime
    onnxruntime/core/session
    core/session
)

set(RC26_VISION_HAS_ONNXRUNTIME FALSE)
if(ONNXRUNTIME_LIB AND ONNXRUNTIME_INCLUDE)
  set(RC26_VISION_HAS_ONNXRUNTIME TRUE)
  message(STATUS "rc26_vision: ONNX Runtime enabled")
  message(STATUS "  ONNXRUNTIME_INCLUDE=${ONNXRUNTIME_INCLUDE}")
  message(STATUS "  ONNXRUNTIME_LIB=${ONNXRUNTIME_LIB}")
else()
  message(WARNING
    "rc26_vision: ONNX Runtime not found. Building with stub YoloEngine; "
    "vision inference targets will compile but runtime inference is unavailable.")
endif()

# ============================================================================
# 头文件目录
# ============================================================================
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)
if(RC26_VISION_HAS_ONNXRUNTIME)
  include_directories(${ONNXRUNTIME_INCLUDE})
endif()

# ============================================================================
# 库：视觉推理
# ============================================================================
set(RC26_VISION_LIB_SRCS
  src/vision_inference_manager.cpp
  src/profile_loader.cpp
)
if(RC26_VISION_HAS_ONNXRUNTIME)
  list(APPEND RC26_VISION_LIB_SRCS src/yolo_engine.cpp)
else()
  list(APPEND RC26_VISION_LIB_SRCS src/yolo_engine_stub.cpp)
endif()

add_library(${PROJECT_NAME} SHARED
  ${RC26_VISION_LIB_SRCS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  message_filters
  behaviortree_cpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  rc26_interfaces
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  yaml-cpp
)
if(RC26_VISION_HAS_ONNXRUNTIME)
  target_link_libraries(${PROJECT_NAME}
    ${ONNXRUNTIME_LIB}
  )
endif()

# ============================================================================
# 测试节点
# ============================================================================
add_executable(vision_test_node src/scripts/vision_test_node.cpp)
target_link_libraries(vision_test_node ${PROJECT_NAME})
ament_target_dependencies(vision_test_node rclcpp)

# 独立推理测试 (不依赖 ROS)
add_executable(yolo_inference_test src/scripts/yolo_inference_test.cpp)
target_link_libraries(yolo_inference_test ${PROJECT_NAME} ${OpenCV_LIBS})

# Tip localizer node
add_executable(tip_localizer_node src/tip_localizer.cpp)
target_link_libraries(tip_localizer_node ${PROJECT_NAME} ${OpenCV_LIBS})
ament_target_dependencies(tip_localizer_node
  rclcpp
  ament_index_cpp
  sensor_msgs
  cv_bridge
  tf2
  tf2_ros
  tf2_geometry_msgs
  rc26_interfaces
)

# ============================================================================
# 安装
# ============================================================================
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

install(
  TARGETS vision_test_node yolo_inference_test tip_localizer_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config models
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================================
# ament 导出
# ============================================================================
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp sensor_msgs cv_bridge image_transport message_filters behaviortree_cpp
  tf2 tf2_ros tf2_geometry_msgs rc26_interfaces
)

ament_package()
