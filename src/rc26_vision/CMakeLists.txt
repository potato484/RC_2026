cmake_minimum_required(VERSION 3.11)
project(rc26_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================================
# 依赖
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# ONNX Runtime
find_library(ONNXRUNTIME_LIB onnxruntime HINTS /usr/local/lib /usr/lib)
find_path(ONNXRUNTIME_INCLUDE onnxruntime_cxx_api.h HINTS /usr/local/include/onnxruntime)

# ============================================================================
# 头文件目录
# ============================================================================
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${ONNXRUNTIME_INCLUDE}
)

# ============================================================================
# 库：视觉推理
# ============================================================================
add_library(${PROJECT_NAME} SHARED
  src/yolo_engine.cpp
  src/vision_inference_manager.cpp
  src/profile_loader.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  message_filters
  behaviortree_cpp
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${ONNXRUNTIME_LIB}
  yaml-cpp
)

# ============================================================================
# 测试节点
# ============================================================================
add_executable(vision_test_node src/scripts/vision_test_node.cpp)
target_link_libraries(vision_test_node ${PROJECT_NAME})
ament_target_dependencies(vision_test_node rclcpp)

# 独立推理测试 (不依赖 ROS)
add_executable(yolo_inference_test src/scripts/yolo_inference_test.cpp)
target_link_libraries(yolo_inference_test ${PROJECT_NAME} ${OpenCV_LIBS})

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
  TARGETS vision_test_node yolo_inference_test
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
)

ament_package()
