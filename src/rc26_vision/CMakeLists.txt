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

# AidLux local ONNX inference chain.
find_path(AIDLITE_INCLUDE aidlux/aidlite/aidlite.hpp HINTS /usr/local/include)
find_library(AIDLITE_LIB aidlite HINTS /usr/local/lib)
find_library(AIDLITE_QNN231_LIB aidlite_qnn231 HINTS /usr/local/lib)

set(RC26_VISION_ENGINE_SOURCES
  src/yolo_engine.cpp
  src/vision_inference_manager.cpp
  src/profile_loader.cpp
  src/engine_factory.cpp
)

if(AIDLITE_INCLUDE AND AIDLITE_LIB)
  message(STATUS "rc26_vision: AidLite detected, enabling real inference backend")
  list(APPEND RC26_VISION_ENGINE_SOURCES src/aidlite_engine.cpp)
else()
  message(WARNING "rc26_vision: AidLite not found, building stub backend; inference will fail at runtime")
  list(APPEND RC26_VISION_ENGINE_SOURCES src/aidlite_engine_stub.cpp)
endif()

if(AIDLITE_INCLUDE AND AIDLITE_LIB AND AIDLITE_QNN231_LIB)
  message(STATUS "rc26_vision: AidLite QNN231 detected, enabling duantou QNN backend")
  list(APPEND RC26_VISION_ENGINE_SOURCES src/aidlite_qnn_yolo_engine.cpp)
else()
  message(WARNING "rc26_vision: AidLite QNN231 not found, building stub duantou backend")
  list(APPEND RC26_VISION_ENGINE_SOURCES src/aidlite_qnn_yolo_engine_stub.cpp)
endif()

# ============================================================================
# 头文件目录
# ============================================================================
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

if(AIDLITE_INCLUDE)
  include_directories(${AIDLITE_INCLUDE})
endif()

# ============================================================================
# 库：视觉推理
# ============================================================================
add_library(${PROJECT_NAME} SHARED
  ${RC26_VISION_ENGINE_SOURCES}
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
  pthread
)

if(AIDLITE_LIB)
  target_link_libraries(${PROJECT_NAME} ${AIDLITE_LIB})
endif()

if(AIDLITE_QNN231_LIB)
  target_link_libraries(${PROJECT_NAME} ${AIDLITE_QNN231_LIB})
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

# USB camera + duantou serial experiment node
add_executable(duantou_vision_node src/duantou_vision_node.cpp)
target_link_libraries(duantou_vision_node ${PROJECT_NAME} ${OpenCV_LIBS})
ament_target_dependencies(duantou_vision_node
  rclcpp
  ament_index_cpp
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
  TARGETS vision_test_node yolo_inference_test tip_localizer_node duantou_vision_node
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
