cmake_minimum_required(VERSION 3.11)
project(rc26_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ============================================================================
# 依赖
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rc26_interfaces REQUIRED)
find_package(rc26_serial REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_path(ONNXRUNTIME_INCLUDE
  NAMES onnxruntime_cxx_api.h
  HINTS /usr/local/include /usr/include /opt/onnxruntime/include
  PATH_SUFFIXES onnxruntime
)
find_library(ONNXRUNTIME_LIB
  NAMES onnxruntime
  HINTS /usr/local/lib /usr/lib /usr/lib/aarch64-linux-gnu /opt/onnxruntime/lib
)

# AidLux local ONNX inference chain.
find_path(AIDLITE_INCLUDE aidlux/aidlite/aidlite.hpp HINTS /usr/local/include)
find_library(AIDLITE_LIB aidlite HINTS /usr/local/lib)
find_library(AIDLITE_QNN231_LIB aidlite_qnn231 HINTS /usr/local/lib)

set(RC26_VISION_LIBRARY_SOURCES
  src/shared/sensors/depth_roi_sampler.cpp
  src/preprocess/yolo/yolo_image_preprocessor.cpp
  src/inference/yolo/yolo_engine.cpp
  src/inference/runtime/vision_inference_manager.cpp
  src/inference/config/model_profile_loader.cpp
  src/inference/runtime/backend_resolver.cpp
  src/inference/runtime/engine_factory.cpp
  src/postprocess/yolo/yolo_detection_postprocessor.cpp
  src/postprocess/localization/tip_localizer.cpp
)

if(ONNXRUNTIME_INCLUDE AND ONNXRUNTIME_LIB)
  message(STATUS "rc26_vision: ONNX Runtime detected, enabling local ONNX backend")
  list(APPEND RC26_VISION_LIBRARY_SOURCES src/inference/onnx/onnx_runtime_engine.cpp)
else()
  message(WARNING "rc26_vision: ONNX Runtime C++ headers/library not found, building local ONNX stub backend")
  list(APPEND RC26_VISION_LIBRARY_SOURCES src/inference/onnx/onnx_runtime_engine_stub.cpp)
endif()

if(AIDLITE_INCLUDE AND AIDLITE_LIB)
  message(STATUS "rc26_vision: AidLite detected, enabling real inference backend")
  list(APPEND RC26_VISION_LIBRARY_SOURCES src/inference/aidlite/aidlite_engine.cpp)
else()
  message(WARNING "rc26_vision: AidLite not found, building stub backend; inference will fail at runtime")
  list(APPEND RC26_VISION_LIBRARY_SOURCES src/inference/aidlite/aidlite_engine_stub.cpp)
endif()

# ============================================================================
# 头文件目录
# ============================================================================
set(COMMON_INCLUDE_DIRS
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ============================================================================
# 库：视觉推理
# ============================================================================
add_library(${PROJECT_NAME} SHARED
  ${RC26_VISION_LIBRARY_SOURCES}
)
target_include_directories(${PROJECT_NAME} PUBLIC ${COMMON_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})
if(ONNXRUNTIME_INCLUDE AND ONNXRUNTIME_LIB)
  target_include_directories(${PROJECT_NAME} PRIVATE
    ${ONNXRUNTIME_INCLUDE}
    ${ONNXRUNTIME_INCLUDE}/onnxruntime
  )
  target_compile_definitions(${PROJECT_NAME} PUBLIC RC26_VISION_HAS_ONNXRUNTIME=1)
else()
  target_compile_definitions(${PROJECT_NAME} PUBLIC RC26_VISION_HAS_ONNXRUNTIME=0)
endif()
if(AIDLITE_INCLUDE AND AIDLITE_LIB)
  target_compile_definitions(${PROJECT_NAME} PUBLIC RC26_VISION_HAS_AIDLITE=1)
else()
  target_compile_definitions(${PROJECT_NAME} PUBLIC RC26_VISION_HAS_AIDLITE=0)
endif()
if(AIDLITE_INCLUDE)
  target_include_directories(${PROJECT_NAME} PRIVATE ${AIDLITE_INCLUDE})
endif()

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  geometry_msgs
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

if(ONNXRUNTIME_INCLUDE AND ONNXRUNTIME_LIB)
  target_link_libraries(${PROJECT_NAME} ${ONNXRUNTIME_LIB})
endif()

if(AIDLITE_LIB)
  target_link_libraries(${PROJECT_NAME} ${AIDLITE_LIB})
endif()

# ============================================================================
# 测试节点
# ============================================================================
add_executable(vision_test_node src/nodes/vision_test_node.cpp)
target_include_directories(vision_test_node PUBLIC ${COMMON_INCLUDE_DIRS})
target_link_libraries(vision_test_node ${PROJECT_NAME})
ament_target_dependencies(vision_test_node rclcpp)

# 独立推理测试 (不依赖 ROS)
add_executable(yolo_inference_test src/tools/yolo_inference_test.cpp)
target_include_directories(yolo_inference_test PUBLIC ${COMMON_INCLUDE_DIRS})
target_link_libraries(yolo_inference_test ${PROJECT_NAME} ${OpenCV_LIBS})

# Tip localizer node
add_executable(tip_localizer_node src/nodes/tip_localizer_node.cpp)
target_include_directories(tip_localizer_node PUBLIC ${COMMON_INCLUDE_DIRS})
target_link_libraries(tip_localizer_node ${PROJECT_NAME})
ament_target_dependencies(tip_localizer_node rclcpp)

# Tip test node
add_executable(tip_vision_test_node
  test/tip_vision_test_node.cpp
)
target_include_directories(tip_vision_test_node PUBLIC ${COMMON_INCLUDE_DIRS})
target_include_directories(tip_vision_test_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/test
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(tip_vision_test_node ${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(tip_vision_test_node rc26_serial::serial_driver)
ament_target_dependencies(tip_vision_test_node ament_index_cpp rclcpp rc26_serial)

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
  TARGETS vision_test_node yolo_inference_test tip_localizer_node tip_vision_test_node
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

install(
  PROGRAMS
    tools/pytorch_to_onnx.py
    tools/calc_dataset_stats.py
  DESTINATION share/${PROJECT_NAME}/tools
)

install(
  FILES tools/dataset_stats.md
  DESTINATION share/${PROJECT_NAME}/tools
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_inference_backend_resolver
    test/test_inference_backend_resolver.cpp
  )
  target_include_directories(test_inference_backend_resolver PUBLIC ${COMMON_INCLUDE_DIRS})
  target_link_libraries(test_inference_backend_resolver ${PROJECT_NAME})
endif()

# ============================================================================
# ament 导出
# ============================================================================
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  ament_index_cpp geometry_msgs rclcpp sensor_msgs cv_bridge image_transport
  message_filters behaviortree_cpp tf2 tf2_ros tf2_geometry_msgs rc26_interfaces
)

ament_package()
