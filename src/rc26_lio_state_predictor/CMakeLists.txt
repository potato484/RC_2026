cmake_minimum_required(VERSION 3.10)
project(rc26_lio_state_predictor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Eigen3 REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rc26_lio_state_predictor::LioStatePredictorNode
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CLANG_FORMAT_VERSION 21)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  set(ament_cmake_clang_tidy_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-tidy")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
)
