cmake_minimum_required(VERSION 3.8)
project(rc26_surface_body_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/graph_loader.cpp
  src/planner.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

add_executable(surface_body_planner_cli src/surface_body_planner_cli.cpp)
target_link_libraries(surface_body_planner_cli ${PROJECT_NAME} yaml-cpp)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(TARGETS surface_body_planner_cli
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_surface_body_planner test/test_surface_body_planner.cpp)
  target_link_libraries(test_surface_body_planner ${PROJECT_NAME} yaml-cpp)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
