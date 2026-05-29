cmake_minimum_required(VERSION 3.8)
project(rc26_mechanism)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rc26_interfaces REQUIRED)
find_package(rc26_serial REQUIRED)

set(RC26_MECHANISM_LIBRARY_SOURCES
  src/catalog/mechanism_command_catalog.cpp
  src/hal/shared_serial/shared_serial_mechanism_hal.cpp
  src/nodes/mechanism_lifecycle_server.cpp
)

set(COMMON_INCLUDE_DIRS
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(${PROJECT_NAME}_component SHARED ${RC26_MECHANISM_LIBRARY_SOURCES})
target_include_directories(${PROJECT_NAME}_component PUBLIC ${COMMON_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  rc26_interfaces
  rc26_serial
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "rc26_mechanism::MechanismLifecycleServer"
  EXECUTABLE mechanism_server_node
)

install(
  TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_shared_serial_transport
    test/transport/test_shared_serial_transport.cpp
  )
  target_link_libraries(test_shared_serial_transport ${PROJECT_NAME}_component)
  target_include_directories(test_shared_serial_transport PUBLIC ${COMMON_INCLUDE_DIRS})
  ament_target_dependencies(test_shared_serial_transport
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    rc26_interfaces
    rc26_serial
  )

  ament_add_gtest(test_mechanism_command_catalog
    test/catalog/test_mechanism_command_catalog.cpp
  )
  target_link_libraries(test_mechanism_command_catalog ${PROJECT_NAME}_component)
  target_include_directories(test_mechanism_command_catalog PUBLIC ${COMMON_INCLUDE_DIRS})
  ament_target_dependencies(test_mechanism_command_catalog
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    rc26_interfaces
    rc26_serial
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_component)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  rc26_interfaces
  rc26_serial
)

ament_package()
