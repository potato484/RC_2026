cmake_minimum_required(VERSION 3.11)
project(rc26_bringup)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_active_side_switch_listener
    test/test_active_side_switch_listener.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_startup_ready_notify_node
    test/test_startup_ready_notify_node.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_startup_odom_gate_config
    test/test_startup_odom_gate_config.py
    TIMEOUT 60
  )
endif()

install(DIRECTORY
  launch
  config
  pcd
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(PROGRAMS
  scripts/startup_ready_notify_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
