cmake_minimum_required(VERSION 3.11)
project(rc26_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  map
  pcd
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(PROGRAMS test/eval_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
