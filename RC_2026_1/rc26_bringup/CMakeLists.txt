cmake_minimum_required(VERSION 3.11)
project(rc26_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  map
  pcd
  rviz
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
