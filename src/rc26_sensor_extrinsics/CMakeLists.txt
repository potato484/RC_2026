cmake_minimum_required(VERSION 3.8)
project(rc26_sensor_extrinsics)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
