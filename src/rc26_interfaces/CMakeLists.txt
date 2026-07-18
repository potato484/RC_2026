cmake_minimum_required(VERSION 3.8)
project(rc26_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TipDetection.msg"
  "msg/TipDetectionArray.msg"
  "msg/MechanismTransportFeedback.msg"
  "srv/SendMechanismTransportCommand.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_package()
