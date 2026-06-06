cmake_minimum_required(VERSION 3.8)
project(rc26_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GrabTip.action"
  "action/AssembleWeapon.action"
  "action/ExecuteMechanism.action"
  "msg/TipDetection.msg"
  "msg/TipDetectionArray.msg"
  "msg/MechanismState.msg"
  "msg/MechanismTransportFeedback.msg"
  "msg/MfKfsCell.msg"
  "msg/MfKfsState.msg"
  "msg/MfBlockOverlayCell.msg"
  "msg/MfBlockOverlay.msg"
  "msg/TerrainFeatureGrid.msg"
  "srv/SetKeepoutRuntime.srv"
  "srv/SendMechanismTransportCommand.srv"
  "msg/MechanismActionHistoryEntry.msg"
  "msg/MechanismActionHistory.msg"
  "msg/DynamicPrediction.msg"
  "msg/DynamicPredictionArray.msg"
  DEPENDENCIES action_msgs builtin_interfaces geometry_msgs nav_msgs std_msgs
)

ament_package()
