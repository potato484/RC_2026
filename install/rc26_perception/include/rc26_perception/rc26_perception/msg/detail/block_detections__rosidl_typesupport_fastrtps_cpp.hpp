// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from rc26_perception:msg/BlockDetections.idl
// generated code does not contain a copyright notice

#ifndef RC26_PERCEPTION__MSG__DETAIL__BLOCK_DETECTIONS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define RC26_PERCEPTION__MSG__DETAIL__BLOCK_DETECTIONS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "rc26_perception/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "rc26_perception/msg/detail/block_detections__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace rc26_perception
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rc26_perception
cdr_serialize(
  const rc26_perception::msg::BlockDetections & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rc26_perception
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  rc26_perception::msg::BlockDetections & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rc26_perception
get_serialized_size(
  const rc26_perception::msg::BlockDetections & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rc26_perception
max_serialized_size_BlockDetections(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace rc26_perception

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rc26_perception
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rc26_perception, msg, BlockDetections)();

#ifdef __cplusplus
}
#endif

#endif  // RC26_PERCEPTION__MSG__DETAIL__BLOCK_DETECTIONS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
