// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rc26_perception:msg/BlockDetections.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rc26_perception/msg/detail/block_detections__rosidl_typesupport_introspection_c.h"
#include "rc26_perception/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rc26_perception/msg/detail/block_detections__functions.h"
#include "rc26_perception/msg/detail/block_detections__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `detections`
#include "rc26_perception/msg/block_detection.h"
// Member `detections`
#include "rc26_perception/msg/detail/block_detection__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rc26_perception__msg__BlockDetections__rosidl_typesupport_introspection_c__BlockDetections_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rc26_perception__msg__BlockDetections__init(message_memory);
}

void rc26_perception__msg__BlockDetections__rosidl_typesupport_introspection_c__BlockDetections_fini_function(void * message_memory)
{
  rc26_perception__msg__BlockDetections__fini(message_memory);
}

size_t rc26_perception__msg__BlockDetections__rosidl_typesupport_introspection_c__size_function__BlockDetections__detections(
  const void * untyped_member)
{
  const rc26_perception__msg__BlockDetection__Sequence * member =
    (const rc26_perception__msg__BlockDetection__Sequence *)(untyped_member);
  return member->size;
}

const void * rc26_perception__msg__BlockDetections__rosidl_typesupport_introspection_c__get_const_function__BlockDetections__detections(
  const void * untyped_member, size_t index)
{
  const rc26_perception__msg__BlockDetection__Sequence * member =
    (const rc26_perception__msg__BlockDetection__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rc26_perception__msg__BlockDetections__rosidl_typesupport_introspection_c__get_function__BlockDetections__detections(
  void * untyped_member, size_t index)
{
  rc26_perception__msg__BlockDetection__Sequence * member =
    (rc26_perception__msg__BlockDetection__Sequence *)(untyped_member);
  return &member->data[index];
}

void rc26_perception__msg__BlockDetections__rosidl_typesupport_introspection_c__fetch_function__BlockDetections__detections(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rc26_perception__msg__BlockDetection * item =
    ((const rc26_perception__msg__BlockDetection *)
    rc26_perception__msg__BlockDetections__rosidl_typesupport_introspection_c__get_const_function__BlockDetections__detections(untyped_member, index));
  rc26_perception__msg__BlockDetection * value =
    (rc26_perception__msg__BlockDetection *)(untyped_value);
  *value = *item;
}

void rc26_perception__msg__BlockDetections__rosidl_typesupport_introspection_c__assign_function__BlockDetections__detections(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rc26_perception__msg__BlockDetection * item =
    ((rc26_perception__msg__BlockDetection *)
    rc26_perception__msg__BlockDetections__rosidl_typesupport_introspection_c__get_function__BlockDetections__detections(untyped_member, index));
  const rc26_perception__msg__BlockDetection * value =
    (const rc26_perception__msg__BlockDetection *)(untyped_value);
  *item = *value;
}

bool rc26_perception__msg__BlockDetections__rosidl_typesupport_introspection_c__resize_function__BlockDetections__detections(
  void * untyped_member, size_t size)
{
  rc26_perception__msg__BlockDetection__Sequence * member =
    (rc26_perception__msg__BlockDetection__Sequence *)(untyped_member);
  rc26_perception__msg__BlockDetection__Sequence__fini(member);
  return rc26_perception__msg__BlockDetection__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember rc26_perception__msg__BlockDetections__rosidl_typesupport_introspection_c__BlockDetections_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rc26_perception__msg__BlockDetections, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "detections",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rc26_perception__msg__BlockDetections, detections),  // bytes offset in struct
    NULL,  // default value
    rc26_perception__msg__BlockDetections__rosidl_typesupport_introspection_c__size_function__BlockDetections__detections,  // size() function pointer
    rc26_perception__msg__BlockDetections__rosidl_typesupport_introspection_c__get_const_function__BlockDetections__detections,  // get_const(index) function pointer
    rc26_perception__msg__BlockDetections__rosidl_typesupport_introspection_c__get_function__BlockDetections__detections,  // get(index) function pointer
    rc26_perception__msg__BlockDetections__rosidl_typesupport_introspection_c__fetch_function__BlockDetections__detections,  // fetch(index, &value) function pointer
    rc26_perception__msg__BlockDetections__rosidl_typesupport_introspection_c__assign_function__BlockDetections__detections,  // assign(index, value) function pointer
    rc26_perception__msg__BlockDetections__rosidl_typesupport_introspection_c__resize_function__BlockDetections__detections  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rc26_perception__msg__BlockDetections__rosidl_typesupport_introspection_c__BlockDetections_message_members = {
  "rc26_perception__msg",  // message namespace
  "BlockDetections",  // message name
  2,  // number of fields
  sizeof(rc26_perception__msg__BlockDetections),
  rc26_perception__msg__BlockDetections__rosidl_typesupport_introspection_c__BlockDetections_message_member_array,  // message members
  rc26_perception__msg__BlockDetections__rosidl_typesupport_introspection_c__BlockDetections_init_function,  // function to initialize message memory (memory has to be allocated)
  rc26_perception__msg__BlockDetections__rosidl_typesupport_introspection_c__BlockDetections_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rc26_perception__msg__BlockDetections__rosidl_typesupport_introspection_c__BlockDetections_message_type_support_handle = {
  0,
  &rc26_perception__msg__BlockDetections__rosidl_typesupport_introspection_c__BlockDetections_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rc26_perception
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rc26_perception, msg, BlockDetections)() {
  rc26_perception__msg__BlockDetections__rosidl_typesupport_introspection_c__BlockDetections_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  rc26_perception__msg__BlockDetections__rosidl_typesupport_introspection_c__BlockDetections_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rc26_perception, msg, BlockDetection)();
  if (!rc26_perception__msg__BlockDetections__rosidl_typesupport_introspection_c__BlockDetections_message_type_support_handle.typesupport_identifier) {
    rc26_perception__msg__BlockDetections__rosidl_typesupport_introspection_c__BlockDetections_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rc26_perception__msg__BlockDetections__rosidl_typesupport_introspection_c__BlockDetections_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
