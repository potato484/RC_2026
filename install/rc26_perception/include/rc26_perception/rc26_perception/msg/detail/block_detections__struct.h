// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rc26_perception:msg/BlockDetections.idl
// generated code does not contain a copyright notice

#ifndef RC26_PERCEPTION__MSG__DETAIL__BLOCK_DETECTIONS__STRUCT_H_
#define RC26_PERCEPTION__MSG__DETAIL__BLOCK_DETECTIONS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'detections'
#include "rc26_perception/msg/detail/block_detection__struct.h"

/// Struct defined in msg/BlockDetections in the package rc26_perception.
/**
  * 多目标检测结果
 */
typedef struct rc26_perception__msg__BlockDetections
{
  std_msgs__msg__Header header;
  rc26_perception__msg__BlockDetection__Sequence detections;
} rc26_perception__msg__BlockDetections;

// Struct for a sequence of rc26_perception__msg__BlockDetections.
typedef struct rc26_perception__msg__BlockDetections__Sequence
{
  rc26_perception__msg__BlockDetections * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rc26_perception__msg__BlockDetections__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RC26_PERCEPTION__MSG__DETAIL__BLOCK_DETECTIONS__STRUCT_H_
