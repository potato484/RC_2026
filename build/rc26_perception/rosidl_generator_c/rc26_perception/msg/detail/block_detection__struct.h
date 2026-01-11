// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rc26_perception:msg/BlockDetection.idl
// generated code does not contain a copyright notice

#ifndef RC26_PERCEPTION__MSG__DETAIL__BLOCK_DETECTION__STRUCT_H_
#define RC26_PERCEPTION__MSG__DETAIL__BLOCK_DETECTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'class_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/BlockDetection in the package rc26_perception.
/**
  * 单个方块检测结果
 */
typedef struct rc26_perception__msg__BlockDetection
{
  /// 类别名: auto_block, manual_block, fake_block, spear, partner, unknown
  rosidl_runtime_c__String class_name;
  /// 检测框中心点 x (像素)
  int32_t center_x;
  /// 检测框中心点 y (像素)
  int32_t center_y;
  /// 深度距离 (米), 0表示无效
  float depth_m;
} rc26_perception__msg__BlockDetection;

// Struct for a sequence of rc26_perception__msg__BlockDetection.
typedef struct rc26_perception__msg__BlockDetection__Sequence
{
  rc26_perception__msg__BlockDetection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rc26_perception__msg__BlockDetection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RC26_PERCEPTION__MSG__DETAIL__BLOCK_DETECTION__STRUCT_H_
