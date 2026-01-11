// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rc26_perception:msg/BlockDetection.idl
// generated code does not contain a copyright notice
#include "rc26_perception/msg/detail/block_detection__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `class_name`
#include "rosidl_runtime_c/string_functions.h"

bool
rc26_perception__msg__BlockDetection__init(rc26_perception__msg__BlockDetection * msg)
{
  if (!msg) {
    return false;
  }
  // class_name
  if (!rosidl_runtime_c__String__init(&msg->class_name)) {
    rc26_perception__msg__BlockDetection__fini(msg);
    return false;
  }
  // center_x
  // center_y
  // depth_m
  return true;
}

void
rc26_perception__msg__BlockDetection__fini(rc26_perception__msg__BlockDetection * msg)
{
  if (!msg) {
    return;
  }
  // class_name
  rosidl_runtime_c__String__fini(&msg->class_name);
  // center_x
  // center_y
  // depth_m
}

bool
rc26_perception__msg__BlockDetection__are_equal(const rc26_perception__msg__BlockDetection * lhs, const rc26_perception__msg__BlockDetection * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // class_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->class_name), &(rhs->class_name)))
  {
    return false;
  }
  // center_x
  if (lhs->center_x != rhs->center_x) {
    return false;
  }
  // center_y
  if (lhs->center_y != rhs->center_y) {
    return false;
  }
  // depth_m
  if (lhs->depth_m != rhs->depth_m) {
    return false;
  }
  return true;
}

bool
rc26_perception__msg__BlockDetection__copy(
  const rc26_perception__msg__BlockDetection * input,
  rc26_perception__msg__BlockDetection * output)
{
  if (!input || !output) {
    return false;
  }
  // class_name
  if (!rosidl_runtime_c__String__copy(
      &(input->class_name), &(output->class_name)))
  {
    return false;
  }
  // center_x
  output->center_x = input->center_x;
  // center_y
  output->center_y = input->center_y;
  // depth_m
  output->depth_m = input->depth_m;
  return true;
}

rc26_perception__msg__BlockDetection *
rc26_perception__msg__BlockDetection__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rc26_perception__msg__BlockDetection * msg = (rc26_perception__msg__BlockDetection *)allocator.allocate(sizeof(rc26_perception__msg__BlockDetection), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rc26_perception__msg__BlockDetection));
  bool success = rc26_perception__msg__BlockDetection__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rc26_perception__msg__BlockDetection__destroy(rc26_perception__msg__BlockDetection * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rc26_perception__msg__BlockDetection__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rc26_perception__msg__BlockDetection__Sequence__init(rc26_perception__msg__BlockDetection__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rc26_perception__msg__BlockDetection * data = NULL;

  if (size) {
    data = (rc26_perception__msg__BlockDetection *)allocator.zero_allocate(size, sizeof(rc26_perception__msg__BlockDetection), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rc26_perception__msg__BlockDetection__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rc26_perception__msg__BlockDetection__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rc26_perception__msg__BlockDetection__Sequence__fini(rc26_perception__msg__BlockDetection__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rc26_perception__msg__BlockDetection__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rc26_perception__msg__BlockDetection__Sequence *
rc26_perception__msg__BlockDetection__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rc26_perception__msg__BlockDetection__Sequence * array = (rc26_perception__msg__BlockDetection__Sequence *)allocator.allocate(sizeof(rc26_perception__msg__BlockDetection__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rc26_perception__msg__BlockDetection__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rc26_perception__msg__BlockDetection__Sequence__destroy(rc26_perception__msg__BlockDetection__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rc26_perception__msg__BlockDetection__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rc26_perception__msg__BlockDetection__Sequence__are_equal(const rc26_perception__msg__BlockDetection__Sequence * lhs, const rc26_perception__msg__BlockDetection__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rc26_perception__msg__BlockDetection__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rc26_perception__msg__BlockDetection__Sequence__copy(
  const rc26_perception__msg__BlockDetection__Sequence * input,
  rc26_perception__msg__BlockDetection__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rc26_perception__msg__BlockDetection);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rc26_perception__msg__BlockDetection * data =
      (rc26_perception__msg__BlockDetection *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rc26_perception__msg__BlockDetection__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rc26_perception__msg__BlockDetection__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rc26_perception__msg__BlockDetection__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
