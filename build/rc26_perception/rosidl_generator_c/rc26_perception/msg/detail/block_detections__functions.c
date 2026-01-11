// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rc26_perception:msg/BlockDetections.idl
// generated code does not contain a copyright notice
#include "rc26_perception/msg/detail/block_detections__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `detections`
#include "rc26_perception/msg/detail/block_detection__functions.h"

bool
rc26_perception__msg__BlockDetections__init(rc26_perception__msg__BlockDetections * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    rc26_perception__msg__BlockDetections__fini(msg);
    return false;
  }
  // detections
  if (!rc26_perception__msg__BlockDetection__Sequence__init(&msg->detections, 0)) {
    rc26_perception__msg__BlockDetections__fini(msg);
    return false;
  }
  return true;
}

void
rc26_perception__msg__BlockDetections__fini(rc26_perception__msg__BlockDetections * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // detections
  rc26_perception__msg__BlockDetection__Sequence__fini(&msg->detections);
}

bool
rc26_perception__msg__BlockDetections__are_equal(const rc26_perception__msg__BlockDetections * lhs, const rc26_perception__msg__BlockDetections * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // detections
  if (!rc26_perception__msg__BlockDetection__Sequence__are_equal(
      &(lhs->detections), &(rhs->detections)))
  {
    return false;
  }
  return true;
}

bool
rc26_perception__msg__BlockDetections__copy(
  const rc26_perception__msg__BlockDetections * input,
  rc26_perception__msg__BlockDetections * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // detections
  if (!rc26_perception__msg__BlockDetection__Sequence__copy(
      &(input->detections), &(output->detections)))
  {
    return false;
  }
  return true;
}

rc26_perception__msg__BlockDetections *
rc26_perception__msg__BlockDetections__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rc26_perception__msg__BlockDetections * msg = (rc26_perception__msg__BlockDetections *)allocator.allocate(sizeof(rc26_perception__msg__BlockDetections), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rc26_perception__msg__BlockDetections));
  bool success = rc26_perception__msg__BlockDetections__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rc26_perception__msg__BlockDetections__destroy(rc26_perception__msg__BlockDetections * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rc26_perception__msg__BlockDetections__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rc26_perception__msg__BlockDetections__Sequence__init(rc26_perception__msg__BlockDetections__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rc26_perception__msg__BlockDetections * data = NULL;

  if (size) {
    data = (rc26_perception__msg__BlockDetections *)allocator.zero_allocate(size, sizeof(rc26_perception__msg__BlockDetections), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rc26_perception__msg__BlockDetections__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rc26_perception__msg__BlockDetections__fini(&data[i - 1]);
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
rc26_perception__msg__BlockDetections__Sequence__fini(rc26_perception__msg__BlockDetections__Sequence * array)
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
      rc26_perception__msg__BlockDetections__fini(&array->data[i]);
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

rc26_perception__msg__BlockDetections__Sequence *
rc26_perception__msg__BlockDetections__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rc26_perception__msg__BlockDetections__Sequence * array = (rc26_perception__msg__BlockDetections__Sequence *)allocator.allocate(sizeof(rc26_perception__msg__BlockDetections__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rc26_perception__msg__BlockDetections__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rc26_perception__msg__BlockDetections__Sequence__destroy(rc26_perception__msg__BlockDetections__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rc26_perception__msg__BlockDetections__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rc26_perception__msg__BlockDetections__Sequence__are_equal(const rc26_perception__msg__BlockDetections__Sequence * lhs, const rc26_perception__msg__BlockDetections__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rc26_perception__msg__BlockDetections__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rc26_perception__msg__BlockDetections__Sequence__copy(
  const rc26_perception__msg__BlockDetections__Sequence * input,
  rc26_perception__msg__BlockDetections__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rc26_perception__msg__BlockDetections);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rc26_perception__msg__BlockDetections * data =
      (rc26_perception__msg__BlockDetections *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rc26_perception__msg__BlockDetections__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rc26_perception__msg__BlockDetections__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rc26_perception__msg__BlockDetections__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
