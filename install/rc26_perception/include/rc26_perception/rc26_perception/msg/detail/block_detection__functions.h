// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from rc26_perception:msg/BlockDetection.idl
// generated code does not contain a copyright notice

#ifndef RC26_PERCEPTION__MSG__DETAIL__BLOCK_DETECTION__FUNCTIONS_H_
#define RC26_PERCEPTION__MSG__DETAIL__BLOCK_DETECTION__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "rc26_perception/msg/rosidl_generator_c__visibility_control.h"

#include "rc26_perception/msg/detail/block_detection__struct.h"

/// Initialize msg/BlockDetection message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rc26_perception__msg__BlockDetection
 * )) before or use
 * rc26_perception__msg__BlockDetection__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rc26_perception
bool
rc26_perception__msg__BlockDetection__init(rc26_perception__msg__BlockDetection * msg);

/// Finalize msg/BlockDetection message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc26_perception
void
rc26_perception__msg__BlockDetection__fini(rc26_perception__msg__BlockDetection * msg);

/// Create msg/BlockDetection message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rc26_perception__msg__BlockDetection__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rc26_perception
rc26_perception__msg__BlockDetection *
rc26_perception__msg__BlockDetection__create();

/// Destroy msg/BlockDetection message.
/**
 * It calls
 * rc26_perception__msg__BlockDetection__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc26_perception
void
rc26_perception__msg__BlockDetection__destroy(rc26_perception__msg__BlockDetection * msg);

/// Check for msg/BlockDetection message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc26_perception
bool
rc26_perception__msg__BlockDetection__are_equal(const rc26_perception__msg__BlockDetection * lhs, const rc26_perception__msg__BlockDetection * rhs);

/// Copy a msg/BlockDetection message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc26_perception
bool
rc26_perception__msg__BlockDetection__copy(
  const rc26_perception__msg__BlockDetection * input,
  rc26_perception__msg__BlockDetection * output);

/// Initialize array of msg/BlockDetection messages.
/**
 * It allocates the memory for the number of elements and calls
 * rc26_perception__msg__BlockDetection__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc26_perception
bool
rc26_perception__msg__BlockDetection__Sequence__init(rc26_perception__msg__BlockDetection__Sequence * array, size_t size);

/// Finalize array of msg/BlockDetection messages.
/**
 * It calls
 * rc26_perception__msg__BlockDetection__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc26_perception
void
rc26_perception__msg__BlockDetection__Sequence__fini(rc26_perception__msg__BlockDetection__Sequence * array);

/// Create array of msg/BlockDetection messages.
/**
 * It allocates the memory for the array and calls
 * rc26_perception__msg__BlockDetection__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rc26_perception
rc26_perception__msg__BlockDetection__Sequence *
rc26_perception__msg__BlockDetection__Sequence__create(size_t size);

/// Destroy array of msg/BlockDetection messages.
/**
 * It calls
 * rc26_perception__msg__BlockDetection__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc26_perception
void
rc26_perception__msg__BlockDetection__Sequence__destroy(rc26_perception__msg__BlockDetection__Sequence * array);

/// Check for msg/BlockDetection message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc26_perception
bool
rc26_perception__msg__BlockDetection__Sequence__are_equal(const rc26_perception__msg__BlockDetection__Sequence * lhs, const rc26_perception__msg__BlockDetection__Sequence * rhs);

/// Copy an array of msg/BlockDetection messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rc26_perception
bool
rc26_perception__msg__BlockDetection__Sequence__copy(
  const rc26_perception__msg__BlockDetection__Sequence * input,
  rc26_perception__msg__BlockDetection__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // RC26_PERCEPTION__MSG__DETAIL__BLOCK_DETECTION__FUNCTIONS_H_
