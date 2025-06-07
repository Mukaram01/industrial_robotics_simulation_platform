// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from apm_msgs:msg/Detection2DArray.idl
// generated code does not contain a copyright notice

#ifndef APM_MSGS__MSG__DETAIL__DETECTION2_D_ARRAY__STRUCT_H_
#define APM_MSGS__MSG__DETAIL__DETECTION2_D_ARRAY__STRUCT_H_

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
#include "apm_msgs/msg/detail/detection2_d__struct.h"

/// Struct defined in msg/Detection2DArray in the package apm_msgs.
/**
  * Detection2DArray message
  * Represents an array of 2D object detections
 */
typedef struct apm_msgs__msg__Detection2DArray
{
  std_msgs__msg__Header header;
  apm_msgs__msg__Detection2D__Sequence detections;
} apm_msgs__msg__Detection2DArray;

// Struct for a sequence of apm_msgs__msg__Detection2DArray.
typedef struct apm_msgs__msg__Detection2DArray__Sequence
{
  apm_msgs__msg__Detection2DArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} apm_msgs__msg__Detection2DArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // APM_MSGS__MSG__DETAIL__DETECTION2_D_ARRAY__STRUCT_H_
