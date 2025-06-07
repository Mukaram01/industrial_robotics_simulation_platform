// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from apm_msgs:msg/DetectedObjectArray.idl
// generated code does not contain a copyright notice

#ifndef APM_MSGS__MSG__DETAIL__DETECTED_OBJECT_ARRAY__STRUCT_H_
#define APM_MSGS__MSG__DETAIL__DETECTED_OBJECT_ARRAY__STRUCT_H_

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
// Member 'objects'
#include "apm_msgs/msg/detail/detected_object__struct.h"

/// Struct defined in msg/DetectedObjectArray in the package apm_msgs.
/**
  * DetectedObjectArray message
  * Represents an array of 3D object detections
 */
typedef struct apm_msgs__msg__DetectedObjectArray
{
  std_msgs__msg__Header header;
  apm_msgs__msg__DetectedObject__Sequence objects;
} apm_msgs__msg__DetectedObjectArray;

// Struct for a sequence of apm_msgs__msg__DetectedObjectArray.
typedef struct apm_msgs__msg__DetectedObjectArray__Sequence
{
  apm_msgs__msg__DetectedObjectArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} apm_msgs__msg__DetectedObjectArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // APM_MSGS__MSG__DETAIL__DETECTED_OBJECT_ARRAY__STRUCT_H_
