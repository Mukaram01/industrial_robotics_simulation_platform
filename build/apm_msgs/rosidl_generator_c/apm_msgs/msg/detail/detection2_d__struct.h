// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from apm_msgs:msg/Detection2D.idl
// generated code does not contain a copyright notice

#ifndef APM_MSGS__MSG__DETAIL__DETECTION2_D__STRUCT_H_
#define APM_MSGS__MSG__DETAIL__DETECTION2_D__STRUCT_H_

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
// Member 'bbox'
#include "sensor_msgs/msg/detail/region_of_interest__struct.h"
// Member 'class_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Detection2D in the package apm_msgs.
/**
  * Detection2D message
  * Represents a 2D object detection with class information
 */
typedef struct apm_msgs__msg__Detection2D
{
  std_msgs__msg__Header header;
  /// Bounding box in pixel coordinates
  sensor_msgs__msg__RegionOfInterest bbox;
  /// Class information
  int32_t class_id;
  rosidl_runtime_c__String class_name;
  float score;
} apm_msgs__msg__Detection2D;

// Struct for a sequence of apm_msgs__msg__Detection2D.
typedef struct apm_msgs__msg__Detection2D__Sequence
{
  apm_msgs__msg__Detection2D * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} apm_msgs__msg__Detection2D__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // APM_MSGS__MSG__DETAIL__DETECTION2_D__STRUCT_H_
