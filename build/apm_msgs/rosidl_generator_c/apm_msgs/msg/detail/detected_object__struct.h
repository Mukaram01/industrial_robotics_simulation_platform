// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from apm_msgs:msg/DetectedObject.idl
// generated code does not contain a copyright notice

#ifndef APM_MSGS__MSG__DETAIL__DETECTED_OBJECT__STRUCT_H_
#define APM_MSGS__MSG__DETAIL__DETECTED_OBJECT__STRUCT_H_

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
// Member 'class_name'
#include "rosidl_runtime_c/string.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'dimensions'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/DetectedObject in the package apm_msgs.
/**
  * DetectedObject message
  * Represents a 3D object detection with pose and dimensions
 */
typedef struct apm_msgs__msg__DetectedObject
{
  std_msgs__msg__Header header;
  /// Object ID for tracking
  int32_t id;
  /// Class information
  int32_t class_id;
  rosidl_runtime_c__String class_name;
  float confidence;
  /// 3D pose in camera frame
  geometry_msgs__msg__Pose pose;
  /// Estimated physical dimensions (meters)
  geometry_msgs__msg__Vector3 dimensions;
} apm_msgs__msg__DetectedObject;

// Struct for a sequence of apm_msgs__msg__DetectedObject.
typedef struct apm_msgs__msg__DetectedObject__Sequence
{
  apm_msgs__msg__DetectedObject * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} apm_msgs__msg__DetectedObject__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // APM_MSGS__MSG__DETAIL__DETECTED_OBJECT__STRUCT_H_
