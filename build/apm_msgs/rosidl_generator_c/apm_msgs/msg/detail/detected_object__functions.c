// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from apm_msgs:msg/DetectedObject.idl
// generated code does not contain a copyright notice
#include "apm_msgs/msg/detail/detected_object__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `class_name`
#include "rosidl_runtime_c/string_functions.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `dimensions`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
apm_msgs__msg__DetectedObject__init(apm_msgs__msg__DetectedObject * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    apm_msgs__msg__DetectedObject__fini(msg);
    return false;
  }
  // id
  // class_id
  // class_name
  if (!rosidl_runtime_c__String__init(&msg->class_name)) {
    apm_msgs__msg__DetectedObject__fini(msg);
    return false;
  }
  // confidence
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    apm_msgs__msg__DetectedObject__fini(msg);
    return false;
  }
  // dimensions
  if (!geometry_msgs__msg__Vector3__init(&msg->dimensions)) {
    apm_msgs__msg__DetectedObject__fini(msg);
    return false;
  }
  return true;
}

void
apm_msgs__msg__DetectedObject__fini(apm_msgs__msg__DetectedObject * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // id
  // class_id
  // class_name
  rosidl_runtime_c__String__fini(&msg->class_name);
  // confidence
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
  // dimensions
  geometry_msgs__msg__Vector3__fini(&msg->dimensions);
}

bool
apm_msgs__msg__DetectedObject__are_equal(const apm_msgs__msg__DetectedObject * lhs, const apm_msgs__msg__DetectedObject * rhs)
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
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // class_id
  if (lhs->class_id != rhs->class_id) {
    return false;
  }
  // class_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->class_name), &(rhs->class_name)))
  {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // dimensions
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->dimensions), &(rhs->dimensions)))
  {
    return false;
  }
  return true;
}

bool
apm_msgs__msg__DetectedObject__copy(
  const apm_msgs__msg__DetectedObject * input,
  apm_msgs__msg__DetectedObject * output)
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
  // id
  output->id = input->id;
  // class_id
  output->class_id = input->class_id;
  // class_name
  if (!rosidl_runtime_c__String__copy(
      &(input->class_name), &(output->class_name)))
  {
    return false;
  }
  // confidence
  output->confidence = input->confidence;
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // dimensions
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->dimensions), &(output->dimensions)))
  {
    return false;
  }
  return true;
}

apm_msgs__msg__DetectedObject *
apm_msgs__msg__DetectedObject__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  apm_msgs__msg__DetectedObject * msg = (apm_msgs__msg__DetectedObject *)allocator.allocate(sizeof(apm_msgs__msg__DetectedObject), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(apm_msgs__msg__DetectedObject));
  bool success = apm_msgs__msg__DetectedObject__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
apm_msgs__msg__DetectedObject__destroy(apm_msgs__msg__DetectedObject * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    apm_msgs__msg__DetectedObject__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
apm_msgs__msg__DetectedObject__Sequence__init(apm_msgs__msg__DetectedObject__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  apm_msgs__msg__DetectedObject * data = NULL;

  if (size) {
    data = (apm_msgs__msg__DetectedObject *)allocator.zero_allocate(size, sizeof(apm_msgs__msg__DetectedObject), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = apm_msgs__msg__DetectedObject__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        apm_msgs__msg__DetectedObject__fini(&data[i - 1]);
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
apm_msgs__msg__DetectedObject__Sequence__fini(apm_msgs__msg__DetectedObject__Sequence * array)
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
      apm_msgs__msg__DetectedObject__fini(&array->data[i]);
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

apm_msgs__msg__DetectedObject__Sequence *
apm_msgs__msg__DetectedObject__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  apm_msgs__msg__DetectedObject__Sequence * array = (apm_msgs__msg__DetectedObject__Sequence *)allocator.allocate(sizeof(apm_msgs__msg__DetectedObject__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = apm_msgs__msg__DetectedObject__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
apm_msgs__msg__DetectedObject__Sequence__destroy(apm_msgs__msg__DetectedObject__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    apm_msgs__msg__DetectedObject__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
apm_msgs__msg__DetectedObject__Sequence__are_equal(const apm_msgs__msg__DetectedObject__Sequence * lhs, const apm_msgs__msg__DetectedObject__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!apm_msgs__msg__DetectedObject__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
apm_msgs__msg__DetectedObject__Sequence__copy(
  const apm_msgs__msg__DetectedObject__Sequence * input,
  apm_msgs__msg__DetectedObject__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(apm_msgs__msg__DetectedObject);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    apm_msgs__msg__DetectedObject * data =
      (apm_msgs__msg__DetectedObject *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!apm_msgs__msg__DetectedObject__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          apm_msgs__msg__DetectedObject__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!apm_msgs__msg__DetectedObject__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
