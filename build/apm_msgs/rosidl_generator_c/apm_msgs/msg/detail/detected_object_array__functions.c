// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from apm_msgs:msg/DetectedObjectArray.idl
// generated code does not contain a copyright notice
#include "apm_msgs/msg/detail/detected_object_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `objects`
#include "apm_msgs/msg/detail/detected_object__functions.h"

bool
apm_msgs__msg__DetectedObjectArray__init(apm_msgs__msg__DetectedObjectArray * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    apm_msgs__msg__DetectedObjectArray__fini(msg);
    return false;
  }
  // objects
  if (!apm_msgs__msg__DetectedObject__Sequence__init(&msg->objects, 0)) {
    apm_msgs__msg__DetectedObjectArray__fini(msg);
    return false;
  }
  return true;
}

void
apm_msgs__msg__DetectedObjectArray__fini(apm_msgs__msg__DetectedObjectArray * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // objects
  apm_msgs__msg__DetectedObject__Sequence__fini(&msg->objects);
}

bool
apm_msgs__msg__DetectedObjectArray__are_equal(const apm_msgs__msg__DetectedObjectArray * lhs, const apm_msgs__msg__DetectedObjectArray * rhs)
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
  // objects
  if (!apm_msgs__msg__DetectedObject__Sequence__are_equal(
      &(lhs->objects), &(rhs->objects)))
  {
    return false;
  }
  return true;
}

bool
apm_msgs__msg__DetectedObjectArray__copy(
  const apm_msgs__msg__DetectedObjectArray * input,
  apm_msgs__msg__DetectedObjectArray * output)
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
  // objects
  if (!apm_msgs__msg__DetectedObject__Sequence__copy(
      &(input->objects), &(output->objects)))
  {
    return false;
  }
  return true;
}

apm_msgs__msg__DetectedObjectArray *
apm_msgs__msg__DetectedObjectArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  apm_msgs__msg__DetectedObjectArray * msg = (apm_msgs__msg__DetectedObjectArray *)allocator.allocate(sizeof(apm_msgs__msg__DetectedObjectArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(apm_msgs__msg__DetectedObjectArray));
  bool success = apm_msgs__msg__DetectedObjectArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
apm_msgs__msg__DetectedObjectArray__destroy(apm_msgs__msg__DetectedObjectArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    apm_msgs__msg__DetectedObjectArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
apm_msgs__msg__DetectedObjectArray__Sequence__init(apm_msgs__msg__DetectedObjectArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  apm_msgs__msg__DetectedObjectArray * data = NULL;

  if (size) {
    data = (apm_msgs__msg__DetectedObjectArray *)allocator.zero_allocate(size, sizeof(apm_msgs__msg__DetectedObjectArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = apm_msgs__msg__DetectedObjectArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        apm_msgs__msg__DetectedObjectArray__fini(&data[i - 1]);
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
apm_msgs__msg__DetectedObjectArray__Sequence__fini(apm_msgs__msg__DetectedObjectArray__Sequence * array)
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
      apm_msgs__msg__DetectedObjectArray__fini(&array->data[i]);
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

apm_msgs__msg__DetectedObjectArray__Sequence *
apm_msgs__msg__DetectedObjectArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  apm_msgs__msg__DetectedObjectArray__Sequence * array = (apm_msgs__msg__DetectedObjectArray__Sequence *)allocator.allocate(sizeof(apm_msgs__msg__DetectedObjectArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = apm_msgs__msg__DetectedObjectArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
apm_msgs__msg__DetectedObjectArray__Sequence__destroy(apm_msgs__msg__DetectedObjectArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    apm_msgs__msg__DetectedObjectArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
apm_msgs__msg__DetectedObjectArray__Sequence__are_equal(const apm_msgs__msg__DetectedObjectArray__Sequence * lhs, const apm_msgs__msg__DetectedObjectArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!apm_msgs__msg__DetectedObjectArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
apm_msgs__msg__DetectedObjectArray__Sequence__copy(
  const apm_msgs__msg__DetectedObjectArray__Sequence * input,
  apm_msgs__msg__DetectedObjectArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(apm_msgs__msg__DetectedObjectArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    apm_msgs__msg__DetectedObjectArray * data =
      (apm_msgs__msg__DetectedObjectArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!apm_msgs__msg__DetectedObjectArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          apm_msgs__msg__DetectedObjectArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!apm_msgs__msg__DetectedObjectArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
