// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from apm_msgs:msg/DetectedObjectArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "apm_msgs/msg/detail/detected_object_array__rosidl_typesupport_introspection_c.h"
#include "apm_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "apm_msgs/msg/detail/detected_object_array__functions.h"
#include "apm_msgs/msg/detail/detected_object_array__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `objects`
#include "apm_msgs/msg/detected_object.h"
// Member `objects`
#include "apm_msgs/msg/detail/detected_object__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void apm_msgs__msg__DetectedObjectArray__rosidl_typesupport_introspection_c__DetectedObjectArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  apm_msgs__msg__DetectedObjectArray__init(message_memory);
}

void apm_msgs__msg__DetectedObjectArray__rosidl_typesupport_introspection_c__DetectedObjectArray_fini_function(void * message_memory)
{
  apm_msgs__msg__DetectedObjectArray__fini(message_memory);
}

size_t apm_msgs__msg__DetectedObjectArray__rosidl_typesupport_introspection_c__size_function__DetectedObjectArray__objects(
  const void * untyped_member)
{
  const apm_msgs__msg__DetectedObject__Sequence * member =
    (const apm_msgs__msg__DetectedObject__Sequence *)(untyped_member);
  return member->size;
}

const void * apm_msgs__msg__DetectedObjectArray__rosidl_typesupport_introspection_c__get_const_function__DetectedObjectArray__objects(
  const void * untyped_member, size_t index)
{
  const apm_msgs__msg__DetectedObject__Sequence * member =
    (const apm_msgs__msg__DetectedObject__Sequence *)(untyped_member);
  return &member->data[index];
}

void * apm_msgs__msg__DetectedObjectArray__rosidl_typesupport_introspection_c__get_function__DetectedObjectArray__objects(
  void * untyped_member, size_t index)
{
  apm_msgs__msg__DetectedObject__Sequence * member =
    (apm_msgs__msg__DetectedObject__Sequence *)(untyped_member);
  return &member->data[index];
}

void apm_msgs__msg__DetectedObjectArray__rosidl_typesupport_introspection_c__fetch_function__DetectedObjectArray__objects(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const apm_msgs__msg__DetectedObject * item =
    ((const apm_msgs__msg__DetectedObject *)
    apm_msgs__msg__DetectedObjectArray__rosidl_typesupport_introspection_c__get_const_function__DetectedObjectArray__objects(untyped_member, index));
  apm_msgs__msg__DetectedObject * value =
    (apm_msgs__msg__DetectedObject *)(untyped_value);
  *value = *item;
}

void apm_msgs__msg__DetectedObjectArray__rosidl_typesupport_introspection_c__assign_function__DetectedObjectArray__objects(
  void * untyped_member, size_t index, const void * untyped_value)
{
  apm_msgs__msg__DetectedObject * item =
    ((apm_msgs__msg__DetectedObject *)
    apm_msgs__msg__DetectedObjectArray__rosidl_typesupport_introspection_c__get_function__DetectedObjectArray__objects(untyped_member, index));
  const apm_msgs__msg__DetectedObject * value =
    (const apm_msgs__msg__DetectedObject *)(untyped_value);
  *item = *value;
}

bool apm_msgs__msg__DetectedObjectArray__rosidl_typesupport_introspection_c__resize_function__DetectedObjectArray__objects(
  void * untyped_member, size_t size)
{
  apm_msgs__msg__DetectedObject__Sequence * member =
    (apm_msgs__msg__DetectedObject__Sequence *)(untyped_member);
  apm_msgs__msg__DetectedObject__Sequence__fini(member);
  return apm_msgs__msg__DetectedObject__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember apm_msgs__msg__DetectedObjectArray__rosidl_typesupport_introspection_c__DetectedObjectArray_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(apm_msgs__msg__DetectedObjectArray, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "objects",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(apm_msgs__msg__DetectedObjectArray, objects),  // bytes offset in struct
    NULL,  // default value
    apm_msgs__msg__DetectedObjectArray__rosidl_typesupport_introspection_c__size_function__DetectedObjectArray__objects,  // size() function pointer
    apm_msgs__msg__DetectedObjectArray__rosidl_typesupport_introspection_c__get_const_function__DetectedObjectArray__objects,  // get_const(index) function pointer
    apm_msgs__msg__DetectedObjectArray__rosidl_typesupport_introspection_c__get_function__DetectedObjectArray__objects,  // get(index) function pointer
    apm_msgs__msg__DetectedObjectArray__rosidl_typesupport_introspection_c__fetch_function__DetectedObjectArray__objects,  // fetch(index, &value) function pointer
    apm_msgs__msg__DetectedObjectArray__rosidl_typesupport_introspection_c__assign_function__DetectedObjectArray__objects,  // assign(index, value) function pointer
    apm_msgs__msg__DetectedObjectArray__rosidl_typesupport_introspection_c__resize_function__DetectedObjectArray__objects  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers apm_msgs__msg__DetectedObjectArray__rosidl_typesupport_introspection_c__DetectedObjectArray_message_members = {
  "apm_msgs__msg",  // message namespace
  "DetectedObjectArray",  // message name
  2,  // number of fields
  sizeof(apm_msgs__msg__DetectedObjectArray),
  apm_msgs__msg__DetectedObjectArray__rosidl_typesupport_introspection_c__DetectedObjectArray_message_member_array,  // message members
  apm_msgs__msg__DetectedObjectArray__rosidl_typesupport_introspection_c__DetectedObjectArray_init_function,  // function to initialize message memory (memory has to be allocated)
  apm_msgs__msg__DetectedObjectArray__rosidl_typesupport_introspection_c__DetectedObjectArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t apm_msgs__msg__DetectedObjectArray__rosidl_typesupport_introspection_c__DetectedObjectArray_message_type_support_handle = {
  0,
  &apm_msgs__msg__DetectedObjectArray__rosidl_typesupport_introspection_c__DetectedObjectArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_apm_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, apm_msgs, msg, DetectedObjectArray)() {
  apm_msgs__msg__DetectedObjectArray__rosidl_typesupport_introspection_c__DetectedObjectArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  apm_msgs__msg__DetectedObjectArray__rosidl_typesupport_introspection_c__DetectedObjectArray_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, apm_msgs, msg, DetectedObject)();
  if (!apm_msgs__msg__DetectedObjectArray__rosidl_typesupport_introspection_c__DetectedObjectArray_message_type_support_handle.typesupport_identifier) {
    apm_msgs__msg__DetectedObjectArray__rosidl_typesupport_introspection_c__DetectedObjectArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &apm_msgs__msg__DetectedObjectArray__rosidl_typesupport_introspection_c__DetectedObjectArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
