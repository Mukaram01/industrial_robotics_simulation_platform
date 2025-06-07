// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from apm_msgs:msg/DetectedObject.idl
// generated code does not contain a copyright notice

#ifndef APM_MSGS__MSG__DETAIL__DETECTED_OBJECT__STRUCT_HPP_
#define APM_MSGS__MSG__DETAIL__DETECTED_OBJECT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'dimensions'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__apm_msgs__msg__DetectedObject __attribute__((deprecated))
#else
# define DEPRECATED__apm_msgs__msg__DetectedObject __declspec(deprecated)
#endif

namespace apm_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DetectedObject_
{
  using Type = DetectedObject_<ContainerAllocator>;

  explicit DetectedObject_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init),
    dimensions(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->class_id = 0l;
      this->class_name = "";
      this->confidence = 0.0f;
    }
  }

  explicit DetectedObject_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    class_name(_alloc),
    pose(_alloc, _init),
    dimensions(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->class_id = 0l;
      this->class_name = "";
      this->confidence = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _id_type =
    int32_t;
  _id_type id;
  using _class_id_type =
    int32_t;
  _class_id_type class_id;
  using _class_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _class_name_type class_name;
  using _confidence_type =
    float;
  _confidence_type confidence;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _dimensions_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _dimensions_type dimensions;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__class_id(
    const int32_t & _arg)
  {
    this->class_id = _arg;
    return *this;
  }
  Type & set__class_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->class_name = _arg;
    return *this;
  }
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__dimensions(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->dimensions = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    apm_msgs::msg::DetectedObject_<ContainerAllocator> *;
  using ConstRawPtr =
    const apm_msgs::msg::DetectedObject_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<apm_msgs::msg::DetectedObject_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<apm_msgs::msg::DetectedObject_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      apm_msgs::msg::DetectedObject_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<apm_msgs::msg::DetectedObject_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      apm_msgs::msg::DetectedObject_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<apm_msgs::msg::DetectedObject_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<apm_msgs::msg::DetectedObject_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<apm_msgs::msg::DetectedObject_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__apm_msgs__msg__DetectedObject
    std::shared_ptr<apm_msgs::msg::DetectedObject_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__apm_msgs__msg__DetectedObject
    std::shared_ptr<apm_msgs::msg::DetectedObject_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DetectedObject_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->class_id != other.class_id) {
      return false;
    }
    if (this->class_name != other.class_name) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->dimensions != other.dimensions) {
      return false;
    }
    return true;
  }
  bool operator!=(const DetectedObject_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DetectedObject_

// alias to use template instance with default allocator
using DetectedObject =
  apm_msgs::msg::DetectedObject_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace apm_msgs

#endif  // APM_MSGS__MSG__DETAIL__DETECTED_OBJECT__STRUCT_HPP_
