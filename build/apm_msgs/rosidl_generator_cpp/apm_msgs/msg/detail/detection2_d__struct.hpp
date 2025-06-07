// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from apm_msgs:msg/Detection2D.idl
// generated code does not contain a copyright notice

#ifndef APM_MSGS__MSG__DETAIL__DETECTION2_D__STRUCT_HPP_
#define APM_MSGS__MSG__DETAIL__DETECTION2_D__STRUCT_HPP_

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
// Member 'bbox'
#include "sensor_msgs/msg/detail/region_of_interest__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__apm_msgs__msg__Detection2D __attribute__((deprecated))
#else
# define DEPRECATED__apm_msgs__msg__Detection2D __declspec(deprecated)
#endif

namespace apm_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Detection2D_
{
  using Type = Detection2D_<ContainerAllocator>;

  explicit Detection2D_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    bbox(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->class_id = 0l;
      this->class_name = "";
      this->score = 0.0f;
    }
  }

  explicit Detection2D_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    bbox(_alloc, _init),
    class_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->class_id = 0l;
      this->class_name = "";
      this->score = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _bbox_type =
    sensor_msgs::msg::RegionOfInterest_<ContainerAllocator>;
  _bbox_type bbox;
  using _class_id_type =
    int32_t;
  _class_id_type class_id;
  using _class_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _class_name_type class_name;
  using _score_type =
    float;
  _score_type score;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__bbox(
    const sensor_msgs::msg::RegionOfInterest_<ContainerAllocator> & _arg)
  {
    this->bbox = _arg;
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
  Type & set__score(
    const float & _arg)
  {
    this->score = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    apm_msgs::msg::Detection2D_<ContainerAllocator> *;
  using ConstRawPtr =
    const apm_msgs::msg::Detection2D_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<apm_msgs::msg::Detection2D_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<apm_msgs::msg::Detection2D_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      apm_msgs::msg::Detection2D_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<apm_msgs::msg::Detection2D_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      apm_msgs::msg::Detection2D_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<apm_msgs::msg::Detection2D_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<apm_msgs::msg::Detection2D_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<apm_msgs::msg::Detection2D_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__apm_msgs__msg__Detection2D
    std::shared_ptr<apm_msgs::msg::Detection2D_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__apm_msgs__msg__Detection2D
    std::shared_ptr<apm_msgs::msg::Detection2D_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Detection2D_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->bbox != other.bbox) {
      return false;
    }
    if (this->class_id != other.class_id) {
      return false;
    }
    if (this->class_name != other.class_name) {
      return false;
    }
    if (this->score != other.score) {
      return false;
    }
    return true;
  }
  bool operator!=(const Detection2D_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Detection2D_

// alias to use template instance with default allocator
using Detection2D =
  apm_msgs::msg::Detection2D_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace apm_msgs

#endif  // APM_MSGS__MSG__DETAIL__DETECTION2_D__STRUCT_HPP_
