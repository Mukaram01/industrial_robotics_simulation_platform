// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from apm_msgs:msg/DetectedObject.idl
// generated code does not contain a copyright notice

#ifndef APM_MSGS__MSG__DETAIL__DETECTED_OBJECT__BUILDER_HPP_
#define APM_MSGS__MSG__DETAIL__DETECTED_OBJECT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "apm_msgs/msg/detail/detected_object__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace apm_msgs
{

namespace msg
{

namespace builder
{

class Init_DetectedObject_dimensions
{
public:
  explicit Init_DetectedObject_dimensions(::apm_msgs::msg::DetectedObject & msg)
  : msg_(msg)
  {}
  ::apm_msgs::msg::DetectedObject dimensions(::apm_msgs::msg::DetectedObject::_dimensions_type arg)
  {
    msg_.dimensions = std::move(arg);
    return std::move(msg_);
  }

private:
  ::apm_msgs::msg::DetectedObject msg_;
};

class Init_DetectedObject_pose
{
public:
  explicit Init_DetectedObject_pose(::apm_msgs::msg::DetectedObject & msg)
  : msg_(msg)
  {}
  Init_DetectedObject_dimensions pose(::apm_msgs::msg::DetectedObject::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_DetectedObject_dimensions(msg_);
  }

private:
  ::apm_msgs::msg::DetectedObject msg_;
};

class Init_DetectedObject_confidence
{
public:
  explicit Init_DetectedObject_confidence(::apm_msgs::msg::DetectedObject & msg)
  : msg_(msg)
  {}
  Init_DetectedObject_pose confidence(::apm_msgs::msg::DetectedObject::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_DetectedObject_pose(msg_);
  }

private:
  ::apm_msgs::msg::DetectedObject msg_;
};

class Init_DetectedObject_class_name
{
public:
  explicit Init_DetectedObject_class_name(::apm_msgs::msg::DetectedObject & msg)
  : msg_(msg)
  {}
  Init_DetectedObject_confidence class_name(::apm_msgs::msg::DetectedObject::_class_name_type arg)
  {
    msg_.class_name = std::move(arg);
    return Init_DetectedObject_confidence(msg_);
  }

private:
  ::apm_msgs::msg::DetectedObject msg_;
};

class Init_DetectedObject_class_id
{
public:
  explicit Init_DetectedObject_class_id(::apm_msgs::msg::DetectedObject & msg)
  : msg_(msg)
  {}
  Init_DetectedObject_class_name class_id(::apm_msgs::msg::DetectedObject::_class_id_type arg)
  {
    msg_.class_id = std::move(arg);
    return Init_DetectedObject_class_name(msg_);
  }

private:
  ::apm_msgs::msg::DetectedObject msg_;
};

class Init_DetectedObject_id
{
public:
  explicit Init_DetectedObject_id(::apm_msgs::msg::DetectedObject & msg)
  : msg_(msg)
  {}
  Init_DetectedObject_class_id id(::apm_msgs::msg::DetectedObject::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_DetectedObject_class_id(msg_);
  }

private:
  ::apm_msgs::msg::DetectedObject msg_;
};

class Init_DetectedObject_header
{
public:
  Init_DetectedObject_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectedObject_id header(::apm_msgs::msg::DetectedObject::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DetectedObject_id(msg_);
  }

private:
  ::apm_msgs::msg::DetectedObject msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::apm_msgs::msg::DetectedObject>()
{
  return apm_msgs::msg::builder::Init_DetectedObject_header();
}

}  // namespace apm_msgs

#endif  // APM_MSGS__MSG__DETAIL__DETECTED_OBJECT__BUILDER_HPP_
