// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from apm_msgs:msg/Detection2D.idl
// generated code does not contain a copyright notice

#ifndef APM_MSGS__MSG__DETAIL__DETECTION2_D__BUILDER_HPP_
#define APM_MSGS__MSG__DETAIL__DETECTION2_D__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "apm_msgs/msg/detail/detection2_d__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace apm_msgs
{

namespace msg
{

namespace builder
{

class Init_Detection2D_score
{
public:
  explicit Init_Detection2D_score(::apm_msgs::msg::Detection2D & msg)
  : msg_(msg)
  {}
  ::apm_msgs::msg::Detection2D score(::apm_msgs::msg::Detection2D::_score_type arg)
  {
    msg_.score = std::move(arg);
    return std::move(msg_);
  }

private:
  ::apm_msgs::msg::Detection2D msg_;
};

class Init_Detection2D_class_name
{
public:
  explicit Init_Detection2D_class_name(::apm_msgs::msg::Detection2D & msg)
  : msg_(msg)
  {}
  Init_Detection2D_score class_name(::apm_msgs::msg::Detection2D::_class_name_type arg)
  {
    msg_.class_name = std::move(arg);
    return Init_Detection2D_score(msg_);
  }

private:
  ::apm_msgs::msg::Detection2D msg_;
};

class Init_Detection2D_class_id
{
public:
  explicit Init_Detection2D_class_id(::apm_msgs::msg::Detection2D & msg)
  : msg_(msg)
  {}
  Init_Detection2D_class_name class_id(::apm_msgs::msg::Detection2D::_class_id_type arg)
  {
    msg_.class_id = std::move(arg);
    return Init_Detection2D_class_name(msg_);
  }

private:
  ::apm_msgs::msg::Detection2D msg_;
};

class Init_Detection2D_bbox
{
public:
  explicit Init_Detection2D_bbox(::apm_msgs::msg::Detection2D & msg)
  : msg_(msg)
  {}
  Init_Detection2D_class_id bbox(::apm_msgs::msg::Detection2D::_bbox_type arg)
  {
    msg_.bbox = std::move(arg);
    return Init_Detection2D_class_id(msg_);
  }

private:
  ::apm_msgs::msg::Detection2D msg_;
};

class Init_Detection2D_header
{
public:
  Init_Detection2D_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Detection2D_bbox header(::apm_msgs::msg::Detection2D::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Detection2D_bbox(msg_);
  }

private:
  ::apm_msgs::msg::Detection2D msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::apm_msgs::msg::Detection2D>()
{
  return apm_msgs::msg::builder::Init_Detection2D_header();
}

}  // namespace apm_msgs

#endif  // APM_MSGS__MSG__DETAIL__DETECTION2_D__BUILDER_HPP_
