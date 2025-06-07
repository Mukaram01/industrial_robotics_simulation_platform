// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from apm_msgs:msg/Detection2D.idl
// generated code does not contain a copyright notice

#ifndef APM_MSGS__MSG__DETAIL__DETECTION2_D__TRAITS_HPP_
#define APM_MSGS__MSG__DETAIL__DETECTION2_D__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "apm_msgs/msg/detail/detection2_d__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'bbox'
#include "sensor_msgs/msg/detail/region_of_interest__traits.hpp"

namespace apm_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Detection2D & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: bbox
  {
    out << "bbox: ";
    to_flow_style_yaml(msg.bbox, out);
    out << ", ";
  }

  // member: class_id
  {
    out << "class_id: ";
    rosidl_generator_traits::value_to_yaml(msg.class_id, out);
    out << ", ";
  }

  // member: class_name
  {
    out << "class_name: ";
    rosidl_generator_traits::value_to_yaml(msg.class_name, out);
    out << ", ";
  }

  // member: score
  {
    out << "score: ";
    rosidl_generator_traits::value_to_yaml(msg.score, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Detection2D & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: bbox
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bbox:\n";
    to_block_style_yaml(msg.bbox, out, indentation + 2);
  }

  // member: class_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "class_id: ";
    rosidl_generator_traits::value_to_yaml(msg.class_id, out);
    out << "\n";
  }

  // member: class_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "class_name: ";
    rosidl_generator_traits::value_to_yaml(msg.class_name, out);
    out << "\n";
  }

  // member: score
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "score: ";
    rosidl_generator_traits::value_to_yaml(msg.score, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Detection2D & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace apm_msgs

namespace rosidl_generator_traits
{

[[deprecated("use apm_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const apm_msgs::msg::Detection2D & msg,
  std::ostream & out, size_t indentation = 0)
{
  apm_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use apm_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const apm_msgs::msg::Detection2D & msg)
{
  return apm_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<apm_msgs::msg::Detection2D>()
{
  return "apm_msgs::msg::Detection2D";
}

template<>
inline const char * name<apm_msgs::msg::Detection2D>()
{
  return "apm_msgs/msg/Detection2D";
}

template<>
struct has_fixed_size<apm_msgs::msg::Detection2D>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<apm_msgs::msg::Detection2D>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<apm_msgs::msg::Detection2D>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // APM_MSGS__MSG__DETAIL__DETECTION2_D__TRAITS_HPP_
