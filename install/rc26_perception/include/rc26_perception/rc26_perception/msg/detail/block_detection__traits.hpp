// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rc26_perception:msg/BlockDetection.idl
// generated code does not contain a copyright notice

#ifndef RC26_PERCEPTION__MSG__DETAIL__BLOCK_DETECTION__TRAITS_HPP_
#define RC26_PERCEPTION__MSG__DETAIL__BLOCK_DETECTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rc26_perception/msg/detail/block_detection__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rc26_perception
{

namespace msg
{

inline void to_flow_style_yaml(
  const BlockDetection & msg,
  std::ostream & out)
{
  out << "{";
  // member: class_name
  {
    out << "class_name: ";
    rosidl_generator_traits::value_to_yaml(msg.class_name, out);
    out << ", ";
  }

  // member: center_x
  {
    out << "center_x: ";
    rosidl_generator_traits::value_to_yaml(msg.center_x, out);
    out << ", ";
  }

  // member: center_y
  {
    out << "center_y: ";
    rosidl_generator_traits::value_to_yaml(msg.center_y, out);
    out << ", ";
  }

  // member: depth_m
  {
    out << "depth_m: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_m, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BlockDetection & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: class_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "class_name: ";
    rosidl_generator_traits::value_to_yaml(msg.class_name, out);
    out << "\n";
  }

  // member: center_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "center_x: ";
    rosidl_generator_traits::value_to_yaml(msg.center_x, out);
    out << "\n";
  }

  // member: center_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "center_y: ";
    rosidl_generator_traits::value_to_yaml(msg.center_y, out);
    out << "\n";
  }

  // member: depth_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "depth_m: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_m, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BlockDetection & msg, bool use_flow_style = false)
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

}  // namespace rc26_perception

namespace rosidl_generator_traits
{

[[deprecated("use rc26_perception::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rc26_perception::msg::BlockDetection & msg,
  std::ostream & out, size_t indentation = 0)
{
  rc26_perception::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rc26_perception::msg::to_yaml() instead")]]
inline std::string to_yaml(const rc26_perception::msg::BlockDetection & msg)
{
  return rc26_perception::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rc26_perception::msg::BlockDetection>()
{
  return "rc26_perception::msg::BlockDetection";
}

template<>
inline const char * name<rc26_perception::msg::BlockDetection>()
{
  return "rc26_perception/msg/BlockDetection";
}

template<>
struct has_fixed_size<rc26_perception::msg::BlockDetection>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rc26_perception::msg::BlockDetection>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rc26_perception::msg::BlockDetection>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RC26_PERCEPTION__MSG__DETAIL__BLOCK_DETECTION__TRAITS_HPP_
