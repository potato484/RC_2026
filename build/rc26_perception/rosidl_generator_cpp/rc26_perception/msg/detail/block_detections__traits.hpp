// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rc26_perception:msg/BlockDetections.idl
// generated code does not contain a copyright notice

#ifndef RC26_PERCEPTION__MSG__DETAIL__BLOCK_DETECTIONS__TRAITS_HPP_
#define RC26_PERCEPTION__MSG__DETAIL__BLOCK_DETECTIONS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rc26_perception/msg/detail/block_detections__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'detections'
#include "rc26_perception/msg/detail/block_detection__traits.hpp"

namespace rc26_perception
{

namespace msg
{

inline void to_flow_style_yaml(
  const BlockDetections & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: detections
  {
    if (msg.detections.size() == 0) {
      out << "detections: []";
    } else {
      out << "detections: [";
      size_t pending_items = msg.detections.size();
      for (auto item : msg.detections) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BlockDetections & msg,
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

  // member: detections
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.detections.size() == 0) {
      out << "detections: []\n";
    } else {
      out << "detections:\n";
      for (auto item : msg.detections) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BlockDetections & msg, bool use_flow_style = false)
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
  const rc26_perception::msg::BlockDetections & msg,
  std::ostream & out, size_t indentation = 0)
{
  rc26_perception::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rc26_perception::msg::to_yaml() instead")]]
inline std::string to_yaml(const rc26_perception::msg::BlockDetections & msg)
{
  return rc26_perception::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rc26_perception::msg::BlockDetections>()
{
  return "rc26_perception::msg::BlockDetections";
}

template<>
inline const char * name<rc26_perception::msg::BlockDetections>()
{
  return "rc26_perception/msg/BlockDetections";
}

template<>
struct has_fixed_size<rc26_perception::msg::BlockDetections>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rc26_perception::msg::BlockDetections>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rc26_perception::msg::BlockDetections>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RC26_PERCEPTION__MSG__DETAIL__BLOCK_DETECTIONS__TRAITS_HPP_
