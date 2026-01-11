// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rc26_perception:msg/BlockDetections.idl
// generated code does not contain a copyright notice

#ifndef RC26_PERCEPTION__MSG__DETAIL__BLOCK_DETECTIONS__BUILDER_HPP_
#define RC26_PERCEPTION__MSG__DETAIL__BLOCK_DETECTIONS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rc26_perception/msg/detail/block_detections__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rc26_perception
{

namespace msg
{

namespace builder
{

class Init_BlockDetections_detections
{
public:
  explicit Init_BlockDetections_detections(::rc26_perception::msg::BlockDetections & msg)
  : msg_(msg)
  {}
  ::rc26_perception::msg::BlockDetections detections(::rc26_perception::msg::BlockDetections::_detections_type arg)
  {
    msg_.detections = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rc26_perception::msg::BlockDetections msg_;
};

class Init_BlockDetections_header
{
public:
  Init_BlockDetections_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BlockDetections_detections header(::rc26_perception::msg::BlockDetections::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_BlockDetections_detections(msg_);
  }

private:
  ::rc26_perception::msg::BlockDetections msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rc26_perception::msg::BlockDetections>()
{
  return rc26_perception::msg::builder::Init_BlockDetections_header();
}

}  // namespace rc26_perception

#endif  // RC26_PERCEPTION__MSG__DETAIL__BLOCK_DETECTIONS__BUILDER_HPP_
