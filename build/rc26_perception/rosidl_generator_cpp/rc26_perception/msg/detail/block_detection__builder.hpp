// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rc26_perception:msg/BlockDetection.idl
// generated code does not contain a copyright notice

#ifndef RC26_PERCEPTION__MSG__DETAIL__BLOCK_DETECTION__BUILDER_HPP_
#define RC26_PERCEPTION__MSG__DETAIL__BLOCK_DETECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rc26_perception/msg/detail/block_detection__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rc26_perception
{

namespace msg
{

namespace builder
{

class Init_BlockDetection_depth_m
{
public:
  explicit Init_BlockDetection_depth_m(::rc26_perception::msg::BlockDetection & msg)
  : msg_(msg)
  {}
  ::rc26_perception::msg::BlockDetection depth_m(::rc26_perception::msg::BlockDetection::_depth_m_type arg)
  {
    msg_.depth_m = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rc26_perception::msg::BlockDetection msg_;
};

class Init_BlockDetection_center_y
{
public:
  explicit Init_BlockDetection_center_y(::rc26_perception::msg::BlockDetection & msg)
  : msg_(msg)
  {}
  Init_BlockDetection_depth_m center_y(::rc26_perception::msg::BlockDetection::_center_y_type arg)
  {
    msg_.center_y = std::move(arg);
    return Init_BlockDetection_depth_m(msg_);
  }

private:
  ::rc26_perception::msg::BlockDetection msg_;
};

class Init_BlockDetection_center_x
{
public:
  explicit Init_BlockDetection_center_x(::rc26_perception::msg::BlockDetection & msg)
  : msg_(msg)
  {}
  Init_BlockDetection_center_y center_x(::rc26_perception::msg::BlockDetection::_center_x_type arg)
  {
    msg_.center_x = std::move(arg);
    return Init_BlockDetection_center_y(msg_);
  }

private:
  ::rc26_perception::msg::BlockDetection msg_;
};

class Init_BlockDetection_class_name
{
public:
  Init_BlockDetection_class_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BlockDetection_center_x class_name(::rc26_perception::msg::BlockDetection::_class_name_type arg)
  {
    msg_.class_name = std::move(arg);
    return Init_BlockDetection_center_x(msg_);
  }

private:
  ::rc26_perception::msg::BlockDetection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rc26_perception::msg::BlockDetection>()
{
  return rc26_perception::msg::builder::Init_BlockDetection_class_name();
}

}  // namespace rc26_perception

#endif  // RC26_PERCEPTION__MSG__DETAIL__BLOCK_DETECTION__BUILDER_HPP_
