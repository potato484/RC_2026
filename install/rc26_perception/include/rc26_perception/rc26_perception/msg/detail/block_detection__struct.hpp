// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rc26_perception:msg/BlockDetection.idl
// generated code does not contain a copyright notice

#ifndef RC26_PERCEPTION__MSG__DETAIL__BLOCK_DETECTION__STRUCT_HPP_
#define RC26_PERCEPTION__MSG__DETAIL__BLOCK_DETECTION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rc26_perception__msg__BlockDetection __attribute__((deprecated))
#else
# define DEPRECATED__rc26_perception__msg__BlockDetection __declspec(deprecated)
#endif

namespace rc26_perception
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BlockDetection_
{
  using Type = BlockDetection_<ContainerAllocator>;

  explicit BlockDetection_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->class_name = "";
      this->center_x = 0l;
      this->center_y = 0l;
      this->depth_m = 0.0f;
    }
  }

  explicit BlockDetection_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : class_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->class_name = "";
      this->center_x = 0l;
      this->center_y = 0l;
      this->depth_m = 0.0f;
    }
  }

  // field types and members
  using _class_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _class_name_type class_name;
  using _center_x_type =
    int32_t;
  _center_x_type center_x;
  using _center_y_type =
    int32_t;
  _center_y_type center_y;
  using _depth_m_type =
    float;
  _depth_m_type depth_m;

  // setters for named parameter idiom
  Type & set__class_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->class_name = _arg;
    return *this;
  }
  Type & set__center_x(
    const int32_t & _arg)
  {
    this->center_x = _arg;
    return *this;
  }
  Type & set__center_y(
    const int32_t & _arg)
  {
    this->center_y = _arg;
    return *this;
  }
  Type & set__depth_m(
    const float & _arg)
  {
    this->depth_m = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rc26_perception::msg::BlockDetection_<ContainerAllocator> *;
  using ConstRawPtr =
    const rc26_perception::msg::BlockDetection_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rc26_perception::msg::BlockDetection_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rc26_perception::msg::BlockDetection_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rc26_perception::msg::BlockDetection_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rc26_perception::msg::BlockDetection_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rc26_perception::msg::BlockDetection_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rc26_perception::msg::BlockDetection_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rc26_perception::msg::BlockDetection_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rc26_perception::msg::BlockDetection_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rc26_perception__msg__BlockDetection
    std::shared_ptr<rc26_perception::msg::BlockDetection_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rc26_perception__msg__BlockDetection
    std::shared_ptr<rc26_perception::msg::BlockDetection_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BlockDetection_ & other) const
  {
    if (this->class_name != other.class_name) {
      return false;
    }
    if (this->center_x != other.center_x) {
      return false;
    }
    if (this->center_y != other.center_y) {
      return false;
    }
    if (this->depth_m != other.depth_m) {
      return false;
    }
    return true;
  }
  bool operator!=(const BlockDetection_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BlockDetection_

// alias to use template instance with default allocator
using BlockDetection =
  rc26_perception::msg::BlockDetection_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rc26_perception

#endif  // RC26_PERCEPTION__MSG__DETAIL__BLOCK_DETECTION__STRUCT_HPP_
