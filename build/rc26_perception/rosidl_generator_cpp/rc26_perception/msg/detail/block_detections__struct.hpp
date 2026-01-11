// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rc26_perception:msg/BlockDetections.idl
// generated code does not contain a copyright notice

#ifndef RC26_PERCEPTION__MSG__DETAIL__BLOCK_DETECTIONS__STRUCT_HPP_
#define RC26_PERCEPTION__MSG__DETAIL__BLOCK_DETECTIONS__STRUCT_HPP_

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
// Member 'detections'
#include "rc26_perception/msg/detail/block_detection__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rc26_perception__msg__BlockDetections __attribute__((deprecated))
#else
# define DEPRECATED__rc26_perception__msg__BlockDetections __declspec(deprecated)
#endif

namespace rc26_perception
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BlockDetections_
{
  using Type = BlockDetections_<ContainerAllocator>;

  explicit BlockDetections_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit BlockDetections_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _detections_type =
    std::vector<rc26_perception::msg::BlockDetection_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<rc26_perception::msg::BlockDetection_<ContainerAllocator>>>;
  _detections_type detections;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__detections(
    const std::vector<rc26_perception::msg::BlockDetection_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<rc26_perception::msg::BlockDetection_<ContainerAllocator>>> & _arg)
  {
    this->detections = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rc26_perception::msg::BlockDetections_<ContainerAllocator> *;
  using ConstRawPtr =
    const rc26_perception::msg::BlockDetections_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rc26_perception::msg::BlockDetections_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rc26_perception::msg::BlockDetections_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rc26_perception::msg::BlockDetections_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rc26_perception::msg::BlockDetections_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rc26_perception::msg::BlockDetections_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rc26_perception::msg::BlockDetections_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rc26_perception::msg::BlockDetections_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rc26_perception::msg::BlockDetections_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rc26_perception__msg__BlockDetections
    std::shared_ptr<rc26_perception::msg::BlockDetections_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rc26_perception__msg__BlockDetections
    std::shared_ptr<rc26_perception::msg::BlockDetections_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BlockDetections_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->detections != other.detections) {
      return false;
    }
    return true;
  }
  bool operator!=(const BlockDetections_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BlockDetections_

// alias to use template instance with default allocator
using BlockDetections =
  rc26_perception::msg::BlockDetections_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rc26_perception

#endif  // RC26_PERCEPTION__MSG__DETAIL__BLOCK_DETECTIONS__STRUCT_HPP_
