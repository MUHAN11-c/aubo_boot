// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from aubo_msgs:msg/RobotStateRTMsg.idl
// generated code does not contain a copyright notice

#ifndef AUBO_MSGS__MSG__DETAIL__ROBOT_STATE_RT_MSG__TRAITS_HPP_
#define AUBO_MSGS__MSG__DETAIL__ROBOT_STATE_RT_MSG__TRAITS_HPP_

#include "aubo_msgs/msg/detail/robot_state_rt_msg__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<aubo_msgs::msg::RobotStateRTMsg>()
{
  return "aubo_msgs::msg::RobotStateRTMsg";
}

template<>
inline const char * name<aubo_msgs::msg::RobotStateRTMsg>()
{
  return "aubo_msgs/msg/RobotStateRTMsg";
}

template<>
struct has_fixed_size<aubo_msgs::msg::RobotStateRTMsg>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<aubo_msgs::msg::RobotStateRTMsg>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<aubo_msgs::msg::RobotStateRTMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUBO_MSGS__MSG__DETAIL__ROBOT_STATE_RT_MSG__TRAITS_HPP_
