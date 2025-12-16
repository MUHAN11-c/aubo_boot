// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from aubo_msgs:msg/MasterboardDataMsg.idl
// generated code does not contain a copyright notice

#ifndef AUBO_MSGS__MSG__DETAIL__MASTERBOARD_DATA_MSG__TRAITS_HPP_
#define AUBO_MSGS__MSG__DETAIL__MASTERBOARD_DATA_MSG__TRAITS_HPP_

#include "aubo_msgs/msg/detail/masterboard_data_msg__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<aubo_msgs::msg::MasterboardDataMsg>()
{
  return "aubo_msgs::msg::MasterboardDataMsg";
}

template<>
inline const char * name<aubo_msgs::msg::MasterboardDataMsg>()
{
  return "aubo_msgs/msg/MasterboardDataMsg";
}

template<>
struct has_fixed_size<aubo_msgs::msg::MasterboardDataMsg>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<aubo_msgs::msg::MasterboardDataMsg>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<aubo_msgs::msg::MasterboardDataMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUBO_MSGS__MSG__DETAIL__MASTERBOARD_DATA_MSG__TRAITS_HPP_
