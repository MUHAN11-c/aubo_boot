// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from aubo_msgs:msg/IOStates.idl
// generated code does not contain a copyright notice

#ifndef AUBO_MSGS__MSG__DETAIL__IO_STATES__TRAITS_HPP_
#define AUBO_MSGS__MSG__DETAIL__IO_STATES__TRAITS_HPP_

#include "aubo_msgs/msg/detail/io_states__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<aubo_msgs::msg::IOStates>()
{
  return "aubo_msgs::msg::IOStates";
}

template<>
inline const char * name<aubo_msgs::msg::IOStates>()
{
  return "aubo_msgs/msg/IOStates";
}

template<>
struct has_fixed_size<aubo_msgs::msg::IOStates>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<aubo_msgs::msg::IOStates>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<aubo_msgs::msg::IOStates>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUBO_MSGS__MSG__DETAIL__IO_STATES__TRAITS_HPP_
