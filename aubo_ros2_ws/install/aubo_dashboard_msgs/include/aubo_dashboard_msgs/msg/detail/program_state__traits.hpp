// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from aubo_dashboard_msgs:msg/ProgramState.idl
// generated code does not contain a copyright notice

#ifndef AUBO_DASHBOARD_MSGS__MSG__DETAIL__PROGRAM_STATE__TRAITS_HPP_
#define AUBO_DASHBOARD_MSGS__MSG__DETAIL__PROGRAM_STATE__TRAITS_HPP_

#include "aubo_dashboard_msgs/msg/detail/program_state__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<aubo_dashboard_msgs::msg::ProgramState>()
{
  return "aubo_dashboard_msgs::msg::ProgramState";
}

template<>
inline const char * name<aubo_dashboard_msgs::msg::ProgramState>()
{
  return "aubo_dashboard_msgs/msg/ProgramState";
}

template<>
struct has_fixed_size<aubo_dashboard_msgs::msg::ProgramState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<aubo_dashboard_msgs::msg::ProgramState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<aubo_dashboard_msgs::msg::ProgramState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUBO_DASHBOARD_MSGS__MSG__DETAIL__PROGRAM_STATE__TRAITS_HPP_
