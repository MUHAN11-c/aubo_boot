// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from aubo_dashboard_msgs:srv/IsProgramRunning.idl
// generated code does not contain a copyright notice

#ifndef AUBO_DASHBOARD_MSGS__SRV__DETAIL__IS_PROGRAM_RUNNING__TRAITS_HPP_
#define AUBO_DASHBOARD_MSGS__SRV__DETAIL__IS_PROGRAM_RUNNING__TRAITS_HPP_

#include "aubo_dashboard_msgs/srv/detail/is_program_running__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<aubo_dashboard_msgs::srv::IsProgramRunning_Request>()
{
  return "aubo_dashboard_msgs::srv::IsProgramRunning_Request";
}

template<>
inline const char * name<aubo_dashboard_msgs::srv::IsProgramRunning_Request>()
{
  return "aubo_dashboard_msgs/srv/IsProgramRunning_Request";
}

template<>
struct has_fixed_size<aubo_dashboard_msgs::srv::IsProgramRunning_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<aubo_dashboard_msgs::srv::IsProgramRunning_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<aubo_dashboard_msgs::srv::IsProgramRunning_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<aubo_dashboard_msgs::srv::IsProgramRunning_Response>()
{
  return "aubo_dashboard_msgs::srv::IsProgramRunning_Response";
}

template<>
inline const char * name<aubo_dashboard_msgs::srv::IsProgramRunning_Response>()
{
  return "aubo_dashboard_msgs/srv/IsProgramRunning_Response";
}

template<>
struct has_fixed_size<aubo_dashboard_msgs::srv::IsProgramRunning_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<aubo_dashboard_msgs::srv::IsProgramRunning_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<aubo_dashboard_msgs::srv::IsProgramRunning_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<aubo_dashboard_msgs::srv::IsProgramRunning>()
{
  return "aubo_dashboard_msgs::srv::IsProgramRunning";
}

template<>
inline const char * name<aubo_dashboard_msgs::srv::IsProgramRunning>()
{
  return "aubo_dashboard_msgs/srv/IsProgramRunning";
}

template<>
struct has_fixed_size<aubo_dashboard_msgs::srv::IsProgramRunning>
  : std::integral_constant<
    bool,
    has_fixed_size<aubo_dashboard_msgs::srv::IsProgramRunning_Request>::value &&
    has_fixed_size<aubo_dashboard_msgs::srv::IsProgramRunning_Response>::value
  >
{
};

template<>
struct has_bounded_size<aubo_dashboard_msgs::srv::IsProgramRunning>
  : std::integral_constant<
    bool,
    has_bounded_size<aubo_dashboard_msgs::srv::IsProgramRunning_Request>::value &&
    has_bounded_size<aubo_dashboard_msgs::srv::IsProgramRunning_Response>::value
  >
{
};

template<>
struct is_service<aubo_dashboard_msgs::srv::IsProgramRunning>
  : std::true_type
{
};

template<>
struct is_service_request<aubo_dashboard_msgs::srv::IsProgramRunning_Request>
  : std::true_type
{
};

template<>
struct is_service_response<aubo_dashboard_msgs::srv::IsProgramRunning_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUBO_DASHBOARD_MSGS__SRV__DETAIL__IS_PROGRAM_RUNNING__TRAITS_HPP_
