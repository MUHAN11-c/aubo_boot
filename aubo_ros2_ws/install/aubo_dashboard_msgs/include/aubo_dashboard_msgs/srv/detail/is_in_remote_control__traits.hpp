// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from aubo_dashboard_msgs:srv/IsInRemoteControl.idl
// generated code does not contain a copyright notice

#ifndef AUBO_DASHBOARD_MSGS__SRV__DETAIL__IS_IN_REMOTE_CONTROL__TRAITS_HPP_
#define AUBO_DASHBOARD_MSGS__SRV__DETAIL__IS_IN_REMOTE_CONTROL__TRAITS_HPP_

#include "aubo_dashboard_msgs/srv/detail/is_in_remote_control__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<aubo_dashboard_msgs::srv::IsInRemoteControl_Request>()
{
  return "aubo_dashboard_msgs::srv::IsInRemoteControl_Request";
}

template<>
inline const char * name<aubo_dashboard_msgs::srv::IsInRemoteControl_Request>()
{
  return "aubo_dashboard_msgs/srv/IsInRemoteControl_Request";
}

template<>
struct has_fixed_size<aubo_dashboard_msgs::srv::IsInRemoteControl_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<aubo_dashboard_msgs::srv::IsInRemoteControl_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<aubo_dashboard_msgs::srv::IsInRemoteControl_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<aubo_dashboard_msgs::srv::IsInRemoteControl_Response>()
{
  return "aubo_dashboard_msgs::srv::IsInRemoteControl_Response";
}

template<>
inline const char * name<aubo_dashboard_msgs::srv::IsInRemoteControl_Response>()
{
  return "aubo_dashboard_msgs/srv/IsInRemoteControl_Response";
}

template<>
struct has_fixed_size<aubo_dashboard_msgs::srv::IsInRemoteControl_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<aubo_dashboard_msgs::srv::IsInRemoteControl_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<aubo_dashboard_msgs::srv::IsInRemoteControl_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<aubo_dashboard_msgs::srv::IsInRemoteControl>()
{
  return "aubo_dashboard_msgs::srv::IsInRemoteControl";
}

template<>
inline const char * name<aubo_dashboard_msgs::srv::IsInRemoteControl>()
{
  return "aubo_dashboard_msgs/srv/IsInRemoteControl";
}

template<>
struct has_fixed_size<aubo_dashboard_msgs::srv::IsInRemoteControl>
  : std::integral_constant<
    bool,
    has_fixed_size<aubo_dashboard_msgs::srv::IsInRemoteControl_Request>::value &&
    has_fixed_size<aubo_dashboard_msgs::srv::IsInRemoteControl_Response>::value
  >
{
};

template<>
struct has_bounded_size<aubo_dashboard_msgs::srv::IsInRemoteControl>
  : std::integral_constant<
    bool,
    has_bounded_size<aubo_dashboard_msgs::srv::IsInRemoteControl_Request>::value &&
    has_bounded_size<aubo_dashboard_msgs::srv::IsInRemoteControl_Response>::value
  >
{
};

template<>
struct is_service<aubo_dashboard_msgs::srv::IsInRemoteControl>
  : std::true_type
{
};

template<>
struct is_service_request<aubo_dashboard_msgs::srv::IsInRemoteControl_Request>
  : std::true_type
{
};

template<>
struct is_service_response<aubo_dashboard_msgs::srv::IsInRemoteControl_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUBO_DASHBOARD_MSGS__SRV__DETAIL__IS_IN_REMOTE_CONTROL__TRAITS_HPP_
