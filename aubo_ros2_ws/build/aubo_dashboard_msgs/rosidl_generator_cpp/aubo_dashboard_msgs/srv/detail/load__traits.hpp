// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from aubo_dashboard_msgs:srv/Load.idl
// generated code does not contain a copyright notice

#ifndef AUBO_DASHBOARD_MSGS__SRV__DETAIL__LOAD__TRAITS_HPP_
#define AUBO_DASHBOARD_MSGS__SRV__DETAIL__LOAD__TRAITS_HPP_

#include "aubo_dashboard_msgs/srv/detail/load__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<aubo_dashboard_msgs::srv::Load_Request>()
{
  return "aubo_dashboard_msgs::srv::Load_Request";
}

template<>
inline const char * name<aubo_dashboard_msgs::srv::Load_Request>()
{
  return "aubo_dashboard_msgs/srv/Load_Request";
}

template<>
struct has_fixed_size<aubo_dashboard_msgs::srv::Load_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<aubo_dashboard_msgs::srv::Load_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<aubo_dashboard_msgs::srv::Load_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<aubo_dashboard_msgs::srv::Load_Response>()
{
  return "aubo_dashboard_msgs::srv::Load_Response";
}

template<>
inline const char * name<aubo_dashboard_msgs::srv::Load_Response>()
{
  return "aubo_dashboard_msgs/srv/Load_Response";
}

template<>
struct has_fixed_size<aubo_dashboard_msgs::srv::Load_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<aubo_dashboard_msgs::srv::Load_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<aubo_dashboard_msgs::srv::Load_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<aubo_dashboard_msgs::srv::Load>()
{
  return "aubo_dashboard_msgs::srv::Load";
}

template<>
inline const char * name<aubo_dashboard_msgs::srv::Load>()
{
  return "aubo_dashboard_msgs/srv/Load";
}

template<>
struct has_fixed_size<aubo_dashboard_msgs::srv::Load>
  : std::integral_constant<
    bool,
    has_fixed_size<aubo_dashboard_msgs::srv::Load_Request>::value &&
    has_fixed_size<aubo_dashboard_msgs::srv::Load_Response>::value
  >
{
};

template<>
struct has_bounded_size<aubo_dashboard_msgs::srv::Load>
  : std::integral_constant<
    bool,
    has_bounded_size<aubo_dashboard_msgs::srv::Load_Request>::value &&
    has_bounded_size<aubo_dashboard_msgs::srv::Load_Response>::value
  >
{
};

template<>
struct is_service<aubo_dashboard_msgs::srv::Load>
  : std::true_type
{
};

template<>
struct is_service_request<aubo_dashboard_msgs::srv::Load_Request>
  : std::true_type
{
};

template<>
struct is_service_response<aubo_dashboard_msgs::srv::Load_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUBO_DASHBOARD_MSGS__SRV__DETAIL__LOAD__TRAITS_HPP_
