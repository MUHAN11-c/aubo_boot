// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from aubo_dashboard_msgs:srv/RawRequest.idl
// generated code does not contain a copyright notice

#ifndef AUBO_DASHBOARD_MSGS__SRV__DETAIL__RAW_REQUEST__TRAITS_HPP_
#define AUBO_DASHBOARD_MSGS__SRV__DETAIL__RAW_REQUEST__TRAITS_HPP_

#include "aubo_dashboard_msgs/srv/detail/raw_request__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<aubo_dashboard_msgs::srv::RawRequest_Request>()
{
  return "aubo_dashboard_msgs::srv::RawRequest_Request";
}

template<>
inline const char * name<aubo_dashboard_msgs::srv::RawRequest_Request>()
{
  return "aubo_dashboard_msgs/srv/RawRequest_Request";
}

template<>
struct has_fixed_size<aubo_dashboard_msgs::srv::RawRequest_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<aubo_dashboard_msgs::srv::RawRequest_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<aubo_dashboard_msgs::srv::RawRequest_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<aubo_dashboard_msgs::srv::RawRequest_Response>()
{
  return "aubo_dashboard_msgs::srv::RawRequest_Response";
}

template<>
inline const char * name<aubo_dashboard_msgs::srv::RawRequest_Response>()
{
  return "aubo_dashboard_msgs/srv/RawRequest_Response";
}

template<>
struct has_fixed_size<aubo_dashboard_msgs::srv::RawRequest_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<aubo_dashboard_msgs::srv::RawRequest_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<aubo_dashboard_msgs::srv::RawRequest_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<aubo_dashboard_msgs::srv::RawRequest>()
{
  return "aubo_dashboard_msgs::srv::RawRequest";
}

template<>
inline const char * name<aubo_dashboard_msgs::srv::RawRequest>()
{
  return "aubo_dashboard_msgs/srv/RawRequest";
}

template<>
struct has_fixed_size<aubo_dashboard_msgs::srv::RawRequest>
  : std::integral_constant<
    bool,
    has_fixed_size<aubo_dashboard_msgs::srv::RawRequest_Request>::value &&
    has_fixed_size<aubo_dashboard_msgs::srv::RawRequest_Response>::value
  >
{
};

template<>
struct has_bounded_size<aubo_dashboard_msgs::srv::RawRequest>
  : std::integral_constant<
    bool,
    has_bounded_size<aubo_dashboard_msgs::srv::RawRequest_Request>::value &&
    has_bounded_size<aubo_dashboard_msgs::srv::RawRequest_Response>::value
  >
{
};

template<>
struct is_service<aubo_dashboard_msgs::srv::RawRequest>
  : std::true_type
{
};

template<>
struct is_service_request<aubo_dashboard_msgs::srv::RawRequest_Request>
  : std::true_type
{
};

template<>
struct is_service_response<aubo_dashboard_msgs::srv::RawRequest_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUBO_DASHBOARD_MSGS__SRV__DETAIL__RAW_REQUEST__TRAITS_HPP_
