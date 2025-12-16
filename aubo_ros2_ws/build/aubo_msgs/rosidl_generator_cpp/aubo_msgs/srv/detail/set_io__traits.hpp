// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from aubo_msgs:srv/SetIO.idl
// generated code does not contain a copyright notice

#ifndef AUBO_MSGS__SRV__DETAIL__SET_IO__TRAITS_HPP_
#define AUBO_MSGS__SRV__DETAIL__SET_IO__TRAITS_HPP_

#include "aubo_msgs/srv/detail/set_io__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<aubo_msgs::srv::SetIO_Request>()
{
  return "aubo_msgs::srv::SetIO_Request";
}

template<>
inline const char * name<aubo_msgs::srv::SetIO_Request>()
{
  return "aubo_msgs/srv/SetIO_Request";
}

template<>
struct has_fixed_size<aubo_msgs::srv::SetIO_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<aubo_msgs::srv::SetIO_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<aubo_msgs::srv::SetIO_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<aubo_msgs::srv::SetIO_Response>()
{
  return "aubo_msgs::srv::SetIO_Response";
}

template<>
inline const char * name<aubo_msgs::srv::SetIO_Response>()
{
  return "aubo_msgs/srv/SetIO_Response";
}

template<>
struct has_fixed_size<aubo_msgs::srv::SetIO_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<aubo_msgs::srv::SetIO_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<aubo_msgs::srv::SetIO_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<aubo_msgs::srv::SetIO>()
{
  return "aubo_msgs::srv::SetIO";
}

template<>
inline const char * name<aubo_msgs::srv::SetIO>()
{
  return "aubo_msgs/srv/SetIO";
}

template<>
struct has_fixed_size<aubo_msgs::srv::SetIO>
  : std::integral_constant<
    bool,
    has_fixed_size<aubo_msgs::srv::SetIO_Request>::value &&
    has_fixed_size<aubo_msgs::srv::SetIO_Response>::value
  >
{
};

template<>
struct has_bounded_size<aubo_msgs::srv::SetIO>
  : std::integral_constant<
    bool,
    has_bounded_size<aubo_msgs::srv::SetIO_Request>::value &&
    has_bounded_size<aubo_msgs::srv::SetIO_Response>::value
  >
{
};

template<>
struct is_service<aubo_msgs::srv::SetIO>
  : std::true_type
{
};

template<>
struct is_service_request<aubo_msgs::srv::SetIO_Request>
  : std::true_type
{
};

template<>
struct is_service_response<aubo_msgs::srv::SetIO_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUBO_MSGS__SRV__DETAIL__SET_IO__TRAITS_HPP_
