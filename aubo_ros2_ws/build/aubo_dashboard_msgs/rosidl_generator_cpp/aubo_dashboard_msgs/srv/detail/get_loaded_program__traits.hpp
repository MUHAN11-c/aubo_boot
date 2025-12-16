// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from aubo_dashboard_msgs:srv/GetLoadedProgram.idl
// generated code does not contain a copyright notice

#ifndef AUBO_DASHBOARD_MSGS__SRV__DETAIL__GET_LOADED_PROGRAM__TRAITS_HPP_
#define AUBO_DASHBOARD_MSGS__SRV__DETAIL__GET_LOADED_PROGRAM__TRAITS_HPP_

#include "aubo_dashboard_msgs/srv/detail/get_loaded_program__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<aubo_dashboard_msgs::srv::GetLoadedProgram_Request>()
{
  return "aubo_dashboard_msgs::srv::GetLoadedProgram_Request";
}

template<>
inline const char * name<aubo_dashboard_msgs::srv::GetLoadedProgram_Request>()
{
  return "aubo_dashboard_msgs/srv/GetLoadedProgram_Request";
}

template<>
struct has_fixed_size<aubo_dashboard_msgs::srv::GetLoadedProgram_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<aubo_dashboard_msgs::srv::GetLoadedProgram_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<aubo_dashboard_msgs::srv::GetLoadedProgram_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<aubo_dashboard_msgs::srv::GetLoadedProgram_Response>()
{
  return "aubo_dashboard_msgs::srv::GetLoadedProgram_Response";
}

template<>
inline const char * name<aubo_dashboard_msgs::srv::GetLoadedProgram_Response>()
{
  return "aubo_dashboard_msgs/srv/GetLoadedProgram_Response";
}

template<>
struct has_fixed_size<aubo_dashboard_msgs::srv::GetLoadedProgram_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<aubo_dashboard_msgs::srv::GetLoadedProgram_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<aubo_dashboard_msgs::srv::GetLoadedProgram_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<aubo_dashboard_msgs::srv::GetLoadedProgram>()
{
  return "aubo_dashboard_msgs::srv::GetLoadedProgram";
}

template<>
inline const char * name<aubo_dashboard_msgs::srv::GetLoadedProgram>()
{
  return "aubo_dashboard_msgs/srv/GetLoadedProgram";
}

template<>
struct has_fixed_size<aubo_dashboard_msgs::srv::GetLoadedProgram>
  : std::integral_constant<
    bool,
    has_fixed_size<aubo_dashboard_msgs::srv::GetLoadedProgram_Request>::value &&
    has_fixed_size<aubo_dashboard_msgs::srv::GetLoadedProgram_Response>::value
  >
{
};

template<>
struct has_bounded_size<aubo_dashboard_msgs::srv::GetLoadedProgram>
  : std::integral_constant<
    bool,
    has_bounded_size<aubo_dashboard_msgs::srv::GetLoadedProgram_Request>::value &&
    has_bounded_size<aubo_dashboard_msgs::srv::GetLoadedProgram_Response>::value
  >
{
};

template<>
struct is_service<aubo_dashboard_msgs::srv::GetLoadedProgram>
  : std::true_type
{
};

template<>
struct is_service_request<aubo_dashboard_msgs::srv::GetLoadedProgram_Request>
  : std::true_type
{
};

template<>
struct is_service_response<aubo_dashboard_msgs::srv::GetLoadedProgram_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUBO_DASHBOARD_MSGS__SRV__DETAIL__GET_LOADED_PROGRAM__TRAITS_HPP_
