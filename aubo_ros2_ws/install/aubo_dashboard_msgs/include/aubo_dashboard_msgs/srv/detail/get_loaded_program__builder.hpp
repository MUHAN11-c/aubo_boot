// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from aubo_dashboard_msgs:srv/GetLoadedProgram.idl
// generated code does not contain a copyright notice

#ifndef AUBO_DASHBOARD_MSGS__SRV__DETAIL__GET_LOADED_PROGRAM__BUILDER_HPP_
#define AUBO_DASHBOARD_MSGS__SRV__DETAIL__GET_LOADED_PROGRAM__BUILDER_HPP_

#include "aubo_dashboard_msgs/srv/detail/get_loaded_program__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace aubo_dashboard_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::aubo_dashboard_msgs::srv::GetLoadedProgram_Request>()
{
  return ::aubo_dashboard_msgs::srv::GetLoadedProgram_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace aubo_dashboard_msgs


namespace aubo_dashboard_msgs
{

namespace srv
{

namespace builder
{

class Init_GetLoadedProgram_Response_success
{
public:
  explicit Init_GetLoadedProgram_Response_success(::aubo_dashboard_msgs::srv::GetLoadedProgram_Response & msg)
  : msg_(msg)
  {}
  ::aubo_dashboard_msgs::srv::GetLoadedProgram_Response success(::aubo_dashboard_msgs::srv::GetLoadedProgram_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aubo_dashboard_msgs::srv::GetLoadedProgram_Response msg_;
};

class Init_GetLoadedProgram_Response_program_name
{
public:
  explicit Init_GetLoadedProgram_Response_program_name(::aubo_dashboard_msgs::srv::GetLoadedProgram_Response & msg)
  : msg_(msg)
  {}
  Init_GetLoadedProgram_Response_success program_name(::aubo_dashboard_msgs::srv::GetLoadedProgram_Response::_program_name_type arg)
  {
    msg_.program_name = std::move(arg);
    return Init_GetLoadedProgram_Response_success(msg_);
  }

private:
  ::aubo_dashboard_msgs::srv::GetLoadedProgram_Response msg_;
};

class Init_GetLoadedProgram_Response_answer
{
public:
  Init_GetLoadedProgram_Response_answer()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetLoadedProgram_Response_program_name answer(::aubo_dashboard_msgs::srv::GetLoadedProgram_Response::_answer_type arg)
  {
    msg_.answer = std::move(arg);
    return Init_GetLoadedProgram_Response_program_name(msg_);
  }

private:
  ::aubo_dashboard_msgs::srv::GetLoadedProgram_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::aubo_dashboard_msgs::srv::GetLoadedProgram_Response>()
{
  return aubo_dashboard_msgs::srv::builder::Init_GetLoadedProgram_Response_answer();
}

}  // namespace aubo_dashboard_msgs

#endif  // AUBO_DASHBOARD_MSGS__SRV__DETAIL__GET_LOADED_PROGRAM__BUILDER_HPP_
