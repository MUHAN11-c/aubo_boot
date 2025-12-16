// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from aubo_dashboard_msgs:srv/IsInRemoteControl.idl
// generated code does not contain a copyright notice

#ifndef AUBO_DASHBOARD_MSGS__SRV__DETAIL__IS_IN_REMOTE_CONTROL__BUILDER_HPP_
#define AUBO_DASHBOARD_MSGS__SRV__DETAIL__IS_IN_REMOTE_CONTROL__BUILDER_HPP_

#include "aubo_dashboard_msgs/srv/detail/is_in_remote_control__struct.hpp"
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
auto build<::aubo_dashboard_msgs::srv::IsInRemoteControl_Request>()
{
  return ::aubo_dashboard_msgs::srv::IsInRemoteControl_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace aubo_dashboard_msgs


namespace aubo_dashboard_msgs
{

namespace srv
{

namespace builder
{

class Init_IsInRemoteControl_Response_success
{
public:
  explicit Init_IsInRemoteControl_Response_success(::aubo_dashboard_msgs::srv::IsInRemoteControl_Response & msg)
  : msg_(msg)
  {}
  ::aubo_dashboard_msgs::srv::IsInRemoteControl_Response success(::aubo_dashboard_msgs::srv::IsInRemoteControl_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aubo_dashboard_msgs::srv::IsInRemoteControl_Response msg_;
};

class Init_IsInRemoteControl_Response_in_remote_control
{
public:
  explicit Init_IsInRemoteControl_Response_in_remote_control(::aubo_dashboard_msgs::srv::IsInRemoteControl_Response & msg)
  : msg_(msg)
  {}
  Init_IsInRemoteControl_Response_success in_remote_control(::aubo_dashboard_msgs::srv::IsInRemoteControl_Response::_in_remote_control_type arg)
  {
    msg_.in_remote_control = std::move(arg);
    return Init_IsInRemoteControl_Response_success(msg_);
  }

private:
  ::aubo_dashboard_msgs::srv::IsInRemoteControl_Response msg_;
};

class Init_IsInRemoteControl_Response_answer
{
public:
  Init_IsInRemoteControl_Response_answer()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IsInRemoteControl_Response_in_remote_control answer(::aubo_dashboard_msgs::srv::IsInRemoteControl_Response::_answer_type arg)
  {
    msg_.answer = std::move(arg);
    return Init_IsInRemoteControl_Response_in_remote_control(msg_);
  }

private:
  ::aubo_dashboard_msgs::srv::IsInRemoteControl_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::aubo_dashboard_msgs::srv::IsInRemoteControl_Response>()
{
  return aubo_dashboard_msgs::srv::builder::Init_IsInRemoteControl_Response_answer();
}

}  // namespace aubo_dashboard_msgs

#endif  // AUBO_DASHBOARD_MSGS__SRV__DETAIL__IS_IN_REMOTE_CONTROL__BUILDER_HPP_
