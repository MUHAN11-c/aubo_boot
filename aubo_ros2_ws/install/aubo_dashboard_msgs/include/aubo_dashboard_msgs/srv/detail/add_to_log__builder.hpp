// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from aubo_dashboard_msgs:srv/AddToLog.idl
// generated code does not contain a copyright notice

#ifndef AUBO_DASHBOARD_MSGS__SRV__DETAIL__ADD_TO_LOG__BUILDER_HPP_
#define AUBO_DASHBOARD_MSGS__SRV__DETAIL__ADD_TO_LOG__BUILDER_HPP_

#include "aubo_dashboard_msgs/srv/detail/add_to_log__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace aubo_dashboard_msgs
{

namespace srv
{

namespace builder
{

class Init_AddToLog_Request_message
{
public:
  Init_AddToLog_Request_message()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::aubo_dashboard_msgs::srv::AddToLog_Request message(::aubo_dashboard_msgs::srv::AddToLog_Request::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aubo_dashboard_msgs::srv::AddToLog_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::aubo_dashboard_msgs::srv::AddToLog_Request>()
{
  return aubo_dashboard_msgs::srv::builder::Init_AddToLog_Request_message();
}

}  // namespace aubo_dashboard_msgs


namespace aubo_dashboard_msgs
{

namespace srv
{

namespace builder
{

class Init_AddToLog_Response_success
{
public:
  explicit Init_AddToLog_Response_success(::aubo_dashboard_msgs::srv::AddToLog_Response & msg)
  : msg_(msg)
  {}
  ::aubo_dashboard_msgs::srv::AddToLog_Response success(::aubo_dashboard_msgs::srv::AddToLog_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aubo_dashboard_msgs::srv::AddToLog_Response msg_;
};

class Init_AddToLog_Response_answer
{
public:
  Init_AddToLog_Response_answer()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AddToLog_Response_success answer(::aubo_dashboard_msgs::srv::AddToLog_Response::_answer_type arg)
  {
    msg_.answer = std::move(arg);
    return Init_AddToLog_Response_success(msg_);
  }

private:
  ::aubo_dashboard_msgs::srv::AddToLog_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::aubo_dashboard_msgs::srv::AddToLog_Response>()
{
  return aubo_dashboard_msgs::srv::builder::Init_AddToLog_Response_answer();
}

}  // namespace aubo_dashboard_msgs

#endif  // AUBO_DASHBOARD_MSGS__SRV__DETAIL__ADD_TO_LOG__BUILDER_HPP_
