// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from aubo_dashboard_msgs:srv/Load.idl
// generated code does not contain a copyright notice

#ifndef AUBO_DASHBOARD_MSGS__SRV__DETAIL__LOAD__BUILDER_HPP_
#define AUBO_DASHBOARD_MSGS__SRV__DETAIL__LOAD__BUILDER_HPP_

#include "aubo_dashboard_msgs/srv/detail/load__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace aubo_dashboard_msgs
{

namespace srv
{

namespace builder
{

class Init_Load_Request_filename
{
public:
  Init_Load_Request_filename()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::aubo_dashboard_msgs::srv::Load_Request filename(::aubo_dashboard_msgs::srv::Load_Request::_filename_type arg)
  {
    msg_.filename = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aubo_dashboard_msgs::srv::Load_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::aubo_dashboard_msgs::srv::Load_Request>()
{
  return aubo_dashboard_msgs::srv::builder::Init_Load_Request_filename();
}

}  // namespace aubo_dashboard_msgs


namespace aubo_dashboard_msgs
{

namespace srv
{

namespace builder
{

class Init_Load_Response_success
{
public:
  explicit Init_Load_Response_success(::aubo_dashboard_msgs::srv::Load_Response & msg)
  : msg_(msg)
  {}
  ::aubo_dashboard_msgs::srv::Load_Response success(::aubo_dashboard_msgs::srv::Load_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aubo_dashboard_msgs::srv::Load_Response msg_;
};

class Init_Load_Response_answer
{
public:
  Init_Load_Response_answer()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Load_Response_success answer(::aubo_dashboard_msgs::srv::Load_Response::_answer_type arg)
  {
    msg_.answer = std::move(arg);
    return Init_Load_Response_success(msg_);
  }

private:
  ::aubo_dashboard_msgs::srv::Load_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::aubo_dashboard_msgs::srv::Load_Response>()
{
  return aubo_dashboard_msgs::srv::builder::Init_Load_Response_answer();
}

}  // namespace aubo_dashboard_msgs

#endif  // AUBO_DASHBOARD_MSGS__SRV__DETAIL__LOAD__BUILDER_HPP_
