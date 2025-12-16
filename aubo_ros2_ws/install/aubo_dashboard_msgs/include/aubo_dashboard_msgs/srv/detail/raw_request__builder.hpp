// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from aubo_dashboard_msgs:srv/RawRequest.idl
// generated code does not contain a copyright notice

#ifndef AUBO_DASHBOARD_MSGS__SRV__DETAIL__RAW_REQUEST__BUILDER_HPP_
#define AUBO_DASHBOARD_MSGS__SRV__DETAIL__RAW_REQUEST__BUILDER_HPP_

#include "aubo_dashboard_msgs/srv/detail/raw_request__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace aubo_dashboard_msgs
{

namespace srv
{

namespace builder
{

class Init_RawRequest_Request_query
{
public:
  Init_RawRequest_Request_query()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::aubo_dashboard_msgs::srv::RawRequest_Request query(::aubo_dashboard_msgs::srv::RawRequest_Request::_query_type arg)
  {
    msg_.query = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aubo_dashboard_msgs::srv::RawRequest_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::aubo_dashboard_msgs::srv::RawRequest_Request>()
{
  return aubo_dashboard_msgs::srv::builder::Init_RawRequest_Request_query();
}

}  // namespace aubo_dashboard_msgs


namespace aubo_dashboard_msgs
{

namespace srv
{

namespace builder
{

class Init_RawRequest_Response_answer
{
public:
  Init_RawRequest_Response_answer()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::aubo_dashboard_msgs::srv::RawRequest_Response answer(::aubo_dashboard_msgs::srv::RawRequest_Response::_answer_type arg)
  {
    msg_.answer = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aubo_dashboard_msgs::srv::RawRequest_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::aubo_dashboard_msgs::srv::RawRequest_Response>()
{
  return aubo_dashboard_msgs::srv::builder::Init_RawRequest_Response_answer();
}

}  // namespace aubo_dashboard_msgs

#endif  // AUBO_DASHBOARD_MSGS__SRV__DETAIL__RAW_REQUEST__BUILDER_HPP_
