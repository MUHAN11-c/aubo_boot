// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from aubo_msgs:srv/SetPayload.idl
// generated code does not contain a copyright notice

#ifndef AUBO_MSGS__SRV__DETAIL__SET_PAYLOAD__BUILDER_HPP_
#define AUBO_MSGS__SRV__DETAIL__SET_PAYLOAD__BUILDER_HPP_

#include "aubo_msgs/srv/detail/set_payload__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace aubo_msgs
{

namespace srv
{

namespace builder
{

class Init_SetPayload_Request_center_of_gravity
{
public:
  explicit Init_SetPayload_Request_center_of_gravity(::aubo_msgs::srv::SetPayload_Request & msg)
  : msg_(msg)
  {}
  ::aubo_msgs::srv::SetPayload_Request center_of_gravity(::aubo_msgs::srv::SetPayload_Request::_center_of_gravity_type arg)
  {
    msg_.center_of_gravity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aubo_msgs::srv::SetPayload_Request msg_;
};

class Init_SetPayload_Request_mass
{
public:
  Init_SetPayload_Request_mass()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetPayload_Request_center_of_gravity mass(::aubo_msgs::srv::SetPayload_Request::_mass_type arg)
  {
    msg_.mass = std::move(arg);
    return Init_SetPayload_Request_center_of_gravity(msg_);
  }

private:
  ::aubo_msgs::srv::SetPayload_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::aubo_msgs::srv::SetPayload_Request>()
{
  return aubo_msgs::srv::builder::Init_SetPayload_Request_mass();
}

}  // namespace aubo_msgs


namespace aubo_msgs
{

namespace srv
{

namespace builder
{

class Init_SetPayload_Response_success
{
public:
  Init_SetPayload_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::aubo_msgs::srv::SetPayload_Response success(::aubo_msgs::srv::SetPayload_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aubo_msgs::srv::SetPayload_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::aubo_msgs::srv::SetPayload_Response>()
{
  return aubo_msgs::srv::builder::Init_SetPayload_Response_success();
}

}  // namespace aubo_msgs

#endif  // AUBO_MSGS__SRV__DETAIL__SET_PAYLOAD__BUILDER_HPP_
