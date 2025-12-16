// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from aubo_msgs:srv/SetIO.idl
// generated code does not contain a copyright notice

#ifndef AUBO_MSGS__SRV__DETAIL__SET_IO__BUILDER_HPP_
#define AUBO_MSGS__SRV__DETAIL__SET_IO__BUILDER_HPP_

#include "aubo_msgs/srv/detail/set_io__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace aubo_msgs
{

namespace srv
{

namespace builder
{

class Init_SetIO_Request_state
{
public:
  explicit Init_SetIO_Request_state(::aubo_msgs::srv::SetIO_Request & msg)
  : msg_(msg)
  {}
  ::aubo_msgs::srv::SetIO_Request state(::aubo_msgs::srv::SetIO_Request::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aubo_msgs::srv::SetIO_Request msg_;
};

class Init_SetIO_Request_pin
{
public:
  explicit Init_SetIO_Request_pin(::aubo_msgs::srv::SetIO_Request & msg)
  : msg_(msg)
  {}
  Init_SetIO_Request_state pin(::aubo_msgs::srv::SetIO_Request::_pin_type arg)
  {
    msg_.pin = std::move(arg);
    return Init_SetIO_Request_state(msg_);
  }

private:
  ::aubo_msgs::srv::SetIO_Request msg_;
};

class Init_SetIO_Request_fun
{
public:
  Init_SetIO_Request_fun()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetIO_Request_pin fun(::aubo_msgs::srv::SetIO_Request::_fun_type arg)
  {
    msg_.fun = std::move(arg);
    return Init_SetIO_Request_pin(msg_);
  }

private:
  ::aubo_msgs::srv::SetIO_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::aubo_msgs::srv::SetIO_Request>()
{
  return aubo_msgs::srv::builder::Init_SetIO_Request_fun();
}

}  // namespace aubo_msgs


namespace aubo_msgs
{

namespace srv
{

namespace builder
{

class Init_SetIO_Response_success
{
public:
  Init_SetIO_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::aubo_msgs::srv::SetIO_Response success(::aubo_msgs::srv::SetIO_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aubo_msgs::srv::SetIO_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::aubo_msgs::srv::SetIO_Response>()
{
  return aubo_msgs::srv::builder::Init_SetIO_Response_success();
}

}  // namespace aubo_msgs

#endif  // AUBO_MSGS__SRV__DETAIL__SET_IO__BUILDER_HPP_
