// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from aubo_dashboard_msgs:msg/SafetyMode.idl
// generated code does not contain a copyright notice

#ifndef AUBO_DASHBOARD_MSGS__MSG__DETAIL__SAFETY_MODE__BUILDER_HPP_
#define AUBO_DASHBOARD_MSGS__MSG__DETAIL__SAFETY_MODE__BUILDER_HPP_

#include "aubo_dashboard_msgs/msg/detail/safety_mode__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace aubo_dashboard_msgs
{

namespace msg
{

namespace builder
{

class Init_SafetyMode_mode
{
public:
  Init_SafetyMode_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::aubo_dashboard_msgs::msg::SafetyMode mode(::aubo_dashboard_msgs::msg::SafetyMode::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aubo_dashboard_msgs::msg::SafetyMode msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::aubo_dashboard_msgs::msg::SafetyMode>()
{
  return aubo_dashboard_msgs::msg::builder::Init_SafetyMode_mode();
}

}  // namespace aubo_dashboard_msgs

#endif  // AUBO_DASHBOARD_MSGS__MSG__DETAIL__SAFETY_MODE__BUILDER_HPP_
