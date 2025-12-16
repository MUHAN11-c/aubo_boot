// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from aubo_dashboard_msgs:msg/ProgramState.idl
// generated code does not contain a copyright notice

#ifndef AUBO_DASHBOARD_MSGS__MSG__DETAIL__PROGRAM_STATE__BUILDER_HPP_
#define AUBO_DASHBOARD_MSGS__MSG__DETAIL__PROGRAM_STATE__BUILDER_HPP_

#include "aubo_dashboard_msgs/msg/detail/program_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace aubo_dashboard_msgs
{

namespace msg
{

namespace builder
{

class Init_ProgramState_state
{
public:
  Init_ProgramState_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::aubo_dashboard_msgs::msg::ProgramState state(::aubo_dashboard_msgs::msg::ProgramState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aubo_dashboard_msgs::msg::ProgramState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::aubo_dashboard_msgs::msg::ProgramState>()
{
  return aubo_dashboard_msgs::msg::builder::Init_ProgramState_state();
}

}  // namespace aubo_dashboard_msgs

#endif  // AUBO_DASHBOARD_MSGS__MSG__DETAIL__PROGRAM_STATE__BUILDER_HPP_
