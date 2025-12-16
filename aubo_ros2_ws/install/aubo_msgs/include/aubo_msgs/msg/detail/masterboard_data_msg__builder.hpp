// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from aubo_msgs:msg/MasterboardDataMsg.idl
// generated code does not contain a copyright notice

#ifndef AUBO_MSGS__MSG__DETAIL__MASTERBOARD_DATA_MSG__BUILDER_HPP_
#define AUBO_MSGS__MSG__DETAIL__MASTERBOARD_DATA_MSG__BUILDER_HPP_

#include "aubo_msgs/msg/detail/masterboard_data_msg__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace aubo_msgs
{

namespace msg
{

namespace builder
{

class Init_MasterboardDataMsg_master_onoff_state
{
public:
  explicit Init_MasterboardDataMsg_master_onoff_state(::aubo_msgs::msg::MasterboardDataMsg & msg)
  : msg_(msg)
  {}
  ::aubo_msgs::msg::MasterboardDataMsg master_onoff_state(::aubo_msgs::msg::MasterboardDataMsg::_master_onoff_state_type arg)
  {
    msg_.master_onoff_state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aubo_msgs::msg::MasterboardDataMsg msg_;
};

class Init_MasterboardDataMsg_master_safety_state
{
public:
  explicit Init_MasterboardDataMsg_master_safety_state(::aubo_msgs::msg::MasterboardDataMsg & msg)
  : msg_(msg)
  {}
  Init_MasterboardDataMsg_master_onoff_state master_safety_state(::aubo_msgs::msg::MasterboardDataMsg::_master_safety_state_type arg)
  {
    msg_.master_safety_state = std::move(arg);
    return Init_MasterboardDataMsg_master_onoff_state(msg_);
  }

private:
  ::aubo_msgs::msg::MasterboardDataMsg msg_;
};

class Init_MasterboardDataMsg_master_io_current
{
public:
  explicit Init_MasterboardDataMsg_master_io_current(::aubo_msgs::msg::MasterboardDataMsg & msg)
  : msg_(msg)
  {}
  Init_MasterboardDataMsg_master_safety_state master_io_current(::aubo_msgs::msg::MasterboardDataMsg::_master_io_current_type arg)
  {
    msg_.master_io_current = std::move(arg);
    return Init_MasterboardDataMsg_master_safety_state(msg_);
  }

private:
  ::aubo_msgs::msg::MasterboardDataMsg msg_;
};

class Init_MasterboardDataMsg_robot_current
{
public:
  explicit Init_MasterboardDataMsg_robot_current(::aubo_msgs::msg::MasterboardDataMsg & msg)
  : msg_(msg)
  {}
  Init_MasterboardDataMsg_master_io_current robot_current(::aubo_msgs::msg::MasterboardDataMsg::_robot_current_type arg)
  {
    msg_.robot_current = std::move(arg);
    return Init_MasterboardDataMsg_master_io_current(msg_);
  }

private:
  ::aubo_msgs::msg::MasterboardDataMsg msg_;
};

class Init_MasterboardDataMsg_robot_voltage_48v
{
public:
  explicit Init_MasterboardDataMsg_robot_voltage_48v(::aubo_msgs::msg::MasterboardDataMsg & msg)
  : msg_(msg)
  {}
  Init_MasterboardDataMsg_robot_current robot_voltage_48v(::aubo_msgs::msg::MasterboardDataMsg::_robot_voltage_48v_type arg)
  {
    msg_.robot_voltage_48v = std::move(arg);
    return Init_MasterboardDataMsg_robot_current(msg_);
  }

private:
  ::aubo_msgs::msg::MasterboardDataMsg msg_;
};

class Init_MasterboardDataMsg_masterboard_temperature
{
public:
  explicit Init_MasterboardDataMsg_masterboard_temperature(::aubo_msgs::msg::MasterboardDataMsg & msg)
  : msg_(msg)
  {}
  Init_MasterboardDataMsg_robot_voltage_48v masterboard_temperature(::aubo_msgs::msg::MasterboardDataMsg::_masterboard_temperature_type arg)
  {
    msg_.masterboard_temperature = std::move(arg);
    return Init_MasterboardDataMsg_robot_voltage_48v(msg_);
  }

private:
  ::aubo_msgs::msg::MasterboardDataMsg msg_;
};

class Init_MasterboardDataMsg_analog_output1
{
public:
  explicit Init_MasterboardDataMsg_analog_output1(::aubo_msgs::msg::MasterboardDataMsg & msg)
  : msg_(msg)
  {}
  Init_MasterboardDataMsg_masterboard_temperature analog_output1(::aubo_msgs::msg::MasterboardDataMsg::_analog_output1_type arg)
  {
    msg_.analog_output1 = std::move(arg);
    return Init_MasterboardDataMsg_masterboard_temperature(msg_);
  }

private:
  ::aubo_msgs::msg::MasterboardDataMsg msg_;
};

class Init_MasterboardDataMsg_analog_output0
{
public:
  explicit Init_MasterboardDataMsg_analog_output0(::aubo_msgs::msg::MasterboardDataMsg & msg)
  : msg_(msg)
  {}
  Init_MasterboardDataMsg_analog_output1 analog_output0(::aubo_msgs::msg::MasterboardDataMsg::_analog_output0_type arg)
  {
    msg_.analog_output0 = std::move(arg);
    return Init_MasterboardDataMsg_analog_output1(msg_);
  }

private:
  ::aubo_msgs::msg::MasterboardDataMsg msg_;
};

class Init_MasterboardDataMsg_analog_output_domain1
{
public:
  explicit Init_MasterboardDataMsg_analog_output_domain1(::aubo_msgs::msg::MasterboardDataMsg & msg)
  : msg_(msg)
  {}
  Init_MasterboardDataMsg_analog_output0 analog_output_domain1(::aubo_msgs::msg::MasterboardDataMsg::_analog_output_domain1_type arg)
  {
    msg_.analog_output_domain1 = std::move(arg);
    return Init_MasterboardDataMsg_analog_output0(msg_);
  }

private:
  ::aubo_msgs::msg::MasterboardDataMsg msg_;
};

class Init_MasterboardDataMsg_analog_output_domain0
{
public:
  explicit Init_MasterboardDataMsg_analog_output_domain0(::aubo_msgs::msg::MasterboardDataMsg & msg)
  : msg_(msg)
  {}
  Init_MasterboardDataMsg_analog_output_domain1 analog_output_domain0(::aubo_msgs::msg::MasterboardDataMsg::_analog_output_domain0_type arg)
  {
    msg_.analog_output_domain0 = std::move(arg);
    return Init_MasterboardDataMsg_analog_output_domain1(msg_);
  }

private:
  ::aubo_msgs::msg::MasterboardDataMsg msg_;
};

class Init_MasterboardDataMsg_analog_input1
{
public:
  explicit Init_MasterboardDataMsg_analog_input1(::aubo_msgs::msg::MasterboardDataMsg & msg)
  : msg_(msg)
  {}
  Init_MasterboardDataMsg_analog_output_domain0 analog_input1(::aubo_msgs::msg::MasterboardDataMsg::_analog_input1_type arg)
  {
    msg_.analog_input1 = std::move(arg);
    return Init_MasterboardDataMsg_analog_output_domain0(msg_);
  }

private:
  ::aubo_msgs::msg::MasterboardDataMsg msg_;
};

class Init_MasterboardDataMsg_analog_input0
{
public:
  explicit Init_MasterboardDataMsg_analog_input0(::aubo_msgs::msg::MasterboardDataMsg & msg)
  : msg_(msg)
  {}
  Init_MasterboardDataMsg_analog_input1 analog_input0(::aubo_msgs::msg::MasterboardDataMsg::_analog_input0_type arg)
  {
    msg_.analog_input0 = std::move(arg);
    return Init_MasterboardDataMsg_analog_input1(msg_);
  }

private:
  ::aubo_msgs::msg::MasterboardDataMsg msg_;
};

class Init_MasterboardDataMsg_analog_input_range1
{
public:
  explicit Init_MasterboardDataMsg_analog_input_range1(::aubo_msgs::msg::MasterboardDataMsg & msg)
  : msg_(msg)
  {}
  Init_MasterboardDataMsg_analog_input0 analog_input_range1(::aubo_msgs::msg::MasterboardDataMsg::_analog_input_range1_type arg)
  {
    msg_.analog_input_range1 = std::move(arg);
    return Init_MasterboardDataMsg_analog_input0(msg_);
  }

private:
  ::aubo_msgs::msg::MasterboardDataMsg msg_;
};

class Init_MasterboardDataMsg_analog_input_range0
{
public:
  explicit Init_MasterboardDataMsg_analog_input_range0(::aubo_msgs::msg::MasterboardDataMsg & msg)
  : msg_(msg)
  {}
  Init_MasterboardDataMsg_analog_input_range1 analog_input_range0(::aubo_msgs::msg::MasterboardDataMsg::_analog_input_range0_type arg)
  {
    msg_.analog_input_range0 = std::move(arg);
    return Init_MasterboardDataMsg_analog_input_range1(msg_);
  }

private:
  ::aubo_msgs::msg::MasterboardDataMsg msg_;
};

class Init_MasterboardDataMsg_digital_output_bits
{
public:
  explicit Init_MasterboardDataMsg_digital_output_bits(::aubo_msgs::msg::MasterboardDataMsg & msg)
  : msg_(msg)
  {}
  Init_MasterboardDataMsg_analog_input_range0 digital_output_bits(::aubo_msgs::msg::MasterboardDataMsg::_digital_output_bits_type arg)
  {
    msg_.digital_output_bits = std::move(arg);
    return Init_MasterboardDataMsg_analog_input_range0(msg_);
  }

private:
  ::aubo_msgs::msg::MasterboardDataMsg msg_;
};

class Init_MasterboardDataMsg_digital_input_bits
{
public:
  Init_MasterboardDataMsg_digital_input_bits()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MasterboardDataMsg_digital_output_bits digital_input_bits(::aubo_msgs::msg::MasterboardDataMsg::_digital_input_bits_type arg)
  {
    msg_.digital_input_bits = std::move(arg);
    return Init_MasterboardDataMsg_digital_output_bits(msg_);
  }

private:
  ::aubo_msgs::msg::MasterboardDataMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::aubo_msgs::msg::MasterboardDataMsg>()
{
  return aubo_msgs::msg::builder::Init_MasterboardDataMsg_digital_input_bits();
}

}  // namespace aubo_msgs

#endif  // AUBO_MSGS__MSG__DETAIL__MASTERBOARD_DATA_MSG__BUILDER_HPP_
