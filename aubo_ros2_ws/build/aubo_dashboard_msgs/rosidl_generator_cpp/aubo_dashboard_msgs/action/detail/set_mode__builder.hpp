// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from aubo_dashboard_msgs:action/SetMode.idl
// generated code does not contain a copyright notice

#ifndef AUBO_DASHBOARD_MSGS__ACTION__DETAIL__SET_MODE__BUILDER_HPP_
#define AUBO_DASHBOARD_MSGS__ACTION__DETAIL__SET_MODE__BUILDER_HPP_

#include "aubo_dashboard_msgs/action/detail/set_mode__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace aubo_dashboard_msgs
{

namespace action
{

namespace builder
{

class Init_SetMode_Goal_play_program
{
public:
  explicit Init_SetMode_Goal_play_program(::aubo_dashboard_msgs::action::SetMode_Goal & msg)
  : msg_(msg)
  {}
  ::aubo_dashboard_msgs::action::SetMode_Goal play_program(::aubo_dashboard_msgs::action::SetMode_Goal::_play_program_type arg)
  {
    msg_.play_program = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aubo_dashboard_msgs::action::SetMode_Goal msg_;
};

class Init_SetMode_Goal_stop_program
{
public:
  explicit Init_SetMode_Goal_stop_program(::aubo_dashboard_msgs::action::SetMode_Goal & msg)
  : msg_(msg)
  {}
  Init_SetMode_Goal_play_program stop_program(::aubo_dashboard_msgs::action::SetMode_Goal::_stop_program_type arg)
  {
    msg_.stop_program = std::move(arg);
    return Init_SetMode_Goal_play_program(msg_);
  }

private:
  ::aubo_dashboard_msgs::action::SetMode_Goal msg_;
};

class Init_SetMode_Goal_target_robot_mode
{
public:
  Init_SetMode_Goal_target_robot_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMode_Goal_stop_program target_robot_mode(::aubo_dashboard_msgs::action::SetMode_Goal::_target_robot_mode_type arg)
  {
    msg_.target_robot_mode = std::move(arg);
    return Init_SetMode_Goal_stop_program(msg_);
  }

private:
  ::aubo_dashboard_msgs::action::SetMode_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::aubo_dashboard_msgs::action::SetMode_Goal>()
{
  return aubo_dashboard_msgs::action::builder::Init_SetMode_Goal_target_robot_mode();
}

}  // namespace aubo_dashboard_msgs


namespace aubo_dashboard_msgs
{

namespace action
{

namespace builder
{

class Init_SetMode_Result_message
{
public:
  explicit Init_SetMode_Result_message(::aubo_dashboard_msgs::action::SetMode_Result & msg)
  : msg_(msg)
  {}
  ::aubo_dashboard_msgs::action::SetMode_Result message(::aubo_dashboard_msgs::action::SetMode_Result::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aubo_dashboard_msgs::action::SetMode_Result msg_;
};

class Init_SetMode_Result_success
{
public:
  Init_SetMode_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMode_Result_message success(::aubo_dashboard_msgs::action::SetMode_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetMode_Result_message(msg_);
  }

private:
  ::aubo_dashboard_msgs::action::SetMode_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::aubo_dashboard_msgs::action::SetMode_Result>()
{
  return aubo_dashboard_msgs::action::builder::Init_SetMode_Result_success();
}

}  // namespace aubo_dashboard_msgs


namespace aubo_dashboard_msgs
{

namespace action
{

namespace builder
{

class Init_SetMode_Feedback_current_safety_mode
{
public:
  explicit Init_SetMode_Feedback_current_safety_mode(::aubo_dashboard_msgs::action::SetMode_Feedback & msg)
  : msg_(msg)
  {}
  ::aubo_dashboard_msgs::action::SetMode_Feedback current_safety_mode(::aubo_dashboard_msgs::action::SetMode_Feedback::_current_safety_mode_type arg)
  {
    msg_.current_safety_mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aubo_dashboard_msgs::action::SetMode_Feedback msg_;
};

class Init_SetMode_Feedback_current_robot_mode
{
public:
  Init_SetMode_Feedback_current_robot_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMode_Feedback_current_safety_mode current_robot_mode(::aubo_dashboard_msgs::action::SetMode_Feedback::_current_robot_mode_type arg)
  {
    msg_.current_robot_mode = std::move(arg);
    return Init_SetMode_Feedback_current_safety_mode(msg_);
  }

private:
  ::aubo_dashboard_msgs::action::SetMode_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::aubo_dashboard_msgs::action::SetMode_Feedback>()
{
  return aubo_dashboard_msgs::action::builder::Init_SetMode_Feedback_current_robot_mode();
}

}  // namespace aubo_dashboard_msgs


namespace aubo_dashboard_msgs
{

namespace action
{

namespace builder
{

class Init_SetMode_SendGoal_Request_goal
{
public:
  explicit Init_SetMode_SendGoal_Request_goal(::aubo_dashboard_msgs::action::SetMode_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::aubo_dashboard_msgs::action::SetMode_SendGoal_Request goal(::aubo_dashboard_msgs::action::SetMode_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aubo_dashboard_msgs::action::SetMode_SendGoal_Request msg_;
};

class Init_SetMode_SendGoal_Request_goal_id
{
public:
  Init_SetMode_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMode_SendGoal_Request_goal goal_id(::aubo_dashboard_msgs::action::SetMode_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_SetMode_SendGoal_Request_goal(msg_);
  }

private:
  ::aubo_dashboard_msgs::action::SetMode_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::aubo_dashboard_msgs::action::SetMode_SendGoal_Request>()
{
  return aubo_dashboard_msgs::action::builder::Init_SetMode_SendGoal_Request_goal_id();
}

}  // namespace aubo_dashboard_msgs


namespace aubo_dashboard_msgs
{

namespace action
{

namespace builder
{

class Init_SetMode_SendGoal_Response_stamp
{
public:
  explicit Init_SetMode_SendGoal_Response_stamp(::aubo_dashboard_msgs::action::SetMode_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::aubo_dashboard_msgs::action::SetMode_SendGoal_Response stamp(::aubo_dashboard_msgs::action::SetMode_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aubo_dashboard_msgs::action::SetMode_SendGoal_Response msg_;
};

class Init_SetMode_SendGoal_Response_accepted
{
public:
  Init_SetMode_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMode_SendGoal_Response_stamp accepted(::aubo_dashboard_msgs::action::SetMode_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_SetMode_SendGoal_Response_stamp(msg_);
  }

private:
  ::aubo_dashboard_msgs::action::SetMode_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::aubo_dashboard_msgs::action::SetMode_SendGoal_Response>()
{
  return aubo_dashboard_msgs::action::builder::Init_SetMode_SendGoal_Response_accepted();
}

}  // namespace aubo_dashboard_msgs


namespace aubo_dashboard_msgs
{

namespace action
{

namespace builder
{

class Init_SetMode_GetResult_Request_goal_id
{
public:
  Init_SetMode_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::aubo_dashboard_msgs::action::SetMode_GetResult_Request goal_id(::aubo_dashboard_msgs::action::SetMode_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aubo_dashboard_msgs::action::SetMode_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::aubo_dashboard_msgs::action::SetMode_GetResult_Request>()
{
  return aubo_dashboard_msgs::action::builder::Init_SetMode_GetResult_Request_goal_id();
}

}  // namespace aubo_dashboard_msgs


namespace aubo_dashboard_msgs
{

namespace action
{

namespace builder
{

class Init_SetMode_GetResult_Response_result
{
public:
  explicit Init_SetMode_GetResult_Response_result(::aubo_dashboard_msgs::action::SetMode_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::aubo_dashboard_msgs::action::SetMode_GetResult_Response result(::aubo_dashboard_msgs::action::SetMode_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aubo_dashboard_msgs::action::SetMode_GetResult_Response msg_;
};

class Init_SetMode_GetResult_Response_status
{
public:
  Init_SetMode_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMode_GetResult_Response_result status(::aubo_dashboard_msgs::action::SetMode_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_SetMode_GetResult_Response_result(msg_);
  }

private:
  ::aubo_dashboard_msgs::action::SetMode_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::aubo_dashboard_msgs::action::SetMode_GetResult_Response>()
{
  return aubo_dashboard_msgs::action::builder::Init_SetMode_GetResult_Response_status();
}

}  // namespace aubo_dashboard_msgs


namespace aubo_dashboard_msgs
{

namespace action
{

namespace builder
{

class Init_SetMode_FeedbackMessage_feedback
{
public:
  explicit Init_SetMode_FeedbackMessage_feedback(::aubo_dashboard_msgs::action::SetMode_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::aubo_dashboard_msgs::action::SetMode_FeedbackMessage feedback(::aubo_dashboard_msgs::action::SetMode_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aubo_dashboard_msgs::action::SetMode_FeedbackMessage msg_;
};

class Init_SetMode_FeedbackMessage_goal_id
{
public:
  Init_SetMode_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMode_FeedbackMessage_feedback goal_id(::aubo_dashboard_msgs::action::SetMode_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_SetMode_FeedbackMessage_feedback(msg_);
  }

private:
  ::aubo_dashboard_msgs::action::SetMode_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::aubo_dashboard_msgs::action::SetMode_FeedbackMessage>()
{
  return aubo_dashboard_msgs::action::builder::Init_SetMode_FeedbackMessage_goal_id();
}

}  // namespace aubo_dashboard_msgs

#endif  // AUBO_DASHBOARD_MSGS__ACTION__DETAIL__SET_MODE__BUILDER_HPP_
