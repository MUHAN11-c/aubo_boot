#include "aubo_ros2_trajectory_action.h"
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <string>

using namespace aubo_ros2_trajectory_action;

JointTrajectoryAction::JointTrajectoryAction(std::string controller_name):Node("aubo_ros2_trajectory_action")
{
  has_active_goal_ = false;
  trajectory_state_recvd_ = false;
  goal_accept_time_sec_ = 0.0;

  using namespace std::placeholders;
  this->action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
    this, controller_name,
    std::bind(&JointTrajectoryAction::handleGoal, this, _1, _2),
    std::bind(&JointTrajectoryAction::handleCancel, this, _1),
    std::bind(&JointTrajectoryAction::handleAccept, this, _1));

  this->declare_parameter<std::vector<std::string>>("joint_name.controller_joint_names", joint_names);

  this->get_parameter("joint_name.controller_joint_names", joint_names);

  for (int i = 0; i < 6; i++)
  {
    RCLCPP_INFO(this->get_logger(), "joint name %d %s", i, joint_names[i].c_str());
  }

  // 仅做关节重映射，轨迹插值由 simulator 端 5 次样条完成
  // 发布完整轨迹到 joint_path_command，由 ROS 1 端的 aubo_joint_trajectory_action 或 aubo_robot_simulator 接收
  // ros1_bridge 会自动桥接 trajectory_msgs::JointTrajectory 消息
  trajectory_command_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_path_command", 100);
  fjt_feedback_sub_ = this->create_subscription<control_msgs::action::FollowJointTrajectory_Feedback>(
    "aubo/feedback_states", 10, std::bind(&JointTrajectoryAction::fjtFeedbackCallback, this, _1));
  moveit_execution_sub_ = this->create_subscription<std_msgs::msg::String>(
    "trajectory_execution_event", 10, std::bind(&JointTrajectoryAction::moveitExecutionCallback, this, _1));

  watch_dog_timer_ = create_wall_timer(std::chrono::seconds(2), std::bind(&JointTrajectoryAction::watchDogTimer, this));
}

void JointTrajectoryAction::watchDogTimer()
{
  if (!has_active_goal_)
    return;
  if (trajectory_state_recvd_)
    return;
  // 仅在“接受目标后已满 2 秒仍无反馈”时才 abort，避免接受后几毫秒内定时器触发误杀
  double now_sec = this->now().seconds();
  if (now_sec - goal_accept_time_sec_ < 2.0)
    return;
  RCLCPP_INFO(this->get_logger(), "abort active goal because driver no feedback (2s)");
  abortActiveGoal("watchdog_no_feedback_2s");
}

void JointTrajectoryAction::fjtFeedbackCallback(const control_msgs::action::FollowJointTrajectory_Feedback::ConstSharedPtr msg)
{
  if (!has_active_goal_ || current_trajectory_.points.empty())
    return;

  trajectory_state_recvd_ = true;
  last_feedback_valid_ = true;
  last_feedback_positions_.clear();
  for (size_t i = 0; i < msg->actual.positions.size() && i < 6u; ++i)
    last_feedback_positions_.push_back(msg->actual.positions[i]);
  active_goal_->publish_feedback(std::const_pointer_cast<control_msgs::action::FollowJointTrajectory_Feedback>(msg));
  bool reached = checkReachTarget(msg, current_trajectory_);
  if (reached)
  {
    RCLCPP_INFO(this->get_logger(), "reach target");
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    result->error_code = result->SUCCESSFUL;
    result->error_string = "successful";
    active_goal_->succeed(result);
    has_active_goal_ = false;
    trajectory_state_recvd_ = false;
  }
}

void JointTrajectoryAction::moveitExecutionCallback(const std_msgs::msg::String::ConstSharedPtr msg)
{
  if (msg->data == "stop")
  {
    RCLCPP_INFO(this->get_logger(), "moveit execution stopped (trajectory_execution_event=stop)");
    if (has_active_goal_)
      abortActiveGoal("trajectory_execution_event_stop");
  }
}

rclcpp_action::GoalResponse JointTrajectoryAction::handleGoal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received new goal");

  if (goal->trajectory.points.empty())
  {
    RCLCPP_INFO(this->get_logger(), "empty trajectory");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if(!isSimilar(joint_names, goal->trajectory.joint_names))
  {
    RCLCPP_INFO(this->get_logger(), "invalid joints");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (has_active_goal_)
  {
    RCLCPP_INFO(this->get_logger(), "Received new goal, canceling current goal");
    abortActiveGoal("new_goal_received");
  }

  (void)uuid;

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse JointTrajectoryAction::handleCancel(const std::shared_ptr<GoalHandleFjt> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "cancel goal (client/MoveIt requested cancel)");
  (void)goal_handle;
  abortActiveGoal("client_cancel");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void JointTrajectoryAction::handleAccept(const std::shared_ptr<GoalHandleFjt> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "accepted new goal");
  active_goal_ = std::move(goal_handle);
  current_trajectory_ = active_goal_->get_goal()->trajectory;
  has_active_goal_ = true;
  trajectory_state_recvd_ = false;  // 新目标重置，避免沿用上次的反馈标志
  last_feedback_valid_ = false;     // 诊断：重置，abort 时可知是否在取消前收到过 feedback
  goal_accept_time_sec_ = this->now().seconds();  // 记录接受时刻，看门狗满 2 秒后再判无反馈

  publishTrajectory();
  return;
}

void JointTrajectoryAction::abortActiveGoal(const char* reason)
{
  const char* r = reason ? reason : "unknown";
  RCLCPP_INFO(this->get_logger(), "JointTrajectoryAction: abort, reason=%s", r);

  auto result = std::make_shared<FollowJointTrajectory::Result>();
  result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
  result->error_string = "aborted";
  active_goal_->abort(result);
  has_active_goal_ = false;
  trajectory_state_recvd_ = false;
}

void JointTrajectoryAction::publishTrajectory()
{
  RCLCPP_INFO(this->get_logger(), "Publishing complete trajectory to ROS 1 for interpolation");

  // 重映射关节顺序（如果需要）
  trajectory_msgs::msg::JointTrajectory remap_traj = remapTrajectoryByJointName(current_trajectory_);
  current_trajectory_ = remap_traj;

  // 直接发布到 joint_path_command，由 simulator 做 5 次样条插值
  trajectory_command_pub_->publish(current_trajectory_);
  RCLCPP_INFO(this->get_logger(), "Published complete trajectory with %zu points to joint_path_command", 
              current_trajectory_.points.size());
}

bool JointTrajectoryAction::isSimilar(std::vector<std::string> lhs, std::vector<std::string> rhs)
{
  if (lhs.size() != rhs.size())
    return false;

  std::sort(lhs.begin(), lhs.end());
  std::sort(rhs.begin(), rhs.end());

  return std::equal(lhs.begin(), lhs.end(), rhs.begin());
}

double JointTrajectoryAction::toSec(const builtin_interfaces::msg::Duration &duration)
{
  return (double)duration.sec + 1e-9*(double)duration.nanosec;
}

builtin_interfaces::msg::Duration JointTrajectoryAction::toDuration(double time_in_seconds)
{
  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<int32_t>(time_in_seconds);
  duration.nanosec = static_cast<uint32_t>((time_in_seconds - duration.sec) * 1e9);
  return duration;
}

bool JointTrajectoryAction::checkReachTarget(const control_msgs::action::FollowJointTrajectory_Feedback::ConstSharedPtr feedback, const trajectory_msgs::msg::JointTrajectory &traj)
{
  if (traj.points.empty() || feedback->actual.positions.size() < 6u ||
      traj.points.back().positions.size() < 6u || feedback->joint_names.size() < 6u)
    return false;
  const size_t last_point = traj.points.size() - 1;
  const double kReachedToleranceRad = 0.01;
  // 按关节名匹配再比较，避免 driver 与轨迹关节顺序不一致导致永远判不到位
  for (size_t i = 0; i < traj.joint_names.size() && i < 6u; i++)
  {
    size_t fb_idx = (size_t)-1;
    for (size_t k = 0; k < feedback->joint_names.size(); k++)
      if (feedback->joint_names[k] == traj.joint_names[i]) { fb_idx = k; break; }
    if (fb_idx == (size_t)-1 || fb_idx >= feedback->actual.positions.size())
      return false;
    if (std::abs(feedback->actual.positions[fb_idx] - traj.points[last_point].positions[i]) > kReachedToleranceRad)
      return false;
  }
  return true;
}

trajectory_msgs::msg::JointTrajectory JointTrajectoryAction::remapTrajectoryByJointName(trajectory_msgs::msg::JointTrajectory &trajectory)
{
  std::vector<int> mapping;

  mapping.resize(6, 6);
  for (uint16_t i = 0; i < trajectory.joint_names.size(); i++)
  {
    for (int j = 0; j < 6; j++)
    {
      if (trajectory.joint_names[i] == joint_names[j])
        mapping[j] = i;
    }
  }

  for(int i = 0; i < 6; i++)
    RCLCPP_INFO(this->get_logger(), "order %d", mapping[i]);

  trajectory_msgs::msg::JointTrajectory new_traj;
  for (int i = 0; i < 6; i++)
    new_traj.joint_names.push_back(joint_names[i]);

  for (uint16_t i = 0; i < trajectory.points.size(); i++)
  {
    trajectory_msgs::msg::JointTrajectoryPoint new_point;
    for(int j = 0; j < 6; j++)
    {
      new_point.positions.push_back(trajectory.points[i].positions[mapping[j]]);
      new_point.velocities.push_back(trajectory.points[i].velocities[mapping[j]]);
      new_point.accelerations.push_back(trajectory.points[i].accelerations[mapping[j]]);
    }
    new_point.time_from_start = trajectory.points[i].time_from_start;
    new_traj.points.push_back(new_point);
  }

  return new_traj;
}
