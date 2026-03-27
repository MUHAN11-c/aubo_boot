#include "aubo_ros2_trajectory_action.h"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <string>

using namespace aubo_ros2_trajectory_action;

namespace {
// H7 曾观测末点残余 ~0.0158–0.0199 rad，原 0.01 rad 导致长时间达不到；对齐 JTC 常用 goal 带
constexpr double kFjGoalToleranceRad = 0.02;
// 反馈 ~50Hz（与 Noetic feedback_states 一致）时 5 帧 ≈ 100ms 稳定窗口，抑制单帧噪声误 succeed
constexpr int kFjGoalHoldConsecutiveFeedback = 5;
}  // namespace

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
  // 发布完整轨迹到 joint_path_command，由 aubo_robot_simulator_ros2 或下游控制器接收 trajectory_msgs::JointTrajectory
  trajectory_command_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_path_command", 100);
  fjt_feedback_sub_ = this->create_subscription<control_msgs::action::FollowJointTrajectory_Feedback>(
    "aubo/feedback_states", 100, std::bind(&JointTrajectoryAction::fjtFeedbackCallback, this, _1));
  moveit_execution_sub_ = this->create_subscription<std_msgs::msg::String>(
    "trajectory_execution_event", 100, std::bind(&JointTrajectoryAction::moveitExecutionCallback, this, _1));

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

  rclcpp::Time stamp_cur(msg->header.stamp, this->get_clock()->get_clock_type());
  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
    stamp_cur = this->get_clock()->now();
  feedback_prev_stamp_ = stamp_cur;
  feedback_stamp_prev_valid_ = true;

  trajectory_state_recvd_ = true;
  last_feedback_valid_ = true;
  last_feedback_positions_.clear();
  for (size_t i = 0; i < msg->actual.positions.size() && i < 6u; ++i)
    last_feedback_positions_.push_back(msg->actual.positions[i]);

  const bool within_goal = checkReachTarget(msg, current_trajectory_);
  if (within_goal)
    ++goal_tolerance_hold_count_;
  else
    goal_tolerance_hold_count_ = 0;

  active_goal_->publish_feedback(std::const_pointer_cast<control_msgs::action::FollowJointTrajectory_Feedback>(msg));
  if (goal_tolerance_hold_count_ >= kFjGoalHoldConsecutiveFeedback)
  {
    RCLCPP_INFO(this->get_logger(), "reach target");
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    result->error_code = result->SUCCESSFUL;
    result->error_string = "successful";
    active_goal_->succeed(result);
    has_active_goal_ = false;
    trajectory_state_recvd_ = false;
    feedback_stamp_prev_valid_ = false;
    goal_tolerance_hold_count_ = 0;
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
  feedback_stamp_prev_valid_ = false;  // 新 goal：避免跨 goal 的 Δt 误算
  goal_tolerance_hold_count_ = 0;
  goal_accept_time_sec_ = this->now().seconds();  // 记录接受时刻，看门狗满 2 秒后再判无反馈
  publishTrajectory();
  return;
}

void JointTrajectoryAction::abortActiveGoal(const char* reason)
{
  const char* r = reason ? reason : "unknown";
  RCLCPP_INFO(this->get_logger(), "JointTrajectoryAction: abort, reason=%s", r);
  goal_tolerance_hold_count_ = 0;

  auto result = std::make_shared<FollowJointTrajectory::Result>();
  result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
  result->error_string = "aborted";
  active_goal_->abort(result);
  has_active_goal_ = false;
  trajectory_state_recvd_ = false;
  feedback_stamp_prev_valid_ = false;
}

void JointTrajectoryAction::publishTrajectory()
{
  RCLCPP_INFO(this->get_logger(), "Publishing complete trajectory to joint_path_command for interpolation");

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
  // 按关节名匹配再比较，避免 driver 与轨迹关节顺序不一致导致永远判不到位
  for (size_t i = 0; i < traj.joint_names.size() && i < 6u; i++)
  {
    size_t fb_idx = (size_t)-1;
    for (size_t k = 0; k < feedback->joint_names.size(); k++)
      if (feedback->joint_names[k] == traj.joint_names[i]) { fb_idx = k; break; }
    if (fb_idx == (size_t)-1 || fb_idx >= feedback->actual.positions.size())
      return false;
    if (std::abs(feedback->actual.positions[fb_idx] - traj.points[last_point].positions[i]) > kFjGoalToleranceRad)
      return false;
  }
  return true;
}

trajectory_msgs::msg::JointTrajectory JointTrajectoryAction::remapTrajectoryByJointName(trajectory_msgs::msg::JointTrajectory &trajectory)
{
  // MoveIt/MTC 有时会发送只包含 positions 的轨迹（velocities/accelerations 为空）。
  // 这里必须做边界检查与缺省填充，否则会越界导致段错误（exit code -11）。

  // 若 joint_names 数量不是 6，则直接返回原轨迹（本驱动仅支持 6 关节重映射）
  if (joint_names.size() != 6u || trajectory.joint_names.size() != 6u)
    return trajectory;

  std::vector<int> mapping(6, -1);
  for (size_t i = 0; i < trajectory.joint_names.size(); ++i)
  {
    for (size_t j = 0; j < 6u; ++j)
    {
      if (trajectory.joint_names[i] == joint_names[j])
        mapping[j] = static_cast<int>(i);
    }
  }

  // mapping 不完整：不要重排，直接返回原轨迹（避免用 -1/越界下标）
  for (size_t j = 0; j < 6u; ++j)
  {
    if (mapping[j] < 0 || static_cast<size_t>(mapping[j]) >= trajectory.joint_names.size())
    {
      RCLCPP_WARN(this->get_logger(), "remapTrajectoryByJointName: mapping incomplete, skip remap");
      return trajectory;
    }
  }

  for (size_t i = 0; i < 6u; ++i)
    RCLCPP_INFO(this->get_logger(), "order %d", mapping[i]);

  trajectory_msgs::msg::JointTrajectory new_traj;
  new_traj.header = trajectory.header;
  new_traj.joint_names = joint_names;

  for (size_t i = 0; i < trajectory.points.size(); ++i)
  {
    const auto& src = trajectory.points[i];
    trajectory_msgs::msg::JointTrajectoryPoint dst;
    dst.time_from_start = src.time_from_start;

    // positions 必须有
    if (src.positions.size() < 6u)
    {
      RCLCPP_WARN(this->get_logger(), "remapTrajectoryByJointName: point[%zu] positions size=%zu < 6, skip remap",
                  i, src.positions.size());
      return trajectory;
    }

    const bool has_vel = (src.velocities.size() >= 6u);
    const bool has_acc = (src.accelerations.size() >= 6u);

    dst.positions.reserve(6);
    dst.velocities.reserve(6);
    dst.accelerations.reserve(6);

    for (size_t j = 0; j < 6u; ++j)
    {
      const size_t k = static_cast<size_t>(mapping[j]);
      dst.positions.push_back(src.positions[k]);
      dst.velocities.push_back(has_vel ? src.velocities[k] : 0.0);
      dst.accelerations.push_back(has_acc ? src.accelerations[k] : 0.0);
    }

    // 若源轨迹带 effort，就同样重排；否则留空
    if (src.effort.size() >= 6u)
    {
      dst.effort.reserve(6);
      for (size_t j = 0; j < 6u; ++j)
      {
        const size_t k = static_cast<size_t>(mapping[j]);
        dst.effort.push_back(src.effort[k]);
      }
    }

    new_traj.points.push_back(std::move(dst));
  }

  return new_traj;
}
