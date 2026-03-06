#include "aubo_ros2_trajectory_action.h"
#include <cmath>
#include <fstream>
#include <chrono>

using namespace aubo_ros2_trajectory_action;

// #region agent log — 调试偶发 -4：记录 accept/abort/success 及时间，写入 NDJSON
static const char* kDebugLogPath = "/home/mu/IVG/.cursor/debug-157b9d.log";
static const char* kLastAbortReasonPath = "/home/mu/IVG/.cursor/last_abort_reason.txt";
static void debugLog(const std::string& jsonLine) {
  std::ofstream f(kDebugLogPath, std::ios::app);
  if (f) { f << jsonLine << "\n"; }
}
static void writeLastAbortReason(const char* reason) {
  std::ofstream f(kLastAbortReasonPath);
  if (f && reason) f << reason << "\n";
}
// #endregion

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

  // 声明并获取速度缩放因子参数（默认值为 1.0，表示不缩放）
  // 可以设置为 0.1-1.0 之间的值来减慢运动速度，例如 0.5 表示 50% 速度
  this->declare_parameter<double>("velocity_scale_factor", 1.0);
  double velocity_scale_factor = this->get_parameter("velocity_scale_factor").as_double();
  RCLCPP_INFO(this->get_logger(), "Velocity scale factor: %f", velocity_scale_factor);

  // 声明并获取笛卡尔路径重采样参数
  // 仅对"异常笛卡尔"重采样：存在点间隔 < 阈值时才重采样，关节空间(间隔~0.095)不受影响
  // 重采样目标：使笛卡尔 segment_T 对齐关节空间（~0.095s）
  this->declare_parameter<double>("cartesian_resample_threshold", 0.095);
  this->declare_parameter<double>("cartesian_min_segment_interval", 0.095);
  double cartesian_threshold = this->get_parameter("cartesian_resample_threshold").as_double();
  double target_interval = this->get_parameter("cartesian_min_segment_interval").as_double();
  RCLCPP_INFO(this->get_logger(), "Cartesian resample threshold: %f, target interval: %f", cartesian_threshold, target_interval);
  
  // 配置重采样过滤器
  resample_filter_.configure(target_interval);

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
    // #region agent log
    {
      double since = this->now().seconds() - goal_accept_time_sec_;
      std::string line = "{\"sessionId\":\"157b9d\",\"event\":\"success\",\"time_since_accept_sec\":" + std::to_string(since) + "}";
      debugLog(line);
    }
    // #endregion
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

  // #region agent log — 诊断 client_cancel：记录轨迹起点，abort 时与 joint_states/feedback 对比
  {
    double dur = 0.0;
    if (!current_trajectory_.points.empty())
      dur = toSec(current_trajectory_.points.back().time_from_start);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    std::string line = "{\"sessionId\":\"157b9d\",\"event\":\"accept\",\"timestamp_ms\":" + std::to_string(ms) +
        ",\"trajectory_points\":" + std::to_string(current_trajectory_.points.size()) +
        ",\"trajectory_duration_sec\":" + std::to_string(dur);
    if (!current_trajectory_.points.empty()) {
      line += ",\"trajectory_start_positions\":[";
      for (size_t i = 0; i < current_trajectory_.points[0].positions.size(); ++i)
        line += (i ? "," : "") + std::to_string(current_trajectory_.points[0].positions[i]);
      line += "]";
    }
    line += "}";
    debugLog(line);
  }
  // #endregion

  publishTrajectory();

  return;
}

void JointTrajectoryAction::abortActiveGoal(const char* reason)
{
  const char* r = reason ? reason : "unknown";
  RCLCPP_INFO(this->get_logger(), "JointTrajectoryAction: abort, reason=%s", r);
  writeLastAbortReason(r);
  // #region agent log — 诊断 client_cancel：轨迹起点 vs 最后 feedback，判断是否起点偏差导致取消
  {
    double since = this->now().seconds() - goal_accept_time_sec_;
    std::string line = "{\"sessionId\":\"157b9d\",\"event\":\"abort\",\"reason\":\"" + std::string(r) +
        "\",\"time_since_accept_sec\":" + std::to_string(since) +
        ",\"feedback_received_before_abort\":" + (last_feedback_valid_ ? "true" : "false");
    if (!current_trajectory_.points.empty()) {
      line += ",\"trajectory_start_positions\":[";
      for (size_t i = 0; i < current_trajectory_.points[0].positions.size(); ++i)
        line += (i ? "," : "") + std::to_string(current_trajectory_.points[0].positions[i]);
      line += "]";
    }
    if (last_feedback_valid_ && !last_feedback_positions_.empty()) {
      line += ",\"last_feedback_positions\":[";
      for (size_t i = 0; i < last_feedback_positions_.size(); ++i)
        line += (i ? "," : "") + std::to_string(last_feedback_positions_[i]);
      line += "]";
      if (!current_trajectory_.points.empty() && current_trajectory_.points[0].positions.size() == last_feedback_positions_.size()) {
        double max_delta = 0.0;
        for (size_t i = 0; i < last_feedback_positions_.size(); ++i) {
          double d = std::abs(current_trajectory_.points[0].positions[i] - last_feedback_positions_[i]);
          if (d > max_delta) max_delta = d;
        }
        line += ",\"max_delta_start_vs_feedback_rad\":" + std::to_string(max_delta);
      }
    }
    line += "}";
    debugLog(line);
  }
  // #endregion
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
  // sometimes current_trajectory.joint_names order: foreArm_joint, shoulder_joint, upperArm_joint, wrist1_joint, wrist2_joint, wrist3_joint
  // will change order: shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint
  trajectory_msgs::msg::JointTrajectory remap_traj = remapTrajectoryByJointName(current_trajectory_);
  current_trajectory_ = remap_traj;

  // ========== 速度缩放处理（可选）==========
  // 获取速度缩放因子（从参数服务器，支持运行时修改）
  double velocity_scale_factor = this->get_parameter("velocity_scale_factor").as_double();
  
  // 如果缩放因子不是 1.0 且有效，对轨迹进行速度缩放
  if (velocity_scale_factor != 1.0 && velocity_scale_factor > 0.0 && velocity_scale_factor <= 1.0)
  {
    RCLCPP_INFO(this->get_logger(), "Scaling trajectory velocity by factor: %f", velocity_scale_factor);
    
    for (auto& point : current_trajectory_.points)
    {
      // 缩放时间：time_from_start = time_from_start / scale
      // 这样轨迹执行时间会变长，速度会变慢
      double time_sec = toSec(point.time_from_start);
      point.time_from_start = toDuration(time_sec / velocity_scale_factor);
      
      // 缩放速度和加速度（如果存在）
      // 速度：velocities = velocities * scale
      // 加速度：accelerations = accelerations * scale * scale
      if (!point.velocities.empty())
      {
        for (size_t j = 0; j < point.velocities.size(); j++)
        {
          point.velocities[j] *= velocity_scale_factor;
        }
      }
      if (!point.accelerations.empty())
      {
        for (size_t j = 0; j < point.accelerations.size(); j++)
        {
          point.accelerations[j] *= velocity_scale_factor * velocity_scale_factor;
        }
      }
    }
    double original_time = toSec(current_trajectory_.points.back().time_from_start) * velocity_scale_factor;
    double scaled_time = toSec(current_trajectory_.points.back().time_from_start);
    RCLCPP_INFO(this->get_logger(), "Trajectory scaled: %zu points, original time: %f s, scaled time: %f s (scale factor: %f)", 
                current_trajectory_.points.size(), 
                original_time, 
                scaled_time, 
                velocity_scale_factor);
  }
  else if (velocity_scale_factor <= 0.0 || velocity_scale_factor > 1.0)
  {
    RCLCPP_WARN(this->get_logger(), "Invalid velocity_scale_factor: %f (must be 0.0-1.0), using 1.0", velocity_scale_factor);
  }

  // ========== 笛卡尔路径重采样处理 ==========
  // 仅对"异常笛卡尔"重采样：存在点间隔 < 阈值时才重采样，关节空间(间隔~0.095)不受影响
  // 重采样目标：使笛卡尔 segment_T 对齐关节空间（~0.095s）
  double cartesian_threshold = this->get_parameter("cartesian_resample_threshold").as_double();
  double target_interval = this->get_parameter("cartesian_min_segment_interval").as_double();
  
  if (current_trajectory_.points.size() >= 3 && target_interval > 0)
  {
    // 计算点间隔
    double min_interval = 1.0;
    for (size_t i = 1; i < current_trajectory_.points.size(); i++)
    {
      double interval = toSec(current_trajectory_.points[i].time_from_start) - 
                        toSec(current_trajectory_.points[i-1].time_from_start);
      if (interval < min_interval)
      {
        min_interval = interval;
      }
    }
    
    // 如果最小间隔小于阈值，说明是密集的笛卡尔路径，需要重采样
    if (min_interval < cartesian_threshold)
    {
      size_t n_before = current_trajectory_.points.size();
      trajectory_msgs::msg::JointTrajectory resampled_traj;
      
      // 使用 UniformSampleFilter 进行重采样（插值重采样，保证均匀时间间隔）
      resample_filter_.configure(target_interval);
      if (resample_filter_.update(current_trajectory_, resampled_traj))
      {
        current_trajectory_ = resampled_traj;
        RCLCPP_INFO(this->get_logger(), "Cartesian resampled: %zu -> %zu points (target_interval=%.3fs, align to joint ~0.095s)", 
                    n_before, current_trajectory_.points.size(), target_interval);
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Cartesian resampling failed, using original trajectory");
      }
    }
  }

  // 直接发布完整轨迹到 joint_path_command
  // ROS 1 端的 aubo_joint_trajectory_action 或 aubo_robot_simulator 会接收并处理插补
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
      traj.points.back().positions.size() < 6u)
    return false;
  const size_t last_point = traj.points.size() - 1;
  const double kReachedToleranceRad = 0.01;
  for (int i = 0; i < 6; i++)
  {
    if (std::abs(feedback->actual.positions[i] - traj.points[last_point].positions[i]) > kReachedToleranceRad)
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
