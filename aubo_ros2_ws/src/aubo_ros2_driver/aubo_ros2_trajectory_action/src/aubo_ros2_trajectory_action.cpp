#include "aubo_ros2_trajectory_action.h"

using namespace aubo_ros2_trajectory_action;

JointTrajectoryAction::JointTrajectoryAction(std::string controller_name):Node("aubo_ros2_trajectory_action")
{
  has_active_goal_ = false;
  trajectory_state_recvd_ = false;

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
  if (has_active_goal_)
  {
    if (!trajectory_state_recvd_)
    {
      RCLCPP_INFO(this->get_logger(), "abort active goal because driver no feedback");
      abortActiveGoal();
    }
  }
}

void JointTrajectoryAction::fjtFeedbackCallback(const control_msgs::action::FollowJointTrajectory_Feedback::ConstSharedPtr msg)
{
  if (!has_active_goal_ || current_trajectory_.points.empty())
    return;

  trajectory_state_recvd_ = true;
  active_goal_->publish_feedback(std::const_pointer_cast<control_msgs::action::FollowJointTrajectory_Feedback>(msg));
  if (checkReachTarget(msg, current_trajectory_))
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
    RCLCPP_INFO(this->get_logger(), "moveit execution stopped");

    if (has_active_goal_)
      abortActiveGoal();
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
    abortActiveGoal();
  }

  (void)uuid;

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse JointTrajectoryAction::handleCancel(const std::shared_ptr<GoalHandleFjt> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "cancel goal");

  auto result = std::make_shared<FollowJointTrajectory::Result>();
  result->error_string = "aborted";
  active_goal_->abort(result);
  has_active_goal_ = false;
  trajectory_state_recvd_ = false;

  return rclcpp_action::CancelResponse::ACCEPT;
}

void JointTrajectoryAction::handleAccept(const std::shared_ptr<GoalHandleFjt> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "accepted new goal");
  active_goal_ = std::move(goal_handle);
  current_trajectory_ = goal_handle->get_goal()->trajectory;
  has_active_goal_ = true;

  // 直接发布完整轨迹，插补在 ROS 1 端完成
  publishTrajectory();

  return;
}

void JointTrajectoryAction::abortActiveGoal()
{
  RCLCPP_INFO(this->get_logger(), "Marks the active goal as aborted");
  auto result = std::make_shared<FollowJointTrajectory::Result>();
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
  bool ret = false;
  int last_point = traj.points.size() - 1;
  ret = true;
  
  for (int i = 0; i < 6; i++)
  {
    if(abs(feedback->actual.positions[i] - traj.points[last_point].positions[i]) > fabs(0.002))
    {
      ret = false;
      break;
    }
  }

  return ret;
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
