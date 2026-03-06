// Copyright 2023 Xie Shaosong
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUBO_JOINT_TRAJECTORY_ACTION_H
#define AUBO_JOINT_TRAJECTORY_ACTION_H

#include <memory>
#include <queue>
#include <thread>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include "std_msgs/msg/string.hpp"

#include <kdl/velocityprofile_spline.hpp>

const double DEFAULT_SAMPLE_DURATION = 0.005;

namespace aubo_ros2_trajectory_action
{

// https://github.com/ros-industrial/industrial_core/blob/melodic-devel/industrial_trajectory_filters/include/industrial_trajectory_filters/uniform_sample_filter.h
class UniformSampleFilter
{
public:
  UniformSampleFilter();

  void configure(const double &sample_duration);
  bool update(const trajectory_msgs::msg::JointTrajectory &in, trajectory_msgs::msg::JointTrajectory &out);
  bool interpolatePt(trajectory_msgs::msg::JointTrajectoryPoint &p1, trajectory_msgs::msg::JointTrajectoryPoint &p2, 
                      double time_from_start, trajectory_msgs::msg::JointTrajectoryPoint &interp_pt);
private:
  double toSec(const builtin_interfaces::msg::Duration &duration);
  builtin_interfaces::msg::Duration toDuration(double time_in_seconds);

  double sample_duration_;
};

class JointTrajectoryAction : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFjt = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  JointTrajectoryAction(std::string controller_name);

private:
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

  // 发布完整轨迹到 joint_path_command，由 ROS 1 端的 aubo_robot_simulator 接收并处理插补
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_command_pub_;
  rclcpp::Subscription<control_msgs::action::FollowJointTrajectory_Feedback>::SharedPtr fjt_feedback_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr moveit_execution_sub_;

  rclcpp::TimerBase::SharedPtr watch_dog_timer_;

  bool has_active_goal_;
  bool trajectory_state_recvd_;
  double goal_accept_time_sec_;  // 接受目标时刻（ROS time 秒），看门狗仅在“接受后满 2 秒仍无反馈”时才 abort
  trajectory_msgs::msg::JointTrajectory current_trajectory_;
  std::shared_ptr<GoalHandleFjt> active_goal_;
  // 诊断 client_cancel：记录最后一次 feedback 的 actual.positions，abort 时与轨迹起点对比
  std::vector<double> last_feedback_positions_;
  bool last_feedback_valid_{false};

  void watchDogTimer();

  void fjtFeedbackCallback(const control_msgs::action::FollowJointTrajectory_Feedback::ConstSharedPtr msg);
  void moveitExecutionCallback(const std_msgs::msg::String::ConstSharedPtr msg);

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleFjt> goal_handle);
  void handleAccept(const std::shared_ptr<GoalHandleFjt> goal_handle);

  void abortActiveGoal(const char* reason = nullptr);

  void publishTrajectory();  // 发布完整轨迹，插补在 ROS 1 端完成
  double toSec(const builtin_interfaces::msg::Duration &duration);
  builtin_interfaces::msg::Duration toDuration(double time_in_seconds);

  bool isSimilar(std::vector<std::string> lhs, std::vector<std::string> rhs);

  trajectory_msgs::msg::JointTrajectory remapTrajectoryByJointName(trajectory_msgs::msg::JointTrajectory &trajectory);

  bool checkReachTarget(const control_msgs::action::FollowJointTrajectory_Feedback::ConstSharedPtr feedback, const trajectory_msgs::msg::JointTrajectory &traj);

  std::vector<std::string> joint_names;
  
  // 轨迹重采样过滤器（用于笛卡尔路径）
  UniformSampleFilter resample_filter_;
};

}

#endif
