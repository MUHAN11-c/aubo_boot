#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit_msgs/action/execute_trajectory.hpp"
#include "moveit_msgs/srv/get_state_validity.hpp"
#include "peach2_manipulation/motion_backend.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace peach2_manipulation
{

struct MoveItBackendConfig
{
  std::string group;
  std::string tip_link;
  std::string base_frame;
  std::string free_pipeline;
  std::string free_planner_id;
  std::string linear_pipeline;
  std::string linear_planner_id;
  double planning_time_s{2.0};
  int planning_attempts{3};
  double cartesian_max_trans_vel_mps{0.25};
  double wait_for_servers_s{30.0};
  double stop_grace_s{5.0};
  int validate_stride{5};
  double at_goal_tolerance_rad{0.002};
};

/// MotionBackend over move_group. Planning goes through MoveGroupInterface; execution goes
/// straight to the `execute_trajectory` action so a cancel is always ours to send and the wait
/// is bounded (the old stack blocked forever in MGI::execute during a TEM stop storm).
///
/// `node` must be spun by an executor the caller owns (MGI spins only its private group).
/// All methods are serialized; plan() and execute() are called from one worker at a time.
class MoveItMotionBackend : public MotionBackend
{
public:
  /// Throws std::runtime_error when the robot model or move_group servers are unavailable.
  MoveItMotionBackend(rclcpp::Node::SharedPtr node, MoveItBackendConfig config);
  ~MoveItMotionBackend() override;

  PlanResult plan(const PlanRequest & request) override;
  ExecResult execute(const JointTrajectory & trajectory, const ExecOptions & options) override;
  bool validate(const JointTrajectory & trajectory, std::string * why) override;
  std::optional<std::vector<double>> current_joints() override;
  std::optional<Eigen::Isometry3d> current_tcp() override;
  void stop() override;
  /// TODO(M4): wrist F/T sensor; until then admittance inserts fall back to LIN.
  bool has_force_sensing() const override {return false;}

private:
  using ExecuteTrajectory = moveit_msgs::action::ExecuteTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteTrajectory>;

  void cancel_active_goal();

  rclcpp::Node::SharedPtr node_;
  MoveItBackendConfig config_;
  rclcpp::Logger logger_;
  std::mutex mgi_mutex_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi_;
  rclcpp_action::Client<ExecuteTrajectory>::SharedPtr exec_client_;
  rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr validity_client_;
  std::mutex goal_mutex_;
  GoalHandle::SharedPtr active_goal_;
};

/// Map a MoveIt error code to FailureCode. Linear (Pilz LIN) failures other than IK /
/// collision count as an incomplete straight segment: LIN is all-or-nothing.
uint32_t plan_failure_code(int32_t moveit_error_code, PlanKind kind);

}  // namespace peach2_manipulation
