#pragma once

#include <Eigen/Geometry>

#include <optional>
#include <string>

#include "aubo_msgs/msg/robot_status.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "peach2_end_effector/types.hpp"
#include "peach2_interfaces/msg/enables.hpp"
#include "peach2_interfaces/msg/grasp_decision.hpp"
#include "peach2_interfaces/msg/harvest_result.hpp"
#include "peach2_interfaces/msg/target_model.hpp"
#include "peach2_interfaces/msg/tool_state.hpp"
#include "peach2_manipulation/command_gate.hpp"
#include "peach2_manipulation/decision_client.hpp"
#include "peach2_manipulation/harvest_cycle.hpp"

namespace peach2_manipulation
{

Eigen::Isometry3d pose_from_msg(const geometry_msgs::msg::Pose & pose);

/// `valid_until_s` is the decision's valid_until on the ROS clock of the producer.
DecisionView decision_from_msg(const peach2_interfaces::msg::GraspDecision & msg);

/// nullopt when bottom / neck are invalid, the axis is degenerate or sizes are not finite.
/// `branch_direction` is set only when `branch_direction_known` and the vector is usable;
/// TargetModel carries no obstacle ("avoid") direction.
std::optional<peach2_end_effector::TargetGeometry> geometry_from_msg(
  const peach2_interfaces::msg::TargetModel & msg);

peach2_interfaces::msg::HarvestResult result_to_msg(const CycleResult & result);

/// Feedback maps to FEEDBACK_UNKNOWN / OPEN / CLOSED (unknown = no independent input).
peach2_interfaces::msg::ToolState tool_state_to_msg(const peach2_end_effector::ToolStatus & s);

RobotStatusSample robot_status_from_msg(
  const aubo_msgs::msg::RobotStatus & msg, double received_s);

EnablesSample enables_from_msg(const peach2_interfaces::msg::Enables & msg, double received_s);

}  // namespace peach2_manipulation
