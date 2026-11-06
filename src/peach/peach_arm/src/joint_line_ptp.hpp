#ifndef PEACH_ARM__JOINT_LINE_PTP_HPP_
#define PEACH_ARM__JOINT_LINE_PTP_HPP_

#include "joint_line_retiming.hpp"

#include <functional>
#include <memory>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

namespace peach_arm
{

// 在PipelinePlanner交给MTC之前替换RobotTrajectory；后续验收、实际执行及
// 原路返程缓存均使用同一个新对象，不只改toMsg()的临时副本。
class JointLinePtpPlanner : public moveit::task_constructor::solvers::PipelinePlanner
{
public:
  using GeometryGate = std::function<bool(const robot_trajectory::RobotTrajectory &)>;
  JointLinePtpPlanner(
    const rclcpp::Node::SharedPtr & node, const std::string & pipeline, GeometryGate gate)
  : PipelinePlanner(node, pipeline, "PTP"), geometry_gate_(std::move(gate)) {}

  using PipelinePlanner::plan;

  Result plan(
    const planning_scene::PlanningSceneConstPtr & from,
    const planning_scene::PlanningSceneConstPtr & to,
    const moveit::core::JointModelGroup * group, double timeout,
    robot_trajectory::RobotTrajectoryPtr & path,
    const moveit_msgs::msg::Constraints & constraints = moveit_msgs::msg::Constraints()) override
  {
    const auto result = PipelinePlanner::plan(from, to, group, timeout, path, constraints);
    if (!result || !path || path->empty() || !group || getPlannerId() != "PTP") {return result;}
    try {
      moveit_msgs::msg::RobotTrajectory msg;
      path->getRobotTrajectoryMsg(msg);
      if (!msg.multi_dof_joint_trajectory.points.empty() ||
        msg.joint_trajectory.joint_names != group->getVariableNames()) {return result;}
      std::vector<JointLineLimit> limits;
      for (const auto & name : msg.joint_trajectory.joint_names) {
        const auto * joint = path->getRobotModel()->getJointOfVariable(name);
        if (!joint || joint->getType() != moveit::core::JointModel::REVOLUTE ||
          joint->getVariableCount() != 1U) {return result;}
        const auto & b = path->getRobotModel()->getVariableBounds(name);
        if (!b.position_bounded_ || !b.velocity_bounded_ || !b.acceleration_bounded_ ||
          !b.jerk_bounded_ || !std::isfinite(b.min_velocity_) || !std::isfinite(b.max_velocity_) ||
          !std::isfinite(b.min_acceleration_) || !std::isfinite(b.max_acceleration_) ||
          !std::isfinite(b.min_jerk_) || !std::isfinite(b.max_jerk_)) {return result;}
        limits.push_back({b.min_position_, b.max_position_,
            std::min(b.max_velocity_, -b.min_velocity_),
            std::min(b.max_acceleration_, -b.min_acceleration_),
            std::min(b.max_jerk_, -b.min_jerk_)});
      }
      JointLineRetimeReport report;
      if (!retimeJointLine(msg.joint_trajectory, limits,
          properties().get<double>("max_velocity_scaling_factor"),
          properties().get<double>("max_acceleration_scaling_factor"), report))
      {
        RCLCPP_DEBUG(node_->get_logger(), "PTP保留原时间参数: %s", report.reason.c_str());
        return result;
      }
      auto retimed = std::make_shared<robot_trajectory::RobotTrajectory>(
        path->getRobotModel(), path->getGroupName());
      retimed->setRobotTrajectoryMsg(path->getFirstWayPoint(), msg);
      // RobotTrajectory用double保存相邻dt；再次序列化可能截断1ns。
      // 必须核同一对象最终toMsg的实际时间，不沿用输入消息的证明。
      moveit_msgs::msg::RobotTrajectory final_msg;
      retimed->getRobotTrajectoryMsg(final_msg);
      JointLineRetimeReport final_bounds;
      if (!jointLineQuinticWithinLimits(
          final_msg.joint_trajectory, report.validated_limits, final_bounds)) {return result;}
      if (!from->isPathValid(*retimed, constraints, group->getName(), false) ||
        !geometry_gate_ || !geometry_gate_(*retimed)) {return result;}
      RCLCPP_INFO(node_->get_logger(),
        "PTP同路径重定时: %.6fs -> %.6fs points=%zu->%zu "
        "quintic_bound_v/a/j=%.6f/%.6f/%.6f",
        report.before_s, report.after_s, path->getWayPointCount(), retimed->getWayPointCount(),
        final_bounds.velocity_ratio, final_bounds.acceleration_ratio, final_bounds.jerk_ratio);
      path = std::move(retimed);
    } catch (const std::exception & error) {
      RCLCPP_WARN(node_->get_logger(), "PTP重定时验证异常，保留原轨迹: %s", error.what());
    }
    return result;
  }

private:
  GeometryGate geometry_gate_;
};

}  // namespace peach_arm

#endif  // PEACH_ARM__JOINT_LINE_PTP_HPP_
