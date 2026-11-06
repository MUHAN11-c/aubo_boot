#ifndef PEACH_ARM__TRAJECTORY_AUDIT_HPP_
#define PEACH_ARM__TRAJECTORY_AUDIT_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <utility>

#include <moveit/robot_model/robot_model.hpp>
#include <moveit/task_constructor/storage.h>
#include <nlohmann/json.hpp>
#include <rclcpp/logging.hpp>
#include <rcutils/logging.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace peach_arm
{

inline const rclcpp::Logger & trajectoryAuditLogger()
{
  static const auto logger = rclcpp::get_logger("peach_arm.trajectory_audit");
  return logger;
}

inline bool trajectoryAuditEnabled()
{
  return rcutils_logging_logger_is_enabled_for(
    trajectoryAuditLogger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
}

// 只记录本包真实下发边界，不声称截获了 DDS/JTC 收到的消息。
// 默认 INFO 在构造 JSON、访问模型或遍历轨迹之前返回；诊断失败不改变执行。
inline void auditJointTrajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const moveit::core::RobotModelConstPtr & model,
  const std::string & kind, const std::string & label,
  const std::string & target_id = {}, std::size_t segment_index = 1,
  std::size_t segment_count = 1) noexcept
{
  if (!trajectoryAuditEnabled() || trajectory.points.empty()) {return;}
  try {
    static std::atomic<std::uint64_t> sequence{0};
    nlohmann::json record = {
      {"schema_version", 1}, {"boundary", "arm_outbound_not_dds_wire"},
      {"kind", kind}, {"label", label}, {"audit_id", ++sequence},
      {"target_id", target_id.empty() ? nlohmann::json(nullptr) : nlohmann::json(target_id)},
      {"cycle_id", nullptr}, {"goal_uuid", nullptr},
      {"segment_index", segment_index}, {"segment_count", segment_count},
      {"joint_names", trajectory.joint_names}, {"frame_id", trajectory.header.frame_id},
      {"header_stamp_ns", static_cast<std::int64_t>(trajectory.header.stamp.sec) *
        1000000000LL + trajectory.header.stamp.nanosec},
      {"steady_ns", std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count()},
      {"points", nlohmann::json::array()}, {"model_bounds", nlohmann::json::array()}};
    for (const auto & point : trajectory.points) {
      record["points"].push_back({
          {"t_ns", static_cast<std::int64_t>(point.time_from_start.sec) *
            1000000000LL + point.time_from_start.nanosec},
          {"q", point.positions}, {"v", point.velocities},
          {"a", point.accelerations}, {"effort", point.effort}});
    }
    record["duration_ns"] = record["points"].back()["t_ns"];
    if (model) {
      for (const auto & name : trajectory.joint_names) {
        const auto & bounds = model->getVariableBounds(name);
        record["model_bounds"].push_back({
            {"name", name}, {"position_bounded", bounds.position_bounded_},
            {"min_position", bounds.min_position_}, {"max_position", bounds.max_position_},
            {"velocity_bounded", bounds.velocity_bounded_},
            {"min_velocity", bounds.min_velocity_}, {"max_velocity", bounds.max_velocity_},
            {"acceleration_bounded", bounds.acceleration_bounded_},
            {"min_acceleration", bounds.min_acceleration_},
            {"max_acceleration", bounds.max_acceleration_},
            {"jerk_bounded", bounds.jerk_bounded_},
            {"min_jerk", bounds.min_jerk_}, {"max_jerk", bounds.max_jerk_}});
      }
    }
    RCLCPP_DEBUG(trajectoryAuditLogger(), "TRAJECTORY_AUDIT_JSON %s", record.dump().c_str());
  } catch (...) {
    RCLCPP_DEBUG(trajectoryAuditLogger(), "轨迹审计失败；原执行流程继续");
  }
}

// Task::execute 只接受 SolutionBase。仅在 DEBUG 时委托同一次 appendTo，
// 读取实际 goal.solution 中的轨迹；不另做一次 toMsg，不改 scene 或解。
// 基类拷贝只保存原状态指针，不调用会注册 incoming/outgoing 的 setter。
class AuditedTaskSolution : public moveit::task_constructor::SolutionBase
{
public:
  AuditedTaskSolution(
    const moveit::task_constructor::SolutionBase & source,
    moveit::core::RobotModelConstPtr model, std::string label, std::string target_id)
  : SolutionBase(source), source_(source), model_(std::move(model)),
    label_(std::move(label)), target_id_(std::move(target_id)) {}

  void appendTo(
    moveit_task_constructor_msgs::msg::Solution & message,
    moveit::task_constructor::Introspection * introspection = nullptr) const override
  {
    source_.appendTo(message, introspection);
    try {
      std::size_t index = 0;
      for (const auto & sub : message.sub_trajectory) {
        ++index;
        auditJointTrajectory(
          sub.trajectory.joint_trajectory, model_, "mtc", label_ + ": " + sub.info.comment,
          target_id_, index, message.sub_trajectory.size());
      }
    } catch (...) {
      RCLCPP_DEBUG(trajectoryAuditLogger(), "MTC轨迹审计失败；原执行流程继续");
    }
  }

  double computeCost(
    const moveit::task_constructor::CostTerm & cost, std::string & comment) const override
  {
    return source_.computeCost(cost, comment);
  }

private:
  const moveit::task_constructor::SolutionBase & source_;
  moveit::core::RobotModelConstPtr model_;
  std::string label_;
  std::string target_id_;
};

}  // namespace peach_arm

#endif  // PEACH_ARM__TRAJECTORY_AUDIT_HPP_
