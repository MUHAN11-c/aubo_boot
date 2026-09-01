// 功能：执行前安全门默认实现（机器人就绪、目标新鲜度）。纯核，零 ROS。
#include "peach_manipulation/safety_gate.hpp"

#include <string>
#include <utility>

namespace peach_manipulation
{
SafetyGate::SafetyGate(SafetyGateConfig config, std::function<double()> clock_s)
: config_(config), clock_s_(std::move(clock_s))
{
}

bool SafetyGate::robotReady(const RobotStatusSample & sample, std::string & reason) const
{
  if (!config_.require_robot_status) {
    return true;
  }
  if (!sample.received) {
    reason = "robot_status_missing";
    return false;
  }
  if (clock_s_() - sample.received_s > config_.robot_status_max_age_s) {
    reason = "robot_status_stale";
    return false;
  }
  if (sample.e_stopped || sample.in_error || !sample.drives_powered ||
    !sample.motion_possible)
  {
    reason = "robot_status_not_motion_ready";
    return false;
  }
  return true;
}

bool SafetyGate::targetReady(
  const TargetGateSample & sample, const std::string & target_id,
  std::string & reason) const
{
  if (sample.id != target_id) {
    reason = "selected_target_changed";
    return false;
  }
  if (!sample.valid) {
    reason = "selected_target_not_observed";
    return false;
  }
  if (clock_s_() - sample.received_s > config_.target_observation_max_age_s) {
    reason = "selected_target_stale";
    return false;
  }
  return true;
}

}  // namespace peach_manipulation
