#include "peach2_manipulation/move_to.hpp"

#include <algorithm>
#include <optional>
#include <string>

#include "peach2_end_effector/failure_codes.hpp"

namespace peach2_manipulation
{

namespace failure = peach2_end_effector::failure;

MoveToResult run_move_to(
  const MoveToRequest & request, const MoveToConfig & config, const MoveToDeps & deps)
{
  MoveToResult out;
  PlanRequest plan;
  if (!request.named_target.empty()) {
    plan.kind = PlanKind::NAMED;
    plan.named_target = request.named_target;
  } else if (request.tcp_pose) {
    plan.kind = PlanKind::FREE;
    plan.tcp_goal = *request.tcp_pose;
  } else {
    out.failure_code = failure::PLAN_FAILED;
    out.message = "no_target";
    return out;
  }
  const double v = request.velocity_scaling > 0.0 ?
    request.velocity_scaling : config.default_velocity_scaling;
  plan.velocity_scaling = std::clamp(v, 0.01, config.max_velocity_scaling);
  plan.acceleration_scaling = config.acceleration_scaling;
  plan.label = "move_to";

  const EffectiveEnables en = deps.enables();
  out.plan_only = !en.execution;
  if (deps.scene) {
    std::string why;
    if (!deps.scene(&why)) {
      out.failure_code = failure::DEPENDENCY_UNAVAILABLE;
      out.message = "scene_update_failed" + (why.empty() ? std::string() : ":" + why);
      return out;
    }
  }
  const PlanResult p = deps.motion->plan(plan);
  if (!p.ok || p.trajectory.empty()) {
    out.failure_code = p.ok ? failure::PLAN_FAILED : p.failure_code;
    out.message = "plan:" + p.reason;
    return out;
  }
  if (out.plan_only) {
    out.failure_code = failure::SAFETY_GATE_CLOSED;
    out.message = "plan_only_ok";
    return out;
  }
  const GateVerdict g = deps.gate(GateStage::TRANSIT, true);
  if (!g.open) {
    out.failure_code = g.failure_code;
    out.message = "gate_TRANSIT:" + g.reason;
    return out;
  }
  const auto current = deps.motion->current_joints();
  if (!current ||
    max_joint_deviation(*current, p.trajectory.points.front().positions) >
    config.start_tolerance_rad)
  {
    out.failure_code = failure::EXEC_FAILED;
    out.message = "start_state_mismatch";
    return out;
  }
  ExecOptions options;
  options.timeout_s = p.trajectory.duration_s() * config.execute_timeout_scale +
    config.execute_timeout_margin_s;
  options.abort_probe = [&deps]() -> std::optional<Abort> {
      if (deps.cancel_requested()) {
        return Abort{failure::CANCELED, "canceled"};
      }
      const GateVerdict gv = deps.gate(GateStage::TRANSIT, false);
      if (!gv.open) {
        return Abort{gv.failure_code, "gate_TRANSIT:" + gv.reason};
      }
      return std::nullopt;
    };
  const ExecResult r = deps.motion->execute(p.trajectory, options);
  out.success = r.ok;
  out.failure_code = r.ok ? failure::NONE : r.failure_code;
  out.message = r.ok ? "ok" : r.reason;
  return out;
}

}  // namespace peach2_manipulation
