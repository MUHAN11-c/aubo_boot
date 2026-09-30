#include "peach2_end_effector/cutter_end_effector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <string>

#include "peach2_end_effector/failure_codes.hpp"

namespace peach2_end_effector
{

namespace
{

bool finite(const Eigen::Vector3d & v) {return v.allFinite();}

}  // namespace

void CutterEndEffector::initialize(const EndEffectorContext & context)
{
  if (!context.io) {
    throw std::invalid_argument("EndEffectorContext.io is null");
  }
  if (!context.now_s || !context.sleep_s) {
    throw std::invalid_argument("EndEffectorContext.now_s / sleep_s must be set");
  }
  if (context.pins.cmd_pin == context.pins.feedback_pin) {
    throw std::invalid_argument("feedback_pin must differ from cmd_pin");
  }
  if (context.profile.tool_id != expected_tool_id()) {
    throw std::invalid_argument(
            std::string("profile '") + context.profile.tool_id + "' loaded into plugin for '" +
            expected_tool_id() + "'");
  }
  if (!(context.timing.poll_period_s > 0.0) || context.timing.poll_period_s > 0.5) {
    throw std::invalid_argument("timing.poll_period_s must be in (0, 0.5]");
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  ctx_ = context;
  ToolStateMachineConfig sm_config;
  sm_config.feedback_timeout_s = ctx_.profile.io.feedback_timeout_s;
  sm_config.min_actuation_s = ctx_.timing.min_actuation_s;
  sm_config.max_close_energized_s = ctx_.timing.max_close_energized_s;
  sm_ = ToolStateMachine(sm_config);
  initialized_ = true;
}

void CutterEndEffector::require_initialized() const
{
  if (!initialized_) {
    throw std::logic_error("end effector used before initialize()");
  }
}

Eigen::Isometry3d CutterEndEffector::blade_in_tcp() const
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(0.0, 0.0, -ctx_.profile.geometry.l_blade);
  return t;
}

Feasibility CutterEndEffector::feasible(
  const TargetGeometry & target, const BudgetView & budget) const
{
  const ToolGeometry & g = ctx_.profile.geometry;
  Feasibility f;
  f.overshoot_m = g.l_blade;
  if (!finite(target.bottom) || !finite(target.neck) || !finite(target.axis) ||
    target.axis.norm() < 0.5 || !(target.d95_m > 0.0) || !(target.length_m > 0.0) ||
    !std::isfinite(target.d95_m) || !std::isfinite(target.length_m))
  {
    f.failure_code = failure::TOOL_NOT_FEASIBLE;
    f.reason = "invalid_target_geometry";
    return f;
  }
  const Eigen::Vector3d axis = target.axis.normalized();
  f.tcp_at_cut = target.neck + g.l_blade * axis;
  f.radial_clearance_m = 0.5 * (g.d_inner - target.d95_m) - g.wall_clearance;
  if (f.radial_clearance_m <= 0.0) {
    f.failure_code = failure::TOOL_NOT_FEASIBLE;
    f.reason = "bag_wider_than_opening";
    return f;
  }
  if (target.length_m > g.l_insert) {
    f.failure_code = failure::TOOL_NOT_FEASIBLE;
    f.reason = "bag_longer_than_L_insert";
    return f;
  }
  f.ok = true;
  f.sleeve_ok = budget.sleeve_allowed;
  f.cut_ok = f.sleeve_ok && budget.cut_allowed;
  if (!f.sleeve_ok) {
    f.failure_code = failure::BUDGET_RADIAL_NEGATIVE;
    f.reason = "sleeve_not_allowed";
  } else if (!f.cut_ok) {
    f.failure_code = failure::BUDGET_AXIAL_NEGATIVE;
    f.reason = "cut_not_allowed";
  } else {
    f.reason = "ok";
  }
  return f;
}

void CutterEndEffector::sample_locked(double now)
{
  const auto fb = ctx_.io->feedback(ctx_.pins.feedback_pin);
  if (fb) {
    const bool closed = ctx_.pins.feedback_active_high ? *fb : !*fb;
    sm_.feedback(closed, now);
  }
  sm_.update(now);
  last_current_ = ctx_.io->current();
  if (last_current_ && sm_.command_closed()) {
    peak_current_ = std::max(peak_current_, *last_current_);
  }
}

ToolResult CutterEndEffector::command(bool close)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto decision = sm_.begin_command(close, ctx_.now_s());
    if (!decision.accepted) {
      const uint32_t code = sm_.state() == ToolState::FAULT ?
        failure::TOOL_FAULT : failure::TOOL_NOT_OPEN;
      return ToolResult::failure(code, decision.reason);
    }
  }
  const bool level = close ? close_level() : !close_level();
  const bool ok = ctx_.io->set_output(ctx_.pins.cmd_pin, level);
  std::lock_guard<std::mutex> lock(state_mutex_);
  sm_.end_command(ok, ctx_.now_s());
  if (!ok) {
    return ToolResult::failure(failure::TOOL_COMMAND_FAILED, "set_output_failed");
  }
  return ToolResult::success(close ? "close_commanded" : "open_commanded");
}

ToolState CutterEndEffector::wait_while(ToolState transient)
{
  const double deadline = ctx_.now_s() + ctx_.profile.io.feedback_timeout_s +
    2.0 * ctx_.timing.poll_period_s + 0.05;
  while (true) {
    ToolState st;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      sample_locked(ctx_.now_s());
      st = sm_.state();
    }
    if (st != transient) {
      return st;
    }
    if (ctx_.now_s() > deadline) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      sm_.fault(transient == ToolState::OPENING ? "open_feedback_timeout" :
        "close_feedback_timeout");
      return sm_.state();
    }
    ctx_.sleep_s(ctx_.timing.poll_period_s);
  }
}

ToolResult CutterEndEffector::open_result(ToolState reached, const std::string & what)
{
  if (reached == ToolState::OPEN_CONFIRMED) {
    return ToolResult::success(what + "_open_confirmed");
  }
  std::string reason;
  bool loopback = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    reason = sm_.fault_reason();
    loopback = sm_.suspected_loopback();
  }
  if (loopback || reason != "open_feedback_timeout") {
    return ToolResult::failure(failure::TOOL_FAULT, what + ":" + reason);
  }
  return ToolResult::failure(failure::TOOL_NOT_OPEN, what + ":" + reason);
}

ToolResult CutterEndEffector::prepare()
{
  require_initialized();
  std::lock_guard<std::mutex> op(op_mutex_);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    sample_locked(ctx_.now_s());
    if (sm_.state() == ToolState::FAULT) {
      return ToolResult::failure(failure::TOOL_FAULT, "prepare:" + sm_.fault_reason());
    }
    if (!sm_.feedback_closed().has_value()) {
      return ToolResult::failure(failure::TOOL_NOT_OPEN, "prepare:feedback_unavailable");
    }
    if (sm_.state() == ToolState::OPEN_CONFIRMED && !sm_.command_closed()) {
      return ToolResult::success("prepare_already_open");
    }
  }
  const ToolResult cmd = command(false);
  if (!cmd.ok) {
    return cmd;
  }
  return open_result(wait_while(ToolState::OPENING), "prepare");
}

ToolResult CutterEndEffector::cut()
{
  require_initialized();
  std::lock_guard<std::mutex> op(op_mutex_);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    sample_locked(ctx_.now_s());
    if (sm_.state() == ToolState::FAULT) {
      return ToolResult::failure(failure::TOOL_FAULT, "cut:" + sm_.fault_reason());
    }
    if (sm_.state() != ToolState::OPEN_CONFIRMED) {
      return ToolResult::failure(
        failure::TOOL_NOT_OPEN, std::string("cut:not_open_confirmed:") + to_string(sm_.state()));
    }
    peak_current_ = 0.0;
  }
  const ToolResult cmd = command(true);
  if (!cmd.ok) {
    log("cut rejected: " + cmd.reason);
  }
  return cmd;
}

CutVerdict CutterEndEffector::confirm_cut(std::chrono::milliseconds timeout)
{
  require_initialized();
  std::lock_guard<std::mutex> op(op_mutex_);
  CutVerdict v;
  const double start = ctx_.now_s();
  const double deadline = start + std::chrono::duration<double>(timeout).count();
  std::optional<double> edge_s;
  while (true) {
    const double now = ctx_.now_s();
    ToolState st;
    std::string fault;
    bool loopback = false;
    std::optional<double> current;
    double peak = 0.0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      sample_locked(now);
      st = sm_.state();
      fault = sm_.fault_reason();
      loopback = sm_.suspected_loopback();
      current = last_current_;
      peak = peak_current_;
    }
    v.elapsed_s = now - start;
    v.peak_current_a = peak;
    if (st == ToolState::FAULT) {
      v.failure_code = (!loopback && fault == "close_feedback_timeout") ?
        failure::TOOL_FEEDBACK_TIMEOUT : failure::TOOL_FAULT;
      v.reason = "confirm_cut:" + fault;
      return v;
    }
    if (st == ToolState::CLOSED_CONFIRMED) {
      v.feedback_edge = true;
      if (!edge_s) {
        edge_s = now;
      }
      if (!ctx_.current.enabled) {
        v.confirmed = true;
        v.reason = "feedback_edge";
        return v;
      }
      if (!current) {
        v.failure_code = failure::CUT_NOT_CONFIRMED;
        v.reason = "confirm_cut:current_unavailable";
        return v;
      }
      v.current_checked = true;
      if (peak >= ctx_.current.peak_min_a && *current <= ctx_.current.drop_ratio * peak) {
        v.confirmed = true;
        v.reason = "feedback_edge_and_current_drop";
        return v;
      }
      if (now - *edge_s > ctx_.current.window_s) {
        v.failure_code = failure::CUT_NOT_CONFIRMED;
        v.reason = "confirm_cut:no_current_signature";
        return v;
      }
    } else if (st != ToolState::CLOSING) {
      v.failure_code = failure::CUT_NOT_CONFIRMED;
      v.reason = std::string("confirm_cut:not_closing:") + to_string(st);
      return v;
    } else if (now > deadline) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      sm_.fault("close_feedback_timeout");
      v.failure_code = failure::TOOL_FEEDBACK_TIMEOUT;
      v.reason = "confirm_cut:close_feedback_timeout";
      return v;
    }
    ctx_.sleep_s(ctx_.timing.poll_period_s);
  }
}

ToolResult CutterEndEffector::abort_safe()
{
  require_initialized();
  std::lock_guard<std::mutex> op(op_mutex_);
  const ToolResult cmd = command(false);
  if (!cmd.ok) {
    return cmd;
  }
  bool in_fault = false;
  std::string reason;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    in_fault = sm_.state() == ToolState::FAULT;
    reason = sm_.fault_reason();
  }
  if (in_fault) {
    return ToolResult::failure(failure::TOOL_FAULT, "abort_safe:opened_in_fault:" + reason);
  }
  return open_result(wait_while(ToolState::OPENING), "abort_safe");
}

ToolResult CutterEndEffector::release()
{
  require_initialized();
  std::lock_guard<std::mutex> op(op_mutex_);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    sample_locked(ctx_.now_s());
    if (sm_.state() == ToolState::FAULT) {
      return ToolResult::failure(failure::TOOL_FAULT, "release:" + sm_.fault_reason());
    }
  }
  const ToolResult cmd = command(false);
  if (!cmd.ok) {
    return cmd;
  }
  return open_result(wait_while(ToolState::OPENING), "release");
}

ToolStatus CutterEndEffector::status()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  ToolStatus s;
  s.tool_id = expected_tool_id();
  s.state = sm_.state();
  s.command_closed = sm_.command_closed();
  s.feedback_closed = sm_.feedback_closed();
  s.current_a = last_current_;
  s.fault_reason = sm_.fault_reason();
  s.suspected_loopback = sm_.suspected_loopback();
  return s;
}

void CutterEndEffector::poll()
{
  if (!initialized_) {
    return;
  }
  std::unique_lock<std::mutex> op(op_mutex_, std::try_to_lock);
  if (!op.owns_lock()) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    sample_locked(ctx_.now_s());
  }
  deenergize_if_needed();
}

void CutterEndEffector::deenergize_if_needed()
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const double now = ctx_.now_s();
    if (!sm_.needs_deenergize() || now - last_deenergize_s_ < 0.5) {
      return;
    }
    last_deenergize_s_ = now;
  }
  const ToolResult r = command(false);
  log("fault with close level commanded, writing open level: " + r.reason);
}

void CutterEndEffector::reset_by_ack()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  sm_.reset_by_ack();
}

void CutterEndEffector::mark_unknown(const std::string & reason)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  sm_.mark_unknown();
  log("tool state marked UNKNOWN: " + reason);
}

void CutterEndEffector::log(const std::string & msg) const
{
  if (ctx_.log) {
    ctx_.log(msg);
  }
}

}  // namespace peach2_end_effector
