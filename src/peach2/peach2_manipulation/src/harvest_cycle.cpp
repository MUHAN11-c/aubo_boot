#include "peach2_manipulation/harvest_cycle.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "peach2_end_effector/failure_codes.hpp"

namespace peach2_manipulation
{

namespace failure = peach2_end_effector::failure;
using peach2_end_effector::ToolResult;
using peach2_end_effector::ToolState;

const char * to_string(CycleStage stage)
{
  switch (stage) {
    case CycleStage::PREPARE_TOOL: return "PREPARE_TOOL";
    case CycleStage::TRANSIT_STAGING: return "TRANSIT_STAGING";
    case CycleStage::APPROACH_PREGRASP: return "APPROACH_PREGRASP";
    case CycleStage::VERIFY_PREGRASP: return "VERIFY_PREGRASP";
    case CycleStage::INSERT: return "INSERT";
    case CycleStage::CUT: return "CUT";
    case CycleStage::CONFIRM: return "CONFIRM";
    case CycleStage::RETREAT: return "RETREAT";
    case CycleStage::TRANSIT_RELEASE: return "TRANSIT_RELEASE";
    case CycleStage::RELEASE: return "RELEASE";
    case CycleStage::DONE: return "DONE";
  }
  return "INVALID";
}

const char * to_string(ScenePhase phase)
{
  switch (phase) {
    case ScenePhase::APPROACH: return "approach";
    case ScenePhase::CONTACT: return "contact";
  }
  return "invalid";
}

struct HarvestCycle::Selection
{
  double roll{0.0};
  Eigen::Isometry3d pregrasp{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d staging{Eigen::Isometry3d::Identity()};
  PlanRequest free_request;
  PlanRequest approach_request;
  JointTrajectory free_trajectory;
  JointTrajectory approach_trajectory;
  double cost{std::numeric_limits<double>::infinity()};
};

struct HarvestCycle::Run
{
  CycleRequest request;
  CycleResult result;
  double t0{0.0};
  std::optional<CycleStage> stage;
  double stage_t0{0.0};
  int stage_index{0};
  peach2_end_effector::TargetGeometry geometry;
  DecisionView decision;
  Selection selection;
  /// In-canopy segments actually sent, in order; reversed for the exit.
  std::vector<JointTrajectory> canopy;
  bool tool_prepared{false};
};

namespace
{

constexpr char kPlannedReason[] = "planned";

bool is_stop_code(uint32_t code)
{
  return code == failure::CANCELED || code == failure::SAFETY_GATE_CLOSED ||
         code == failure::ROBOT_NOT_READY;
}

Reached max_reached(Reached a, Reached b)
{
  return static_cast<uint8_t>(a) >= static_cast<uint8_t>(b) ? a : b;
}

peach2_end_effector::BudgetView budget_of(const DecisionView & d)
{
  peach2_end_effector::BudgetView b;
  b.sleeve_allowed = d.sleeve_allowed;
  b.cut_allowed = d.cut_allowed;
  b.radial_margin_m = d.radial_margin_m;
  b.axial_margin_m = d.axial_margin_m;
  return b;
}

}  // namespace

HarvestCycle::HarvestCycle(CycleConfig config, CycleDeps deps)
: config_(std::move(config)), deps_(std::move(deps))
{
  if (!deps_.motion || !deps_.ee || !deps_.decisions || !deps_.targets || !deps_.gate ||
    !deps_.enables || !deps_.cancel_requested || !deps_.now_s || !deps_.sleep_s)
  {
    throw std::invalid_argument("HarvestCycle: missing dependency");
  }
  if (config_.roll_samples < 1 || config_.max_corrections < 0 || config_.cut_retry_max < 0) {
    throw std::invalid_argument("HarvestCycle: invalid config");
  }
}

void HarvestCycle::log(const std::string & msg) const
{
  if (deps_.log) {
    deps_.log(msg);
  }
}

void HarvestCycle::begin_stage(Run & run, CycleStage stage)
{
  const double now = deps_.now_s();
  if (run.stage) {
    run.result.stage_names.emplace_back(to_string(*run.stage));
    run.result.stage_times_s.push_back(now - run.stage_t0);
  }
  run.stage = stage;
  run.stage_t0 = now;
  if (deps_.on_stage) {
    deps_.on_stage(stage, run.stage_index, now - run.t0);
  }
  ++run.stage_index;
}

CycleResult HarvestCycle::finish(
  Run & run, Outcome outcome, uint32_t code, const std::string & reason)
{
  const double now = deps_.now_s();
  if (run.stage) {
    run.result.stage_names.emplace_back(to_string(*run.stage));
    run.result.stage_times_s.push_back(now - run.stage_t0);
    run.stage.reset();
  }
  // Plan-only reachability keys on failure_code == NONE: only a fully planned chain may carry it.
  if (run.result.plan_only && code == failure::NONE && reason != kPlannedReason) {
    code = failure::PLAN_FAILED;
  }
  run.result.outcome = outcome;
  run.result.failure_code = code;
  run.result.reason = reason;
  run.result.cycle_time_s = now - run.t0;
  if (!run.canopy.empty() && outcome != Outcome::SUCCEEDED) {
    run.result.recovery_required = true;
  }
  log(
    std::string("cycle ") + run.request.target_id + " -> outcome=" +
    std::to_string(static_cast<int>(outcome)) + " code=" + std::to_string(code) + " " + reason);
  return run.result;
}

bool HarvestCycle::set_scene(const Run & run, ScenePhase phase, std::string & why)
{
  if (!deps_.scene) {
    return true;
  }
  std::string detail;
  if (deps_.scene(phase, run.request.target_id, &detail)) {
    return true;
  }
  why = std::string("scene_update_failed:") + to_string(phase) + (detail.empty() ? "" : ":") +
    detail;
  return false;
}

bool HarvestCycle::canceled(Run & run)
{
  (void)run;
  return deps_.cancel_requested();
}

void HarvestCycle::note_tool_fault(Run & run)
{
  if (deps_.ee->status().state == ToolState::FAULT) {
    run.result.recovery_required = true;
  }
}

std::optional<DecisionView> HarvestCycle::requery(
  Run & run, const DecisionView & previous, uint32_t & code, std::string & reason)
{
  auto d = deps_.decisions->get(run.request.target_id, run.request.tool_id, previous.revision);
  if (!d) {
    code = failure::MODEL_EXPIRED;
    reason = "decision_missing";
    return std::nullopt;
  }
  if (d->valid_until_s < deps_.decisions->now_s()) {
    code = failure::MODEL_EXPIRED;
    reason = "decision_expired";
    return std::nullopt;
  }
  if (d->revision != previous.revision) {
    const double shift = std::max(
      (d->pregrasp_tcp.translation() - previous.pregrasp_tcp.translation()).norm(),
      (d->blade_target - previous.blade_target).norm());
    if (shift > config_.model_shift_tol_m) {
      code = failure::MODEL_STALE;
      reason = "model_revision_moved";
      return std::nullopt;
    }
  }
  run.result.radial_margin_m = d->radial_margin_m;
  run.result.axial_margin_m = d->axial_margin_m;
  return d;
}

std::optional<HarvestCycle::Selection> HarvestCycle::select_roll(
  Run & run, uint32_t & code, std::string & reason)
{
  const DecisionView & d = run.decision;
  const Eigen::Vector3d axis = d.pregrasp_tcp.linear().col(2).normalized();
  peach2_end_effector::TargetGeometry g = run.geometry;
  g.axis = axis;
  const auto rolls = deps_.ee->roll_constraint(g).samples(config_.roll_samples);

  std::optional<Selection> best;
  int best_failure_rank = -1;
  for (double roll : rolls) {
    Selection s;
    s.roll = roll;
    s.pregrasp = pose_with_roll(d.pregrasp_tcp.translation(), axis, roll);
    s.staging = staging_pose(s.pregrasp, run.geometry.bottom, config_.staging_distance_m);

    s.free_request.kind = PlanKind::FREE;
    s.free_request.tcp_goal = s.staging;
    s.free_request.velocity_scaling = config_.transit_velocity_scaling;
    s.free_request.acceleration_scaling = config_.transit_acceleration_scaling;
    s.free_request.label = "transit_staging";
    const PlanResult pf = deps_.motion->plan(s.free_request);
    if (!pf.ok || pf.trajectory.empty()) {
      if (best_failure_rank < 1) {
        best_failure_rank = 1;
        code = pf.ok ? failure::PLAN_FAILED : pf.failure_code;
        reason = "transit_staging:" + pf.reason;
      }
      continue;
    }
    s.approach_request.kind = PlanKind::LINEAR;
    s.approach_request.tcp_goal = s.pregrasp;
    s.approach_request.linear_speed_mps = config_.approach_speed_mps;
    s.approach_request.acceleration_scaling = config_.transit_acceleration_scaling;
    s.approach_request.start_joints = pf.trajectory.points.back().positions;
    s.approach_request.label = "approach_pregrasp";
    const PlanResult pl = deps_.motion->plan(s.approach_request);
    if (!pl.ok || pl.trajectory.empty()) {
      if (best_failure_rank < 2) {
        best_failure_rank = 2;
        code = pl.ok ? failure::PLAN_CARTESIAN_INCOMPLETE : pl.failure_code;
        reason = "approach_pregrasp:" + pl.reason;
      }
      continue;
    }
    s.free_trajectory = pf.trajectory;
    s.approach_trajectory = pl.trajectory;
    s.cost = joint_path_length(pf.trajectory) + joint_path_length(pl.trajectory);
    if (!best || s.cost < best->cost) {
      best = s;
    }
  }
  if (rolls.empty()) {
    code = failure::TOOL_NOT_FEASIBLE;
    reason = "no_roll_samples";
  }
  return best;
}

ExecResult HarvestCycle::exec_segment(
  Run & run, JointTrajectory trajectory, std::optional<PlanRequest> replan, GateStage stage,
  bool into_canopy)
{
  const GateVerdict v = deps_.gate(stage, true);
  if (!v.open) {
    return {false, v.failure_code, std::string("gate_") + to_string(stage) + ":" + v.reason};
  }
  if (trajectory.empty()) {
    return {false, failure::EXEC_FAILED, "empty_trajectory"};
  }
  auto current = deps_.motion->current_joints();
  if (!current) {
    return {false, failure::EXEC_FAILED, "joint_state_unavailable"};
  }
  if (max_joint_deviation(*current, trajectory.points.front().positions) >
    config_.start_tolerance_rad)
  {
    if (!replan) {
      return {false, failure::EXEC_FAILED, "start_state_mismatch"};
    }
    replan->start_joints.reset();
    const PlanResult p = deps_.motion->plan(*replan);
    if (!p.ok || p.trajectory.empty()) {
      return {false, p.ok ? failure::PLAN_FAILED : p.failure_code, "replan:" + p.reason};
    }
    trajectory = p.trajectory;
    current = deps_.motion->current_joints();
    if (!current || max_joint_deviation(*current, trajectory.points.front().positions) >
      config_.start_tolerance_rad)
    {
      return {false, failure::EXEC_FAILED, "start_state_mismatch"};
    }
  }
  if (is_null_motion(trajectory)) {
    // Nothing moves, so nothing to back out of the canopy either.
    return {true, failure::NONE, "at_goal"};
  }
  ExecOptions options;
  options.timeout_s = trajectory.duration_s() * config_.execute_timeout_scale +
    config_.execute_timeout_margin_s;
  options.abort_probe = [this, stage]() -> std::optional<Abort> {
      if (deps_.cancel_requested()) {
        return Abort{failure::CANCELED, "canceled"};
      }
      const GateVerdict g = deps_.gate(stage, false);
      if (!g.open) {
        return Abort{g.failure_code, std::string("gate_") + to_string(stage) + ":" + g.reason};
      }
      return std::nullopt;
    };
  // Recorded before executing: a partially executed segment still has to be backed out.
  if (into_canopy) {
    run.canopy.push_back(trajectory);
  }
  return deps_.motion->execute(trajectory, options);
}

bool HarvestCycle::retreat(Run & run, std::string & why, uint32_t & stop_code)
{
  begin_stage(run, CycleStage::RETREAT);
  stop_code = failure::NONE;
  if (run.canopy.empty()) {
    return true;
  }
  const JointTrajectory reverse = reverse_path(run.canopy);
  std::string invalid;
  if (deps_.motion->validate(reverse, &invalid)) {
    const ExecResult r = exec_segment(run, reverse, std::nullopt, GateStage::RETREAT, false);
    if (r.ok) {
      run.canopy.clear();
      mark_retreated(run);
      return true;
    }
    if (is_stop_code(r.failure_code)) {
      why = r.reason;
      stop_code = r.failure_code;
      return false;
    }
    log("reverse exit failed (" + r.reason + "), falling back to axial LIN to staging");
  } else {
    log("reverse exit invalid (" + invalid + "), falling back to axial LIN to staging");
  }
  PlanRequest lin;
  lin.kind = PlanKind::LINEAR;
  lin.tcp_goal = run.selection.staging;
  lin.linear_speed_mps = config_.retreat_speed_mps;
  lin.acceleration_scaling = config_.transit_acceleration_scaling;
  lin.label = "retreat_fallback";
  const PlanResult p = deps_.motion->plan(lin);
  if (!p.ok) {
    why = "retreat_plan:" + p.reason;
    return false;
  }
  const ExecResult r = exec_segment(run, p.trajectory, lin, GateStage::RETREAT, false);
  if (!r.ok) {
    why = "retreat_exec:" + r.reason;
    if (is_stop_code(r.failure_code)) {
      stop_code = r.failure_code;
    }
    return false;
  }
  run.canopy.clear();
  mark_retreated(run);
  return true;
}

void HarvestCycle::mark_retreated(Run & run)
{
  // reached is ordered progress and RETREATED implies the cut; a pregrasp-only exit is visible
  // in stage_names / failure_code instead.
  if (run.request.mode == CycleMode::FULL) {
    run.result.reached = max_reached(run.result.reached, Reached::RETREATED);
  }
}

CycleResult HarvestCycle::fail_in_canopy(
  Run & run, Outcome outcome, uint32_t code, const std::string & reason)
{
  if (code == failure::CANCELED) {
    return finish(run, Outcome::CANCELED, code, reason);
  }
  if (is_stop_code(code)) {
    // Gate closed under us: no further motion. Opening the blade is still allowed (TOOL_SAFE).
    if (run.tool_prepared) {
      const ToolResult a = deps_.ee->abort_safe();
      if (!a.ok) {
        log("abort_safe after gate close: " + a.reason);
      }
      note_tool_fault(run);
    }
    return finish(run, outcome, code, reason);
  }
  std::string why;
  uint32_t stop_code = failure::NONE;
  if (!retreat(run, why, stop_code)) {
    run.result.recovery_required = true;
    const uint32_t final_code = stop_code != failure::NONE ? stop_code : failure::RETREAT_FAILED;
    const Outcome o = stop_code == failure::CANCELED ? Outcome::CANCELED : Outcome::FAILED;
    return finish(run, o, final_code, reason + ";retreat_failed:" + why);
  }
  return finish(run, outcome, code, reason);
}

CycleResult HarvestCycle::plan_chain(Run & run)
{
  begin_stage(run, CycleStage::TRANSIT_STAGING);
  uint32_t code = 0;
  std::string reason;
  auto sel = select_roll(run, code, reason);
  if (!sel) {
    return finish(run, Outcome::SKIPPED, code, reason);
  }
  run.selection = *sel;
  if (run.request.mode == CycleMode::FULL) {
    begin_stage(run, CycleStage::INSERT);
    std::string why;
    if (!set_scene(run, ScenePhase::CONTACT, why)) {
      return finish(run, Outcome::SKIPPED, failure::DEPENDENCY_UNAVAILABLE, why);
    }
    const Eigen::Isometry3d goal = tcp_for_blade(
      run.decision.blade_target, sel->pregrasp.linear(), deps_.ee->blade_in_tcp());
    const InsertGeometry ig = insert_geometry(sel->pregrasp, goal.translation());
    if (ig.travel_m <= 0.0 || ig.lateral_m > config_.insert_lateral_tol_m) {
      return finish(run, Outcome::SKIPPED, failure::PLAN_FAILED, "insert_geometry");
    }
    PlanRequest ins;
    ins.kind = PlanKind::LINEAR;
    ins.tcp_goal = goal;
    ins.linear_speed_mps = deps_.ee->insert(run.geometry).speed_mps;
    ins.acceleration_scaling = config_.transit_acceleration_scaling;
    ins.start_joints = sel->approach_trajectory.points.back().positions;
    ins.label = "insert";
    const PlanResult p = deps_.motion->plan(ins);
    if (!p.ok) {
      return finish(run, Outcome::SKIPPED, p.failure_code, "insert:" + p.reason);
    }
  }
  return finish(run, Outcome::SKIPPED, failure::NONE, kPlannedReason);
}

CycleResult HarvestCycle::run(const CycleRequest & request)
{
  Run run;
  run.request = request;
  run.t0 = deps_.now_s();
  run.result.target_id = request.target_id;
  run.result.tool_id = request.tool_id;

  const EffectiveEnables en = deps_.enables();
  const bool execute = en.execution && !request.plan_only;
  const bool full = request.mode == CycleMode::FULL;
  run.result.plan_only = !execute;

  if (request.tool_id != deps_.ee->tool_id()) {
    return finish(
      run, Outcome::SKIPPED, failure::TOOL_NOT_FEASIBLE, "tool_not_loaded:" + deps_.ee->tool_id());
  }
  if (execute && full && !(en.grasp && en.tool)) {
    return finish(
      run, Outcome::SKIPPED, failure::SAFETY_GATE_CLOSED, "full_mode_requires_grasp_and_tool");
  }
  const auto geometry = deps_.targets->get(request.target_id);
  if (!geometry) {
    return finish(run, Outcome::SKIPPED, failure::MODEL_EXPIRED, "target_model_missing");
  }
  run.geometry = *geometry;
  const auto decision = deps_.decisions->get(request.target_id, request.tool_id, 0);
  if (!decision) {
    return finish(run, Outcome::SKIPPED, failure::MODEL_EXPIRED, "decision_missing");
  }
  if (decision->valid_until_s < deps_.decisions->now_s()) {
    return finish(run, Outcome::SKIPPED, failure::MODEL_EXPIRED, "decision_expired");
  }
  run.decision = *decision;
  run.result.radial_margin_m = decision->radial_margin_m;
  run.result.axial_margin_m = decision->axial_margin_m;
  if (!decision->approach_allowed) {
    const uint32_t code = decision->failure_code != 0 ?
      decision->failure_code : failure::BUDGET_RADIAL_NEGATIVE;
    return finish(run, Outcome::SKIPPED, code, "approach_not_allowed:" + decision->reason);
  }
  const auto feas = deps_.ee->feasible(run.geometry, budget_of(*decision));
  if (!feas.ok) {
    return finish(run, Outcome::SKIPPED, feas.failure_code, feas.reason);
  }
  {
    std::string why;
    if (!set_scene(run, ScenePhase::APPROACH, why)) {
      return finish(run, Outcome::SKIPPED, failure::DEPENDENCY_UNAVAILABLE, why);
    }
  }

  if (!execute) {
    return plan_chain(run);
  }

  // ---- PREPARE_TOOL ----
  if (full) {
    begin_stage(run, CycleStage::PREPARE_TOOL);
    const GateVerdict g = deps_.gate(GateStage::TOOL, false);
    if (!g.open) {
      return finish(run, Outcome::SKIPPED, g.failure_code, "gate_TOOL:" + g.reason);
    }
    const ToolResult p = deps_.ee->prepare();
    if (!p.ok) {
      note_tool_fault(run);
      return finish(run, Outcome::FAILED, p.failure_code, p.reason);
    }
    run.tool_prepared = true;
  }
  if (canceled(run)) {
    return finish(run, Outcome::CANCELED, failure::CANCELED, "canceled");
  }

  // ---- TRANSIT_STAGING ----
  begin_stage(run, CycleStage::TRANSIT_STAGING);
  uint32_t code = 0;
  std::string reason;
  auto sel = select_roll(run, code, reason);
  if (!sel) {
    return finish(run, Outcome::SKIPPED, code, reason);
  }
  run.selection = *sel;
  PlanRequest free_replan = sel->free_request;
  ExecResult r = exec_segment(
    run, sel->free_trajectory, free_replan, GateStage::TRANSIT, false);
  if (!r.ok) {
    const Outcome o = r.failure_code == failure::CANCELED ? Outcome::CANCELED : Outcome::FAILED;
    return fail_in_canopy(run, o, r.failure_code, "transit_staging:" + r.reason);
  }

  // ---- APPROACH_PREGRASP ----
  begin_stage(run, CycleStage::APPROACH_PREGRASP);
  PlanRequest approach_replan = sel->approach_request;
  r = exec_segment(run, sel->approach_trajectory, approach_replan, GateStage::APPROACH, true);
  if (!r.ok) {
    return fail_in_canopy(run, Outcome::FAILED, r.failure_code, "approach_pregrasp:" + r.reason);
  }

  // ---- VERIFY_PREGRASP ----
  begin_stage(run, CycleStage::VERIFY_PREGRASP);
  for (int corrections = 0;; ++corrections) {
    const auto tcp = deps_.motion->current_tcp();
    if (!tcp) {
      return fail_in_canopy(run, Outcome::FAILED, failure::EXEC_FAILED, "tcp_unavailable");
    }
    const Residual res = pregrasp_residual(*tcp, sel->pregrasp, config_.residual);
    if (res.ok) {
      break;
    }
    if (corrections >= config_.max_corrections) {
      return fail_in_canopy(
        run, Outcome::FAILED, failure::PREGRASP_RESIDUAL, "pregrasp_residual:" + res.reason);
    }
    PlanRequest fix = sel->approach_request;
    fix.start_joints.reset();
    fix.label = "pregrasp_correction";
    fix.collapse_at_goal = false;
    const PlanResult p = deps_.motion->plan(fix);
    if (!p.ok) {
      return fail_in_canopy(run, Outcome::FAILED, p.failure_code, "correction:" + p.reason);
    }
    r = exec_segment(run, p.trajectory, fix, GateStage::APPROACH, true);
    if (!r.ok) {
      return fail_in_canopy(run, Outcome::FAILED, r.failure_code, "correction:" + r.reason);
    }
  }
  run.result.reached = Reached::PREGRASP;
  if (canceled(run)) {
    return finish(run, Outcome::CANCELED, failure::CANCELED, "canceled");
  }

  if (!full) {
    if (!config_.pregrasp_only_retreat) {
      // Arm left at pregrasp by configuration; not a failure, so no recovery latch.
      return finish(run, Outcome::SUCCEEDED, failure::NONE, "pregrasp_reached");
    }
    std::string why;
    uint32_t stop_code = failure::NONE;
    if (!retreat(run, why, stop_code)) {
      run.result.recovery_required = true;
      const uint32_t c = stop_code != failure::NONE ? stop_code : failure::RETREAT_FAILED;
      return finish(run, Outcome::FAILED, c, "retreat_failed:" + why);
    }
    return finish(run, Outcome::SUCCEEDED, failure::NONE, "pregrasp_reached");
  }

  // ---- INSERT / CUT / CONFIRM (one retry on CUT_NOT_CONFIRMED) ----
  DecisionView current = run.decision;
  for (int attempt = 0;; ++attempt) {
    begin_stage(run, CycleStage::INSERT);
    {
      std::string why;
      if (!set_scene(run, ScenePhase::CONTACT, why)) {
        return fail_in_canopy(run, Outcome::FAILED, failure::DEPENDENCY_UNAVAILABLE, why);
      }
    }
    auto d2 = requery(run, current, code, reason);
    if (!d2) {
      return fail_in_canopy(run, Outcome::SKIPPED, code, "before_insert:" + reason);
    }
    current = *d2;
    if (!current.sleeve_allowed) {
      const uint32_t c = current.failure_code != 0 ?
        current.failure_code : failure::BUDGET_RADIAL_NEGATIVE;
      return fail_in_canopy(run, Outcome::SKIPPED, c, "sleeve_not_allowed:" + current.reason);
    }
    const auto plan_ins = deps_.ee->insert(run.geometry);
    if (plan_ins.mode == peach2_end_effector::InsertMode::ADMITTANCE &&
      !deps_.motion->has_force_sensing() && !plan_ins.linear_fallback)
    {
      return fail_in_canopy(
        run, Outcome::SKIPPED, failure::TOOL_NOT_FEASIBLE, "insert_needs_force_sensing");
    }
    const Eigen::Isometry3d goal = tcp_for_blade(
      current.blade_target, sel->pregrasp.linear(), deps_.ee->blade_in_tcp());
    const InsertGeometry ig = insert_geometry(sel->pregrasp, goal.translation());
    if (ig.travel_m <= 0.0 || ig.lateral_m > config_.insert_lateral_tol_m) {
      return fail_in_canopy(run, Outcome::SKIPPED, failure::PLAN_FAILED, "insert_geometry");
    }
    PlanRequest ins;
    ins.kind = PlanKind::LINEAR;
    ins.tcp_goal = goal;
    ins.linear_speed_mps = plan_ins.speed_mps;
    ins.acceleration_scaling = config_.transit_acceleration_scaling;
    ins.label = "insert";
    const PlanResult pi = deps_.motion->plan(ins);
    if (!pi.ok) {
      return fail_in_canopy(run, Outcome::FAILED, pi.failure_code, "insert:" + pi.reason);
    }
    const std::size_t canopy_before = run.canopy.size();
    r = exec_segment(run, pi.trajectory, ins, GateStage::CONTACT, true);
    const bool insert_moved = run.canopy.size() > canopy_before;
    if (!r.ok) {
      return fail_in_canopy(run, Outcome::FAILED, r.failure_code, "insert:" + r.reason);
    }
    if (plan_ins.dwell_s > 0.0) {
      deps_.sleep_s(plan_ins.dwell_s);
    }
    run.result.reached = max_reached(run.result.reached, Reached::INSERTED);
    if (canceled(run)) {
      return finish(run, Outcome::CANCELED, failure::CANCELED, "canceled");
    }

    begin_stage(run, CycleStage::CUT);
    auto d3 = requery(run, current, code, reason);
    if (!d3) {
      return fail_in_canopy(run, Outcome::SKIPPED, code, "before_cut:" + reason);
    }
    current = *d3;
    if (!current.cut_allowed) {
      const uint32_t c = current.failure_code != 0 ?
        current.failure_code : failure::BUDGET_AXIAL_NEGATIVE;
      return fail_in_canopy(run, Outcome::SKIPPED, c, "cut_not_allowed:" + current.reason);
    }
    const GateVerdict gt = deps_.gate(GateStage::TOOL, false);
    if (!gt.open) {
      return fail_in_canopy(run, Outcome::FAILED, gt.failure_code, "gate_TOOL:" + gt.reason);
    }
    const ToolResult c = deps_.ee->cut();
    if (!c.ok) {
      if (c.failure_code == failure::TOOL_FAULT || c.failure_code == failure::TOOL_COMMAND_FAILED) {
        deps_.ee->abort_safe();
      }
      note_tool_fault(run);
      return fail_in_canopy(run, Outcome::FAILED, c.failure_code, c.reason);
    }

    begin_stage(run, CycleStage::CONFIRM);
    const double timeout_s = deps_.ee->profile().io.feedback_timeout_s + config_.confirm_margin_s;
    const auto verdict = deps_.ee->confirm_cut(
      std::chrono::milliseconds(static_cast<int64_t>(timeout_s * 1000.0)));
    if (verdict.confirmed) {
      run.result.reached = max_reached(run.result.reached, Reached::CUT_CONFIRMED);
      break;
    }
    const ToolResult opened = deps_.ee->abort_safe();
    if (verdict.failure_code == failure::CUT_NOT_CONFIRMED && opened.ok &&
      attempt < config_.cut_retry_max)
    {
      // Back out of the bag along the insert, then re-insert with a fresh decision.
      if (insert_moved) {
        JointTrajectory insert_segment = run.canopy.back();
        const ExecResult back = exec_segment(
          run, reverse_trajectory(insert_segment), std::nullopt, GateStage::RETREAT, false);
        if (!back.ok) {
          return fail_in_canopy(
            run, Outcome::FAILED, back.failure_code, "cut_retry_backout:" + back.reason);
        }
        run.canopy.pop_back();
      }
      log("cut not confirmed (" + verdict.reason + "), retrying");
      continue;
    }
    if (!opened.ok || verdict.failure_code != failure::CUT_NOT_CONFIRMED) {
      run.result.recovery_required = true;
    }
    note_tool_fault(run);
    return fail_in_canopy(run, Outcome::FAILED, verdict.failure_code, verdict.reason);
  }

  // ---- RETREAT ----
  std::string why;
  uint32_t stop_code = failure::NONE;
  if (!retreat(run, why, stop_code)) {
    run.result.recovery_required = true;
    const uint32_t c = stop_code != failure::NONE ? stop_code : failure::RETREAT_FAILED;
    return finish(run, Outcome::FAILED, c, "retreat_failed:" + why);
  }
  if (canceled(run)) {
    return finish(run, Outcome::CANCELED, failure::CANCELED, "canceled");
  }

  // ---- TRANSIT_RELEASE ----
  begin_stage(run, CycleStage::TRANSIT_RELEASE);
  PlanRequest rel;
  rel.kind = PlanKind::NAMED;
  rel.named_target = config_.release_named_target;
  rel.velocity_scaling = config_.transit_velocity_scaling;
  rel.acceleration_scaling = config_.transit_acceleration_scaling;
  rel.label = "transit_release";
  const PlanResult pr = deps_.motion->plan(rel);
  if (!pr.ok) {
    return finish(run, Outcome::FAILED, pr.failure_code, "transit_release:" + pr.reason);
  }
  r = exec_segment(run, pr.trajectory, rel, GateStage::TRANSIT, false);
  if (!r.ok) {
    const Outcome o = r.failure_code == failure::CANCELED ? Outcome::CANCELED : Outcome::FAILED;
    return finish(run, o, r.failure_code, "transit_release:" + r.reason);
  }

  // ---- RELEASE ----
  begin_stage(run, CycleStage::RELEASE);
  const GateVerdict gr = deps_.gate(GateStage::RELEASE, false);
  if (!gr.open) {
    return finish(run, Outcome::FAILED, gr.failure_code, "gate_RELEASE:" + gr.reason);
  }
  const ToolResult released = deps_.ee->release();
  if (!released.ok) {
    note_tool_fault(run);
    return finish(run, Outcome::FAILED, released.failure_code, released.reason);
  }
  run.result.reached = Reached::RELEASED;
  begin_stage(run, CycleStage::DONE);
  return finish(run, Outcome::SUCCEEDED, failure::NONE, "released");
}

}  // namespace peach2_manipulation
