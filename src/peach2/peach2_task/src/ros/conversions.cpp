// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
#include "peach2_task/ros/conversions.hpp"

namespace peach2_task::ros
{

namespace msg = peach2_interfaces::msg;

core::Candidate to_candidate(const msg::TargetObservation & obs)
{
  core::Candidate c;
  c.target_id = obs.target_id;
  c.category = obs.category;
  c.confirmed = obs.confirmed;
  c.edge_touch = obs.edge_touch;
  c.has_geometry = obs.bottom.valid || obs.neck.valid;
  c.mask_quality = obs.mask_quality;
  c.depth_coverage = obs.depth_coverage;
  c.camera_distance_m = obs.camera_distance_m;
  c.height_m = obs.neck.valid ? obs.neck.position.z : obs.bottom.position.z;
  c.roi_area_px = static_cast<double>(obs.roi.width) * static_cast<double>(obs.roi.height);
  return c;
}

core::ObservationSet to_observation_set(const msg::TargetObservationArray & array)
{
  core::ObservationSet set;
  set.target_set_locked = array.target_set_locked;
  set.scene_epoch = array.scene_epoch;
  set.locked_target_ids = array.locked_target_ids;
  set.candidates.reserve(array.observations.size());
  for (const auto & obs : array.observations) {
    set.candidates.push_back(to_candidate(obs));
  }
  return set;
}

bool to_reach_map(
  const peach2_interfaces::srv::CheckReachability::Response & response, core::ReachMap * out)
{
  const size_t n = response.target_ids.size();
  const bool has_reasons = !response.reasons.empty();
  if (response.reachable.size() != n || response.failure_codes.size() != n ||
    (has_reasons && response.reasons.size() != n))
  {
    return false;
  }
  out->clear();
  for (size_t i = 0; i < n; ++i) {
    (*out)[response.target_ids[i]] = core::Reach{
      static_cast<bool>(response.reachable[i]), response.failure_codes[i],
      has_reasons ? response.reasons[i] : std::string{}};
  }
  return true;
}

core::DecisionOutcome to_decision(bool found, const msg::GraspDecision & d, int64_t now_ns)
{
  core::DecisionOutcome out;
  out.found = found;
  if (!found) {
    out.failure_code = d.failure_code;
    out.reason = d.reason;
    return out;
  }
  const int64_t valid_until_ns =
    static_cast<int64_t>(d.valid_until.sec) * 1000000000LL + d.valid_until.nanosec;
  out.expired = valid_until_ns != 0 && valid_until_ns < now_ns;
  out.approach_allowed = d.approach_allowed;
  out.sleeve_allowed = d.sleeve_allowed;
  out.cut_allowed = d.cut_allowed;
  out.model_revision = d.model_revision;
  out.failure_code = d.failure_code;
  out.reason = d.reason;
  out.radial_margin_m = d.radial_margin_m;
  out.axial_margin_m = d.axial_margin_m;
  return out;
}

core::ObserveOutcome to_observe(const peach2_interfaces::action::ObserveTarget::Result & result)
{
  core::ObserveOutcome out;
  out.converged = result.converged;
  out.failure_code = result.failure_code;
  out.model_revision = result.model.model_revision;
  out.n_views = result.model.n_views;
  return out;
}

core::HarvestOutcome to_outcome(const msg::HarvestResult & m)
{
  core::HarvestOutcome o;
  o.target_id = m.target_id;
  o.tool_id = m.tool_id;
  o.outcome = m.outcome;
  o.reached = m.reached;
  o.failure_code = m.failure_code;
  o.reason = m.reason;
  o.recovery_required = m.recovery_required;
  o.plan_only = m.plan_only;
  o.cycle_time_s = m.cycle_time_s;
  o.stage_names = m.stage_names;
  o.stage_times_s = m.stage_times_s;
  o.radial_margin_m = m.radial_margin_m;
  o.axial_margin_m = m.axial_margin_m;
  return o;
}

msg::HarvestResult to_msg(const core::HarvestOutcome & o)
{
  msg::HarvestResult m;
  m.target_id = o.target_id;
  m.tool_id = o.tool_id;
  m.outcome = o.outcome;
  m.reached = o.reached;
  m.failure_code = o.failure_code;
  m.reason = o.reason;
  m.recovery_required = o.recovery_required;
  m.plan_only = o.plan_only;
  m.cycle_time_s = o.cycle_time_s;
  m.stage_names = o.stage_names;
  m.stage_times_s = o.stage_times_s;
  m.radial_margin_m = o.radial_margin_m;
  m.axial_margin_m = o.axial_margin_m;
  return m;
}

msg::BatchState to_msg(
  const core::StateSnapshot & s, const builtin_interfaces::msg::Time & stamp)
{
  msg::BatchState m;
  m.header.stamp = stamp;
  m.request_id = s.request_id;
  m.phase = static_cast<uint8_t>(s.phase);
  m.current_target_id = s.current_target_id;
  m.blockers = s.blockers;
  m.attempted = s.counts.attempted;
  m.succeeded = s.counts.succeeded;
  m.skipped = s.counts.skipped;
  m.failed = s.counts.failed;
  m.recovery_required = s.recovery_required;
  m.message = s.message;
  return m;
}

uint8_t harvest_mode_for(core::Intent intent)
{
  using Goal = peach2_interfaces::action::HarvestTarget::Goal;
  return intent == core::Intent::FULL ? Goal::MODE_FULL : Goal::MODE_PREGRASP_ONLY;
}

uint8_t reachability_mode_for(core::Intent intent)
{
  using Req = peach2_interfaces::srv::CheckReachability::Request;
  return intent == core::Intent::FULL ? Req::MODE_FULL : Req::MODE_PREGRASP_ONLY;
}

}  // namespace peach2_task::ros
