// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <peach2_interfaces/action/harvest_target.hpp>
#include <peach2_interfaces/action/observe_target.hpp>
#include <peach2_interfaces/msg/batch_state.hpp>
#include <peach2_interfaces/msg/grasp_decision.hpp>
#include <peach2_interfaces/msg/harvest_result.hpp>
#include <peach2_interfaces/msg/target_observation_array.hpp>
#include <peach2_interfaces/srv/check_reachability.hpp>

#include <string>
#include <vector>

#include "peach2_task/core/batch_session.hpp"
#include "peach2_task/core/selection.hpp"

/// Message <-> pure-core conversions. No node, no clock: callers pass times in.
namespace peach2_task::ros
{

core::Candidate to_candidate(const peach2_interfaces::msg::TargetObservation & obs);
core::ObservationSet to_observation_set(const peach2_interfaces::msg::TargetObservationArray & msg);

/// False when the response arrays disagree in length; `reasons` may also be empty.
bool to_reach_map(
  const peach2_interfaces::srv::CheckReachability::Response & response, core::ReachMap * out);

/// `now_ns`: current time on the clock that stamps valid_until. valid_until == 0 never expires.
core::DecisionOutcome to_decision(
  bool found, const peach2_interfaces::msg::GraspDecision & decision, int64_t now_ns);

core::ObserveOutcome to_observe(const peach2_interfaces::action::ObserveTarget::Result & result);

core::HarvestOutcome to_outcome(const peach2_interfaces::msg::HarvestResult & msg);
peach2_interfaces::msg::HarvestResult to_msg(const core::HarvestOutcome & outcome);

peach2_interfaces::msg::BatchState to_msg(
  const core::StateSnapshot & snapshot, const builtin_interfaces::msg::Time & stamp);

uint8_t harvest_mode_for(core::Intent intent);
uint8_t reachability_mode_for(core::Intent intent);

}  // namespace peach2_task::ros
