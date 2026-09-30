// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
#include "peach2_task/core/batch_session.hpp"

#include <algorithm>
#include <utility>

namespace peach2_task::core
{

const char * phase_name(Phase phase)
{
  switch (phase) {
    case Phase::IDLE: return "IDLE";
    case Phase::SURVEYING: return "SURVEYING";
    case Phase::SELECTING: return "SELECTING";
    case Phase::OBSERVING: return "OBSERVING";
    case Phase::HARVESTING: return "HARVESTING";
    case Phase::WAITING_ACK: return "WAITING_ACK";
    case Phase::PAUSED: return "PAUSED";
    case Phase::COMPLETED: return "COMPLETED";
    case Phase::ABORTED: return "ABORTED";
  }
  return "UNKNOWN";
}

const char * outcome_name(uint8_t value)
{
  switch (value) {
    case outcome::SUCCEEDED: return "SUCCEEDED";
    case outcome::SKIPPED: return "SKIPPED";
    case outcome::FAILED: return "FAILED";
    case outcome::CANCELED: return "CANCELED";
    default: return "UNKNOWN";
  }
}

const char * reached_name(uint8_t value)
{
  switch (value) {
    case reached::NONE: return "NONE";
    case reached::PREGRASP: return "PREGRASP";
    case reached::INSERTED: return "INSERTED";
    case reached::CUT_CONFIRMED: return "CUT_CONFIRMED";
    case reached::RETREATED: return "RETREATED";
    case reached::RELEASED: return "RELEASED";
    default: return "UNKNOWN";
  }
}

Policy Failure::policy() const
{
  if (recovery_required) {
    return Policy::RECOVER;
  }
  if (policy_override) {
    return *policy_override;
  }
  if (code == fc::NONE) {
    return Policy::SKIP;
  }
  return policy_for(code);
}

bool decision_level_from_name(const std::string & name, DecisionLevel * out)
{
  if (name == "approach") {
    *out = DecisionLevel::APPROACH;
  } else if (name == "sleeve") {
    *out = DecisionLevel::SLEEVE;
  } else if (name == "cut") {
    *out = DecisionLevel::CUT;
  } else {
    return false;
  }
  return true;
}

BatchSession::BatchSession()
: BatchSession(
    [] {
      return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    },
    [] {return std::chrono::system_clock::now();})
{
}

BatchSession::BatchSession(Clock clock, WallClock wall_clock)
: clock_(std::move(clock)), wall_clock_(std::move(wall_clock))
{
}

void BatchSession::begin(
  BatchRequest request, SessionConfig config, std::unique_ptr<Ledger> ledger)
{
  const uint64_t revision = revision_;
  const bool peer_recovery = peer_recovery_;
  *this = BatchSession(clock_, wall_clock_);
  revision_ = revision + 1;
  peer_recovery_ = peer_recovery;
  request_ = std::move(request);
  config_ = std::move(config);
  ledger_ = std::move(ledger);
  active_ = true;
  started_utc_ = utc_iso8601(wall_clock_());
  phase_ = Phase::SURVEYING;
  write_ledger();
}

void BatchSession::set_phase(Phase phase)
{
  if (phase != phase_) {
    phase_ = phase;
    bump();
  }
}

void BatchSession::set_message(const std::string & message)
{
  if (message != message_) {
    message_ = message;
    bump();
  }
}

void BatchSession::set_safety_blockers(const std::vector<std::string> & blockers)
{
  if (blockers != safety_blockers_) {
    safety_blockers_ = blockers;
    bump();
  }
}

StateSnapshot BatchSession::snapshot() const
{
  StateSnapshot s;
  s.request_id = request_.request_id;
  s.phase = phase_;
  s.current_target_id = current_target_;
  s.blockers = safety_blockers_;
  if (recovery_required_) {
    s.blockers.emplace_back("recovery_required");
  }
  if (peer_recovery_) {
    s.blockers.emplace_back("manipulation_recovery_required");
  }
  if (ledger_failures_ > 0) {
    s.blockers.emplace_back("ledger_write_failed");
  }
  s.counts = counts_;
  s.recovery_required = recovery_required_;
  s.message = message_;
  return s;
}

void BatchSession::on_scene_begun(uint32_t scene_epoch)
{
  scene_begun_ = true;
  scene_epoch_ = scene_epoch;
  bump();
  write_ledger();
}

void BatchSession::on_locked_snapshot(ObservationSet set)
{
  snapshot_ = std::move(set);
  has_snapshot_ = true;
  const Eligibility seen = filter_eligible(snapshot_, {}, request_.target_ids, config_.selection);
  for (const auto & id : seen.eligible) {
    if (discovered_set_.insert(id).second) {
      discovered_ids_.push_back(id);
    }
  }
  counts_.discovered = static_cast<uint32_t>(discovered_ids_.size());
  bump();
}

std::vector<std::string> BatchSession::reach_query_ids() const
{
  if (!has_snapshot_) {
    return {};
  }
  return filter_eligible(snapshot_, claimed_, request_.target_ids, config_.selection).eligible;
}

std::string BatchSession::select(const ReachMap & reach)
{
  if (!has_snapshot_) {
    last_filtered_.clear();
    return "";
  }
  const Selection sel =
    select_next(snapshot_, claimed_, request_.target_ids, reach, config_.selection);
  last_filtered_ = sel.filtered;
  for (const auto & [id, r] : sel.unreachable) {
    unreachable_[id] = r;
  }
  for (const auto & [id, r] : reach) {
    reachability_[id] =
      ReachabilityRecord{id, r.reachable, r.failure_code, failure_name(r.failure_code), r.reason};
  }
  bump();
  if (sel.target_id.empty()) {
    if (!reach.empty()) {
      write_ledger();
    }
    return "";
  }
  start_target(sel.target_id);
  return sel.target_id;
}

void BatchSession::start_target(const std::string & target_id)
{
  claimed_.insert(target_id);
  current_target_ = target_id;
  attempts_ = 0;
  failure_ = Failure{};
  model_revision_ = 0;
  last_decision_.reset();
  last_harvest_.reset();
  target_plan_only_ = false;
  neck_phase_ = false;
  neck_remeasured_ = false;
  target_started_s_ = now();
  target_started_wall_ = wall_clock_();
  deadline_.start(target_started_s_, request_.limits.per_target_timeout_s);
  empty_rounds_ = 0;
  bump();
}

bool BatchSession::on_empty_round()
{
  if (!abort_reason_.empty()) {
    return true;
  }
  ++empty_rounds_;
  bump();
  if (empty_limit_reached(empty_rounds_, request_.limits)) {
    settle("no_targets");
    return true;
  }
  return false;
}

bool BatchSession::gate_allows_next()
{
  if (!abort_reason_.empty() || settled_) {
    return false;
  }
  if (!request_.target_ids.empty()) {
    const bool all_done = std::all_of(
      request_.target_ids.begin(), request_.target_ids.end(),
      [this](const std::string & id) {return claimed_.count(id) != 0;});
    if (all_done) {
      settle("target_list_done");
      return false;
    }
  }
  const Gate gate = evaluate_gate(request_.limits, counts_);
  if (gate != Gate::CONTINUE) {
    settle(gate_reason(gate));
    return false;
  }
  return true;
}

void BatchSession::begin_attempt()
{
  neck_phase_ = attempts_ > 0 && failure_.policy() == Policy::REMEASURE_NECK;
  neck_remeasured_ = neck_phase_;
  ++attempts_;
  failure_ = Failure{};
  last_harvest_.reset();
  bump();
}

bool BatchSession::target_deadline_exceeded() const
{
  return !current_target_.empty() && deadline_.exceeded(now());
}

void BatchSession::set_failure(Failure failure)
{
  failure_ = std::move(failure);
  bump();
}

bool BatchSession::on_observe(const ObserveOutcome & observed)
{
  model_revision_ = observed.model_revision;
  if (observed.converged && observed.failure_code == fc::NONE) {
    bump();
    return true;
  }
  Failure f;
  f.code = observed.failure_code != fc::NONE ? observed.failure_code : fc::MODEL_NOT_CONVERGED;
  f.reason = observed.message.empty() ? "observe:" + failure_name(f.code) : observed.message;
  set_failure(std::move(f));
  return false;
}

bool BatchSession::on_decision(const DecisionOutcome & decision, DecisionLevel level)
{
  last_decision_ = decision;
  if (decision.model_revision > model_revision_) {
    model_revision_ = decision.model_revision;
  }
  Failure f;
  if (!decision.found) {
    f.code = fc::MODEL_STALE;
    f.reason = "decision_not_found";
  } else if (decision.expired) {
    f.code = fc::MODEL_EXPIRED;
    f.reason = "decision_expired";
  } else {
    bool allowed = false;
    uint32_t fallback = fc::NONE;
    switch (level) {
      case DecisionLevel::APPROACH:
        allowed = decision.approach_allowed;
        fallback = fc::TOOL_NOT_FEASIBLE;
        break;
      case DecisionLevel::SLEEVE:
        allowed = decision.approach_allowed && decision.sleeve_allowed;
        fallback = fc::BUDGET_RADIAL_NEGATIVE;
        break;
      case DecisionLevel::CUT:
        allowed = decision.approach_allowed && decision.sleeve_allowed && decision.cut_allowed;
        fallback = fc::BUDGET_AXIAL_NEGATIVE;
        break;
    }
    if (allowed) {
      bump();
      return true;
    }
    f.code = decision.failure_code != fc::NONE ? decision.failure_code : fallback;
    f.reason = decision.reason.empty() ? "decision:" + failure_name(f.code) : decision.reason;
  }
  set_failure(std::move(f));
  return false;
}

bool BatchSession::on_harvest(const HarvestOutcome & result)
{
  last_harvest_ = result;
  target_plan_only_ = target_plan_only_ || result.plan_only;
  if (result.outcome == outcome::SUCCEEDED && result.failure_code == fc::NONE) {
    bump();
    return true;
  }
  Failure f;
  f.code = result.failure_code;
  if (f.code == fc::NONE) {
    if (result.outcome == outcome::CANCELED) {
      f.code = fc::CANCELED;
    } else if (result.outcome != outcome::SKIPPED) {
      f.code = fc::EXEC_FAILED;
    }
  }
  f.reason = result.reason.empty() ? "harvest:" + failure_name(f.code) : result.reason;
  f.recovery_required = result.recovery_required;
  set_failure(std::move(f));
  return false;
}

RetryVerdict BatchSession::retry_verdict(uint32_t attempts_done, uint32_t max_attempts) const
{
  if (recovery_required_) {
    return RetryVerdict::GIVE_UP;
  }
  const Policy policy = failure_.policy();
  if (policy == Policy::REMEASURE_NECK) {
    // The arm waits at pregrasp: neither the attempt cap nor the observe+decision budget
    // applies to the one re-measure that follows a normal attempt.
    const bool allowed = request_.intent == Intent::FULL && !neck_remeasured_;
    return allowed ? RetryVerdict::RETRY_NOW : RetryVerdict::GIVE_UP;
  }
  if (attempts_done >= max_attempts || target_deadline_exceeded()) {
    return RetryVerdict::GIVE_UP;
  }
  switch (policy) {
    case Policy::RETRY_VIEW: return RetryVerdict::RETRY_NOW;
    case Policy::WAIT: return RetryVerdict::RETRY_AFTER_WAIT;
    default: return RetryVerdict::GIVE_UP;
  }
}

TargetRecord BatchSession::base_record() const
{
  TargetRecord r;
  r.target_id = current_target_;
  r.tool_id = request_.tool_id;
  r.attempts = attempts_;
  r.model_revision = model_revision_;
  r.cycle_time_s = now() - target_started_s_;
  r.started_utc = utc_iso8601(target_started_wall_);
  r.finished_utc = utc_iso8601(wall_clock_());
  if (last_decision_) {
    r.radial_margin_m = last_decision_->radial_margin_m;
    r.axial_margin_m = last_decision_->axial_margin_m;
  }
  if (last_harvest_) {
    r.reached = reached_name(last_harvest_->reached);
    r.stage_names = last_harvest_->stage_names;
    r.stage_times_s = last_harvest_->stage_times_s;
    if (last_harvest_->radial_margin_m != 0.0 || last_harvest_->axial_margin_m != 0.0) {
      r.radial_margin_m = last_harvest_->radial_margin_m;
      r.axial_margin_m = last_harvest_->axial_margin_m;
    }
  } else {
    r.reached = reached_name(reached::NONE);
  }
  return r;
}

void BatchSession::close_target(TargetRecord record, HarvestOutcome result)
{
  record.plan_only = target_plan_only_;
  result.plan_only = target_plan_only_;
  if (target_plan_only_) {
    ++counts_.plan_only;
    rework_.push_back(
      ReworkEntry{current_target_, "plan_only",
        record.reason.empty() ? "planned_not_executed" : record.reason, record.failure_code,
        false});
  } else {
    ++counts_.attempted;
  }
  records_.push_back(std::move(record));
  results_.push_back(std::move(result));
  current_target_.clear();
  deadline_.clear();
  bump();
  write_ledger();
}

void BatchSession::record_success()
{
  if (current_target_.empty()) {
    return;
  }
  TargetRecord r = base_record();
  r.outcome = outcome_name(outcome::SUCCEEDED);
  r.failure_name = failure_name(fc::NONE);
  r.policy = policy_name(Policy::NONE);
  HarvestOutcome result = last_harvest_.value_or(HarvestOutcome{});
  result.target_id = current_target_;
  result.tool_id = request_.tool_id;
  result.outcome = outcome::SUCCEEDED;
  r.recovery_required = result.recovery_required;
  const bool plan_only = target_plan_only_;
  if (!plan_only) {
    ++counts_.succeeded;
  }
  const bool manipulation_ack = result.recovery_required;
  close_target(std::move(r), std::move(result));
  if (manipulation_ack) {
    require_recovery("manipulation_requested_ack");
  } else if (!plan_only && request_.intent == Intent::PREGRASP_ONLY && config_.ack_each_pregrasp) {
    require_recovery("pregrasp_checkpoint");
  }
}

bool BatchSession::record_skip()
{
  if (current_target_.empty()) {
    return abort_reason_.empty();
  }
  Failure f = failure_;
  if (!f.active()) {
    f.reason = "unspecified_failure";
  }
  const Policy policy = f.policy();
  uint8_t out = outcome::SKIPPED;
  if (f.code == fc::CANCELED) {
    out = outcome::CANCELED;
  } else if (policy == Policy::RECOVER || policy == Policy::STOP_BATCH) {
    out = outcome::FAILED;
  }
  // Nothing moved on a plan-only target: only an explicit manipulation request needs an ACK.
  const bool plan_only = target_plan_only_;
  const bool needs_ack = plan_only ? f.recovery_required : policy == Policy::RECOVER;
  TargetRecord r = base_record();
  r.outcome = outcome_name(out);
  r.failure_code = f.code;
  r.failure_name = failure_name(f.code);
  r.policy = policy_name(policy);
  r.reason = f.reason;
  r.recovery_required = needs_ack;
  r.rework_kind = f.rework_override.empty() ? rework_kind(f.code, policy) : f.rework_override;

  HarvestOutcome result;
  if (last_harvest_) {
    result = *last_harvest_;
  } else {
    result.reached = reached::NONE;
    result.radial_margin_m = r.radial_margin_m;
    result.axial_margin_m = r.axial_margin_m;
  }
  result.target_id = current_target_;
  result.tool_id = request_.tool_id;
  result.outcome = out;
  result.failure_code = f.code;
  result.reason = f.reason;
  result.recovery_required = r.recovery_required;
  result.cycle_time_s = r.cycle_time_s;

  if (!plan_only) {
    rework_.push_back(ReworkEntry{current_target_, r.rework_kind, f.reason, f.code, true});
    if (out == outcome::FAILED) {
      ++counts_.failed;
    } else {
      ++counts_.skipped;
    }
  }
  if (policy == Policy::STOP_BATCH) {
    std::string why = "stop_batch";
    if (f.code != fc::NONE) {
      why += ":" + failure_name(f.code);
    }
    if (!f.reason.empty()) {
      why += ":" + f.reason;
    }
    abort(why);
  }
  close_target(std::move(r), std::move(result));
  if (needs_ack) {
    require_recovery(f.reason.empty() ? failure_name(f.code) : f.reason);
  }
  return abort_reason_.empty();
}

void BatchSession::require_recovery(const std::string & reason)
{
  recovery_required_ = true;
  ack_granted_ = false;
  recovery_reason_ = reason;
  bump();
}

bool BatchSession::grant_ack()
{
  if (!recovery_required_) {
    return false;
  }
  ack_granted_ = true;
  bump();
  return true;
}

bool BatchSession::consume_ack()
{
  if (!recovery_required_) {
    return true;
  }
  if (!ack_granted_ || peer_recovery_) {
    return false;
  }
  recovery_required_ = false;
  ack_granted_ = false;
  recovery_reason_.clear();
  bump();
  return true;
}

void BatchSession::on_peer_recovery(bool required)
{
  if (required == peer_recovery_) {
    return;
  }
  peer_recovery_ = required;
  bump();
  if (required && active_ && !recovery_required_) {
    require_recovery("manipulation_recovery_required");
  }
}

void BatchSession::settle(const std::string & reason)
{
  if (!settled_ && abort_reason_.empty()) {
    settled_ = true;
    settle_reason_ = reason;
    bump();
  }
}

void BatchSession::abort(const std::string & reason)
{
  if (abort_reason_.empty() && !settled_) {
    abort_reason_ = reason;
    bump();
  }
}

std::string BatchSession::termination_reason() const
{
  if (!termination_reason_.empty()) {
    return termination_reason_;
  }
  if (settled()) {
    return settle_reason_;
  }
  if (!abort_reason_.empty()) {
    return abort_reason_;
  }
  if (failure_.active()) {
    return "failed:" + (failure_.reason.empty() ? failure_name(failure_.code) : failure_.reason);
  }
  return "";
}

std::vector<ReworkEntry> BatchSession::final_rework(BatchEnd end) const
{
  std::vector<ReworkEntry> out = rework_;
  std::set<std::string> listed;
  for (const auto & e : out) {
    listed.insert(e.target_id);
  }
  auto add_unattempted = [&](const std::string & id, bool observed) {
      if (claimed_.count(id) != 0 || !listed.insert(id).second) {
        return;
      }
      ReworkEntry e;
      e.target_id = id;
      e.attempted = false;
      const auto unreach = unreachable_.find(id);
      const auto filtered = last_filtered_.find(id);
      if (!observed) {
        e.kind = "not_observed";
        e.reason = "target_list id never eligible in a locked set";
      } else if (unreach != unreachable_.end()) {
        e.kind = "unreachable";
        e.failure_code = unreach->second.failure_code;
        e.reason = "check_reachability:" + failure_name(unreach->second.failure_code);
        if (!unreach->second.reason.empty()) {
          e.reason += ":" + unreach->second.reason;
        }
      } else if (end == BatchEnd::SUCCEEDED && settle_reason_ == "ratio_reached") {
        e.kind = "ratio_satisfied";
        e.reason = settle_reason_;
      } else if (end == BatchEnd::SUCCEEDED && settle_reason_ == "max_targets") {
        e.kind = "max_targets";
        e.reason = settle_reason_;
      } else {
        e.kind = "not_attempted";
        e.reason = filtered != last_filtered_.end() ? filtered->second : termination_reason_;
      }
      out.push_back(std::move(e));
    };
  for (const auto & id : discovered_ids_) {
    add_unattempted(id, true);
  }
  for (const auto & id : request_.target_ids) {
    add_unattempted(id, discovered_set_.count(id) != 0);
  }
  return out;
}

std::string BatchSession::finish(BatchEnd end)
{
  if (!active_) {
    return termination_reason_;
  }
  if (end == BatchEnd::CANCELED) {
    abort("canceled");
  }
  if (!current_target_.empty()) {
    if (end == BatchEnd::CANCELED) {
      failure_ = Failure{fc::CANCELED, "batch_canceled", false, {}, {}};
    } else if (!failure_.active()) {
      failure_ = Failure{
        fc::NONE, abort_reason_.empty() ? "batch_aborted" : abort_reason_, false,
        Policy::STOP_BATCH, "not_finished"};
    }
    // record_skip may abort with the target's own reason; keep the batch-level one first.
    const std::string abort_before = abort_reason_;
    record_skip();
    if (!abort_before.empty()) {
      abort_reason_ = abort_before;
    }
  }
  if (end == BatchEnd::CANCELED) {
    termination_reason_ = "canceled";
  } else if (end == BatchEnd::SUCCEEDED) {
    termination_reason_ = settled() ? settle_reason_ : "completed";
  } else {
    termination_reason_ = termination_reason();
    if (termination_reason_.empty()) {
      termination_reason_ = "tree_failure";
    }
  }
  phase_ = end == BatchEnd::SUCCEEDED ? Phase::COMPLETED : Phase::ABORTED;
  finished_utc_ = utc_iso8601(wall_clock_());
  rework_ = final_rework(end);
  if (ledger_) {
    std::string error;
    if (!ledger_->write_rework(request_.request_id, rework_, &error)) {
      ++ledger_failures_;
      last_ledger_error_ = error;
    }
  }
  write_ledger();
  active_ = false;
  bump();
  return termination_reason_;
}

LedgerDoc BatchSession::ledger_doc() const
{
  LedgerDoc doc;
  doc.request_id = request_.request_id;
  doc.intent = intent_name(request_.intent);
  doc.tool_id = request_.tool_id;
  doc.target_list = request_.target_ids;
  doc.limits = request_.limits;
  doc.scene_epoch = scene_epoch_;
  doc.started_utc = started_utc_;
  doc.finished_utc = finished_utc_;
  doc.termination_reason = finished_utc_.empty() ? "" : termination_reason_;
  doc.counts = counts_;
  doc.discovered_ids = discovered_ids_;
  for (const auto & [id, record] : reachability_) {
    doc.reachability.push_back(record);
  }
  doc.targets = records_;
  return doc;
}

void BatchSession::write_ledger()
{
  if (!ledger_) {
    return;
  }
  std::string error;
  if (!ledger_->write(ledger_doc(), &error)) {
    ++ledger_failures_;
    last_ledger_error_ = error;
    bump();
  }
}

}  // namespace peach2_task::core
