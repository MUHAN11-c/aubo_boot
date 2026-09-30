// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
#include "peach2_task/ros/ros_nodes.hpp"

#include <string>
#include <utility>

#include "peach2_task/bt/ports.hpp"
#include "peach2_task/ros/conversions.hpp"

namespace peach2_task::ros
{

using BT::NodeStatus;
namespace fc = core::fc;

const char * leaf_error_name(LeafError error)
{
  switch (error) {
    case LeafError::SERVER_UNAVAILABLE: return "server_unavailable";
    case LeafError::REJECTED: return "rejected";
    case LeafError::TIMEOUT: return "timeout";
    case LeafError::NO_RESULT: return "no_result";
    case LeafError::BAD_REQUEST: return "bad_request";
  }
  return "unknown";
}

namespace
{

std::string describe(const std::string & what, LeafError error, const std::string & detail)
{
  std::string out = what + ":" + leaf_error_name(error);
  if (!detail.empty()) {
    out += ":" + detail;
  }
  return out;
}

/// A peer server that is missing or refuses to talk takes every later target down with it.
core::Failure dependency_failure(const std::string & reason)
{
  return core::Failure{fc::DEPENDENCY_UNAVAILABLE, reason, false, {}, {}};
}

/// Survey has no current target: any failure there ends the batch.
void fail_survey(
  const SessionPtr & session, const rclcpp::Logger & logger, const std::string & reason)
{
  RCLCPP_ERROR(logger, "survey failed: %s", reason.c_str());
  session->abort("survey_failed:" + reason);
}

template<typename T>
T require_input(const BT::TreeNode & node, const std::string & key)
{
  auto value = node.getInput<T>(key);
  if (!value) {
    throw BT::RuntimeError(node.name() + ": missing port '" + key + "': " + value.error());
  }
  return value.value();
}

void require_current_target(
  const SessionPtr & session, const std::string & tid, const std::string & who)
{
  if (tid.empty() || tid != session->current_target()) {
    throw BT::RuntimeError(
      who + ": target_id '" + tid + "' != current target '" + session->current_target() + "'");
  }
}

}  // namespace

// ---------------------------------------------------------------- CheckSafety

CheckSafety::CheckSafety(
  const std::string & name, const BT::NodeConfig & config, RosContextPtr ctx,
  SessionPtr session)
: BT::ConditionNode(name, config), ctx_(std::move(ctx)), session_(std::move(session)) {}

NodeStatus CheckSafety::tick()
{
  core::SafetyInputs in;
  in.robot = ctx_->robot_sample(SteadyClock::now());
  in.enables = ctx_->enables;
  in.intent = session_->request().intent;
  in.tool_fault = ctx_->tool_fault();
  const core::SafetyVerdict verdict = core::evaluate_safety(ctx_->safety, in);
  session_->set_safety_blockers(verdict.blockers);
  if (verdict.ok) {
    return NodeStatus::SUCCESS;
  }
  const std::string reason = "safety:" + verdict.reason();
  RCLCPP_WARN(ctx_->logger, "CheckSafety failed, aborting batch: %s", reason.c_str());
  session_->set_failure(core::Failure{verdict.failure_code, reason, false, {}, {}});
  session_->abort(reason);
  return NodeStatus::FAILURE;
}

// ---------------------------------------------------------------- MoveToNamed

MoveToNamed::MoveToNamed(
  const std::string & name, const BT::NodeConfig & config, RosContextPtr ctx,
  SessionPtr session)
: AsyncActionLeaf(name, config, ctx, ctx->move_to, ctx->timeouts.move_to_s),
  session_(std::move(session)) {}

void MoveToNamed::on_started()
{
  target_ = require_input<std::string>(*this, "target");
  session_->set_phase(core::Phase::SURVEYING);
  session_->set_message("moving to " + target_);
}

bool MoveToNamed::make_goal(Goal * goal)
{
  if (target_.empty()) {
    on_error(LeafError::BAD_REQUEST, "empty named target");
    return false;
  }
  goal->named_target = target_;
  goal->velocity_scaling = getInput<double>("velocity_scaling").value_or(0.0);
  return true;
}

NodeStatus MoveToNamed::on_result(const WrappedResult & result)
{
  if (!result.result) {
    on_error(LeafError::NO_RESULT, "");
    return NodeStatus::FAILURE;
  }
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success) {
    return NodeStatus::SUCCESS;
  }
  uint32_t code = result.result->failure_code;
  if (code == fc::NONE) {
    code = result.code == rclcpp_action::ResultCode::CANCELED ? fc::CANCELED : fc::EXEC_FAILED;
  }
  const std::string reason =
    "move_to " + target_ + ":" + core::failure_name(code) + ":" + result.result->message;
  session_->set_failure(core::Failure{code, reason, false, {}, {}});
  fail_survey(session_, ctx_->logger, reason);
  return NodeStatus::FAILURE;
}

void MoveToNamed::on_error(LeafError error, const std::string & detail)
{
  const std::string reason = describe("move_to " + target_, error, detail);
  if (error == LeafError::TIMEOUT) {
    session_->set_failure(core::Failure{fc::EXEC_TIMEOUT, reason, false, {}, {}});
  } else {
    session_->set_failure(dependency_failure(reason));
  }
  fail_survey(session_, ctx_->logger, reason);
}

// ---------------------------------------------------------------- BuildSceneSnapshot

BuildSceneSnapshot::BuildSceneSnapshot(
  const std::string & name, const BT::NodeConfig & config, RosContextPtr ctx,
  SessionPtr session)
: AsyncServiceLeaf(name, config, ctx, ctx->snapshot, ctx->timeouts.snapshot_s),
  session_(std::move(session)) {}

void BuildSceneSnapshot::on_started()
{
  session_->set_phase(core::Phase::SURVEYING);
  session_->set_message("building scene snapshot");
}

bool BuildSceneSnapshot::make_request(Request * request)
{
  request->request_id = session_->request().request_id;
  request->clear_previous = getInput<bool>("clear_previous").value_or(true);
  return true;
}

NodeStatus BuildSceneSnapshot::on_response(const Response & response)
{
  if (!response.success) {
    fail_survey(session_, ctx_->logger, "scene_snapshot:" + response.message);
    return NodeStatus::FAILURE;
  }
  RCLCPP_INFO(
    ctx_->logger,
    "scene snapshot: %u hard objects%s, %u soft voxels, %u frames, frame age %.2f s",
    response.n_hard_objects, response.truncated ? " (truncated)" : "", response.n_soft_voxels,
    response.n_frames, response.frame_age_s);
  return NodeStatus::SUCCESS;
}

NodeStatus BuildSceneSnapshot::on_error(LeafError error, const std::string & detail)
{
  const std::string reason = describe("scene_snapshot", error, detail);
  session_->set_failure(dependency_failure(reason));
  fail_survey(session_, ctx_->logger, reason);
  return NodeStatus::FAILURE;
}

// ---------------------------------------------------------------- BeginScene

BeginScene::BeginScene(
  const std::string & name, const BT::NodeConfig & config, RosContextPtr ctx,
  SessionPtr session)
: AsyncServiceLeaf(name, config, ctx, ctx->begin_scene, ctx->timeouts.begin_scene_s),
  session_(std::move(session)) {}

void BeginScene::on_started()
{
  session_->set_phase(core::Phase::SURVEYING);
  session_->set_message("beginning scene");
}

bool BeginScene::make_request(Request * request)
{
  request->request_id = session_->request().request_id;
  return true;
}

NodeStatus BeginScene::on_response(const Response & response)
{
  if (!response.accepted) {
    fail_survey(session_, ctx_->logger, "begin_scene:rejected:" + response.message);
    return NodeStatus::FAILURE;
  }
  session_->on_scene_begun(response.scene_epoch);
  RCLCPP_INFO(ctx_->logger, "scene begun: epoch %u", response.scene_epoch);
  return NodeStatus::SUCCESS;
}

NodeStatus BeginScene::on_error(LeafError error, const std::string & detail)
{
  const std::string reason = describe("begin_scene", error, detail);
  session_->set_failure(dependency_failure(reason));
  fail_survey(session_, ctx_->logger, reason);
  return NodeStatus::FAILURE;
}

// ---------------------------------------------------------------- WaitTargetSetLocked

WaitTargetSetLocked::WaitTargetSetLocked(
  const std::string & name, const BT::NodeConfig & config, RosContextPtr ctx,
  SessionPtr session)
: BT::StatefulActionNode(name, config), ctx_(std::move(ctx)), session_(std::move(session)) {}

NodeStatus WaitTargetSetLocked::onStart()
{
  started_stamp_ = ctx_->ros_clock->now();
  started_at_ = SteadyClock::now();
  session_->set_phase(core::Phase::SURVEYING);
  session_->set_message("waiting for target_set_locked");
  return onRunning();
}

NodeStatus WaitTargetSetLocked::onRunning()
{
  const auto & obs = ctx_->observations;
  if (obs && session_->scene_begun() && obs->scene_epoch > session_->scene_epoch()) {
    // Someone else began a scene: the claimed target ids of this batch no longer exist.
    fail_survey(
      session_, ctx_->logger, "scene_epoch_changed:" + std::to_string(obs->scene_epoch) +
      "!=" + std::to_string(session_->scene_epoch()));
    return NodeStatus::FAILURE;
  }
  const bool epoch_ok =
    obs && (!session_->scene_begun() || obs->scene_epoch == session_->scene_epoch());
  if (epoch_ok && obs->target_set_locked) {
    const rclcpp::Time stamp(obs->header.stamp, started_stamp_.get_clock_type());
    const bool fresh = stamp.nanoseconds() != 0 ?
      stamp >= started_stamp_ : ctx_->observations_received_at >= started_at_;
    if (fresh) {
      session_->on_locked_snapshot(to_observation_set(*obs));
      RCLCPP_INFO(
        ctx_->logger, "target set locked: epoch %u, %zu locked ids, %zu observations",
        obs->scene_epoch, obs->locked_target_ids.size(), obs->observations.size());
      return NodeStatus::SUCCESS;
    }
  }
  if (seconds_since(started_at_) > ctx_->timeouts.lock_wait_s) {
    fail_survey(session_, ctx_->logger, "lock_timeout");
    return NodeStatus::FAILURE;
  }
  return NodeStatus::RUNNING;
}

// ---------------------------------------------------------------- SelectTarget

SelectTarget::SelectTarget(
  const std::string & name, const BT::NodeConfig & config, RosContextPtr ctx,
  SessionPtr session)
: AsyncServiceLeaf(name, config, ctx, ctx->reachability, ctx->timeouts.reachability_s),
  session_(std::move(session)) {}

NodeStatus SelectTarget::onStart()
{
  session_->set_phase(core::Phase::SELECTING);
  session_->set_message("selecting next target");
  if (session_->reach_query_ids().empty()) {
    return finish({});
  }
  return AsyncServiceLeaf::onStart();
}

bool SelectTarget::make_request(Request * request)
{
  request->target_ids = session_->reach_query_ids();
  request->tool_id = session_->request().tool_id;
  request->mode = reachability_mode_for(session_->request().intent);
  return true;
}

NodeStatus SelectTarget::on_response(const Response & response)
{
  core::ReachMap reach;
  if (!to_reach_map(response, &reach)) {
    return on_error(LeafError::NO_RESULT, "response arrays differ in length");
  }
  for (const auto & [id, r] : reach) {
    if (!r.reachable) {
      RCLCPP_INFO(
        ctx_->logger, "unreachable %s: %s %s", id.c_str(),
        core::failure_name(r.failure_code).c_str(), r.reason.c_str());
    }
  }
  return finish(reach);
}

NodeStatus SelectTarget::on_error(LeafError error, const std::string & detail)
{
  const std::string reason = describe("check_reachability", error, detail);
  if (session_->config().selection.require_reachability) {
    RCLCPP_ERROR(ctx_->logger, "%s: aborting batch (reachability required)", reason.c_str());
    session_->abort(core::failure_name(fc::DEPENDENCY_UNAVAILABLE) + ":" + reason);
    return NodeStatus::FAILURE;
  }
  RCLCPP_WARN(ctx_->logger, "%s: selecting without reachability", reason.c_str());
  return finish({});
}

NodeStatus SelectTarget::finish(const core::ReachMap & reach)
{
  const std::string tid = session_->select(reach);
  if (tid.empty()) {
    return NodeStatus::FAILURE;
  }
  setOutput("target_id", tid);
  RCLCPP_INFO(ctx_->logger, "selected target %s", tid.c_str());
  return NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------- ObserveTarget

ObserveTarget::ObserveTarget(
  const std::string & name, const BT::NodeConfig & config, RosContextPtr ctx,
  SessionPtr session)
: AsyncActionLeaf(name, config, ctx, ctx->observe, ctx->timeouts.observe_s),
  session_(std::move(session)) {}

void ObserveTarget::on_started()
{
  session_->set_phase(core::Phase::OBSERVING);
  session_->set_message("observing " + session_->current_target());
}

bool ObserveTarget::make_goal(Goal * goal)
{
  goal->target_id = require_input<std::string>(*this, "target_id");
  require_current_target(session_, goal->target_id, name());
  goal->max_views = require_input<unsigned>(*this, "max_views");
  goal->neck_remeasure = getInput<bool>("neck_remeasure").value_or(false);
  return true;
}

NodeStatus ObserveTarget::on_result(const WrappedResult & result)
{
  if (!result.result) {
    on_error(LeafError::NO_RESULT, "");
    return NodeStatus::FAILURE;
  }
  if (result.code == rclcpp_action::ResultCode::CANCELED) {
    session_->set_failure(core::Failure{fc::CANCELED, "observe canceled by server", false, {},
        {}});
    return NodeStatus::FAILURE;
  }
  core::ObserveOutcome observed = to_observe(*result.result);
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    observed.converged = false;
  }
  setOutput<uint64_t>("model_revision", observed.model_revision);
  return session_->on_observe(observed) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

void ObserveTarget::on_error(LeafError error, const std::string & detail)
{
  const std::string reason = describe("observe", error, detail);
  switch (error) {
    case LeafError::TIMEOUT:
      session_->set_failure(core::Failure{fc::MODEL_NOT_CONVERGED, reason, false, {}, {}});
      break;
    case LeafError::REJECTED:
      session_->set_failure(core::Failure{fc::PERCEPTION_NO_TARGET, reason, false, {}, {}});
      break;
    default:
      session_->set_failure(dependency_failure(reason));
      break;
  }
}

void ObserveTarget::on_feedback(const Feedback & fb)
{
  session_->set_message(
    "observing " + session_->current_target() + ": views=" + std::to_string(fb.n_views) +
    " sigma_lat95=" + std::to_string(fb.sigma_lateral95_m) +
    " sigma_ax95=" + std::to_string(fb.sigma_axial95_m));
}

// ---------------------------------------------------------------- CheckDecision

CheckDecision::CheckDecision(
  const std::string & name, const BT::NodeConfig & config, RosContextPtr ctx,
  SessionPtr session)
: AsyncServiceLeaf(name, config, ctx, ctx->decision, ctx->timeouts.decision_s),
  session_(std::move(session)) {}

void CheckDecision::on_started()
{
  const auto level = require_input<std::string>(*this, "level");
  if (!core::decision_level_from_name(level, &level_)) {
    throw BT::RuntimeError(name() + ": invalid level '" + level + "'");
  }
}

bool CheckDecision::make_request(Request * request)
{
  request->target_id = require_input<std::string>(*this, "target_id");
  require_current_target(session_, request->target_id, name());
  request->tool_id = require_input<std::string>(*this, "tool_id");
  request->min_model_revision = getInput<uint64_t>("min_model_revision").value_or(0U);
  return true;
}

NodeStatus CheckDecision::on_response(const Response & response)
{
  const auto decision =
    to_decision(response.found, response.decision, ctx_->ros_clock->now().nanoseconds());
  return session_->on_decision(decision, level_) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus CheckDecision::on_error(LeafError error, const std::string & detail)
{
  const std::string reason = describe("get_decision", error, detail);
  if (error == LeafError::TIMEOUT) {
    session_->set_failure(core::Failure{fc::MODEL_STALE, reason, false, {}, {}});
  } else {
    session_->set_failure(dependency_failure(reason));
  }
  return NodeStatus::FAILURE;
}

// ---------------------------------------------------------------- HarvestTarget

HarvestTarget::HarvestTarget(
  const std::string & name, const BT::NodeConfig & config, RosContextPtr ctx,
  SessionPtr session)
: AsyncActionLeaf(name, config, ctx, ctx->harvest, ctx->timeouts.harvest_s),
  session_(std::move(session)) {}

void HarvestTarget::on_started()
{
  target_id_ = require_input<std::string>(*this, "target_id");
  require_current_target(session_, target_id_, name());
  session_->set_phase(core::Phase::HARVESTING);
  session_->set_message("harvesting " + target_id_);
}

bool HarvestTarget::make_goal(Goal * goal)
{
  const int intent_value = require_input<int>(*this, "intent");
  core::Intent intent = core::Intent::PREGRASP_ONLY;
  if (intent_value < 0 || !core::intent_from_uint(static_cast<unsigned>(intent_value), &intent)) {
    throw BT::RuntimeError(name() + ": invalid intent " + std::to_string(intent_value));
  }
  goal->request_id = session_->request().request_id;
  goal->target_id = target_id_;
  goal->tool_id = require_input<std::string>(*this, "tool_id");
  goal->mode = harvest_mode_for(intent);
  return true;
}

NodeStatus HarvestTarget::on_result(const WrappedResult & result)
{
  if (!result.result) {
    on_error(LeafError::NO_RESULT, "");
    return NodeStatus::FAILURE;
  }
  core::HarvestOutcome outcome = to_outcome(result.result->result);
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED &&
    outcome.outcome == core::outcome::SUCCEEDED)
  {
    // A default-constructed result reads as OUTCOME_SUCCEEDED (0); trust the action status.
    outcome.outcome = result.code == rclcpp_action::ResultCode::CANCELED ?
      core::outcome::CANCELED : core::outcome::FAILED;
  }
  if (outcome.target_id.empty()) {
    outcome.target_id = target_id_;
  } else if (outcome.target_id != target_id_) {
    outcome.outcome = core::outcome::FAILED;
    outcome.failure_code = fc::EXEC_FAILED;
    outcome.recovery_required = true;
    outcome.reason = "result target_id '" + outcome.target_id + "' != goal '" + target_id_ + "'";
    outcome.target_id = target_id_;
  }
  return session_->on_harvest(outcome) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

void HarvestTarget::on_error(LeafError error, const std::string & detail)
{
  const std::string reason = describe("harvest", error, detail);
  switch (error) {
    case LeafError::TIMEOUT:
      // Arm state unknown after an unanswered cycle: the operator must look before continuing.
      session_->set_failure(core::Failure{fc::EXEC_TIMEOUT, reason, true, {}, {}});
      break;
    case LeafError::REJECTED:
      session_->set_failure(core::Failure{fc::SAFETY_GATE_CLOSED, reason, false, {}, {}});
      break;
    case LeafError::NO_RESULT:
      session_->set_failure(core::Failure{fc::EXEC_FAILED, reason, true, {}, {}});
      break;
    default:
      session_->set_failure(dependency_failure(reason));
      break;
  }
}

void HarvestTarget::on_feedback(const Feedback & fb)
{
  session_->set_message(
    "harvesting " + target_id_ + ": " + fb.stage + " (" + std::to_string(fb.stage_index) +
    ", " + std::to_string(fb.elapsed_s) + " s)");
}

// ---------------------------------------------------------------- registration

void register_ros_nodes(
  BT::BehaviorTreeFactory & factory, const RosContextPtr & ctx, const SessionPtr & session)
{
  factory.registerNodeType<CheckSafety>("CheckSafety", bt::ports::check_safety(), ctx, session);
  factory.registerNodeType<MoveToNamed>(
    "MoveToNamed", bt::ports::move_to_named(), ctx, session);
  factory.registerNodeType<BuildSceneSnapshot>(
    "BuildSceneSnapshot", bt::ports::build_scene_snapshot(), ctx, session);
  factory.registerNodeType<BeginScene>("BeginScene", bt::ports::begin_scene(), ctx, session);
  factory.registerNodeType<WaitTargetSetLocked>(
    "WaitTargetSetLocked", bt::ports::wait_target_set_locked(), ctx, session);
  factory.registerNodeType<SelectTarget>(
    "SelectTarget", bt::ports::select_target(), ctx, session);
  factory.registerNodeType<ObserveTarget>(
    "ObserveTarget", bt::ports::observe_target(), ctx, session);
  factory.registerNodeType<CheckDecision>(
    "CheckDecision", bt::ports::check_decision(), ctx, session);
  factory.registerNodeType<HarvestTarget>(
    "HarvestTarget", bt::ports::harvest_target(), ctx, session);
}

}  // namespace peach2_task::ros
