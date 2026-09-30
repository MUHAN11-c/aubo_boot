// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
#include "peach2_task/bt/logic_nodes.hpp"

#include <utility>

namespace peach2_task::bt
{

using BT::NodeStatus;

IntentIs::IntentIs(const std::string & name, const BT::NodeConfig & config, SessionPtr session)
: BT::ConditionNode(name, config), session_(std::move(session)) {}

BT::PortsList IntentIs::providedPorts()
{
  return {BT::InputPort<std::string>("intent", "SURVEY_ONLY | PREGRASP_ONLY | FULL")};
}

NodeStatus IntentIs::tick()
{
  const auto name = getInput<std::string>("intent");
  core::Intent wanted;
  if (!name || !core::intent_from_name(name.value(), &wanted)) {
    throw BT::RuntimeError("IntentIs: invalid port 'intent'");
  }
  return session_->request().intent == wanted ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

SettleBatch::SettleBatch(
  const std::string & name, const BT::NodeConfig & config, SessionPtr session)
: BT::SyncActionNode(name, config), session_(std::move(session)) {}

BT::PortsList SettleBatch::providedPorts()
{
  return {BT::InputPort<std::string>("reason")};
}

NodeStatus SettleBatch::tick()
{
  session_->settle(getInput<std::string>("reason").value_or("completed"));
  return NodeStatus::SUCCESS;
}

BatchGate::BatchGate(const std::string & name, const BT::NodeConfig & config, SessionPtr session)
: BT::ConditionNode(name, config), session_(std::move(session)) {}

NodeStatus BatchGate::tick()
{
  return session_->gate_allows_next() ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NoTargetRound::NoTargetRound(
  const std::string & name, const BT::NodeConfig & config, SessionPtr session)
: BT::SyncActionNode(name, config), session_(std::move(session)) {}

NodeStatus NoTargetRound::tick()
{
  return session_->on_empty_round() ? NodeStatus::FAILURE : NodeStatus::SUCCESS;
}

RecordResult::RecordResult(
  const std::string & name, const BT::NodeConfig & config, SessionPtr session)
: BT::SyncActionNode(name, config), session_(std::move(session)) {}

BT::PortsList RecordResult::providedPorts()
{
  return {BT::InputPort<std::string>("target_id")};
}

NodeStatus RecordResult::tick()
{
  const auto tid = getInput<std::string>("target_id");
  if (tid && tid.value() != session_->current_target()) {
    throw BT::RuntimeError(
      "RecordResult: target_id '" + tid.value() + "' != current '" +
      session_->current_target() + "'");
  }
  session_->record_success();
  return NodeStatus::SUCCESS;
}

RecordSkip::RecordSkip(
  const std::string & name, const BT::NodeConfig & config, SessionPtr session)
: BT::SyncActionNode(name, config), session_(std::move(session)) {}

BT::PortsList RecordSkip::providedPorts()
{
  return {BT::InputPort<std::string>("target_id")};
}

NodeStatus RecordSkip::tick()
{
  const auto tid = getInput<std::string>("target_id");
  if (tid && tid.value() != session_->current_target()) {
    throw BT::RuntimeError(
      "RecordSkip: target_id '" + tid.value() + "' != current '" +
      session_->current_target() + "'");
  }
  return session_->record_skip() ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

IsRecoveryRequired::IsRecoveryRequired(
  const std::string & name, const BT::NodeConfig & config, SessionPtr session)
: BT::ConditionNode(name, config), session_(std::move(session)) {}

NodeStatus IsRecoveryRequired::tick()
{
  return session_->recovery_required() ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

WaitForAck::WaitForAck(
  const std::string & name, const BT::NodeConfig & config, SessionPtr session)
: BT::StatefulActionNode(name, config), session_(std::move(session)) {}

NodeStatus WaitForAck::onStart()
{
  session_->set_phase(core::Phase::WAITING_ACK);
  session_->set_message("waiting for operator ACK: " + session_->recovery_reason());
  return onRunning();
}

NodeStatus WaitForAck::onRunning()
{
  if (!session_->consume_ack()) {
    if (session_->ack_granted() && session_->peer_recovery_required()) {
      session_->set_message("ACK granted; waiting for manipulation recovery_required=false");
    }
    return NodeStatus::RUNNING;
  }
  session_->set_message("ACK received");
  return NodeStatus::SUCCESS;
}

BatchSettled::BatchSettled(
  const std::string & name, const BT::NodeConfig & config, SessionPtr session)
: BT::ConditionNode(name, config), session_(std::move(session)) {}

NodeStatus BatchSettled::tick()
{
  return session_->settled() ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

RetryOnPolicy::RetryOnPolicy(
  const std::string & name, const BT::NodeConfig & config, SessionPtr session)
: BT::DecoratorNode(name, config), session_(std::move(session)) {}

BT::PortsList RetryOnPolicy::providedPorts()
{
  return {BT::InputPort<unsigned>("num_attempts", 2U, "total attempts (2 = retry once)")};
}

void RetryOnPolicy::halt()
{
  attempts_ = 0;
  waiting_ = false;
  BT::DecoratorNode::halt();
}

NodeStatus RetryOnPolicy::tick()
{
  const auto max_attempts = getInput<unsigned>("num_attempts");
  if (!max_attempts || max_attempts.value() < 1) {
    throw BT::RuntimeError("RetryOnPolicy: invalid port 'num_attempts'");
  }
  if (status() == NodeStatus::IDLE) {
    attempts_ = 0;
    waiting_ = false;
  }
  setStatus(NodeStatus::RUNNING);
  if (waiting_) {
    if (session_->now() < wait_until_s_) {
      return NodeStatus::RUNNING;
    }
    waiting_ = false;
  }
  if (child_node_->status() == NodeStatus::IDLE) {
    session_->begin_attempt();
    if (!session_->neck_remeasure_pending()) {
      ++attempts_;
    }
  }
  const NodeStatus child = child_node_->executeTick();
  switch (child) {
    case NodeStatus::RUNNING:
      return NodeStatus::RUNNING;
    case NodeStatus::FAILURE: {
        resetChild();
        const auto verdict = session_->retry_verdict(attempts_, max_attempts.value());
        if (verdict == core::RetryVerdict::GIVE_UP) {
          attempts_ = 0;
          return NodeStatus::FAILURE;
        }
        if (verdict == core::RetryVerdict::RETRY_AFTER_WAIT) {
          waiting_ = true;
          wait_until_s_ = session_->now() + session_->config().wait_retry_s;
          session_->set_message("waiting before retry: " + session_->failure().reason);
        }
        return NodeStatus::RUNNING;
      }
    default:
      resetChild();
      attempts_ = 0;
      return child;
  }
}

WithinTargetDeadline::WithinTargetDeadline(
  const std::string & name, const BT::NodeConfig & config, SessionPtr session)
: BT::DecoratorNode(name, config), session_(std::move(session)) {}

NodeStatus WithinTargetDeadline::tick()
{
  setStatus(NodeStatus::RUNNING);
  if (session_->target_deadline_exceeded()) {
    haltChild();
    session_->set_failure(core::Failure{core::fc::TARGET_TIMEOUT, "per_target_timeout", false,
        {}, {}});
    return NodeStatus::FAILURE;
  }
  const NodeStatus child = child_node_->executeTick();
  if (child != NodeStatus::RUNNING) {
    resetChild();
  }
  return child;
}

NeckRemeasurePending::NeckRemeasurePending(
  const std::string & name, const BT::NodeConfig & config, SessionPtr session)
: BT::ConditionNode(name, config), session_(std::move(session)) {}

NodeStatus NeckRemeasurePending::tick()
{
  return session_->neck_remeasure_pending() ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NoRecoveryPending::NoRecoveryPending(
  const std::string & name, const BT::NodeConfig & config, SessionPtr session)
: BT::ConditionNode(name, config), session_(std::move(session)) {}

NodeStatus NoRecoveryPending::tick()
{
  if (!session_->recovery_required()) {
    return NodeStatus::SUCCESS;
  }
  session_->set_failure(
    core::Failure{core::fc::RECOVERY_REQUIRED, "recovery_pending:" + session_->recovery_reason(),
      false, {}, {}});
  return NodeStatus::FAILURE;
}

void register_logic_nodes(BT::BehaviorTreeFactory & factory, const SessionPtr & session)
{
  factory.registerNodeType<NeckRemeasurePending>("NeckRemeasurePending", session);
  factory.registerNodeType<NoRecoveryPending>("NoRecoveryPending", session);
  factory.registerNodeType<IntentIs>("IntentIs", session);
  factory.registerNodeType<SettleBatch>("SettleBatch", session);
  factory.registerNodeType<BatchGate>("BatchGate", session);
  factory.registerNodeType<NoTargetRound>("NoTargetRound", session);
  factory.registerNodeType<RecordResult>("RecordResult", session);
  factory.registerNodeType<RecordSkip>("RecordSkip", session);
  factory.registerNodeType<IsRecoveryRequired>("IsRecoveryRequired", session);
  factory.registerNodeType<WaitForAck>("WaitForAck", session);
  factory.registerNodeType<BatchSettled>("BatchSettled", session);
  factory.registerNodeType<RetryOnPolicy>("RetryOnPolicy", session);
  factory.registerNodeType<WithinTargetDeadline>("WithinTargetDeadline", session);
}

}  // namespace peach2_task::bt
