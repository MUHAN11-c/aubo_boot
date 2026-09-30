// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/decorator_node.h"
#include "peach2_task/core/batch_session.hpp"

/// BT nodes whose behavior is pure batch logic (no ROS): they only read/write the shared
/// BatchSession. Registered identically in the node and in the tree tests.
namespace peach2_task::bt
{

using SessionPtr = std::shared_ptr<core::BatchSession>;

/// SUCCESS if the batch intent equals port `intent` (SURVEY_ONLY | PREGRASP_ONLY | FULL).
class IntentIs : public BT::ConditionNode
{
public:
  IntentIs(const std::string & name, const BT::NodeConfig & config, SessionPtr session);
  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;
  SessionPtr session_;
};

/// Marks the batch as settled (normal termination) with port `reason`.
class SettleBatch : public BT::SyncActionNode
{
public:
  SettleBatch(const std::string & name, const BT::NodeConfig & config, SessionPtr session);
  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;
  SessionPtr session_;
};

/// FAILURE (and settles) when max_targets, target_harvest_ratio or the explicit target list
/// says the batch is done.
class BatchGate : public BT::ConditionNode
{
public:
  BatchGate(const std::string & name, const BT::NodeConfig & config, SessionPtr session);
  static BT::PortsList providedPorts() {return {};}

private:
  BT::NodeStatus tick() override;
  SessionPtr session_;
};

/// Counts a survey round without a selectable target. SUCCESS = re-survey and try again;
/// FAILURE = empty_survey_limit reached (batch settled "no_targets").
class NoTargetRound : public BT::SyncActionNode
{
public:
  NoTargetRound(const std::string & name, const BT::NodeConfig & config, SessionPtr session);
  static BT::PortsList providedPorts() {return {};}

private:
  BT::NodeStatus tick() override;
  SessionPtr session_;
};

/// Ledger entry for a succeeded target. Always SUCCESS (ledger I/O errors become a blocker).
class RecordResult : public BT::SyncActionNode
{
public:
  RecordResult(const std::string & name, const BT::NodeConfig & config, SessionPtr session);
  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;
  SessionPtr session_;
};

/// Ledger + rework entry for a failed target using the FailureCode policy table.
/// FAILURE only for STOP_BATCH (batch aborted); RECOVER arms the ACK gate.
class RecordSkip : public BT::SyncActionNode
{
public:
  RecordSkip(const std::string & name, const BT::NodeConfig & config, SessionPtr session);
  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;
  SessionPtr session_;
};

class IsRecoveryRequired : public BT::ConditionNode
{
public:
  IsRecoveryRequired(const std::string & name, const BT::NodeConfig & config, SessionPtr session);
  static BT::PortsList providedPorts() {return {};}

private:
  BT::NodeStatus tick() override;
  SessionPtr session_;
};

/// RUNNING until the operator ACK has been forwarded to manipulation and granted on the
/// session (task service /peach/task/acknowledge_recovery). Never times out: the batch stays
/// parked (phase WAITING_ACK) until ACK or cancel. Blocks the whole tree, not a subtree.
class WaitForAck : public BT::StatefulActionNode
{
public:
  WaitForAck(const std::string & name, const BT::NodeConfig & config, SessionPtr session);
  static BT::PortsList providedPorts() {return {};}

private:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {}
  SessionPtr session_;
};

/// SUCCESS if the batch ended normally (settled and not aborted).
class BatchSettled : public BT::ConditionNode
{
public:
  BatchSettled(const std::string & name, const BT::NodeConfig & config, SessionPtr session);
  static BT::PortsList providedPorts() {return {};}

private:
  BT::NodeStatus tick() override;
  SessionPtr session_;
};

/// Retries its child only when the session failure policy is RETRY_VIEW (immediately), WAIT
/// (after SessionConfig::wait_retry_s) or REMEASURE_NECK (immediately, not counted), up to
/// `num_attempts` counted attempts. Any other policy, a pending recovery or an expired
/// per-target deadline returns FAILURE at once.
class RetryOnPolicy : public BT::DecoratorNode
{
public:
  RetryOnPolicy(const std::string & name, const BT::NodeConfig & config, SessionPtr session);
  static BT::PortsList providedPorts();
  void halt() override;

private:
  BT::NodeStatus tick() override;
  SessionPtr session_;
  uint32_t attempts_ = 0;
  bool waiting_ = false;
  double wait_until_s_ = 0.0;
};

/// SUCCESS during a neck re-measure attempt (previous attempt failed NECK_REMEASURE_PENDING).
class NeckRemeasurePending : public BT::ConditionNode
{
public:
  NeckRemeasurePending(
    const std::string & name, const BT::NodeConfig & config, SessionPtr session);
  static BT::PortsList providedPorts() {return {};}

private:
  BT::NodeStatus tick() override;
  SessionPtr session_;
};

/// Guard in front of HarvestTarget: FAILURE (RECOVERY_REQUIRED) when a recovery is pending,
/// e.g. manipulation latched recovery_required while this target was being observed.
class NoRecoveryPending : public BT::ConditionNode
{
public:
  NoRecoveryPending(const std::string & name, const BT::NodeConfig & config, SessionPtr session);
  static BT::PortsList providedPorts() {return {};}

private:
  BT::NodeStatus tick() override;
  SessionPtr session_;
};

/// Halts its child and fails (TARGET_TIMEOUT: policy SKIP, rework "timeout") once the budget
/// started at selection is exhausted. Wraps observe+decision only: an in-flight HarvestTarget
/// is never preempted by the scheduling budget (it has its own action timeout).
class WithinTargetDeadline : public BT::DecoratorNode
{
public:
  WithinTargetDeadline(
    const std::string & name, const BT::NodeConfig & config, SessionPtr session);
  static BT::PortsList providedPorts() {return {};}

private:
  BT::NodeStatus tick() override;
  SessionPtr session_;
};

void register_logic_nodes(BT::BehaviorTreeFactory & factory, const SessionPtr & session);

}  // namespace peach2_task::bt
