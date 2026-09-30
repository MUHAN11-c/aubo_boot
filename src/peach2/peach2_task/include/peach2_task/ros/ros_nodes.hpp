// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>

#include "peach2_task/bt/logic_nodes.hpp"
#include "peach2_task/ros/async_leaves.hpp"
#include "peach2_task/ros/ros_context.hpp"

/// ROS-facing leaves. Each converts its answer into a core struct and hands it to the
/// session; the batch logic stays in core::BatchSession.
namespace peach2_task::ros
{

using bt::SessionPtr;

class CheckSafety : public BT::ConditionNode
{
public:
  CheckSafety(
    const std::string & name, const BT::NodeConfig & config, RosContextPtr ctx,
    SessionPtr session);
  BT::NodeStatus tick() override;

private:
  RosContextPtr ctx_;
  SessionPtr session_;
};

class MoveToNamed : public AsyncActionLeaf<peach2_interfaces::action::MoveTo>
{
public:
  MoveToNamed(
    const std::string & name, const BT::NodeConfig & config, RosContextPtr ctx,
    SessionPtr session);

protected:
  void on_started() override;
  bool make_goal(Goal * goal) override;
  BT::NodeStatus on_result(const WrappedResult & result) override;
  void on_error(LeafError error, const std::string & detail) override;

private:
  SessionPtr session_;
  std::string target_;
};

class BuildSceneSnapshot : public AsyncServiceLeaf<peach2_interfaces::srv::BuildSceneSnapshot>
{
public:
  BuildSceneSnapshot(
    const std::string & name, const BT::NodeConfig & config, RosContextPtr ctx,
    SessionPtr session);

protected:
  void on_started() override;
  bool make_request(Request * request) override;
  BT::NodeStatus on_response(const Response & response) override;
  BT::NodeStatus on_error(LeafError error, const std::string & detail) override;

private:
  SessionPtr session_;
};

/// /peach/perception/begin_scene on the first survey: tracks cleared, new scene_epoch stored in
/// the session. Not accepted or unreachable = survey failure (batch aborted).
class BeginScene : public AsyncServiceLeaf<peach2_interfaces::srv::BeginScene>
{
public:
  BeginScene(
    const std::string & name, const BT::NodeConfig & config, RosContextPtr ctx,
    SessionPtr session);

protected:
  void on_started() override;
  bool make_request(Request * request) override;
  BT::NodeStatus on_response(const Response & response) override;
  BT::NodeStatus on_error(LeafError error, const std::string & detail) override;

private:
  SessionPtr session_;
};

/// Waits for a target_set_locked observation array of the session's scene_epoch produced after
/// this node started, so a lock from the previous survey (or from before the photo pose was
/// reached) never counts. The selectable set is its locked_target_ids.
class WaitTargetSetLocked : public BT::StatefulActionNode
{
public:
  WaitTargetSetLocked(
    const std::string & name, const BT::NodeConfig & config, RosContextPtr ctx,
    SessionPtr session);

protected:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {}

private:
  RosContextPtr ctx_;
  SessionPtr session_;
  rclcpp::Time started_stamp_;
  SteadyClock::time_point started_at_;
};

class SelectTarget : public AsyncServiceLeaf<peach2_interfaces::srv::CheckReachability>
{
public:
  SelectTarget(
    const std::string & name, const BT::NodeConfig & config, RosContextPtr ctx,
    SessionPtr session);

protected:
  BT::NodeStatus onStart() override;
  bool make_request(Request * request) override;
  BT::NodeStatus on_response(const Response & response) override;
  BT::NodeStatus on_error(LeafError error, const std::string & detail) override;

private:
  BT::NodeStatus finish(const core::ReachMap & reach);
  SessionPtr session_;
};

class ObserveTarget : public AsyncActionLeaf<peach2_interfaces::action::ObserveTarget>
{
public:
  ObserveTarget(
    const std::string & name, const BT::NodeConfig & config, RosContextPtr ctx,
    SessionPtr session);

protected:
  void on_started() override;
  bool make_goal(Goal * goal) override;
  BT::NodeStatus on_result(const WrappedResult & result) override;
  void on_error(LeafError error, const std::string & detail) override;
  void on_feedback(const Feedback & feedback) override;

private:
  SessionPtr session_;
};

class CheckDecision : public AsyncServiceLeaf<peach2_interfaces::srv::GetDecision>
{
public:
  CheckDecision(
    const std::string & name, const BT::NodeConfig & config, RosContextPtr ctx,
    SessionPtr session);

protected:
  void on_started() override;
  bool make_request(Request * request) override;
  BT::NodeStatus on_response(const Response & response) override;
  BT::NodeStatus on_error(LeafError error, const std::string & detail) override;

private:
  SessionPtr session_;
  core::DecisionLevel level_ = core::DecisionLevel::APPROACH;
};

class HarvestTarget : public AsyncActionLeaf<peach2_interfaces::action::HarvestTarget>
{
public:
  HarvestTarget(
    const std::string & name, const BT::NodeConfig & config, RosContextPtr ctx,
    SessionPtr session);

protected:
  void on_started() override;
  bool make_goal(Goal * goal) override;
  BT::NodeStatus on_result(const WrappedResult & result) override;
  void on_error(LeafError error, const std::string & detail) override;
  void on_feedback(const Feedback & feedback) override;

private:
  SessionPtr session_;
  std::string target_id_;
};

void register_ros_nodes(
  BT::BehaviorTreeFactory & factory, const RosContextPtr & ctx, const SessionPtr & session);

}  // namespace peach2_task::ros
