// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <aubo_msgs/msg/robot_status.hpp>
#include <peach2_interfaces/action/run_batch.hpp>
#include <peach2_interfaces/msg/batch_state.hpp>
#include <peach2_interfaces/msg/enables.hpp>
#include <peach2_interfaces/msg/target_observation_array.hpp>
#include <peach2_interfaces/msg/tool_state.hpp>
#include <peach2_interfaces/srv/set_enables.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <bondcpp/bond.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "peach2_task/core/batch_session.hpp"
#include "peach2_task/peach2_task_parameters.hpp"
#include "peach2_task/ros/ros_context.hpp"

namespace peach2_task
{

/// Batch orchestrator: RunBatch starts trees/harvest_batch.xml, a wall timer ticks it with
/// Tree::tickOnce, cancel halts it (in-flight goals are canceled by the leaves' onHalted).
/// Also the single source of the operator enables (/peach/enables, default all false).
class TaskNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using RunBatch = peach2_interfaces::action::RunBatch;
  using GoalHandle = rclcpp_action::ServerGoalHandle<RunBatch>;
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit TaskNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~TaskNode() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

private:
  struct PendingAck
  {
    std::shared_ptr<rmw_request_id_t> header;
    int64_t forward_id = 0;
    std::chrono::steady_clock::time_point deadline;
  };

  // run_batch
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const RunBatch::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(std::shared_ptr<GoalHandle> goal_handle);
  std::string check_goal(const RunBatch::Goal & goal, core::BatchLimits * limits) const;
  void start_batch(std::shared_ptr<GoalHandle> goal_handle);
  void tick();
  void end_batch(core::BatchEnd end);
  void abort_running_batch(const std::string & reason);

  // enables / ack
  void on_set_enables(
    const std::shared_ptr<peach2_interfaces::srv::SetEnables::Request> request,
    std::shared_ptr<peach2_interfaces::srv::SetEnables::Response> response);
  void on_acknowledge(
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request);
  void expire_pending_acks();
  void set_enables(const core::Enables & enables);
  void publish_enables();

  // state / diagnostics
  void heartbeat();
  void publish_state(bool force);
  void diagnose_batch(diagnostic_updater::DiagnosticStatusWrapper & status);
  void diagnose_servers(diagnostic_updater::DiagnosticStatusWrapper & status);
  void diagnose_safety(diagnostic_updater::DiagnosticStatusWrapper & status);
  core::SafetyVerdict idle_safety() const;

  void refresh_context(const Params & params);
  void release();
  std::filesystem::path resolve_runs_dir(const Params & params) const;
  std::filesystem::path resolve_tree_file(const Params & params) const;

  std::shared_ptr<ParamListener> param_listener_;
  std::filesystem::path runs_dir_;
  std::filesystem::path tree_file_;

  ros::RosContextPtr ctx_;
  std::shared_ptr<core::BatchSession> session_;
  std::unique_ptr<BT::BehaviorTreeFactory> factory_;
  std::unique_ptr<BT::Tree> tree_;
  std::shared_ptr<GoalHandle> goal_;
  uint64_t published_revision_ = 0;
  std::chrono::steady_clock::time_point last_state_publish_;

  core::Enables enables_;
  uint32_t enables_seq_ = 0;
  bool active_ = false;

  rclcpp_action::Server<RunBatch>::SharedPtr run_batch_server_;
  rclcpp_lifecycle::LifecyclePublisher<peach2_interfaces::msg::Enables>::SharedPtr enables_pub_;
  rclcpp_lifecycle::LifecyclePublisher<peach2_interfaces::msg::BatchState>::SharedPtr state_pub_;
  rclcpp::Subscription<peach2_interfaces::msg::TargetObservationArray>::SharedPtr obs_sub_;
  rclcpp::Subscription<aubo_msgs::msg::RobotStatus>::SharedPtr robot_status_sub_;
  rclcpp::Subscription<peach2_interfaces::msg::ToolState>::SharedPtr tool_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr peer_recovery_sub_;
  rclcpp::Service<peach2_interfaces::srv::SetEnables>::SharedPtr set_enables_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ack_srv_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr ack_forward_client_;
  std::vector<PendingAck> pending_acks_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
  std::unique_ptr<bond::Bond> bond_;
};

}  // namespace peach2_task
