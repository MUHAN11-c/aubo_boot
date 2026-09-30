// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <peach2_interfaces/action/harvest_target.hpp>
#include <peach2_interfaces/action/move_to.hpp>
#include <peach2_interfaces/action/observe_target.hpp>
#include <peach2_interfaces/msg/target_observation_array.hpp>
#include <peach2_interfaces/msg/tool_state.hpp>
#include <peach2_interfaces/srv/begin_scene.hpp>
#include <peach2_interfaces/srv/build_scene_snapshot.hpp>
#include <peach2_interfaces/srv/check_reachability.hpp>
#include <peach2_interfaces/srv/get_decision.hpp>

#include <chrono>
#include <memory>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "peach2_task/core/enables_policy.hpp"
#include "peach2_task/core/safety_gate.hpp"

/// Everything the ROS leaves share: clients, latest-message caches written by the node's
/// subscriptions, and timeouts. Single-threaded executor: no locking between the
/// subscription callbacks, the tick timer and the leaves.
namespace peach2_task::ros
{

using SteadyClock = std::chrono::steady_clock;

struct LeafTimeouts
{
  double server_wait_s = 5.0;
  double move_to_s = 60.0;
  double snapshot_s = 15.0;
  double begin_scene_s = 5.0;
  double lock_wait_s = 15.0;
  double reachability_s = 10.0;
  double observe_s = 90.0;
  double decision_s = 5.0;
  double harvest_s = 180.0;
};

struct RobotStatusCache
{
  bool received = false;
  SteadyClock::time_point received_at;
  bool drives_powered = false;
  bool e_stopped = false;
  bool in_error = false;
};

struct RosContext
{
  rclcpp::Logger logger = rclcpp::get_logger("peach2_task");
  rclcpp::Clock::SharedPtr ros_clock;  ///< node clock; compares against message stamps
  rclcpp::Time node_start;

  rclcpp_action::Client<peach2_interfaces::action::MoveTo>::SharedPtr move_to;
  rclcpp_action::Client<peach2_interfaces::action::ObserveTarget>::SharedPtr observe;
  rclcpp_action::Client<peach2_interfaces::action::HarvestTarget>::SharedPtr harvest;
  rclcpp::Client<peach2_interfaces::srv::BuildSceneSnapshot>::SharedPtr snapshot;
  rclcpp::Client<peach2_interfaces::srv::BeginScene>::SharedPtr begin_scene;
  rclcpp::Client<peach2_interfaces::srv::CheckReachability>::SharedPtr reachability;
  rclcpp::Client<peach2_interfaces::srv::GetDecision>::SharedPtr decision;

  peach2_interfaces::msg::TargetObservationArray::ConstSharedPtr observations;
  SteadyClock::time_point observations_received_at;
  RobotStatusCache robot;
  peach2_interfaces::msg::ToolState::ConstSharedPtr tool_state;
  core::Enables enables;

  core::SafetyConfig safety;
  LeafTimeouts timeouts;

  core::RobotStatusSample robot_sample(SteadyClock::time_point now) const
  {
    core::RobotStatusSample s;
    s.received = robot.received;
    s.age_s = robot.received ?
      std::chrono::duration<double>(now - robot.received_at).count() : 0.0;
    s.drives_powered = robot.drives_powered;
    s.e_stopped = robot.e_stopped;
    s.in_error = robot.in_error;
    return s;
  }

  bool tool_fault() const
  {
    return tool_state && tool_state->state == peach2_interfaces::msg::ToolState::FAULT;
  }
};

using RosContextPtr = std::shared_ptr<RosContext>;

}  // namespace peach2_task::ros
