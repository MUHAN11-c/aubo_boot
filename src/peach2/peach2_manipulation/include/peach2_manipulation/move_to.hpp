#pragma once

#include <Eigen/Geometry>

#include <cstdint>
#include <functional>
#include <optional>
#include <string>

#include "peach2_manipulation/command_gate.hpp"
#include "peach2_manipulation/motion_backend.hpp"

namespace peach2_manipulation
{

struct MoveToRequest
{
  std::string named_target;                     ///< SRDF group_state; empty = tcp_pose
  std::optional<Eigen::Isometry3d> tcp_pose;    ///< base_link
  double velocity_scaling{0.0};                 ///< 0 = config default
};

struct MoveToConfig
{
  double default_velocity_scaling{0.1};
  double acceleration_scaling{0.1};
  double max_velocity_scaling{0.5};
  double start_tolerance_rad{0.02};
  double execute_timeout_scale{2.0};
  double execute_timeout_margin_s{5.0};
};

struct MoveToResult
{
  bool success{false};
  uint32_t failure_code{0};
  std::string message;
  bool plan_only{false};
};

struct MoveToDeps
{
  MotionBackend * motion{nullptr};
  std::function<GateVerdict(GateStage, bool new_trajectory)> gate;
  std::function<EffectiveEnables()> enables;
  std::function<bool()> cancel_requested;
  /// Optional: write every bag as a collision object before planning; false = not applied.
  std::function<bool(std::string * why)> scene;
};

/// Free-space transit (OMPL, whole-arm collision checked) through the TRANSIT gate. With
/// execution disabled it only plans and reports `plan_only_ok` with SAFETY_GATE_CLOSED
/// (MoveTo.Result has no plan-only flag). Scene update failure is DEPENDENCY_UNAVAILABLE.
MoveToResult run_move_to(
  const MoveToRequest & request, const MoveToConfig & config, const MoveToDeps & deps);

}  // namespace peach2_manipulation
