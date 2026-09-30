#pragma once

#include <Eigen/Geometry>

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "peach2_manipulation/trajectory_reverse.hpp"

namespace peach2_manipulation
{

enum class PlanKind : uint8_t
{
  FREE,     ///< OMPL, whole-arm collision checked
  LINEAR,   ///< Pilz LIN; either complete or failed (no partial Cartesian fraction)
  NAMED,    ///< OMPL to an SRDF group_state
};

struct PlanRequest
{
  PlanKind kind{PlanKind::FREE};
  Eigen::Isometry3d tcp_goal{Eigen::Isometry3d::Identity()};  ///< base_link, TCP frame
  std::string named_target;
  double velocity_scaling{0.1};       ///< FREE / NAMED
  double acceleration_scaling{0.1};
  double linear_speed_mps{0.02};      ///< LINEAR TCP speed
  /// Plan from these joints instead of the current state (chained segments).
  std::optional<std::vector<double>> start_joints;
  std::string label;
  /// Residual-correction LINs set this false: a 3 mm TCP residual is ~0.004 rad at 0.8 m,
  /// inside the transit at-goal band, but must still be sent.
  bool collapse_at_goal{true};
};

/// `ok` with a single-point trajectory is a null motion (see is_null_motion): the start already
/// satisfies the goal, nothing is sent to the controller and the segment counts as reached.
struct PlanResult
{
  bool ok{false};
  uint32_t failure_code{0};
  std::string reason;
  JointTrajectory trajectory;
};

/// Planner output -> PlanResult. When `collapse_at_goal` is true, a trajectory whose every point
/// lies within `at_goal_tolerance_rad` (max |dq|) of `start` (the first point when absent)
/// collapses to a null motion at `start`. Residual-correction LINs pass false so a TCP residual
/// smaller than the joint at-goal band is still executed. Otherwise fewer than two points is a
/// failure (PLAN_CARTESIAN_INCOMPLETE for LINEAR, PLAN_FAILED otherwise, reason
/// `<label>:empty_trajectory`): an empty trajectory cannot prove the goal is the start.
PlanResult finalize_plan(
  JointTrajectory trajectory, const std::optional<std::vector<double>> & start, PlanKind kind,
  const std::string & label, double at_goal_tolerance_rad, bool collapse_at_goal = true);

struct Abort
{
  uint32_t failure_code{0};
  std::string reason;
};

struct ExecOptions
{
  double timeout_s{10.0};
  /// Polled while executing (~20 ms); a value aborts: backend cancels and stops the arm.
  std::function<std::optional<Abort>()> abort_probe;
};

struct ExecResult
{
  bool ok{false};
  uint32_t failure_code{0};
  std::string reason;
};

/// Motion planning / execution seam. The MoveIt implementation lives in
/// moveit_motion_backend.hpp; tests use a fake.
class MotionBackend
{
public:
  virtual ~MotionBackend() = default;
  virtual PlanResult plan(const PlanRequest & request) = 0;
  /// Blocking, bounded by options.timeout_s. On abort / timeout the backend stops the arm
  /// (trajectory cancel -> driver RobotMoveStop; an application stop, not an e-stop).
  virtual ExecResult execute(const JointTrajectory & trajectory, const ExecOptions & options) = 0;
  /// Collision-check every waypoint against the current planning scene.
  virtual bool validate(const JointTrajectory & trajectory, std::string * why) = 0;
  virtual std::optional<std::vector<double>> current_joints() = 0;
  virtual std::optional<Eigen::Isometry3d> current_tcp() = 0;
  virtual void stop() = 0;
  virtual bool has_force_sensing() const = 0;
};

}  // namespace peach2_manipulation
