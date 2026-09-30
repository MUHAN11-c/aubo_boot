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
};

struct PlanResult
{
  bool ok{false};
  uint32_t failure_code{0};
  std::string reason;
  JointTrajectory trajectory;
};

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
