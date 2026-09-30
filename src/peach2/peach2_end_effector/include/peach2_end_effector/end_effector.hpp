#pragma once

#include <Eigen/Geometry>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "peach2_end_effector/io_backend.hpp"
#include "peach2_end_effector/tool_profile.hpp"
#include "peach2_end_effector/types.hpp"

namespace peach2_end_effector
{

struct IoPins
{
  int cmd_pin{0};
  int feedback_pin{1};           ///< must differ from cmd_pin
  bool feedback_active_high{true};
};

struct ToolTiming
{
  double min_actuation_s{0.03};     ///< faster feedback flips are treated as loopback
  double poll_period_s{0.01};
  double max_close_energized_s{0.0};  ///< 0 disables
};

/// Optional second cut criterion: current rises above peak_min_a while closing and then
/// falls below drop_ratio * peak within window_s after the feedback edge.
struct CurrentSignatureConfig
{
  bool enabled{false};
  double peak_min_a{0.5};
  double drop_ratio{0.5};
  double window_s{0.3};
};

/// Everything a plugin needs. Deliberately ROS-free so plugins are unit-testable; the
/// manipulation node supplies a gated IoBackend and a steady clock.
struct EndEffectorContext
{
  ToolProfile profile;
  std::shared_ptr<IoBackend> io;
  IoPins pins;
  ToolTiming timing;
  CurrentSignatureConfig current;
  std::function<double()> now_s;          ///< monotonic seconds
  std::function<void(double)> sleep_s;    ///< blocking sleep (tests may advance a fake clock)
  std::function<void(const std::string &)> log;  ///< optional
};

/// Tool plugin interface (方案 §7.1). Loaded with pluginlib by class name; default-constructed
/// then initialize()d. IO methods block for at most the profile feedback timeout and must be
/// called from a worker thread, never from an executor callback.
class EndEffector
{
public:
  virtual ~EndEffector() = default;

  /// Throws std::invalid_argument on an unusable context (missing io, cmd_pin == feedback_pin,
  /// profile of another tool).
  virtual void initialize(const EndEffectorContext & context) = 0;

  virtual std::string tool_id() const = 0;
  virtual const ToolProfile & profile() const = 0;

  /// Blade plane pose in the TCP frame: translation (0, 0, -L_blade). TCP = mouth, +Z = opening
  /// direction = bag axis. Blade on the neck => tcp = neck + L_blade * axis.
  virtual Eigen::Isometry3d blade_in_tcp() const = 0;
  virtual RollConstraint roll_constraint(const TargetGeometry & target) const = 0;
  virtual Feasibility feasible(const TargetGeometry & target, const BudgetView & budget) const = 0;
  virtual InsertPlan insert(const TargetGeometry & target) const = 0;

  /// Open and confirm open through the independent feedback channel.
  virtual ToolResult prepare() = 0;
  /// Command close. Requires OPEN_CONFIRMED; does not wait for feedback.
  virtual ToolResult cut() = 0;
  /// Feedback edge (+ optional current signature) within `timeout`.
  virtual CutVerdict confirm_cut(std::chrono::milliseconds timeout) = 0;
  /// Open the blade (also from FAULT). Returns failure if the tool stays in FAULT.
  virtual ToolResult abort_safe() = 0;
  /// Open to drop the fruit and confirm. Does not look at any perception permission.
  virtual ToolResult release() = 0;

  virtual ToolStatus status() = 0;
  /// Sample feedback / timeouts; call periodically when no IO method is running.
  virtual void poll() = 0;
  virtual void reset_by_ack() = 0;
  /// Robot e-stop / protective stop observed: blade state no longer trusted.
  virtual void mark_unknown(const std::string & reason) = 0;
};

}  // namespace peach2_end_effector
