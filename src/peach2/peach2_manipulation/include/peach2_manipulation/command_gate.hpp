#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace peach2_manipulation
{

/// Which permission a motion / IO step needs (方案 §10.2).
enum class GateStage : uint8_t
{
  TRANSIT,     ///< free-space move: execution
  APPROACH,    ///< staging -> pregrasp LIN: execution
  CONTACT,     ///< insert: execution && grasp
  TOOL,        ///< close the blade: execution && grasp && tool
  RETREAT,     ///< reverse out of the canopy: execution (no perception permission)
  RELEASE,     ///< open to drop the fruit: tool chain (no perception permission)
  TOOL_SAFE,   ///< open the blade after a fault/abort: robot fresh && not e-stopped only
};

const char * to_string(GateStage stage);

/// aubo_msgs/RobotStatus fields (0/1) + receipt time on the node's steady clock.
struct RobotStatusSample
{
  int mode{-1};
  int e_stopped{0};
  int drives_powered{0};
  int motion_possible{0};
  int in_motion{0};
  int in_error{0};
  int error_code{0};
  double received_s{0.0};
};

struct EnablesSample
{
  uint32_t seq{0};
  bool execution{false};
  bool grasp{false};
  bool tool{false};
  double received_s{0.0};
};

struct EffectiveEnables
{
  bool heartbeat_ok{false};
  bool execution{false};
  bool grasp{false};     ///< already chained: grasp && execution
  bool tool{false};      ///< already chained: tool && grasp && execution
};

struct GateConfig
{
  /// false only for hardware_mode:=mock (no aubo_io_controller).
  bool require_robot_status{true};
  double robot_status_max_age_s{0.3};
  /// Enables heartbeat is 1 Hz; older than this counts as all false.
  double enables_timeout_s{3.0};
};

struct GateVerdict
{
  bool open{false};
  uint32_t failure_code{0};
  std::string reason;
};

/// Single command gate for every trajectory and SetIO. Pure: the node feeds samples and asks.
/// Continuous conditions (drives, e-stop, error, freshness) apply always; motion_possible is
/// only required before a *new* trajectory because the driver reports 0 while one streams.
/// There is no fallback to local parameters when the heartbeat is lost (old P1).
class CommandGate
{
public:
  explicit CommandGate(GateConfig config = {});

  void set_active(bool active) {active_ = active;}
  bool active() const {return active_;}
  void on_robot_status(const RobotStatusSample & sample) {robot_ = sample;}
  void on_enables(const EnablesSample & sample) {enables_ = sample;}
  void set_cancel(bool cancel) {cancel_ = cancel;}
  bool cancel() const {return cancel_;}

  EffectiveEnables enables(double now_s) const;
  GateVerdict robot_ready(double now_s, bool new_trajectory) const;
  GateVerdict check(GateStage stage, double now_s, bool new_trajectory) const;

  const GateConfig & config() const {return config_;}
  std::optional<RobotStatusSample> last_robot_status() const {return robot_;}
  std::optional<EnablesSample> last_enables() const {return enables_;}

private:
  GateConfig config_;
  bool active_{false};
  bool cancel_{false};
  std::optional<RobotStatusSample> robot_;
  std::optional<EnablesSample> enables_;
};

/// True exactly once on each open -> closed transition (the node stops motion on it).
class ClosedEdge
{
public:
  bool update(bool open)
  {
    const bool edge = was_open_ && !open;
    was_open_ = open;
    return edge;
  }
  void reset() {was_open_ = false;}

private:
  bool was_open_{false};
};

}  // namespace peach2_manipulation
