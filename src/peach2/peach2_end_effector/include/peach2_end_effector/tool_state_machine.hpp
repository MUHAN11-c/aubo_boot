#pragma once

#include <optional>
#include <string>

#include "peach2_end_effector/types.hpp"

namespace peach2_end_effector
{

struct ToolStateMachineConfig
{
  double feedback_timeout_s{1.5};   ///< from the tool profile io.feedback_timeout_s
  /// A feedback flip seen sooner than this after the command write returned is physically
  /// implausible for a blade and is treated as the DI reading back the DO (P0-3).
  double min_actuation_s{0.03};
  /// Longest time the close level may stay commanded; 0 disables. TODO(M0): per actuator.
  double max_close_energized_s{0.0};
};

struct CommandDecision
{
  bool accepted{false};
  std::string reason;
};

/// Pure blade state machine (no ROS, no IO). Time is caller-provided, monotonic seconds.
///
/// UNKNOWN -> OPENING -> OPEN_CONFIRMED -> CLOSING -> CLOSED_CONFIRMED -> OPENING -> ...
/// Any timeout / implausible feedback -> FAULT. FAULT is left only through reset_by_ack();
/// while in FAULT an *open* command is still accepted (energy-reducing) but the state stays
/// FAULT, and close commands are rejected.
class ToolStateMachine
{
public:
  explicit ToolStateMachine(ToolStateMachineConfig config = {});

  /// Validate and transition before writing the output. Rejected commands must not be written.
  CommandDecision begin_command(bool close, double now_s);
  /// Report the output write result; `now_s` is taken after the write returned.
  void end_command(bool write_ok, double now_s);
  /// One feedback sample; `closed` is already mapped to "blade closed".
  void feedback(bool closed, double now_s);
  /// Timeouts.
  void update(double now_s);
  /// External fault (e.g. confirm window shorter than the profile timeout expired).
  void fault(const std::string & reason);
  /// Human acknowledgement: back to UNKNOWN, clears fault and loopback suspicion.
  void reset_by_ack();
  /// Robot e-stop / protective stop: output hold behaviour is unknown. FAULT stays FAULT.
  void mark_unknown();

  ToolState state() const {return state_;}
  bool command_closed() const {return command_closed_;}
  bool command_pending() const {return pending_;}
  std::optional<bool> feedback_closed() const {return feedback_;}
  bool suspected_loopback() const {return suspected_loopback_;}
  const std::string & fault_reason() const {return fault_reason_;}
  /// FAULT while the close level is still commanded: caller should write the open level.
  bool needs_deenergize() const {return state_ == ToolState::FAULT && command_closed_;}
  const ToolStateMachineConfig & config() const {return config_;}

private:
  void enter_fault(const std::string & reason);

  ToolStateMachineConfig config_;
  ToolState state_{ToolState::UNKNOWN};
  bool command_closed_{false};
  bool pending_{false};
  bool writing_{false};
  bool pending_close_{false};
  double command_s_{0.0};
  std::optional<bool> feedback_;
  std::optional<bool> feedback_at_command_;
  bool suspected_loopback_{false};
  std::string fault_reason_;
};

}  // namespace peach2_end_effector
