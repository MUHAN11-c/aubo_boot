#pragma once

#include <chrono>
#include <mutex>
#include <optional>
#include <string>

#include "peach2_end_effector/end_effector.hpp"
#include "peach2_end_effector/tool_state_machine.hpp"

namespace peach2_end_effector
{

/// Shared implementation for the three sleeve cutters: common geometry checks and the
/// open / close / confirm IO flow on top of ToolStateMachine. Subclasses supply the tool id,
/// the roll constraint and the insert strategy.
class CutterEndEffector : public EndEffector
{
public:
  void initialize(const EndEffectorContext & context) override;

  std::string tool_id() const override {return expected_tool_id();}
  const ToolProfile & profile() const override {return ctx_.profile;}
  Eigen::Isometry3d blade_in_tcp() const override;
  Feasibility feasible(const TargetGeometry & target, const BudgetView & budget) const override;

  ToolResult prepare() override;
  ToolResult cut() override;
  CutVerdict confirm_cut(std::chrono::milliseconds timeout) override;
  ToolResult abort_safe() override;
  ToolResult release() override;

  ToolStatus status() override;
  void poll() override;
  void reset_by_ack() override;
  void mark_unknown(const std::string & reason) override;

protected:
  virtual const char * expected_tool_id() const = 0;

private:
  void require_initialized() const;
  bool close_level() const {return ctx_.profile.io.close_state > 0.5;}
  /// Reads feedback + current into the state machine. Caller holds state_mutex_.
  void sample_locked(double now);
  ToolResult command(bool close);
  /// Samples until the state leaves `transient` or the profile timeout (+margin) elapses.
  ToolState wait_while(ToolState transient);
  ToolResult open_result(ToolState reached, const std::string & what);
  void deenergize_if_needed();
  void log(const std::string & msg) const;

  EndEffectorContext ctx_;
  bool initialized_{false};
  std::mutex op_mutex_;      ///< serialises IO operations
  std::mutex state_mutex_;   ///< guards sm_ and the sampled values
  ToolStateMachine sm_;
  std::optional<double> last_current_;
  double peak_current_{0.0};
  double last_deenergize_s_{-1e9};
};

}  // namespace peach2_end_effector
