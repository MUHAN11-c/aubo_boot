// Copyright 2026 aubo_e5_ros2_ws authors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the aubo_e5_ros2_ws authors nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#ifndef PEACH_HARVEST_ORCHESTRATOR__STATE_MACHINE_HPP_
#define PEACH_HARVEST_ORCHESTRATOR__STATE_MACHINE_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace peach_harvest_orchestrator
{
enum class OperationMode : uint8_t {AUTO, PAUSED, MAINTENANCE};
enum class BatchState : uint8_t
{
  WAITING_READY, DISCOVERY, RUNNING, PAUSE_PENDING, PAUSED, MAINTENANCE,
  COMPLETED, FAULT, RECOVERY_REQUIRED, INTERRUPTED
};
enum class TargetPhase : uint8_t
{
  IDLE, SELECTING, OBSERVING, FINALIZING, VALIDATING, APPROACHING,
  TOOL_ACTION, RETREATING, COMPLETING, SUCCEEDED, SKIPPED, FAILED
};
enum class ControlCommand : uint8_t
{
  PAUSE, RESUME, ENTER_MAINTENANCE, EXIT_MAINTENANCE, CANCEL_NOW,
  RETRY_TARGET, SKIP_TARGET, ACKNOWLEDGE_RECOVERY
};

struct Readiness
{
  bool perception{false};
  bool reconstruction{false};
  bool motion{false};
  bool web{false};
};
struct OperationPolicy
{
  bool auto_start_enabled{true};
  bool execution_enabled{false};
  bool grasp_enabled{false};
  bool tool_enabled{false};
};
struct HarvestSnapshot
{
  uint64_t revision{0};
  OperationMode mode{OperationMode::AUTO};
  BatchState batch_state{BatchState::WAITING_READY};
  TargetPhase target_phase{TargetPhase::IDLE};
  bool run_active{false};
  bool target_active{false};
  bool recovery_required{false};
  std::string target_id;
  std::string message{"等待系统就绪"};
  std::vector<std::string> blockers;
  OperationPolicy policy;
};
struct CommandResult
{
  bool accepted{false};
  std::string message;
  uint64_t revision{0};
};

class HarvestStateMachine
{
public:
  const HarvestSnapshot & snapshot() const noexcept;
  void update_readiness(const Readiness & readiness);
  bool begin_target(const std::string & target_id);
  void reach_safe_checkpoint();
  void require_recovery(const std::string & reason);
  CommandResult control(
    ControlCommand command, const std::string & request_id, uint64_t expected_revision);
  CommandResult set_policy(
    const OperationPolicy & policy, const std::string & request_id, uint64_t expected_revision);

private:
  CommandResult finish_request(
    const std::string & request_id, bool accepted, const std::string & message,
    bool state_changed);
  bool all_ready() const noexcept;
  void refresh_blockers();
  Readiness readiness_;
  HarvestSnapshot state_;
  std::unordered_map<std::string, CommandResult> request_results_;
};
}  // namespace peach_harvest_orchestrator
#endif  // PEACH_HARVEST_ORCHESTRATOR__STATE_MACHINE_HPP_
