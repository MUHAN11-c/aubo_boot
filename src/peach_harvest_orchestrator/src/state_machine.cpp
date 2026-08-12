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


#include "peach_harvest_orchestrator/state_machine.hpp"

namespace peach_harvest_orchestrator
{
const HarvestSnapshot & HarvestStateMachine::snapshot() const noexcept {return state_;}

bool HarvestStateMachine::all_ready() const noexcept
{
  return readiness_.perception && readiness_.reconstruction && readiness_.motion &&
         readiness_.web;
}

void HarvestStateMachine::refresh_blockers()
{
  state_.blockers.clear();
  if (!readiness_.perception) {state_.blockers.emplace_back("perception");}
  if (!readiness_.reconstruction) {state_.blockers.emplace_back("reconstruction");}
  if (!readiness_.motion) {state_.blockers.emplace_back("motion");}
  if (!readiness_.web) {state_.blockers.emplace_back("web");}
}

void HarvestStateMachine::update_readiness(const Readiness & readiness)
{
  const bool readiness_changed =
    readiness.perception != readiness_.perception ||
    readiness.reconstruction != readiness_.reconstruction ||
    readiness.motion != readiness_.motion || readiness.web != readiness_.web;
  const auto previous_state = state_.batch_state;
  const bool previous_active = state_.run_active;
  const std::string previous_message = state_.message;
  const std::vector<std::string> previous_blockers = state_.blockers;
  readiness_ = readiness;
  refresh_blockers();
  if (state_.batch_state == BatchState::WAITING_READY && all_ready() &&
    state_.policy.auto_start_enabled)
  {
    state_.batch_state = BatchState::DISCOVERY;
    state_.run_active = true;
    state_.message = "系统就绪，开始发现目标";
  } else if (state_.batch_state == BatchState::WAITING_READY) {
    state_.message = "等待系统就绪";
  }
  if (readiness_changed || previous_state != state_.batch_state ||
    previous_active != state_.run_active || previous_message != state_.message ||
    previous_blockers != state_.blockers)
  {
    ++state_.revision;
  }
}

bool HarvestStateMachine::begin_target(const std::string & target_id)
{
  if (target_id.empty() || state_.recovery_required || state_.target_active ||
    !state_.policy.execution_enabled ||
    (state_.batch_state != BatchState::DISCOVERY && state_.batch_state != BatchState::RUNNING))
  {
    return false;
  }
  state_.target_id = target_id;
  state_.target_active = true;
  state_.batch_state = BatchState::RUNNING;
  state_.target_phase = TargetPhase::SELECTING;
  state_.message = "目标周期运行中";
  ++state_.revision;
  return true;
}

void HarvestStateMachine::reach_safe_checkpoint()
{
  state_.target_active = false;
  state_.target_phase = TargetPhase::IDLE;
  if (state_.batch_state == BatchState::PAUSE_PENDING) {
    state_.batch_state = BatchState::PAUSED;
    state_.mode = OperationMode::PAUSED;
    state_.message = "已在安全检查点暂停";
  }
  ++state_.revision;
}

void HarvestStateMachine::require_recovery(const std::string & reason)
{
  state_.recovery_required = true;
  state_.target_active = false;
  state_.batch_state = BatchState::RECOVERY_REQUIRED;
  state_.mode = OperationMode::PAUSED;
  state_.message = reason;
  ++state_.revision;
}

CommandResult HarvestStateMachine::finish_request(
  const std::string & request_id, bool accepted, const std::string & message, bool state_changed)
{
  if (state_changed) {++state_.revision;}
  const CommandResult result{accepted, message, state_.revision};
  if (!request_id.empty()) {request_results_[request_id] = result;}
  return result;
}

CommandResult HarvestStateMachine::control(
  ControlCommand command, const std::string & request_id, uint64_t expected_revision)
{
  const auto existing = request_results_.find(request_id);
  if (!request_id.empty() && existing != request_results_.end()) {return existing->second;}
  if (expected_revision != state_.revision) {
    return finish_request(request_id, false, "状态版本已过期", false);
  }
  switch (command) {
    case ControlCommand::PAUSE:
      if (state_.mode == OperationMode::MAINTENANCE) {
        return finish_request(request_id, false, "维护模式无需暂停", false);
      }
      state_.batch_state = state_.target_active ? BatchState::PAUSE_PENDING : BatchState::PAUSED;
      if (!state_.target_active) {state_.mode = OperationMode::PAUSED;}
      state_.message = state_.target_active ? "将在安全检查点暂停" : "已暂停";
      return finish_request(request_id, true, state_.message, true);
    case ControlCommand::RESUME:
      if (state_.recovery_required) {
        return finish_request(request_id, false, "需要先确认恢复", false);
      }
      if (state_.mode == OperationMode::MAINTENANCE) {
        return finish_request(request_id, false, "请先退出维护模式", false);
      }
      state_.mode = OperationMode::AUTO;
      state_.batch_state = all_ready() ? BatchState::DISCOVERY : BatchState::WAITING_READY;
      state_.message = all_ready() ? "恢复自动运行" : "恢复后等待系统就绪";
      return finish_request(request_id, true, state_.message, true);
    case ControlCommand::ENTER_MAINTENANCE:
      if (state_.target_active || state_.batch_state == BatchState::PAUSE_PENDING) {
        return finish_request(request_id, false, "活动目标尚未到达安全检查点", false);
      }
      state_.mode = OperationMode::MAINTENANCE;
      state_.batch_state = BatchState::MAINTENANCE;
      state_.message = "已进入维护模式";
      return finish_request(request_id, true, state_.message, true);
    case ControlCommand::EXIT_MAINTENANCE:
      if (state_.mode != OperationMode::MAINTENANCE) {
        return finish_request(request_id, false, "当前不在维护模式", false);
      }
      state_.mode = OperationMode::PAUSED;
      state_.batch_state = BatchState::PAUSED;
      state_.message = "已退出维护模式，保持暂停";
      return finish_request(request_id, true, state_.message, true);
    case ControlCommand::CANCEL_NOW:
      state_.target_active = false;
      state_.run_active = false;
      state_.mode = OperationMode::PAUSED;
      state_.batch_state = BatchState::INTERRUPTED;
      state_.message = "已请求立即取消";
      return finish_request(request_id, true, state_.message, true);
    case ControlCommand::ACKNOWLEDGE_RECOVERY:
      if (!state_.recovery_required) {
        return finish_request(request_id, false, "当前没有待确认恢复", false);
      }
      state_.recovery_required = false;
      state_.batch_state = BatchState::PAUSED;
      state_.mode = OperationMode::PAUSED;
      state_.message = "恢复已确认，保持暂停";
      return finish_request(request_id, true, state_.message, true);
    case ControlCommand::RETRY_TARGET:
    case ControlCommand::SKIP_TARGET:
      return finish_request(request_id, false, "当前阶段不允许该命令", false);
  }
  return finish_request(request_id, false, "未知命令", false);
}

CommandResult HarvestStateMachine::set_policy(
  const OperationPolicy & policy, const std::string & request_id, uint64_t expected_revision)
{
  const auto existing = request_results_.find(request_id);
  if (!request_id.empty() && existing != request_results_.end()) {return existing->second;}
  if (expected_revision != state_.revision) {
    return finish_request(request_id, false, "状态版本已过期", false);
  }
  if ((policy.grasp_enabled && !policy.execution_enabled) ||
    (policy.tool_enabled && !policy.grasp_enabled))
  {
    return finish_request(request_id, false, "使能依赖必须满足 execution→grasp→tool", false);
  }
  if (state_.target_active) {
    return finish_request(request_id, false, "目标周期运行时不能修改操作策略", false);
  }
  state_.policy = policy;
  state_.message = "操作策略已更新";
  return finish_request(request_id, true, state_.message, true);
}
}  // namespace peach_harvest_orchestrator
