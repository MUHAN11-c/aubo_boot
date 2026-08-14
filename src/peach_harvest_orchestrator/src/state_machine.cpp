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
Readiness ReadinessTracker::evaluate(const ReadinessSample & sample) const
{
  // fresh：收到过（时刻>0）且未超龄；与原节点 fresh() 判定一致。
  const auto fresh = [&sample, this](double received_s) {
      return received_s > 0.0 && sample.now_s - received_s <= timeout_s_;
    };
  Readiness readiness;
  // 感知就绪只看话题新鲜度：锁定由派发前置保证，避免收齐窗口期卡回 WAITING_READY。
  readiness.perception = fresh(sample.targets_received_s);
  readiness.reconstruction = fresh(sample.reconstruction_received_s);
  const bool robot_ready = !require_robot_status_ || (
    fresh(sample.robot_received_s) && !sample.robot_e_stopped && !sample.robot_in_error &&
    sample.robot_drives_powered && sample.robot_motion_possible);
  readiness.motion = sample.action_server_ready && robot_ready;
  readiness.web = sample.web_ready;
  return readiness;
}

std::vector<std::string> ReadinessTracker::blockers(const Readiness & readiness)
{
  std::vector<std::string> blockers;
  if (!readiness.perception) {blockers.emplace_back("perception");}
  if (!readiness.reconstruction) {blockers.emplace_back("reconstruction");}
  if (!readiness.motion) {blockers.emplace_back("motion");}
  if (!readiness.web) {blockers.emplace_back("web");}
  return blockers;
}

bool allow_dispatch(
  bool photo_step_done, const std::string & selected_target_id,
  const std::string & last_dispatched_target, bool target_active,
  bool action_server_ready)
{
  return photo_step_done && !selected_target_id.empty() &&
         selected_target_id != last_dispatched_target && !target_active &&
         action_server_ready;
}

RoundVerdict decide_round(
  bool locked, uint32_t target_count, uint32_t processed_count,
  uint32_t round, uint32_t max_rounds, bool rescan_enabled)
{
  if (!locked || processed_count < target_count) {
    return {RoundDecision::WAIT, ""};
  }
  if (target_count == 0) {
    return {RoundDecision::COMPLETE, "本轮未锁定到目标，批次完成"};
  }
  if (rescan_enabled && round < max_rounds) {
    return {RoundDecision::RESCAN, ""};
  }
  if (rescan_enabled) {
    return {RoundDecision::COMPLETE, "达到最大复拍轮次，批次完成"};
  }
  return {RoundDecision::COMPLETE, "本轮目标已处理完，批次完成"};
}

const HarvestSnapshot & HarvestStateMachine::snapshot() const noexcept {return state_;}

bool HarvestStateMachine::all_ready() const noexcept
{
  return readiness_.perception && readiness_.reconstruction && readiness_.motion &&
         readiness_.web;
}

void HarvestStateMachine::refresh_blockers()
{
  state_.blockers = ReadinessTracker::blockers(readiness_);
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

void HarvestStateMachine::record_target_outcome(
  const std::string & target_id, TargetOutcome outcome, const std::string & reason)
{
  // 无活动目标时不记账：终态回调可能迟到于 CANCEL_NOW 等已清场路径。
  if (!state_.target_active) {return;}
  ++state_.counters.attempted;
  const char * default_message = "目标周期完成";
  switch (outcome) {
    case TargetOutcome::SUCCEEDED:
      ++state_.counters.succeeded;
      break;
    case TargetOutcome::SKIPPED_QUALITY:
      ++state_.counters.skipped_quality;
      default_message = "质量门未通过，已跳过目标";
      break;
    case TargetOutcome::SKIPPED_UNREACHABLE:
      ++state_.counters.skipped_unreachable;
      default_message = "目标不可达，已跳过";
      break;
    case TargetOutcome::FAILED:
      ++state_.counters.failed;
      default_message = "目标周期失败";
      break;
    case TargetOutcome::CANCELED:
      ++state_.counters.canceled;
      default_message = "目标周期已取消";
      break;
  }
  state_.outcomes.push_back(TargetOutcomeRecord{target_id, outcome, reason});
  state_.target_active = false;
  state_.target_phase = TargetPhase::IDLE;
  if (state_.batch_state == BatchState::PAUSE_PENDING) {
    // 暂停请求挂起期间周期结束：直接落到 PAUSED。
    state_.batch_state = BatchState::PAUSED;
    state_.mode = OperationMode::PAUSED;
    state_.message = "已在安全检查点暂停";
  } else {
    state_.message = reason.empty() ? default_message : reason;
  }
  ++state_.revision;
}

bool HarvestStateMachine::complete_batch(const std::string & message)
{
  if (state_.target_active ||
    (state_.batch_state != BatchState::DISCOVERY && state_.batch_state != BatchState::RUNNING))
  {
    return false;
  }
  state_.batch_state = BatchState::COMPLETED;
  state_.run_active = false;
  state_.message = message;
  ++state_.revision;
  return true;
}

bool HarvestStateMachine::reset_batch()
{
  if (state_.batch_state != BatchState::COMPLETED &&
    state_.batch_state != BatchState::INTERRUPTED)
  {
    return false;
  }
  state_.counters = BatchCounters{};
  state_.outcomes.clear();
  state_.target_id.clear();
  state_.target_phase = TargetPhase::IDLE;
  state_.mode = OperationMode::AUTO;
  state_.batch_state = all_ready() ? BatchState::DISCOVERY : BatchState::WAITING_READY;
  state_.run_active = all_ready();
  state_.message = all_ready() ? "批次已复位，重新开始发现目标" : "批次已复位，等待系统就绪";
  ++state_.revision;
  return true;
}

bool HarvestStateMachine::set_target_phase(TargetPhase phase)
{
  if (!state_.target_active || state_.target_phase == phase) {return false;}
  state_.target_phase = phase;
  ++state_.revision;
  return true;
}

CommandResult HarvestStateMachine::finish_request(
  const std::string & request_id, bool accepted, const std::string & message, bool state_changed)
{
  if (state_changed) {++state_.revision;}
  const CommandResult result{accepted, message, state_.revision};
  if (!request_id.empty()) {
    if (request_results_.find(request_id) == request_results_.end()) {
      // 新条目入列；超过上限时淘汰最旧，防止长期运行缓存无界增长。
      request_order_.push_back(request_id);
      if (request_order_.size() > kMaxRequestResults) {
        request_results_.erase(request_order_.front());
        request_order_.pop_front();
      }
    }
    request_results_[request_id] = result;
  }
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
      if (state_.recovery_required) {
        return finish_request(request_id, false, "需先确认恢复", false);
      }
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
      // 预留，未启用：重试语义待复扫阶段定义，当前恒拒。
      return finish_request(request_id, false, "当前阶段不允许该命令", false);
    case ControlCommand::SKIP_TARGET:
      // 仅登记跳过意图；真正的 goal 取消由节点层向能力端传播。
      if (!state_.target_active) {
        return finish_request(request_id, false, "当前没有活动目标可跳过", false);
      }
      state_.message = "已请求跳过当前目标";
      return finish_request(request_id, true, state_.message, true);
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
