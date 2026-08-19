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
  // fresh：收到过（时刻>0）且未超龄；各路阈值按 2.11 自适应
  // （max(配置下限, 2.5×实测发布周期EMA)，无实测回退配置下限）。
  const auto fresh = [&sample](double received_s, double limit_s) {
      return received_s > 0.0 && sample.now_s - received_s <= limit_s;
    };
  Readiness readiness;
  // 感知就绪只看话题新鲜度：锁定由派发前置保证，避免收齐窗口期卡回 WAITING_READY。
  readiness.perception = fresh(
    sample.targets_received_s,
    adaptive_fresh_threshold(timeout_s_, sample.targets_period_s));
  readiness.reconstruction = fresh(
    sample.reconstruction_received_s,
    adaptive_fresh_threshold(timeout_s_, sample.reconstruction_period_s));
  const double robot_limit_s = adaptive_fresh_threshold(timeout_s_, sample.robot_period_s);
  const bool robot_ready = !require_robot_status_ || (
    fresh(sample.robot_received_s, robot_limit_s) && !sample.robot_e_stopped &&
    !sample.robot_in_error &&
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

std::vector<std::string> evaluate_preflight(const PreflightFacts & facts)
{
  // 开关关闭直通（旧行为：不做就位自校）。
  if (!facts.check_enabled) {return {};}
  std::vector<std::string> failures;
  if (!facts.tf_extrinsics_ready) {failures.emplace_back("tf_extrinsics");}
  if (facts.robot_check_required) {
    if (!facts.robot_status_received) {
      failures.emplace_back("robot_status_missing");
    } else if (facts.robot_fault) {
      failures.emplace_back("robot_fault");
    }
  }
  return failures;
}

bool allow_dispatch(
  bool photo_step_done, const std::string & selected_target_id,
  const std::string & last_dispatched_target, bool target_active,
  bool action_server_ready, bool motion_ready)
{
  // G1–G6 合取（协议 2.5 G-DISP）：G6 失能门缺失时，急停/未上电窗口内照样
  // 派发，goal 必被能力端拒绝并误入拒绝熔断链（R3 的根源之一）。
  return photo_step_done && !selected_target_id.empty() &&
         selected_target_id != last_dispatched_target && !target_active &&
         action_server_ready && motion_ready;
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

std::string pick_observe_retry_candidate(
  const std::vector<TargetOutcomeRecord> & outcomes,
  const std::vector<std::string> & priority_order,
  const std::unordered_set<std::string> & known_target_ids,
  const std::unordered_map<std::string, uint32_t> & retry_counts,
  const std::unordered_set<std::string> & retired_this_round,
  uint32_t max_retries)
{
  // 每目标最新终局账：outcomes 按发生顺序追加，后者覆盖前者。
  // 锁定集规模小（十级）、感知帧率低（0.78FPS），线性建表开销可忽略。
  std::unordered_map<std::string, TargetOutcome> latest_outcome;
  latest_outcome.reserve(outcomes.size());
  for (const auto & record : outcomes) {
    latest_outcome[record.target_id] = record.outcome;
  }
  for (const auto & id : priority_order) {
    // 资格 1：最新终局账为 SKIPPED_QUALITY
    const auto outcome_it = latest_outcome.find(id);
    if (outcome_it == latest_outcome.end() ||
      outcome_it->second != TargetOutcome::SKIPPED_QUALITY)
    {
      continue;
    }
    // 资格 2：仍在当前锁定集（感知锚点未移除）
    if (known_target_ids.count(id) == 0) {continue;}
    // 资格 4：本轮未抬过（每轮每目标至多一次）
    if (retired_this_round.count(id) != 0) {continue;}
    // 资格 3：批次内次数预算未耗尽
    const auto count_it = retry_counts.find(id);
    if (count_it != retry_counts.end() && count_it->second >= max_retries) {continue;}
    return id;
  }
  return "";
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
  const auto previous_state = state_.batch_state;
  const bool previous_active = state_.run_active;
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
  // revision 只随真状态迁移递增（I7）：本函数可能的真迁移仅
  // WAITING_READY→DISCOVERY（BatchState/run_active 变化）；blockers 投影
  // 变化、readiness 翻转、message 刷新一律不推 revision——否则 0.78FPS 感知
  // 在固定 2s 新鲜度门下的就绪 flap 会持续 invalidate 乐观锁，
  // 控制命令被批量误拒（"状态版本已过期"）。
  if (previous_state != state_.batch_state || previous_active != state_.run_active) {
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
  const std::string & target_id, TargetOutcome outcome, const std::string & reason,
  double elapsed_s, float quality_score)
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
  state_.outcomes.push_back(
    TargetOutcomeRecord{target_id, outcome, reason, elapsed_s, quality_score});
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

bool HarvestStateMachine::record_passive_outcome(
  const std::string & target_id, TargetOutcome outcome, const std::string & reason)
{
  // 批次可记账窗口：采摘态（含暂停/暂停挂起）且未待恢复；批次终态
  // （COMPLETED/INTERRUPTED/RECOVERY_REQUIRED）与未开批次无账可记。
  if (state_.recovery_required ||
    (state_.batch_state != BatchState::DISCOVERY &&
    state_.batch_state != BatchState::RUNNING &&
    state_.batch_state != BatchState::PAUSE_PENDING &&
    state_.batch_state != BatchState::PAUSED))
  {
    return false;
  }
  ++state_.counters.attempted;
  switch (outcome) {
    case TargetOutcome::SUCCEEDED:
      ++state_.counters.succeeded;
      break;
    case TargetOutcome::SKIPPED_QUALITY:
      ++state_.counters.skipped_quality;
      break;
    case TargetOutcome::SKIPPED_UNREACHABLE:
      ++state_.counters.skipped_unreachable;
      break;
    case TargetOutcome::FAILED:
      ++state_.counters.failed;
      break;
    case TargetOutcome::CANCELED:
      ++state_.counters.canceled;
      break;
  }
  state_.outcomes.push_back(TargetOutcomeRecord{target_id, outcome, reason, 0.0, 0.0f});
  // 账本变化属真状态迁移（counters/outcomes 进 summary），推 revision；
  // message 不覆写——批次过程文案（轮次/收齐进度）比单条丢失原因更有用。
  ++state_.revision;
  return true;
}

void HarvestStateMachine::record_observe_retry_outcome(
  const std::string & target_id, TargetOutcome outcome, const std::string & reason,
  double elapsed_s)
{
  // 无活动目标时不记账（迟到终局，I9；与 record_target_outcome 同一防御）。
  if (!state_.target_active) {return;}
  // 只追加审计条目，不动 counters/attempted（见头文件注释：observe 终局
  // 不等于采摘成功/失败；轮次终止口径不受扰动）。
  state_.outcomes.push_back(TargetOutcomeRecord{target_id, outcome, reason, elapsed_s, 0.0f});
  state_.target_active = false;
  state_.target_phase = TargetPhase::IDLE;
  if (state_.batch_state == BatchState::PAUSE_PENDING) {
    // 暂停请求挂起期间周期结束：直接落到 PAUSED（与正常终局同语义）。
    state_.batch_state = BatchState::PAUSED;
    state_.mode = OperationMode::PAUSED;
    state_.message = "已在安全检查点暂停";
  } else {
    state_.message = reason.empty() ? "残局抬质量周期完成" : reason;
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
  // TargetPhase 透写是能力端反馈的投影（I7）：不推 revision，否则高频阶段
  // 推进会让乐观锁持续漂移；返回 true 仅供节点侧节流发布状态。
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

std::vector<ControlCommand> allowed_commands(const HarvestSnapshot & snapshot)
{
  // 允许表与 control() 的接受规则一一对应（见头文件注释）；批次已终结
  // （COMPLETED/INTERRUPTED）时不再接受任何控制命令，重开走 RunHarvest goal。
  switch (snapshot.batch_state) {
    case BatchState::COMPLETED:
    case BatchState::INTERRUPTED:
      return {};
    case BatchState::RECOVERY_REQUIRED:
      // 恢复未确认前所有运动类命令都被 control() 拒绝，只允许确认恢复。
      return {ControlCommand::ACKNOWLEDGE_RECOVERY};
    case BatchState::MAINTENANCE:
      return {ControlCommand::EXIT_MAINTENANCE, ControlCommand::CANCEL_NOW};
    case BatchState::PAUSED:
      return {
        ControlCommand::RESUME, ControlCommand::ENTER_MAINTENANCE,
        ControlCommand::CANCEL_NOW};
    case BatchState::RUNNING:
    case BatchState::PAUSE_PENDING:
      if (snapshot.target_active) {
        return {
          ControlCommand::PAUSE, ControlCommand::SKIP_TARGET,
          ControlCommand::CANCEL_NOW};
      }
      return {
        ControlCommand::PAUSE, ControlCommand::ENTER_MAINTENANCE,
        ControlCommand::CANCEL_NOW};
    case BatchState::WAITING_READY:
    case BatchState::DISCOVERY:
      return {
        ControlCommand::PAUSE, ControlCommand::ENTER_MAINTENANCE,
        ControlCommand::CANCEL_NOW};
  }
  return {};
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
