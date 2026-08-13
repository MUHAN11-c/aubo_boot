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

#include <cstddef>
#include <cstdint>
#include <deque>
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
// 单目标周期终态，取值与 peach_harvest_msgs/TargetOutcome.msg 常量一一对应。
enum class TargetOutcome : uint8_t
{
  SUCCEEDED, SKIPPED_QUALITY, SKIPPED_UNREACHABLE, FAILED, CANCELED
};

// 复扫轮次终止判定结果（decide_round 的返回枚举）。
enum class RoundDecision : uint8_t
{
  WAIT,      // 本轮尚未结束：未锁定或锁定目标未全部处理完
  RESCAN,    // 开启新一轮复扫（回拍照位姿并重置感知）
  COMPLETE   // 终止批次（空集/复扫关闭/达到最大轮次）
};
struct RoundVerdict
{
  RoundDecision decision{RoundDecision::WAIT};
  // COMPLETE 时的批次完成原因（写入状态机 message），其余情况为空。
  std::string message;
};

// 复扫轮次终止判定（纯函数，零 ROS 依赖，供节点层与单测复用）：
// - locked=false 或本轮已处理数 < 锁定目标数：WAIT（继续等锁定/周期终态；
//   锁定后 selected 暂时为空可能只是质量待恢复，不算本轮结束）；
// - 本轮锁定空集：COMPLETE（没有可摘目标）；
// - 本轮目标全部处理完且允许复扫且未达轮次上限：RESCAN；
// - 达到最大轮次或复扫关闭：COMPLETE（message 注明原因）。
// round 从 1 起计；max_rounds 为总轮次上限（含首轮）。
RoundVerdict decide_round(
  bool locked, uint32_t target_count, uint32_t processed_count,
  uint32_t round, uint32_t max_rounds, bool rescan_enabled);

struct Readiness
{
  bool perception{false};
  bool reconstruction{false};
  bool motion{false};
  bool web{false};
};

// 四路就绪输入样本（纯值）：时刻为秒且与 now_s 同源，0 表示从未收到；
// robot 四标志由节点从 int8 消息字段转成布尔后传入，纯核不认识消息类型。
struct ReadinessSample
{
  double now_s{0.0};
  double targets_received_s{0.0};
  double reconstruction_received_s{0.0};
  double robot_received_s{0.0};
  bool robot_e_stopped{false};
  bool robot_in_error{false};
  bool robot_drives_powered{false};
  bool robot_motion_possible{false};
  bool action_server_ready{false};
  bool web_ready{true};
};

// 四路就绪推算（纯核，零 ROS）：fresh 判定 + robot_status 判读 + motion 合成。
// 时间以秒注入，不接触 ROS clock；语义与原 refresh() 内联推算完全一致。
class ReadinessTracker
{
public:
  ReadinessTracker(double timeout_s, bool require_robot_status)
  : timeout_s_(timeout_s), require_robot_status_(require_robot_status) {}

  Readiness evaluate(const ReadinessSample & sample) const;

  // 未就绪路名（blockers 投影），顺序固定为 perception/reconstruction/motion/web。
  static std::vector<std::string> blockers(const Readiness & readiness);

private:
  double timeout_s_;
  bool require_robot_status_;
};

// 目标派发前置判定（纯函数）：拍照前置完成、selected 非空、非重复派发、
// 无活动目标、能力端 action 服务就绪，五项齐备才允许派发。
bool allow_dispatch(
  bool photo_step_done, const std::string & selected_target_id,
  const std::string & last_dispatched_target, bool target_active,
  bool action_server_ready);
struct OperationPolicy
{
  bool auto_start_enabled{true};
  bool execution_enabled{false};
  bool grasp_enabled{false};
  bool tool_enabled{false};
};
// 批次计数账：各终态次数，attempted 为全部已处理目标数。
struct BatchCounters
{
  uint32_t attempted{0};
  uint32_t succeeded{0};
  uint32_t skipped_quality{0};
  uint32_t skipped_unreachable{0};
  uint32_t failed{0};
  uint32_t canceled{0};
};
// 单个已处理目标的记账条目。
struct TargetOutcomeRecord
{
  std::string target_id;
  TargetOutcome outcome{TargetOutcome::SUCCEEDED};
  std::string reason;
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
  BatchCounters counters;
  std::vector<TargetOutcomeRecord> outcomes;
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
  // 记录单目标周期终态：清活动目标与阶段、记账 counters/outcomes，
  // 并把 PAUSE_PENDING 落到 PAUSED；无活动目标时忽略（防重复记账）。
  void record_target_outcome(
    const std::string & target_id, TargetOutcome outcome, const std::string & reason);
  // 批次完成：仅 DISCOVERY/RUNNING 且无活动目标时转移到 COMPLETED；
  // message 可注明完成原因（如达到最大复拍轮次），默认“批次完成”。
  bool complete_batch(const std::string & message = "批次完成");
  // 批次复位：COMPLETED/INTERRUPTED 清账后按就绪度回到 DISCOVERY/WAITING_READY；
  // recovery_required 保持不变（仍需人工确认）。供复扫/重开一轮复用。
  bool reset_batch();
  // 仅活动目标期间更新阶段；阶段无变化或无活动目标时返回 false 且不动 revision。
  bool set_target_phase(TargetPhase phase);
  CommandResult control(
    ControlCommand command, const std::string & request_id, uint64_t expected_revision);
  CommandResult set_policy(
    const OperationPolicy & policy, const std::string & request_id, uint64_t expected_revision);

private:
  // 幂等结果缓存上限：超过后按插入顺序淘汰最旧条目。
  static constexpr size_t kMaxRequestResults{256};
  CommandResult finish_request(
    const std::string & request_id, bool accepted, const std::string & message,
    bool state_changed);
  bool all_ready() const noexcept;
  void refresh_blockers();
  Readiness readiness_;
  HarvestSnapshot state_;
  std::unordered_map<std::string, CommandResult> request_results_;
  std::deque<std::string> request_order_;
};
}  // namespace peach_harvest_orchestrator
#endif  // PEACH_HARVEST_ORCHESTRATOR__STATE_MACHINE_HPP_
