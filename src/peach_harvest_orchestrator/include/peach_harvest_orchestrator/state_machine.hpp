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

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace peach_harvest_orchestrator
{
// 以下各枚举底层值与 peach_harvest_msgs 对应常量按声明顺序一一对应；纯核零 ROS
// 不 include 消息头，对齐由节点 TU 的 static_assert 编译期钉死
// （harvest_orchestrator_node.cpp）。新增枚举值必须同步消息常量并补断言。
enum class OperationMode : uint8_t {AUTO, PAUSED, MAINTENANCE};
enum class BatchState : uint8_t
{
  WAITING_READY, DISCOVERY, RUNNING, PAUSE_PENDING, PAUSED, MAINTENANCE,
  COMPLETED, RECOVERY_REQUIRED, INTERRUPTED
};
enum class TargetPhase : uint8_t
{
  IDLE, SELECTING, OBSERVING, FINALIZING, VALIDATING, APPROACHING,
  TOOL_ACTION, RETREATING, COMPLETING, SUCCEEDED, SKIPPED, FAILED
};
enum class ControlCommand : uint8_t
{
  PAUSE, RESUME, ENTER_MAINTENANCE, EXIT_MAINTENANCE, CANCEL_NOW,
  SKIP_TARGET, ACKNOWLEDGE_RECOVERY
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

struct TargetOutcomeRecord;

// 残局抬质量候选选择（纯函数，协议 2.8 残局 OBSERVE_ONLY，阶段 E3；节点层与
// 单测复用）：一轮正常派发耗尽后、RESCAN 复扫判定之前，从锁定集中挑出下一个
// 值得派发 OBSERVE_ONLY 周期（只观察+精化验证，不抓取）尝试抬质量的残局目标。
// 资格判据（合取）：
//   1. 该目标最新一条终局账为 SKIPPED_QUALITY（质量门未达标被跳过；
//      其后已成功/失败/取消的目标不再抬，observe 自身的终局账也会覆盖
//      资格——observe 成功记账后目标自然退出残局集）；
//   2. 仍在当前锁定集（known_target_ids，掉出锁定集说明感知已移除锚点，
//      派发必被能力端拒绝，不浪费次数）；
//   3. 本批次已派发的抬质量次数 < max_retries（次数在派发时消耗，
//      拒绝/取消不归还，保证能力端系统性拒绝下批次仍有界收口）；
//   4. 本轮尚未抬过（retired_this_round——每轮每目标至多一次，终局未改善
//      时让位下一个残局目标而非原地反复；复扫开新一轮时清册）。
// 返回优先级序（priority_order）中第一个满足资格的目标 ID，无候选返回空串。
// priority_order 由调用方给：自动批次=最近锁定帧观测序（感知固定优先级），
// 显式清单批次=清单自身顺序（清单外目标永不在序中，天然满足"不引入清单外
// 目标"约束）。
std::string pick_observe_retry_candidate(
  const std::vector<TargetOutcomeRecord> & outcomes,
  const std::vector<std::string> & priority_order,
  const std::unordered_set<std::string> & known_target_ids,
  const std::unordered_map<std::string, uint32_t> & retry_counts,
  const std::unordered_set<std::string> & retired_this_round,
  uint32_t max_retries);

// 发布周期 EMA 估算器（协议 2.11 新鲜度自适应的唯一实现，纯核零 ROS）：
// 订阅回调对每次消息到达调 observe()，相邻到达间隔做 EMA。线程安全由调用方
// 保证（编排器三路订阅均在同一把节点互斥锁下回调）。生命周期随节点成员，
// 不重置；对端重启后周期突变由 EMA 平滑吸收。
class PeriodEstimator
{
public:
  explicit PeriodEstimator(double alpha = 0.3)
  : alpha_(alpha) {}

  // 记录一次消息到达（秒，必须与新鲜度判定同一时钟源）。
  // 非正间隔（时钟回拨/同刻重复）跳过不计，防污染 EMA。
  void observe(double arrival_s)
  {
    if (has_last_) {
      const double interval = arrival_s - last_arrival_s_;
      if (interval > 0.0) {
        ema_s_ = has_ema_ ? alpha_ * interval + (1.0 - alpha_) * ema_s_ : interval;
        has_ema_ = true;
      }
    }
    last_arrival_s_ = arrival_s;
    has_last_ = true;
  }

  // 实测发布周期 EMA（秒）；不足两次到达返回 0（无实测通道，调用方回退配置下限）。
  double period_ema() const {return has_ema_ ? ema_s_ : 0.0;}

private:
  double alpha_;
  double last_arrival_s_{0.0};
  double ema_s_{0.0};
  bool has_last_{false};
  bool has_ema_{false};
};

// 自适应新鲜度阈值（协议 2.11，I4）：fresh ⟺ age ≤ max(配置下限, 2.5×实测发布周期EMA)。
// period_ema_s=0（从未收齐两次到达）时回退配置下限。语义：节点真死时 EMA 停留
// 在最后实测周期，断流超 2.5×EMA 即判失连；慢速话题（如 0.78FPS 感知，帧间隔
// ≈1.3s）阈值自动放宽到 ≈3.3s，避免固定 2s 下限裕度极薄造成的就绪 flap
// （flap 曾推 revision 导致控制命令批量被拒，I7 防抖的另一道闸）。
inline double adaptive_fresh_threshold(double min_s, double period_ema_s)
{
  return period_ema_s > 0.0 ? std::max(min_s, 2.5 * period_ema_s) : min_s;
}

struct Readiness
{
  bool perception{false};
  bool reconstruction{false};
  bool motion{false};
  bool web{false};
};

// 四路就绪输入样本（纯值）：时刻为秒且与 now_s 同源，0 表示从未收到；
// robot 四标志由节点从 int8 消息字段转成布尔后传入，纯核不认识消息类型。
// *_period_s 为各路实测发布周期 EMA（PeriodEstimator，0=无实测回退配置下限）。
struct ReadinessSample
{
  double now_s{0.0};
  double targets_received_s{0.0};
  double reconstruction_received_s{0.0};
  double robot_received_s{0.0};
  double targets_period_s{0.0};
  double reconstruction_period_s{0.0};
  double robot_period_s{0.0};
  bool robot_e_stopped{false};
  bool robot_in_error{false};
  bool robot_drives_powered{false};
  bool robot_motion_possible{false};
  bool action_server_ready{false};
  bool web_ready{true};
};

// 四路就绪推算（纯核，零 ROS）：fresh 判定（2.11 自适应阈值：max(配置下限,
// 2.5×实测发布周期EMA)）+ robot_status 判读 + motion 合成。
// 时间以秒注入，不接触 ROS clock；语义与原 refresh() 内联推算一致。
class ReadinessTracker
{
public:
  // timeout_s 为各路新鲜度的配置下限（readiness.timeout_s，热生效）。
  ReadinessTracker(double timeout_s, bool require_robot_status)
  : timeout_s_(timeout_s), require_robot_status_(require_robot_status) {}

  Readiness evaluate(const ReadinessSample & sample) const;

  // 未就绪路名（blockers 投影），顺序固定为 perception/reconstruction/motion/web。
  static std::vector<std::string> blockers(const Readiness & readiness);

private:
  double timeout_s_;
  bool require_robot_status_;
};

// 批次启动就位自校（阶段 F3，harvest.preflight_check）输入事实（纯值）：
// 四路就绪门只覆盖"话题心跳新鲜度"，不覆盖"静态几何就位"——TF 外参链
// （base_link→camera_link，extrinsics_publisher 静态发布）若没起来，拍照前置
// 与后续重建全部白跑且故障表现晦涩。节点层在批次首轮进入拍照前置链之前
// 采集本结构并调 evaluate_preflight 判定。
// 取舍说明：joint_states 不在检查项内——编排器节点本无 joint_states 订阅，
// 仅为自校新增一路高频订阅代价不合算；robot_status（已含上电/故障标志）+
// TF 外参链已足以覆盖"静态几何就位"语义，故只查 TF+robot_status。
struct PreflightFacts
{
  bool check_enabled{true};         // harvest.preflight_check 总开关（false 直通）
  bool tf_extrinsics_ready{false};  // TF base_link→camera_link 可查（静态链）
  // robot 检查是否启用：跟随 readiness.require_robot_status（热读）——该路
  // 被配置为不要求时，robot_status 可能根本无发布端，自校不应把它卡在门外。
  bool robot_check_required{true};
  bool robot_status_received{false};  // robot_status 已收到过（节点缓存时刻>0）
  bool robot_fault{false};            // 已收到且 e_stopped/in_error 任一置位
};

// 就位自校判定（纯函数，零 ROS 依赖，节点层与单测复用；幂等无状态——同一
// 事实输入恒得同一输出，节点层"下拍 refresh 重试"天然成立，连续失败不熔断，
// 环境稍后就位后自动放行）。
// 返回失败项标签清单（固定序：tf_extrinsics / robot_status_missing /
// robot_fault；check_enabled=false 或全部就绪时为空=放行）。
// robot_check_required=false 时跳过 robot 两项（对齐就绪门的可选语义）。
std::vector<std::string> evaluate_preflight(const PreflightFacts & facts);

// 目标派发前置判定（纯函数，协议 2.5 守卫 G-DISP 的纯核部分）：
// G1 拍照前置完成、G2 selected 非空、G3 非重复派发、G4 无活动目标、
// G5 能力端 action 服务就绪（含对端 lifecycle Active，节点侧折叠）、
// G6 motion 就绪（robot_status 新鲜∧非急停∧非故障∧已上电，I5；节点侧由
// ReadinessTracker 同源样本折算后传入）。六项齐备才允许派发。
bool allow_dispatch(
  bool photo_step_done, const std::string & selected_target_id,
  const std::string & last_dispatched_target, bool target_active,
  bool action_server_ready, bool motion_ready);

struct HarvestSnapshot;

// 当前快照下允许的控制命令列表（HarvestState.permissions 的填充源，纯函数）：
// 以 control() 的接受规则为允许表，按批次状态/活动目标/恢复标志投影——
// 活动目标：PAUSE/SKIP_TARGET/CANCEL_NOW；DISCOVERY/WAITING_READY：PAUSE/
// ENTER_MAINTENANCE/CANCEL_NOW；PAUSED：RESUME/ENTER_MAINTENANCE/CANCEL_NOW；
// MAINTENANCE：EXIT_MAINTENANCE/CANCEL_NOW；RECOVERY_REQUIRED：仅
// ACKNOWLEDGE_RECOVERY；COMPLETED/INTERRUPTED：无可控命令（批次已终结，
// 重开走 RunHarvest goal）。
std::vector<ControlCommand> allowed_commands(const HarvestSnapshot & snapshot);
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
// 单个已处理目标的记账条目。elapsed_s 为派发→终局墙钟（秒）；quality_score
// 为质量占位分（协议 2.13-E5 埋点，占位语义见节点侧填充注释，待阶段 E 接真实质量分）。
struct TargetOutcomeRecord
{
  std::string target_id;
  TargetOutcome outcome{TargetOutcome::SUCCEEDED};
  std::string reason;
  double elapsed_s{0.0};
  float quality_score{0.0f};
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
  // 更新四路就绪并刷新 blockers 投影。revision 语义（I7）：仅随真状态迁移
  // 递增——本函数只在 BatchState/run_active 变化时推 revision；blockers 投影
  // 变化与 message 刷新不推 revision（就绪 flap 不再 invalidate 乐观锁）。
  void update_readiness(const Readiness & readiness);
  bool begin_target(const std::string & target_id);
  void reach_safe_checkpoint();
  void require_recovery(const std::string & reason);
  // 记录单目标周期终态：清活动目标与阶段、记账 counters/outcomes，
  // 并把 PAUSE_PENDING 落到 PAUSED；无活动目标时忽略（防重复记账）。
  // elapsed_s=派发→终局墙钟（秒），quality_score=质量占位分（见
  // TargetOutcomeRecord 注释）；节点侧正常终局填入，拒绝/显式跳过路径留默认 0。
  void record_target_outcome(
    const std::string & target_id, TargetOutcome outcome, const std::string & reason,
    double elapsed_s = 0.0, float quality_score = 0.0f);
  // 被动终局记账（协议 2.4 目标丢失，阶段 D2）：锁定集目标在未派发、无活动
  // 周期的情况下被感知 anchor_drop 移除时入账——只动 counters/outcomes，
  // 不触碰 target_active/阶段/批次状态（与 record_target_outcome 的活动周期
  // 语义互斥：目标从未 begin_target，无周期可清）。批次可记账窗口
  // （DISCOVERY/RUNNING/PAUSE_PENDING/PAUSED 且未待恢复）外返回 false，
  // 由节点侧决定后续处置。elapsed/quality 无来源，恒记 0。
  bool record_passive_outcome(
    const std::string & target_id, TargetOutcome outcome, const std::string & reason);
  // 残局抬质量（OBSERVE_ONLY）周期的终局记账（协议 2.8 残局，阶段 E3）：
  // 与 record_target_outcome 同为"活动周期终局"语义（清 target_active/阶段、
  // 把 PAUSE_PENDING 落到 PAUSED、推 revision、无活动目标时忽略防迟到双记），
  // 差异在**不动 counters/attempted**——observe 周期的 SUCCEEDED 只是
  // "观察+精化验证完成"而非采摘成功，计入 succeeded 会虚报批次成果；
  // decide_round 的轮次终止口径（attempted 对 target_count）也不应被抬质量
  // 周期扰动。条目仍追加进 outcomes（reason 由节点侧带 observe_retry 标注），
  // HarvestSummary 可见、可审计（协议要求：记账但不重复计为采摘成功/失败）。
  // quality_score 无来源恒记 0。
  void record_observe_retry_outcome(
    const std::string & target_id, TargetOutcome outcome, const std::string & reason,
    double elapsed_s = 0.0);
  // 批次完成：仅 DISCOVERY/RUNNING 且无活动目标时转移到 COMPLETED；
  // message 可注明完成原因（如达到最大复拍轮次），默认“批次完成”。
  bool complete_batch(const std::string & message = "批次完成");
  // 批次复位：COMPLETED/INTERRUPTED 清账后按就绪度回到 DISCOVERY/WAITING_READY；
  // recovery_required 保持不变（仍需人工确认）。供复扫/重开一轮复用。
  bool reset_batch();
  // 仅活动目标期间更新阶段；阶段无变化或无活动目标时返回 false。
  // TargetPhase 透写属投影（I7）：阶段变化只更新字段并返回 true（供节点
  // 节流发布状态），不推 revision。
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
