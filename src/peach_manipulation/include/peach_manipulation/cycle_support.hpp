// 功能：周期支撑（扫描预算、阶段墙钟、回调耗时 RAII）。包内私用，零业务分支。
#ifndef PEACH_MANIPULATION__CYCLE_SUPPORT_HPP_
#define PEACH_MANIPULATION__CYCLE_SUPPORT_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include "peach_manipulation/cycle_state.hpp"

// 功能：观察段扫描预算。覆盖够就停；否则把 maximum_moves 走完。

namespace peach_manipulation
{

// 停准则对齐体积重建惯例（Open3D TSDF 对一组相机位姿逐张 integrate；
// 工业 NBV 在 max_views 或信息增益够了才停），而不是用「上次移动+等帧」
// 预测下一视点买不买得起：
//   * 质量门放行 → CONVERGED（不再为凑次数运动）；
//   * moves < maximum_moves → CONTINUE（把设计好的短移走完）；
//   * 否则 MOVES_EXHAUSTED。
// 墙钟 time_budget_s 只作日志对照。等帧超时在 waitForFreshTarget /
// waitForNewStation，不在这里用 EMA 放大预算或预测收口。
// 08-31 1633/1636：等帧把 EMA 抬到 ~8 s，15 s 预算剩 6 s，8 cm 第二机位被
// 预测收口砍掉，基线永远不够。
// 纯核零 ROS、零阻塞、零时钟依赖。

struct ScanBudgetConfig
{
  int maximum_moves{2};
  int min_effective_views{1};
  double time_budget_s{15.0};
};

enum class ScanVerdict
{
  CONTINUE,
  CONVERGED,
  BUDGET_EXHAUSTED,
  MOVES_EXHAUSTED
};

class ScanBudget
{
public:
  explicit ScanBudget(ScanBudgetConfig config = ScanBudgetConfig())
  : config_(config)
  {
  }

  double effectiveBudgetS(double /*move_cost_ema_s*/) const
  {
    return config_.time_budget_s;
  }

  ScanVerdict poll(
    bool gate_allowed, int moves, int /*effective_views*/, double /*elapsed_s*/,
    double /*move_cost_ema_s*/) const
  {
    if (gate_allowed) {
      return ScanVerdict::CONVERGED;
    }
    if (moves < config_.maximum_moves) {
      return ScanVerdict::CONTINUE;
    }
    return ScanVerdict::MOVES_EXHAUSTED;
  }

private:
  ScanBudgetConfig config_;
};

}  // namespace peach_manipulation

// 功能：周期各阶段墙钟耗时累计。终局投影只认 CycleState 枚举。

namespace peach_manipulation
{
// 周期阶段耗时埋点（重构阶段 C）纯核：CycleState → 阶段名投影与墙钟累计。
// 阶段名集合与 ExecuteTarget.action Result 的 stage_names 契约一一对应
// （test_action_contract 钉死）；名字一律小写串，与 action 注释保持同步。
// prepare：周期受理（onStart/previewContact 放行）到首次进入后续阶段的隐式段；
// reconfirm：抓取前再确认段（阶段 E1，CycleState::RECONFIRM 投影）。
inline constexpr std::array<const char *, 7> kStageNames = {
  "prepare", "observe", "finalize", "reconfirm", "approach_insert", "tool", "retreat"};

// CycleState → 阶段名投影；非阶段态（IDLE/全部终态/plan-only 圆满态）返回
// nullptr，表示"当前不处于任何计时阶段"（StageTimer 据此收口）。
// observe 合并观察段三态（PLAN_OBSERVATION/MOVE_TO_VIEW/WAIT_FRAME）；
// approach_insert 合并 MTC 靠近插入与接触轨迹预览规划（PREVIEW 周期同段计时）。
inline const char * stageForState(CycleState state)
{
  switch (state) {
    case CycleState::PLAN_OBSERVATION:
    case CycleState::MOVE_TO_VIEW:
    case CycleState::WAIT_FRAME:
      return "observe";
    case CycleState::FINALIZE:
      return "finalize";
    case CycleState::RECONFIRM:
      return "reconfirm";
    case CycleState::MTC_APPROACH_INSERT:
    case CycleState::PREVIEW_CONTACT_PLANNING:
      return "approach_insert";
    case CycleState::ACTUATE_TOOL:
      return "tool";
    case CycleState::MTC_RETREAT:
      return "retreat";
    default:
      return nullptr;
  }
}

// 单个阶段的累计耗时（插入序排列，同名阶段多次进入累计到同一桶）。
struct StageDuration
{
  std::string name;
  std::chrono::duration<double> elapsed{0.0};
};

// 阶段墙钟计时器：steady_clock 时间点由调用方注入（节点侧在 setState 的
// cycle_state_ 变更点喂入；测试用合成时间点）。start 进入 prepare 段，
// 每次 onStateChange 把上一段累计进对应桶并切换当前段；投影为 nullptr 的
// 状态（终态/IDLE）使计时收口，此后一切变更被忽略。未 start 前全部 no-op。
// 非线程安全：节点侧一律在 state_mutex_ 内访问。
class StageTimer
{
public:
  using Clock = std::chrono::steady_clock;

  // 开始一个新周期：清空上一周期记录，当前段置为 prepare。
  void start(Clock::time_point now)
  {
    entries_.clear();
    started_ = true;
    closed_ = false;
    last_ = now;
    enterStage("prepare");
  }

  // 状态切换点：累计上一段耗时，切到 state 投影的新段；nullptr 投影即收口。
  void onStateChange(CycleState state, Clock::time_point now)
  {
    if (!started_ || closed_) {
      return;
    }
    accrue(now);
    const char * stage = stageForState(state);
    if (stage == nullptr) {
      closed_ = true;
      current_ = nullptr;
      return;
    }
    enterStage(stage);
  }

  // 显式收口（终局组装 result 时兜底）：把当前段累计完成后关闭计时。
  void close(Clock::time_point now)
  {
    if (!started_ || closed_) {
      return;
    }
    accrue(now);
    closed_ = true;
    current_ = nullptr;
  }

  bool active() const {return started_ && !closed_;}

  // 已历经阶段（含 prepare），按首次进入顺序；未经历的阶段不出现。
  const std::vector<StageDuration> & entries() const {return entries_;}

private:
  void enterStage(const char * stage)
  {
    current_ = stage;
    for (auto & entry : entries_) {
      if (entry.name == stage) {
        return;
      }
    }
    entries_.push_back(StageDuration{stage, std::chrono::duration<double>{0.0}});
  }

  void accrue(Clock::time_point now)
  {
    if (current_ == nullptr || now <= last_) {
      last_ = now > last_ ? now : last_;
      return;
    }
    for (auto & entry : entries_) {
      if (entry.name == current_) {
        entry.elapsed += now - last_;
        break;
      }
    }
    last_ = now;
  }

  bool started_{false};
  bool closed_{false};
  const char * current_{nullptr};
  Clock::time_point last_{};
  std::vector<StageDuration> entries_;
};

}  // namespace peach_manipulation

// 功能：回调耗时 RAII 计时器与累计表。随 ~/status 发布，不新增话题。
// 用法：回调入口栈上构造 ScopedTimer；析构按标签记耗时（steady_clock）。

namespace peach_manipulation
{
// 回调耗时累计注册表：线程安全自给（节点回调跨默认组/规划组并发）。
// 输出为标签 → {count, total_ms, max_ms, last_ms} 的 JSON 投影。
class CallbackTimingRegistry
{
public:
  struct Entry
  {
    uint64_t count{0};
    double total_ms{0.0};
    double max_ms{0.0};
    double last_ms{0.0};
  };

  void record(const std::string & label, double elapsed_ms)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto & entry = entries_[label];
    entry.count += 1;
    entry.total_ms += elapsed_ms;
    entry.max_ms = elapsed_ms > entry.max_ms ? elapsed_ms : entry.max_ms;
    entry.last_ms = elapsed_ms;
  }

  nlohmann::json toJson() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    nlohmann::json out = nlohmann::json::object();
    for (const auto & [label, entry] : entries_) {
      out[label] = {
        {"count", entry.count},
        {"total_ms", entry.total_ms},
        {"max_ms", entry.max_ms},
        {"last_ms", entry.last_ms},
      };
    }
    return out;
  }

  // 测试用快照（值拷贝，避免暴露内部锁）。
  std::map<std::string, Entry> snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return entries_;
  }

private:
  mutable std::mutex mutex_;
  std::map<std::string, Entry> entries_;
};

// RAII 计时器：构造取起始点，析构计算耗时——注入注册表则累计一条记录，
// 并按节流周期输出 DEBUG 日志。禁止拷贝/移动（析构只应发生一次）。
class ScopedTimer
{
public:
  explicit ScopedTimer(
    rclcpp::Logger logger, std::string label,
    CallbackTimingRegistry * registry = nullptr, double throttle_ms = 2000.0)
  : logger_(std::move(logger)),
    label_(std::move(label)),
    registry_(registry),
    throttle_ms_(throttle_ms),
    start_(std::chrono::steady_clock::now())
  {
  }

  ScopedTimer(const ScopedTimer &) = delete;
  ScopedTimer & operator=(const ScopedTimer &) = delete;

  ~ScopedTimer()
  {
    const double elapsed = elapsedMs();
    if (registry_ != nullptr) {
      registry_->record(label_, elapsed);
    }
    RCLCPP_DEBUG_THROTTLE(
      logger_, steady_clock_, throttle_ms_,
      "回调耗时 %s: %.2f ms", label_.c_str(), elapsed);
  }

  double elapsedMs() const
  {
    return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start_).count();
  }

private:
  rclcpp::Logger logger_;
  std::string label_;
  CallbackTimingRegistry * registry_;
  double throttle_ms_;
  std::chrono::steady_clock::time_point start_;
  // 节流宏需要时钟左值；RCL_STEADY_TIME 与耗时口径一致。
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
};

}  // namespace peach_manipulation

#endif  // PEACH_MANIPULATION__CYCLE_SUPPORT_HPP_
