// 功能：默认安全门（机器人就绪、目标新鲜度）。纯核，零 ROS。
#ifndef PEACH_MANIPULATION__SAFETY_GATE_HPP_
#define PEACH_MANIPULATION__SAFETY_GATE_HPP_

#include <algorithm>
#include <atomic>
#include <functional>
#include <string>

namespace peach_manipulation
{

// robot_status 的纯值快照：节点把 int8 消息字段转成布尔后传入，纯核不认识消息类型。
struct RobotStatusSample
{
  bool received{false};    // 是否已收到过 robot_status
  double received_s{0.0};  // 接收时刻（秒，与注入时钟同源）
  bool e_stopped{false};
  bool in_error{false};
  bool drives_powered{false};
  bool motion_possible{false};
};

// 周期目标的纯值快照：id 在观测无效时也可能非空（身份一致性判定先于有效性判定）。
struct TargetGateSample
{
  std::string id;
  bool valid{false};
  double received_s{0.0};
};

// 执行前安全门抽象基类：机器人状态门 + 周期目标门。
// 用途：所有运动下发前判定硬件状态与目标身份/新鲜度。
// 生命周期：由节点构造期/参数重载时重建，unique_ptr 独占持有；
//   时钟由实现构造时注入（秒），实现内部不得直接读系统时钟。
// 线程安全：判定方法为 const，可被周期工作线程与 executor 回调并发调用；
//   set_target_observation_max_age_s 由订阅回调（executor 线程）调用，实现
//   须保证与判定方法的并发安全（默认实现走 std::atomic<double>，消撕裂写）。
// 可替换性：唯一实现 SafetyGate（节点直接构造）。
class SafetyGateBase
{
public:
  virtual ~SafetyGateBase() = default;

  // 机器人状态门（I5：任何实现不得旁路）。
  // 前置：sample 由调用端在锁内组装，时刻与实现注入时钟同源。
  // 后置：返回 false 时 reason 为稳定原因串（robot_status_missing/stale/
  //   not_motion_ready 等），调用端据此中止运动。
  virtual bool robotReady(
    const RobotStatusSample & sample, std::string & reason) const = 0;
  // 周期目标门：目标身份一致、观测有效、未超龄。
  virtual bool targetReady(
    const TargetGateSample & sample, const std::string & target_id,
    std::string & reason) const = 0;
  // 运行期按实测帧率自适应调整目标观测新鲜度上限。
  virtual void set_target_observation_max_age_s(double value) = 0;
};

// 帧率自适应超时（纯函数）：等待预算以帧数表达（per_frame_mult），按实测帧
// 间隔 EMA 折成秒并夹在 [floor_s, cap_s]。帧率以运行状态为准——高帧率自动
// 收紧提速，低帧率自动放宽防误判；ema 未测得（≤0）由调用方回退到配置值。
inline double adaptive_timeout_s(
  double frame_interval_ema_s, double per_frame_mult, double margin_s,
  double floor_s, double cap_s)
{
  return std::clamp(
    per_frame_mult * frame_interval_ema_s + margin_s, floor_s, cap_s);
}

struct SafetyGateConfig
{
  bool require_robot_status{true};
  double robot_status_max_age_s{1.0};
  double target_observation_max_age_s{3.0};
};

// 执行前安全门默认实现（纯逻辑，零 ROS）：机器人状态门 + 周期目标门。
// 时钟以 std::function 注入（秒），数据以值入参；判定结果与原因字符串语义
// 与原 ManipulationSkillsNode::safetyReady/cycleTargetReady 内联实现完全一致。
// I5：robotReady 为硬件安全门，实现不得旁路（见 SafetyGateBase 契约注释）。
class SafetyGate : public SafetyGateBase
{
public:
  SafetyGate(SafetyGateConfig config, std::function<double()> clock_s);

  // 机器人状态门：require_robot_status=false 时直接放行；否则要求已收到、
  // 未超龄、无急停/错误且驱动已上电可运动。
  bool robotReady(const RobotStatusSample & sample, std::string & reason) const override;
  // 周期目标门：目标身份一致、观测有效、未超龄。
  bool targetReady(
    const TargetGateSample & sample, const std::string & target_id,
    std::string & reason) const override;

  // 运行期按实测帧率自适应调整目标观测新鲜度上限（见 adaptive_timeout_s）。
  void set_target_observation_max_age_s(double value) override
  {
    target_observation_max_age_s_.store(value, std::memory_order_relaxed);
  }

private:
  SafetyGateConfig config_;
  // 自适应新鲜度上限：订阅回调写、周期 worker 读；构造后 config_ 字段
  // 不再变更，运行期调整只走这个原子量（平凡 double 并发读写是 UB）。
  std::atomic<double> target_observation_max_age_s_;
  std::function<double()> clock_s_;
};

}  // namespace peach_manipulation

#endif  // PEACH_MANIPULATION__SAFETY_GATE_HPP_
