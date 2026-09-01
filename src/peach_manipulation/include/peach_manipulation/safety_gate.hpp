// 功能：默认安全门（机器人就绪、目标新鲜度）。纯核，零 ROS。
#ifndef PEACH_MANIPULATION__SAFETY_GATE_HPP_
#define PEACH_MANIPULATION__SAFETY_GATE_HPP_

#include <algorithm>
#include <functional>
#include <string>

#include "peach_manipulation/safety_gate_base.hpp"

namespace peach_manipulation
{

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

// 执行前安全门默认实现（注册名 robot_status_gate，纯逻辑，零 ROS）：机器人
// 状态门 + 周期目标门。时钟以 std::function 注入（秒），数据以值入参；判定
// 结果与原因字符串语义与原 ManipulationSkillsNode::safetyReady/cycleTargetReady
// 内联实现完全一致。I5：robotReady 为硬件安全门，实现不得旁路（见
// safety_gate_base.hpp 契约注释）。
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
    config_.target_observation_max_age_s = value;
  }

private:
  SafetyGateConfig config_;
  std::function<double()> clock_s_;
};

}  // namespace peach_manipulation

#endif  // PEACH_MANIPULATION__SAFETY_GATE_HPP_
