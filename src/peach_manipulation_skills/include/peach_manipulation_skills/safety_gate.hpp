// Copyright 2026, aubo_e5_ros2_ws authors
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
//    * Neither the name of the copyright holder nor the names of its
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
#ifndef PEACH_MANIPULATION_SKILLS__SAFETY_GATE_HPP_
#define PEACH_MANIPULATION_SKILLS__SAFETY_GATE_HPP_

#include <algorithm>
#include <functional>
#include <string>

#include "peach_manipulation_skills/safety_gate_base.hpp"

namespace peach_manipulation_skills
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
// 结果与原因字符串语义与原 ApproachGraspNode::safetyReady/cycleTargetReady
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

}  // namespace peach_manipulation_skills

#endif  // PEACH_MANIPULATION_SKILLS__SAFETY_GATE_HPP_
