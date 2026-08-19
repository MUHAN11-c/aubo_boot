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
// 职责：执行前安全门职责的可替换接口（重构协议 2.14：抽象基类、签名最小化、
// 纯数据输入输出、零 ROS 类型）。
// 契约（I5，接口级安全不变量）：robotReady 是硬件安全门，任何实现都不得以
// 任何形式旁路或恒放行——所有真实运动下发前必须经过它；替换实现只能让门
// 更严，不得更松（require_robot_status=false 的放行是部署期显式配置，不属
// 于实现层旁路）。调用端（节点 safetyReady/cycleTargetReady 薄壳）只持
// std::unique_ptr<SafetyGateBase>，经 impl_factory.hpp 按名字创建。
#ifndef PEACH_MANIPULATION_SKILLS__SAFETY_GATE_BASE_HPP_
#define PEACH_MANIPULATION_SKILLS__SAFETY_GATE_BASE_HPP_

#include <string>

namespace peach_manipulation_skills
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
// 生命周期：由节点构造期/参数重载时经工厂创建，unique_ptr 独占持有；
//   时钟由实现构造时注入（秒），实现内部不得直接读系统时钟。
// 线程安全：判定方法为 const，可被 BT 工作线程与 executor 回调并发调用；
//   set_target_observation_max_age_s 由订阅回调（executor 线程）调用，实现
//   须保证与判定方法的并发安全（默认实现为平凡双精度写，沿用既有语义）。
// 可替换性：注册名见 impl_factory.hpp（默认实现 robot_status_gate）。
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

}  // namespace peach_manipulation_skills

#endif  // PEACH_MANIPULATION_SKILLS__SAFETY_GATE_BASE_HPP_
