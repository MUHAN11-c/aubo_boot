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
// 职责：可替换实现的显式注册与按名创建（重构协议 2.14：工厂函数按实现名
// 字符串创建，显式注册清单，一处 if 链，不用自注册宏）。
// 本头只覆盖纯核三件套（视点规划/质量门/安全门）；运动接口工厂涉及
// MoveIt/TF 类型，见包内私有头 src/motion_factory.hpp。
// 失败语义：未知名抛 std::invalid_argument，信息列出全部可用注册名。
#ifndef PEACH_APPROACH_GRASP__IMPL_FACTORY_HPP_
#define PEACH_APPROACH_GRASP__IMPL_FACTORY_HPP_

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "peach_approach_grasp/quality_gate.hpp"
#include "peach_approach_grasp/safety_gate.hpp"
#include "peach_approach_grasp/view_planner.hpp"

namespace peach_approach_grasp
{

// 视点规划器注册表：spherical_adaptive=球面自适应候选生成（当前唯一实现）。
inline std::unique_ptr<ViewPlannerBase> createViewPlanner(
  const std::string & name, const ViewPlannerConfig & config)
{
  if (name == "spherical_adaptive") {
    return std::make_unique<ViewPlanner>(config);
  }
  throw std::invalid_argument(
          "未知 view_planner.impl: '" + name + "'（可用: spherical_adaptive）");
}

// 质量门注册表：threshold=固定阈值档身份/覆盖/精化判定（当前唯一实现）。
inline std::unique_ptr<QualityGateBase> createQualityGate(
  const std::string & name, const QualityGateConfig & config)
{
  if (name == "threshold") {
    return std::make_unique<QualityGate>(config);
  }
  throw std::invalid_argument(
          "未知 quality_gate.impl: '" + name + "'（可用: threshold）");
}

// 安全门注册表：robot_status_gate=robot_status 状态门 + 目标新鲜度门
// （当前唯一实现）。I5：任何注册的实现都不得旁路硬件安全门。
inline std::unique_ptr<SafetyGateBase> createSafetyGate(
  const std::string & name, const SafetyGateConfig & config,
  std::function<double()> clock_s)
{
  if (name == "robot_status_gate") {
    return std::make_unique<SafetyGate>(config, std::move(clock_s));
  }
  throw std::invalid_argument(
          "未知 safety_gate.impl: '" + name + "'（可用: robot_status_gate）");
}

}  // namespace peach_approach_grasp

#endif  // PEACH_APPROACH_GRASP__IMPL_FACTORY_HPP_
