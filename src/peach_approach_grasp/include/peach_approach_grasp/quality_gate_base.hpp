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
// 职责：重建/精化质量门职责的可替换接口（重构协议 2.14：抽象基类、签名最小
// 化、纯数据输入输出、零 ROS 类型）。
// 契约：判定为纯函数——同一份 QualitySnapshot 必须给出同一结果，实现不得
// 依赖调用时刻或外部数据源。调用端（bt_nodes / 预览服务）只持
// std::unique_ptr<QualityGateBase>，经 impl_factory.hpp 按名字创建。
#ifndef PEACH_APPROACH_GRASP__QUALITY_GATE_BASE_HPP_
#define PEACH_APPROACH_GRASP__QUALITY_GATE_BASE_HPP_

#include <cstddef>
#include <string>

namespace peach_approach_grasp
{

// 质量门判定的全部输入（纯值快照）：感知选中目标、重建状态与覆盖指标、
// 精化拟合指标、数据时效与许可标志。无效标量约定 -1 或极大值（见字段默认）。
struct QualitySnapshot
{
  std::string selected_target_id;
  std::string reconstruction_target_id;
  std::string refined_target_id;
  std::string reconstruction_state;
  std::size_t captured_views{0};
  double max_baseline_deg{0.0};
  double mean_nearest_baseline_deg{0.0};
  double mean_depth_ratio{0.0};
  double refined_rmse_m{-1.0};
  double refined_inlier_ratio{-1.0};
  double data_age_s{1.0e9};
  bool refined_accept{false};
  bool grasp_allowed{false};
};

// 门判定结果：allowed=false 时 reason 为稳定的机器可读原因串（调用端按
// 字符串做分支判断，实现不得随意改名）。
struct GateResult
{
  bool allowed{false};
  std::string reason;
};

// 质量门抽象基类。
// 用途：判定观察覆盖是否可 finalize、精化质量是否可预览/可抓取。
// 生命周期：由节点构造期/参数重载时经工厂创建，unique_ptr 独占持有。
// 线程安全：三个判定方法为 const 纯函数，只在 BT 工作线程/executor 回调调用。
// 可替换性：注册名见 impl_factory.hpp（默认实现 threshold）。
class QualityGateBase
{
public:
  virtual ~QualityGateBase() = default;

  // finalize 门：覆盖证据（视图数/基线/深度）与身份、时效是否达标。
  virtual GateResult readyToFinalize(const QualitySnapshot & snapshot) const = 0;
  // 接触轨迹预览门：预览只读锁存几何、不执行运动，时效要求由实现自定。
  virtual GateResult readyToPreviewContact(const QualitySnapshot & snapshot) const = 0;
  // 抓取门：finalize 后精化质量与许可是否达标（真实执行的最终质量门）。
  virtual GateResult readyToGrasp(const QualitySnapshot & snapshot) const = 0;
};

}  // namespace peach_approach_grasp

#endif  // PEACH_APPROACH_GRASP__QUALITY_GATE_BASE_HPP_
