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
#ifndef PEACH_APPROACH_GRASP__QUALITY_GATE_HPP_
#define PEACH_APPROACH_GRASP__QUALITY_GATE_HPP_

#include <cstddef>
#include <string>

#include "peach_approach_grasp/quality_gate_base.hpp"

namespace peach_approach_grasp
{

// 默认值以 config/approach_grasp.yaml 为权威源，此处仅为直接构造兜底
// （2026-08-14 对齐 yaml 验证期放宽档）。
struct QualityGateConfig
{
  std::size_t minimum_views{3};
  double minimum_baseline_deg{15.0};
  double minimum_mean_nearest_baseline_deg{6.0};
  double minimum_mean_depth_ratio{0.40};
  double maximum_refined_rmse_m{0.01};
  double minimum_refined_inlier_ratio{0.35};
  double maximum_data_age_s{2.0};
};

// 默认质量门实现（注册名 threshold）：固定阈值档的身份一致性 + 视图覆盖 +
// 精化拟合质量判定。线程安全与生命周期约定见 QualityGateBase。
class QualityGate : public QualityGateBase
{
public:
  explicit QualityGate(QualityGateConfig config = QualityGateConfig());

  GateResult readyToFinalize(const QualitySnapshot & snapshot) const override;
  GateResult readyToPreviewContact(const QualitySnapshot & snapshot) const override;
  GateResult readyToGrasp(const QualitySnapshot & snapshot) const override;

private:
  GateResult commonIdentityGate(const QualitySnapshot & snapshot) const;
  QualityGateConfig config_;
};

}  // namespace peach_approach_grasp

#endif  // PEACH_APPROACH_GRASP__QUALITY_GATE_HPP_
