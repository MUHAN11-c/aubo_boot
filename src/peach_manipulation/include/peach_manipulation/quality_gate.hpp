// 功能：默认质量门。阈值以 config/peach_manipulation.yaml 为准。
#ifndef PEACH_MANIPULATION__QUALITY_GATE_HPP_
#define PEACH_MANIPULATION__QUALITY_GATE_HPP_

#include <cstddef>
#include <string>

#include "peach_manipulation/quality_gate_base.hpp"

namespace peach_manipulation
{

// 默认值以 config/peach_manipulation.yaml 为权威源，此处仅为直接构造兜底
// （yaml：当前位+一次 0.15 m 短移，覆盖门 8°）。
struct QualityGateConfig
{
  std::size_t minimum_views{2};
  double minimum_baseline_deg{8.0};
  double minimum_mean_nearest_baseline_deg{6.0};
  double minimum_mean_depth_ratio{0.40};
  double maximum_data_age_s{3.0};
  double maximum_axis_angle_deg{35.0};
};

// 默认质量门实现（注册名 threshold）：固定阈值档的身份一致性 + 视图覆盖 +
// 精化拟合质量判定。线程安全与生命周期约定见 QualityGateBase。
class QualityGate : public QualityGateBase
{
public:
  explicit QualityGate(QualityGateConfig config = QualityGateConfig());

  GateResult readyToFinalize(const QualitySnapshot & snapshot) const override;
  GateResult readyToPreviewContact(const QualitySnapshot & snapshot) const override;
  GateResult readyToApproach(const QualitySnapshot & snapshot) const override;
  GateResult readyToGrasp(const QualitySnapshot & snapshot) const override;

private:
  GateResult commonIdentityGate(const QualitySnapshot & snapshot) const;
  GateResult axisConsistencyGate(const QualitySnapshot & snapshot) const;
  QualityGateConfig config_;
};

}  // namespace peach_manipulation

#endif  // PEACH_MANIPULATION__QUALITY_GATE_HPP_
