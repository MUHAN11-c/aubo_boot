// 功能：默认质量门。阈值以 config/peach_manipulation.yaml 为准。
#ifndef PEACH_MANIPULATION__QUALITY_GATE_HPP_
#define PEACH_MANIPULATION__QUALITY_GATE_HPP_

#include <cstddef>
#include <string>

namespace peach_manipulation
{

// 质量门判定的全部输入（纯值快照）：感知选中目标、重建状态与覆盖指标、
// 精化拟合指标、检测—精化轴夹角、数据时效与许可标志。无效标量约定 -1
// 或极大值（见字段默认；axis_angle_deg<0 表示夹角不可算）。
struct QualitySnapshot
{
  std::string selected_target_id;
  std::string reconstruction_target_id;
  std::string refined_target_id;
  std::string reconstruction_state;
  std::size_t captured_views{0};
  std::size_t station_count{0};
  double max_baseline_deg{0.0};
  double mean_nearest_baseline_deg{0.0};
  double mean_depth_ratio{0.0};
  double refined_rmse_m{-1.0};
  double refined_inlier_ratio{-1.0};
  double data_age_s{1.0e9};
  double axis_angle_deg{-1.0};
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
// 生命周期：由节点构造期/参数重载时重建，unique_ptr 独占持有。
// 线程安全：判定方法为 const 纯函数，只在周期工作线程/executor 回调调用。
// 可替换性：唯一实现 QualityGate（节点直接构造）。
class QualityGateBase
{
public:
  virtual ~QualityGateBase() = default;

  // finalize 门：覆盖证据（机位数/基线/深度）与身份、时效是否达标。
  virtual GateResult readyToFinalize(const QualitySnapshot & snapshot) const = 0;
  // 接触轨迹预览门：预览只读锁存几何、不执行运动，时效要求由实现自定。
  virtual GateResult readyToPreviewContact(const QualitySnapshot & snapshot) const = 0;
  // 预抓取门：融合几何可接近，不要求 GraspDecision.allowed。
  virtual GateResult readyToApproach(const QualitySnapshot & snapshot) const
  {
    return readyToGrasp(snapshot);
  }
  // 抓取门：接近几何 + 接触许可（真实套入/剪切的最终质量门）。
  virtual GateResult readyToGrasp(const QualitySnapshot & snapshot) const = 0;
};

// 默认值以 config/manipulation_parameters.yaml 为权威源，此处仅为直接构造兜底
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

// 默认质量门实现：固定阈值档的身份一致性 + 视图覆盖 +
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
