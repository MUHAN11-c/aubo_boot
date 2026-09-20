// 功能：默认质量门。阈值以 config/peach_arm.yaml 为准。
#ifndef PEACH_MANIPULATION__QUALITY_GATE_HPP_
#define PEACH_MANIPULATION__QUALITY_GATE_HPP_

#include <cstddef>
#include <string>

namespace peach_arm
{

// 质量门判定的全部输入（纯值快照）：感知选中目标、重建状态与覆盖指标、
// 精化拟合指标、检测—精化轴夹角、数据时效与许可标志。无效标量约定 -1
// 或极大值（见字段默认；axis_angle_deg<0 表示夹角不可算）。
struct QualitySnapshot
{
  std::string selected_target_id;      ///< 感知当前选中目标。
  std::string reconstruction_target_id;  ///< 重建绑定目标（空=未绑定）。
  std::string refined_target_id;       ///< 精化结果绑定目标。
  std::string reconstruction_state;    ///< 重建状态机（READY/FINISHING/…）。
  std::size_t captured_views{0};       ///< 已积分帧数（机位数优先于本值）。
  std::size_t station_count{0};        ///< 已采机位数（>0 时覆盖 captured_views）。
  double max_baseline_deg{0.0};        ///< 最大角基线 [deg]（覆盖证据）。
  double mean_nearest_baseline_deg{0.0};  ///< 平均最近基线 [deg]（视角分布）。
  double mean_depth_ratio{0.0};        ///< 平均有效深度比例 [0,1]。
  double refined_rmse_m{-1.0};         ///< 精化拟合 RMSE [m]；-1=未提供。
  double refined_inlier_ratio{-1.0};   ///< 精化内点比 [0,1]；-1=未提供。
  double data_age_s{1.0e9};            ///< 质量证据年龄 [s]（超龄拒）。
  double axis_angle_deg{-1.0};         ///< 检测—精化轴夹角 [deg]；<0=不可算。
  bool refined_accept{false};          ///< 精化几何可用（接近门要求）。
  bool grasp_allowed{false};           ///< GraspDecision.allowed（真实套入/剪切许可）。
};

// 门判定结果：allowed=false 时 reason 为稳定的机器可读原因串（调用端按
// 字符串做分支判断，实现不得随意改名）。axis_* 两字段为轴一致性诊断
// （axisConsistencyGate 填充、外层门向上透传）：恒不参与 allowed 判定，
// 只供日志/诊断观测检测—精化轴夹角（W13-B；-1=夹角不可算）。
struct GateResult
{
  bool allowed{false};
  std::string reason;
  double axis_angle_deg{-1.0};   ///< 检测—精化轴夹角观测值 [deg]；-1=不可算。
  bool axis_mismatch{false};     ///< 轴夹角超 maximum_axis_angle_deg（仅诊断，不拒）。
};

// 默认值以 config/peach_arm.yaml 为权威源，此处仅为直接构造兜底
// （yaml：当前位+一次 0.15 m 短移，覆盖门 8°）。
struct QualityGateConfig
{
  std::size_t minimum_views{2};                        ///< 最少机位数（station_count 优先）。
  double minimum_baseline_deg{8.0};                    ///< 最大基线角下限 [deg]（0.15 m 短移实测口径）。
  double minimum_mean_nearest_baseline_deg{6.0};       ///< 平均最近基线下限 [deg]（视角分布）。
  double minimum_mean_depth_ratio{0.40};               ///< 平均有效深度比例下限 [0,1]。
  double maximum_data_age_s{3.0};                      ///< 质量证据最大年龄 [s]（低帧率诊断心跳口径）。
  double maximum_axis_angle_deg{35.0};                 ///< 完全错轴诊断上限 [deg]（只诊断不拒接触）。
};

// 默认质量门（唯一实现，零第二实现虚基类已删，W5-8；将来需要第二实现时
// 按 AGENTS 走 pluginlib 新缝）：固定阈值档的身份一致性 + 视图覆盖 +
// 精化拟合质量判定。判定方法为 const 纯函数，只在周期工作线程/executor
// 回调调用；生命周期=节点构造期/参数重载时重建，unique_ptr 独占持有。
class QualityGate
{
public:
  explicit QualityGate(QualityGateConfig config = QualityGateConfig());

  // finalize 门：覆盖证据（机位数/基线/深度）与身份、时效是否达标。
  GateResult readyToFinalize(const QualitySnapshot & snapshot) const;
  // 接触轨迹预览门：预览只读锁存几何、不执行运动，时效要求较宽。
  GateResult readyToPreviewContact(const QualitySnapshot & snapshot) const;
  // 预抓取门：融合几何可接近，不要求 GraspDecision.allowed。
  GateResult readyToApproach(const QualitySnapshot & snapshot) const;
  // 抓取门：接近几何 + 接触许可（真实套入/剪切的最终质量门）。
  GateResult readyToGrasp(const QualitySnapshot & snapshot) const;

private:
  GateResult commonIdentityGate(const QualitySnapshot & snapshot) const;
  GateResult axisConsistencyGate(const QualitySnapshot & snapshot) const;
  QualityGateConfig config_;
};

}  // namespace peach_arm

#endif  // PEACH_MANIPULATION__QUALITY_GATE_HPP_
