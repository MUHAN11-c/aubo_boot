// 功能：预抓取残差纯核（W5-4，自 stageVerifyPregrasp 的残差计算与门限
// 判定抽出）。输入：两次采样（间隔 200ms）的 tool_axis / sleeve_mouth /
// cutting_plane 位姿 + 精化几何（袋底/袋颈/袋轴）；输出：帧间一致性/对轴
// 角/横向/轴向残差与三阈值判定。修正回路编排（TF 轮询、重规划迭代）留在
// 阶段函数。门限来自 yaml grasp.pregrasp_residual.*。
#ifndef PEACH_MANIPULATION__PREGRASP_RESIDUAL_HPP_
#define PEACH_MANIPULATION__PREGRASP_RESIDUAL_HPP_

#include <Eigen/Geometry>

#include "peach_arm/angles.hpp"

namespace peach_arm
{

/// 预抓取残差三阈值（GPL yaml grasp.pregrasp_residual.*；默认=原硬编码）。
struct PregraspThresholds
{
  double frame_consistent_deg{1.5};  ///< 帧间一致门 [deg]（严格小于才 consistent）。
  double axis_deg{2.0};              ///< 对轴角门 [deg]（<=）。
  double lateral_m{0.003};           ///< 横向残差门 [m]（<=；axial 只记录不判定）。
};

/// 一次采样的三个工具 TF 位姿（base 系）。
struct PregraspPoseSample
{
  Eigen::Isometry3d tool_axis{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d sleeve_mouth{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d cutting_plane{Eigen::Isometry3d::Identity()};
};

/// 残差评估结果：angle/lateral 为判定值，axial 仅记录；thresholds 原样带回
/// 供消息填充与诊断。
struct ResidualReport
{
  bool consistent{false};  ///< 帧间一致（frames_deg 严格小于门限）。
  double frames_deg{0.0};  ///< 两次采样 tool Z 的帧间夹角 [deg]。
  double angle_deg{0.0};   ///< 第二次采样 tool Z 与袋轴夹角 [deg]。
  double lateral_m{0.0};   ///< sleeve_mouth/cutting_plane 相对袋几何的横向偏差 [m]。
  double axial_m{0.0};     ///< cutting_plane 相对袋颈的轴向偏差 [m]（只记录）。
  bool passed{false};      ///< consistent ∧ angle<=axis_deg ∧ lateral<=lateral_m。
  PregraspThresholds thresholds;
};

/// 残差计算与三阈值判定（公式逐字自原 stageVerifyPregrasp 内联段搬移）：
///   frames_deg = 夹角(first.tool_axis Z, second.tool_axis Z)（退化按 180°，
///   让门判定走拒绝侧而非误放行）；
///   angle      = 夹角(second.tool_axis Z, bag_axis)（退化同上）；
///   delta_b    = second.sleeve_mouth 平移 − bag_bottom；
///   delta_c    = second.cutting_plane 平移 − bag_neck；
///   axial      = delta_c·bag_axis；
///   lateral    = max(|delta_b 的袋轴垂分量|, |delta_c − axial·bag_axis|)。
inline ResidualReport evaluatePregraspResidual(
  const PregraspThresholds & thresholds,
  const PregraspPoseSample & first,
  const PregraspPoseSample & second,
  const Eigen::Vector3d & bag_bottom,
  const Eigen::Vector3d & bag_neck,
  const Eigen::Vector3d & bag_axis)
{
  ResidualReport report;
  report.thresholds = thresholds;
  const Eigen::Vector3d tool_z = second.tool_axis.linear().col(2);
  report.frames_deg = angleDeg(
    first.tool_axis.linear().col(2), tool_z, AngleDegenerate::MaxMismatch);
  report.angle_deg = angleDeg(tool_z, bag_axis, AngleDegenerate::MaxMismatch);
  const Eigen::Vector3d delta_b =
    second.sleeve_mouth.translation() - bag_bottom;
  const Eigen::Vector3d delta_c =
    second.cutting_plane.translation() - bag_neck;
  report.axial_m = delta_c.dot(bag_axis);
  report.lateral_m = std::max(
    (delta_b - delta_b.dot(bag_axis) * bag_axis).norm(),
    (delta_c - report.axial_m * bag_axis).norm());
  report.consistent = report.frames_deg < thresholds.frame_consistent_deg;
  report.passed = report.consistent &&
    report.angle_deg <= thresholds.axis_deg &&
    report.lateral_m <= thresholds.lateral_m;
  return report;
}

}  // namespace peach_arm

#endif  // PEACH_MANIPULATION__PREGRASP_RESIDUAL_HPP_
