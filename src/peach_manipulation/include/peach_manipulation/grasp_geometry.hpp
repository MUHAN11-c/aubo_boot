// 功能：套入入口点纯函数。入口 = 锚点 − 轴·(行程 + standoff)。
#ifndef PEACH_MANIPULATION__GRASP_GEOMETRY_HPP_
#define PEACH_MANIPULATION__GRASP_GEOMETRY_HPP_

#include "peach_manipulation/math_utils.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

namespace peach_manipulation
{

// 最小旋转：把 current_R 的 Z 转到 axis，滚转跟着走。接触对轴只用这个，
// 不要用检测位拼出的绝对姿态（那会多转一截滚转，远处 LIN 常无 IK）。
inline Eigen::Matrix3d alignFrameZ(
  const Eigen::Matrix3d & current_R, const Eigen::Vector3d & axis)
{
  const Eigen::Vector3d z = current_R.col(2);
  if (!z.allFinite() || z.norm() < 1.0e-9 || !axis.allFinite() ||
    axis.norm() < 1.0e-9)
  {
    return current_R;
  }
  const Eigen::Quaterniond delta =
    Eigen::Quaterniond::FromTwoVectors(z, axis.normalized());
  return (delta * Eigen::Quaterniond(current_R)).normalized().toRotationMatrix();
}

// 对轴后再绕工具 Z 滚转。对应 MTC GenerateGraspPose 的 angle_delta 采样：
// 圆筒刀口方位自由；keep-roll 直线若自碰，换滚转仍走同一条位置弦。
inline Eigen::Matrix3d alignFrameZRolled(
  const Eigen::Matrix3d & current_R,
  const Eigen::Vector3d & axis,
  double roll_rad)
{
  const Eigen::Matrix3d aligned = alignFrameZ(current_R, axis);
  if (std::abs(roll_rad) < 1.0e-12) {
    return aligned;
  }
  return aligned * Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitZ());
}

// 刀口滚转：keep-roll 优先，只扫 ±30°/±60°。更大滚转会让 PTP 把 TCP
// 拧过 90°+（09-11 mock 30 例里出现 180° 姿态行程）。
inline std::vector<double> toolRollsRad()
{
  constexpr double step = kPi / 6.0;
  return {0.0, step, -step, 2.0 * step, -2.0 * step};
}

// 预抓取 = 入口沿 −axis 后撤 standoff_m，姿态与入口一致。
inline Eigen::Isometry3d pregraspAlongAxis(
  const Eigen::Isometry3d & entry, const Eigen::Vector3d & axis, double standoff_m)
{
  Eigen::Isometry3d pose = entry;
  if (!axis.allFinite() || axis.norm() < 1.0e-9) {
    return pose;
  }
  const double retreat = standoff_m > 0.0 ? standoff_m : 0.0;
  pose.translation() -= axis.normalized() * retreat;
  return pose;
}

// SELECT / CheckReachability 与 MovePregrasp 同一停位：请求当入口（Z=袋轴），
// 位置沿 −Z 后撤 standoff，姿态 alignFrameZ 保留当前 TCP 滚转。轴无效则原样返回。
inline Eigen::Isometry3d pregraspFromEntryKeepRoll(
  const Eigen::Isometry3d & entry,
  const Eigen::Matrix3d & current_R,
  double standoff_m)
{
  const Eigen::Vector3d axis = entry.linear().col(2);
  Eigen::Isometry3d pose = pregraspAlongAxis(entry, axis, standoff_m);
  if (!axis.allFinite() || axis.norm() < 1.0e-9) {
    return pose;
  }
  pose.linear() = alignFrameZ(current_R, axis);
  return pose;
}

// 袋囊 keepout：入口沿 +axis 的半无限圆柱。TCP 在 s≥0 且 r<radius
// 即从口侧/上方进入，接近段禁止。半径默认 0.12 m。axial_m<=0 关闭。
// （G/under 单弦档已删：photo→G 弦 fraction 0.41–0.73，2026-09-10 探针。）
struct BagKeepout
{
  Eigen::Vector3d entry{Eigen::Vector3d::Zero()};
  Eigen::Vector3d axis{Eigen::Vector3d::UnitZ()};
  double radius_m{0.12};
  double axial_m{0.12};
};

inline bool bagKeepoutDisabled(const BagKeepout & keepout)
{
  return keepout.radius_m <= 1.0e-6 || keepout.axial_m <= 1.0e-6 ||
         !keepout.axis.allFinite() || keepout.axis.norm() < 1.0e-9 ||
         !keepout.entry.allFinite();
}

inline void axialRadial(
  const Eigen::Vector3d & point, const BagKeepout & keepout,
  double & axial_m, double & radial_m)
{
  const Eigen::Vector3d axis = keepout.axis.normalized();
  const Eigen::Vector3d delta = point - keepout.entry;
  axial_m = delta.dot(axis);
  radial_m = (delta - axial_m * axis).norm();
}

inline bool pointHitsBagKeepout(
  const Eigen::Vector3d & point, const BagKeepout & keepout)
{
  if (bagKeepoutDisabled(keepout) || !point.allFinite()) {
    return false;
  }
  double axial_m = 0.0;
  double radial_m = 0.0;
  axialRadial(point, keepout, axial_m, radial_m);
  return axial_m >= 0.0 && radial_m < keepout.radius_m;
}

inline bool segmentHitsBagKeepout(
  const Eigen::Vector3d & start, const Eigen::Vector3d & end,
  const BagKeepout & keepout, std::size_t samples = 40U)
{
  if (bagKeepoutDisabled(keepout)) {
    return false;
  }
  const std::size_t count = samples < 2U ? 2U : samples;
  for (std::size_t i = 0; i <= count; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(count);
    if (pointHitsBagKeepout((1.0 - t) * start + t * end, keepout)) {
      return true;
    }
  }
  return false;
}

struct BagKeepoutReport
{
  bool allowed{true};
  std::string reason{"袋囊 keepout 通过"};
};

// audit_climb=false 只查圆柱穿越（staging 转移的 PTP 弧用：拍照位本就在
// 袋口上方，锚定起点的反爬门会把关节弧的自然拱高误判成绕行；袋口安全由
// 圆柱本体检查保证）。=true 另查反爬：s 不得超过 max(本段起点s, 0)+2 cm。
inline BagKeepoutReport inspectTcpBagKeepout(
  const std::vector<Eigen::Vector3d> & points, const BagKeepout & keepout,
  bool audit_climb = true)
{
  BagKeepoutReport report;
  if (bagKeepoutDisabled(keepout)) {
    report.reason = "袋囊 keepout 关闭";
    return report;
  }
  if (points.size() < 2U) {
    report.reason = "袋囊 keepout：点列不足，跳过";
    return report;
  }
  double start_s = 0.0;
  double start_r = 0.0;
  axialRadial(points.front(), keepout, start_s, start_r);
  (void)start_r;
  // 禁止从口侧上方绕：s 不得超过 max(起点s, 0)+2 cm。已在袋底（s<0）
  // 允许朝入口增大 s；套入段不走本审查。
  const double s_max = (start_s > 0.0 ? start_s : 0.0) + 0.02;
  for (std::size_t i = 0; i < points.size(); ++i) {
    double axial_m = 0.0;
    double radial_m = 0.0;
    axialRadial(points[i], keepout, axial_m, radial_m);
    if (audit_climb && axial_m > s_max) {
      report.allowed = false;
      std::ostringstream reason;
      reason << "从口侧/上方绕行 s=" << axial_m << "m > 口侧上限 "
             << s_max << "m";
      report.reason = reason.str();
      return report;
    }
    if (!pointHitsBagKeepout(points[i], keepout)) {
      continue;
    }
    report.allowed = false;
    std::ostringstream reason;
    reason << "TCP 进入袋囊 keepout（口侧）s=" << axial_m << "m r=" <<
      radial_m << "m < " << keepout.radius_m << "m";
    report.reason = reason.str();
    return report;
  }
  report.reason = "袋囊 keepout 通过";
  return report;
}

}  // namespace peach_manipulation
#endif  // PEACH_MANIPULATION__GRASP_GEOMETRY_HPP_
