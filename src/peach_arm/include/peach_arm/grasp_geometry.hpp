// 功能：套入入口点纯函数。入口 = 锚点 − 轴·(行程 + standoff)。
// 工具筒体尺寸常量已删（W5-6）：审查函数的 tool_length/tool_radius 一律
// 由调用方注入（GraspTaskConfig.tool_body_*，yaml tool.body_*，与
// tcp.xacro tool_body_link 对齐）。
#ifndef PEACH_MANIPULATION__GRASP_GEOMETRY_HPP_
#define PEACH_MANIPULATION__GRASP_GEOMETRY_HPP_

#include "peach_arm/math_utils.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace peach_arm
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
  constexpr double step = static_cast<double>(EIGEN_PI) / 6.0;
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

// 果实胶囊（2026-09-14 约束重设计）：感知拟合圆柱 bottom→neck 的有限段
// + 逐目标半径（直径/2 + 固定膨胀）。保护果实的口径是「工具有限圆柱不
// 与本胶囊在轴向投影重叠区内相交」；从下方接近由反爬规则（锚定果底
// 平面）另行保证。
// diameter<=0（感知无效）时用 fallback_radius_m 保守值，调用方告警。
// 半无限 BagKeepout 已删（G 弦时代产物；2026-09-10 探针）。
struct FruitCapsule
{
  Eigen::Vector3d bottom{Eigen::Vector3d::Zero()};
  Eigen::Vector3d neck{Eigen::Vector3d::Zero()};
  Eigen::Vector3d axis{Eigen::Vector3d::UnitZ()};
  double radius_m{0.05};
  bool enabled{true};
};

inline double fruitRadiusM(
  double diameter_m, double inflation_m, double fallback_radius_m)
{
  const double base = diameter_m > 1.0e-6 ? diameter_m / 2.0 : fallback_radius_m;
  return std::max(0.025, base) + std::max(0.0, inflation_m);
}

inline FruitCapsule fruitCapsuleFrom(
  const Eigen::Vector3d & bottom, const Eigen::Vector3d & neck,
  const Eigen::Vector3d & axis, double diameter_m, double inflation_m,
  double fallback_radius_m, bool enabled)
{
  FruitCapsule fruit;
  fruit.bottom = bottom;
  fruit.neck = neck;
  fruit.axis = axis.norm() > 1.0e-9 ? axis.normalized() : Eigen::Vector3d::UnitZ();
  fruit.radius_m = fruitRadiusM(diameter_m, inflation_m, fallback_radius_m);
  fruit.enabled = enabled;
  return fruit;
}

inline bool fruitCapsuleDisabled(const FruitCapsule & fruit)
{
  return !fruit.enabled || fruit.radius_m <= 1.0e-6 ||
         !fruit.axis.allFinite() || !fruit.bottom.allFinite() ||
         !fruit.neck.allFinite();
}

// 点相对果底平面的轴向/径向（反爬与诊断用；s=0 在果底）。
inline void axialRadial(
  const Eigen::Vector3d & point, const FruitCapsule & fruit,
  double & axial_m, double & radial_m)
{
  const Eigen::Vector3d delta = point - fruit.bottom;
  axial_m = delta.dot(fruit.axis);
  radial_m = (delta - axial_m * fruit.axis).norm();
}

// 线段-线段最近距离（clamp 到两端，退化共线取端点距）。
// d(s,t)²=|r+s·d1−t·d2|² 的驻点解两步 clamp：先解 s 并 clamp，再回代解 t
// 并 clamp，最后对 s 复核一次（审计用途，近似即够）。
inline double segmentSegmentDistance(
  const Eigen::Vector3d & p1, const Eigen::Vector3d & q1,
  const Eigen::Vector3d & p2, const Eigen::Vector3d & q2)
{
  const Eigen::Vector3d d1 = q1 - p1;
  const Eigen::Vector3d d2 = q2 - p2;
  const Eigen::Vector3d r = p1 - p2;
  const double a = d1.squaredNorm();
  const double b = d1.dot(d2);
  const double c = d2.squaredNorm();
  const double f = r.dot(d1);
  const double g = r.dot(d2);
  const double denom = a * c - b * b;
  double s = 0.0;
  double t = 0.0;
  if (a <= 1.0e-12 && c <= 1.0e-12) {
    return r.norm();
  }
  if (a <= 1.0e-12) {
    t = std::clamp(g / c, 0.0, 1.0);
  } else if (c <= 1.0e-12) {
    s = std::clamp(-f / a, 0.0, 1.0);
  } else {
    s = denom > 1.0e-12 ?
      std::clamp((b * g - c * f) / denom, 0.0, 1.0) : 0.0;
    t = std::clamp((b * s + g) / c, 0.0, 1.0);
    s = std::clamp((b * t - f) / a, 0.0, 1.0);
  }
  return (r + s * d1 - t * d2).norm();
}

inline Eigen::Vector3d toolTailPoint(
  const Eigen::Vector3d & tcp, const Eigen::Quaterniond & tcp_quat,
  double tool_length_m)
{
  const Eigen::Vector3d tool_z = tcp_quat.normalized() * Eigen::Vector3d::UnitZ();
  return tcp - tool_z * tool_length_m;
}

inline bool axialRangesOverlap(double a0, double a1, double b0, double b1)
{
  const double amin = std::min(a0, a1);
  const double amax = std::max(a0, a1);
  const double bmin = std::min(b0, b1);
  const double bmax = std::max(b0, b1);
  return amin <= bmax + 1.0e-9 && bmin <= amax + 1.0e-9;
}

inline double toolCapsuleClearance(
  const Eigen::Vector3d & tcp, const Eigen::Quaterniond & tcp_quat,
  const FruitCapsule & fruit, double tool_length_m, double tool_radius_m)
{
  if (fruitCapsuleDisabled(fruit)) {
    return std::numeric_limits<double>::infinity();
  }
  const Eigen::Vector3d tail = toolTailPoint(tcp, tcp_quat, tool_length_m);
  double tool_s0 = 0.0;
  double tool_s1 = 0.0;
  double fruit_s0 = 0.0;
  double fruit_s1 = 0.0;
  double unused = 0.0;
  axialRadial(tcp, fruit, tool_s0, unused);
  axialRadial(tail, fruit, tool_s1, unused);
  axialRadial(fruit.bottom, fruit, fruit_s0, unused);
  axialRadial(fruit.neck, fruit, fruit_s1, unused);
  if (!axialRangesOverlap(tool_s0, tool_s1, fruit_s0, fruit_s1)) {
    return std::numeric_limits<double>::infinity();
  }
  return segmentSegmentDistance(tcp, tail, fruit.bottom, fruit.neck) -
         tool_radius_m - fruit.radius_m;
}

struct FruitAuditReport
{
  bool allowed{true};
  std::string reason{"果实胶囊审查通过"};
  double min_clearance_m{std::numeric_limits<double>::infinity()};
};

// audit_climb=false 只查工具×果实胶囊接触（staging 转移首段 PTP 弧用：
// 拍照位本就在果上方，锚定起点的反爬门会把关节弧 2–4 cm 拱高误判绕行）。
// =true 另查反爬：TCP 的 s 不得超过 max(本段起点 s, 0)+2 cm（锚定果底）。
// 套入/撤退段不走本审查（任务性穿果）。
template<typename Waypoint>
inline FruitAuditReport inspectToolVsFruit(
  const std::vector<Waypoint> & points, const FruitCapsule & fruit,
  bool audit_climb, double tool_length_m, double tool_radius_m)
{
  FruitAuditReport report;
  if (fruitCapsuleDisabled(fruit)) {
    report.reason = "果实胶囊审查关闭";
    return report;
  }
  if (points.size() < 2U) {
    report.reason = "果实胶囊审查：点列不足，跳过";
    return report;
  }
  double start_s = 0.0;
  double start_r = 0.0;
  axialRadial(
    Eigen::Vector3d(points.front().x, points.front().y, points.front().z),
    fruit, start_s, start_r);
  (void)start_r;
  const double s_max = (start_s > 0.0 ? start_s : 0.0) + 0.02;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const Eigen::Vector3d tcp(points[i].x, points[i].y, points[i].z);
    const Eigen::Quaterniond quat(
      points[i].qw, points[i].qx, points[i].qy, points[i].qz);
    double axial_m = 0.0;
    double radial_m = 0.0;
    axialRadial(tcp, fruit, axial_m, radial_m);
    if (audit_climb && axial_m > s_max) {
      report.allowed = false;
      std::ostringstream reason;
      reason << "从果上方绕行 s=" << axial_m << "m > 上限 " << s_max << "m";
      report.reason = reason.str();
      return report;
    }
    const double clearance = toolCapsuleClearance(
      tcp, quat, fruit, tool_length_m, tool_radius_m);
    report.min_clearance_m = std::min(report.min_clearance_m, clearance);
    if (clearance <= 0.0) {
      report.allowed = false;
      std::ostringstream reason;
      reason << "工具筒体接触果实胶囊 s=" << axial_m << "m r=" << radial_m
             << "m 间隙=" << clearance << "m (果半径 " << fruit.radius_m << "m)";
      report.reason = reason.str();
      return report;
    }
  }
  return report;
}

// 直连 LIN 资格用：从 start 到 goal 的工具扫掠（位置 lerp + 姿态 slerp）
// 是否接触果实胶囊。姿态用两端四元数（调用方给对齐后姿态）。
inline bool toolSweepHitsFruit(
  const Eigen::Isometry3d & start, const Eigen::Isometry3d & goal,
  const FruitCapsule & fruit, double tool_length_m, double tool_radius_m,
  std::size_t samples = 24U)
{
  if (fruitCapsuleDisabled(fruit)) {
    return false;
  }
  const Eigen::Quaterniond q0(start.linear());
  const Eigen::Quaterniond q1(goal.linear());
  const std::size_t count = samples < 2U ? 2U : samples;
  struct W
  {
    double x, y, z, qx, qy, qz, qw;
  };
  std::vector<W> points;
  points.reserve(count + 1U);
  for (std::size_t i = 0; i <= count; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(count);
    const Eigen::Vector3d p =
      (1.0 - t) * start.translation() + t * goal.translation();
    const Eigen::Quaterniond q = q0.slerp(t, q1);
    points.push_back({p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w()});
  }
  const FruitAuditReport report = inspectToolVsFruit(
    points, fruit, false, tool_length_m, tool_radius_m);
  return !report.allowed;
}

}  // namespace peach_arm
#endif  // PEACH_MANIPULATION__GRASP_GEOMETRY_HPP_
