// 功能：套入入口点纯函数。入口 = 锚点 − 轴·(行程 + standoff)。
#ifndef PEACH_MANIPULATION__GRASP_GEOMETRY_HPP_
#define PEACH_MANIPULATION__GRASP_GEOMETRY_HPP_

#include <Eigen/Geometry>

#include <cmath>
#include <cstddef>
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

// 笛卡尔直线采样（含终点，不含起点）：平移 lerp + 姿态 slerp。
// 接触回退走同一条弦，禁止另造关节空间目标。
inline std::vector<Eigen::Isometry3d> cartesianLineSamples(
  const Eigen::Isometry3d & start,
  const Eigen::Isometry3d & goal,
  std::size_t inner_count = 1U)
{
  std::vector<Eigen::Isometry3d> out;
  const Eigen::Quaterniond q0(start.linear());
  const Eigen::Quaterniond q1(goal.linear());
  const std::size_t steps = inner_count + 1U;
  out.reserve(steps);
  for (std::size_t i = 1; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() =
      (1.0 - t) * start.translation() + t * goal.translation();
    pose.linear() = q0.slerp(t, q1).toRotationMatrix();
    out.push_back(pose);
  }
  return out;
}

}  // namespace peach_manipulation
#endif  // PEACH_MANIPULATION__GRASP_GEOMETRY_HPP_
