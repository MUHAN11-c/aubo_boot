// 功能：套入入口点纯函数。入口 = 锚点 − 轴·(行程 + standoff)。
#ifndef PEACH_MANIPULATION__GRASP_GEOMETRY_HPP_
#define PEACH_MANIPULATION__GRASP_GEOMETRY_HPP_

#include <Eigen/Geometry>

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

// SELECT / CheckReachability 与 MovePregrasp 同一停位：请求当入口（Z=袋轴），
// 位置沿 −Z 后撤 standoff，姿态 alignFrameZ 保留当前 TCP 滚转。轴无效则原样返回。
inline Eigen::Isometry3d pregraspFromEntryKeepRoll(
  const Eigen::Isometry3d & entry,
  const Eigen::Matrix3d & current_R,
  double standoff_m)
{
  const Eigen::Vector3d axis = entry.linear().col(2);
  Eigen::Isometry3d pose = entry;
  if (!axis.allFinite() || axis.norm() < 1.0e-9) {
    return pose;
  }
  const double retreat = standoff_m > 0.0 ? standoff_m : 0.0;
  pose.translation() -= axis.normalized() * retreat;
  pose.linear() = alignFrameZ(current_R, axis);
  return pose;
}

}  // namespace peach_manipulation
#endif  // PEACH_MANIPULATION__GRASP_GEOMETRY_HPP_
