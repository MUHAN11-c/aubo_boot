// 运动纯核实现：零 MoveIt 依赖（Eigen 头文件用于旋转），可被 gtest 直测。
#include "ivg_demo_services/motion_utils.hpp"

#include <Eigen/Geometry>
#include <cmath>

namespace ivg_demo_services
{

geometry_msgs::msg::Quaternion slerp(
  const geometry_msgs::msg::Quaternion & q0,
  const geometry_msgs::msg::Quaternion & q1, double t)
{
  double dot = q0.w * q1.w + q0.x * q1.x + q0.y * q1.y + q0.z * q1.z;

  // q 与 -q 同旋转，选同号半球保证最短路径
  geometry_msgs::msg::Quaternion q1f = q1;
  if (dot < 0.0) {
    dot = -dot;
    q1f.w = -q1f.w;
    q1f.x = -q1f.x;
    q1f.y = -q1f.y;
    q1f.z = -q1f.z;
  }

  const double eps = 0.9995;
  geometry_msgs::msg::Quaternion r;
  if (dot > eps) {
    double s = 1.0 - t;
    r.w = s * q0.w + t * q1f.w;
    r.x = s * q0.x + t * q1f.x;
    r.y = s * q0.y + t * q1f.y;
    r.z = s * q0.z + t * q1f.z;
    double n = std::sqrt(r.w * r.w + r.x * r.x + r.y * r.y + r.z * r.z);
    r.w /= n;
    r.x /= n;
    r.y /= n;
    r.z /= n;
    return r;
  }

  double theta = std::acos(dot);
  double sin_theta = std::sin(theta);
  double s0 = std::sin((1.0 - t) * theta) / sin_theta;
  double s1 = std::sin(t * theta) / sin_theta;
  r.w = s0 * q0.w + s1 * q1f.w;
  r.x = s0 * q0.x + s1 * q1f.x;
  r.y = s0 * q0.y + s1 * q1f.y;
  r.z = s0 * q0.z + s1 * q1f.z;
  return r;
}

std::vector<geometry_msgs::msg::Pose> interpolateCartesian(
  const geometry_msgs::msg::Pose & from,
  const geometry_msgs::msg::Pose & to, int steps)
{
  std::vector<geometry_msgs::msg::Pose> waypoints;
  for (int i = 1; i <= steps; ++i) {
    double t = static_cast<double>(i) / steps;
    geometry_msgs::msg::Pose p;
    p.position.x = from.position.x + t * (to.position.x - from.position.x);
    p.position.y = from.position.y + t * (to.position.y - from.position.y);
    p.position.z = from.position.z + t * (to.position.z - from.position.z);
    p.orientation = slerp(from.orientation, to.orientation, t);
    waypoints.push_back(p);
  }
  return waypoints;
}

geometry_msgs::msg::Quaternion quatSameHemisphere(
  const geometry_msgs::msg::Quaternion & q_ref,
  const geometry_msgs::msg::Quaternion & q)
{
  double dot = q_ref.x * q.x + q_ref.y * q.y + q_ref.z * q.z + q_ref.w * q.w;
  geometry_msgs::msg::Quaternion out;
  if (dot >= 0) {
    out = q;
  } else {
    out.x = -q.x;
    out.y = -q.y;
    out.z = -q.z;
    out.w = -q.w;
  }
  return out;
}

geometry_msgs::msg::Pose applyGraspZOffset(
  const geometry_msgs::msg::Pose & grasp, double offset)
{
  // 简化变换（与 aubo_boot ExecuteGraspPoseWorker 最终版一致）：只沿世界 Z 抬高
  geometry_msgs::msg::Pose out;
  out.position.x = grasp.position.x;
  out.position.y = grasp.position.y;
  out.position.z = grasp.position.z + offset;
  out.orientation = grasp.orientation;
  return out;
}

std::vector<geometry_msgs::msg::Pose> buildApproachWaypoints(
  const geometry_msgs::msg::Pose & current,
  const geometry_msgs::msg::Pose & target,
  double height_above, double z_min_limit)
{
  double gx = target.position.x;
  double gy = target.position.y;
  double gz = target.position.z;
  double z_above = gz + height_above;
  if (gz < z_min_limit) {
    gz = z_min_limit;
    z_above = gz + height_above;
  }

  // 目标朝向取与当前同半球后，绕世界 Z 最短角旋转（先抬升后旋转，姿态过渡平滑）
  const auto grasp_short = quatSameHemisphere(current.orientation, target.orientation);
  Eigen::Quaterniond q_after_up(
    current.orientation.w, current.orientation.x, current.orientation.y,
    current.orientation.z);
  Eigen::Quaterniond q_goal(grasp_short.w, grasp_short.x, grasp_short.y, grasp_short.z);
  q_after_up.normalize();
  q_goal.normalize();
  Eigen::Quaterniond q_delta = q_after_up.conjugate() * q_goal;
  q_delta.normalize();
  Eigen::Matrix3d delta_rot = q_delta.toRotationMatrix();
  double delta_yaw = std::atan2(delta_rot(1, 0), delta_rot(0, 0));
  // 归一到 (-pi, pi] 的短角等价
  delta_yaw = std::remainder(delta_yaw, 2.0 * M_PI);
  Eigen::Quaterniond q_delta_short(Eigen::AngleAxisd(delta_yaw, Eigen::Vector3d::UnitZ()));
  Eigen::Quaterniond q_target = q_after_up * q_delta_short;
  q_target.normalize();

  geometry_msgs::msg::Pose p_x;
  p_x.position.x = gx;
  p_x.position.y = current.position.y;
  p_x.position.z = current.position.z;
  p_x.orientation = current.orientation;

  geometry_msgs::msg::Pose p_y;
  p_y.position.x = gx;
  p_y.position.y = gy;
  p_y.position.z = current.position.z;
  p_y.orientation = current.orientation;

  geometry_msgs::msg::Pose p_up;
  p_up.position.x = gx;
  p_up.position.y = gy;
  p_up.position.z = z_above;
  p_up.orientation = current.orientation;

  geometry_msgs::msg::Pose p_rot;
  p_rot.position = p_up.position;
  p_rot.orientation.x = q_target.x();
  p_rot.orientation.y = q_target.y();
  p_rot.orientation.z = q_target.z();
  p_rot.orientation.w = q_target.w();

  geometry_msgs::msg::Pose p_down;
  p_down.position.x = gx;
  p_down.position.y = gy;
  p_down.position.z = gz;
  p_down.orientation = p_rot.orientation;

  return {p_x, p_y, p_up, p_rot, p_down};
}

std::vector<geometry_msgs::msg::Pose> buildSegmentWaypoints(
  const geometry_msgs::msg::Pose & start,
  const std::vector<CartesianSegment> & segments, double z_min_limit)
{
  std::vector<geometry_msgs::msg::Pose> waypoints;
  if (segments.empty()) {
    return waypoints;
  }
  waypoints.push_back(start);
  for (const auto & seg : segments) {
    auto next = waypoints.back();
    switch (seg.axis) {
      case 'x': next.position.x += seg.offset; break;
      case 'y': next.position.y += seg.offset; break;
      case 'z': next.position.z += seg.offset; break;
    }
    if (next.position.z < z_min_limit) {
      next.position.z = z_min_limit;
    }
    waypoints.push_back(next);
  }
  waypoints.erase(waypoints.begin());  // 去掉起点
  return waypoints;
}

}  // namespace ivg_demo_services
