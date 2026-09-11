// 功能：接近轨迹护栏。关节行程拦绕腕；袋囊 keepout 拦口侧穿果（grasp_geometry）。
// 笛卡尔绕行比/弦偏离/回退与 TCP 姿态行程：任一项 <=0 则跳过该项。
// 时长仅记录，max_duration_s<=0 不按时长拒发。
#ifndef PEACH_MANIPULATION__TRAJECTORY_GUARD_HPP_
#define PEACH_MANIPULATION__TRAJECTORY_GUARD_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace peach_manipulation
{

inline double durationToSec(const builtin_interfaces::msg::Duration & d)
{
  return static_cast<double>(d.sec) + static_cast<double>(d.nanosec) * 1.0e-9;
}

inline builtin_interfaces::msg::Duration secToDuration(double s)
{
  builtin_interfaces::msg::Duration d;
  if (s < 0.0) {
    s = 0.0;
  }
  d.sec = static_cast<int32_t>(s);
  d.nanosec = static_cast<uint32_t>((s - static_cast<double>(d.sec)) * 1.0e9);
  return d;
}

// 多段接近拼成一条关节轨迹（时间轴首尾相接）。关节名须一致，否则返回空。
inline trajectory_msgs::msg::JointTrajectory concatJointTrajectories(
  const std::vector<trajectory_msgs::msg::JointTrajectory> & parts)
{
  trajectory_msgs::msg::JointTrajectory out;
  if (parts.empty()) {
    return out;
  }
  out.header = parts.front().header;
  out.joint_names = parts.front().joint_names;
  double offset_s = 0.0;
  bool skip_stitch = false;
  for (const auto & part : parts) {
    if (part.joint_names != out.joint_names || part.points.empty()) {
      return trajectory_msgs::msg::JointTrajectory();
    }
    for (const auto & point : part.points) {
      if (skip_stitch) {
        skip_stitch = false;
        continue;
      }
      auto copy = point;
      copy.time_from_start = secToDuration(
        offset_s + durationToSec(point.time_from_start));
      out.points.push_back(std::move(copy));
    }
    offset_s += durationToSec(part.points.back().time_from_start);
    skip_stitch = true;
  }
  return out;
}

// 接近轨迹原路返程：点序倒放、时间轴从 0 起、速度/加速度取反。
inline trajectory_msgs::msg::JointTrajectory reverseJointTrajectory(
  const trajectory_msgs::msg::JointTrajectory & forward)
{
  trajectory_msgs::msg::JointTrajectory out;
  out.header = forward.header;
  out.joint_names = forward.joint_names;
  if (forward.points.size() < 2U) {
    return out;
  }
  const double t_end = durationToSec(forward.points.back().time_from_start);
  out.points.reserve(forward.points.size());
  for (std::size_t i = 0; i < forward.points.size(); ++i) {
    const auto & src = forward.points[forward.points.size() - 1U - i];
    auto point = src;
    point.time_from_start = secToDuration(t_end - durationToSec(src.time_from_start));
    for (double & v : point.velocities) {
      v = -v;
    }
    for (double & a : point.accelerations) {
      a = -a;
    }
    out.points.push_back(std::move(point));
  }
  if (!out.points.empty()) {
    out.points.front().time_from_start = secToDuration(0.0);
    if (!out.points.front().velocities.empty()) {
      std::fill(
        out.points.front().velocities.begin(),
        out.points.front().velocities.end(), 0.0);
    }
    if (!out.points.back().velocities.empty()) {
      std::fill(
        out.points.back().velocities.begin(),
        out.points.back().velocities.end(), 0.0);
    }
  }
  return out;
}

struct TrajectoryGuardLimits
{
  // <=0：不按时长拒发（时长随速度变，不能当绕行判据）。
  double max_duration_s{0.0};
  double max_total_joint_travel_rad{12.0};
  double max_single_joint_travel_rad{6.1};
};

struct CartesianWaypoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
};

struct CartesianDetourLimits
{
  // <=0：不查该项。宁可不执行，不许先远离目标再绕回来。
  double max_detour_ratio{2.2};
  double max_chord_deviation_m{0.25};
  double max_recede_m{0.08};
};

struct CartesianDetourReport
{
  bool allowed{false};
  double path_m{0.0};
  double chord_m{0.0};
  double detour_ratio{0.0};
  double max_dev_m{0.0};
  double max_recede_m{0.0};
  std::string reason;
};

struct TrajectoryGuardReport
{
  bool allowed{false};
  double duration_s{0.0};
  double total_joint_travel_rad{0.0};
  double max_single_joint_travel_rad{0.0};
  std::size_t point_count{0U};
  std::string reason;
};

// 关节行程审查：多段接近轨迹（staging PTP + LIN 拼接）按点名对齐后累计每轴 |Δq|，
// 对「时长（可选）/ 累计行程 / 单轴行程」三项逐一拒发；段空、关节名或维度
// 不一致、含非有限值均直接拒。往返重复调用同一 limits 必须同一结果。
inline TrajectoryGuardReport inspectApproachTrajectories(
  const std::vector<trajectory_msgs::msg::JointTrajectory> & parts,
  const TrajectoryGuardLimits & limits)
{
  TrajectoryGuardReport report;
  if (parts.empty()) {
    report.reason = "接近轨迹为空";
    return report;
  }
  std::vector<std::string> names;
  std::vector<double> per_joint;
  std::vector<double> previous_end;
  for (const auto & trajectory : parts) {
    if (trajectory.points.empty() || trajectory.joint_names.empty()) {
      report.reason = "接近轨迹为空";
      return report;
    }
    const auto joint_count = trajectory.joint_names.size();
    if (names.empty()) {
      names = trajectory.joint_names;
      per_joint.assign(joint_count, 0.0);
    } else if (trajectory.joint_names != names) {
      report.reason = "接近轨迹关节名不一致";
      return report;
    }
    for (const auto & point : trajectory.points) {
      if (point.positions.size() != joint_count) {
        report.reason = "接近轨迹关节维度不一致";
        return report;
      }
    }
    // 段间接缝也计入行程：过渡点边界处腕部 ±π 翻转不会出现在任一段内部。
    if (!previous_end.empty()) {
      const auto & next_start = trajectory.points.front().positions;
      for (std::size_t joint = 0; joint < joint_count; ++joint) {
        const double delta = std::abs(next_start[joint] - previous_end[joint]);
        if (!std::isfinite(delta)) {
          report.reason = "接近轨迹含非有限关节值";
          return report;
        }
        per_joint[joint] += delta;
        report.total_joint_travel_rad += delta;
      }
    }
    for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
      const auto & previous = trajectory.points[i - 1].positions;
      const auto & current = trajectory.points[i].positions;
      for (std::size_t joint = 0; joint < joint_count; ++joint) {
        const double delta = std::abs(current[joint] - previous[joint]);
        if (!std::isfinite(delta)) {
          report.reason = "接近轨迹含非有限关节值";
          return report;
        }
        per_joint[joint] += delta;
        report.total_joint_travel_rad += delta;
      }
    }
    const auto & end = trajectory.points.back().time_from_start;
    report.duration_s += static_cast<double>(end.sec) +
      static_cast<double>(end.nanosec) * 1e-9;
    report.point_count += trajectory.points.size();
    previous_end = trajectory.points.back().positions;
  }
  report.max_single_joint_travel_rad =
    *std::max_element(per_joint.begin(), per_joint.end());

  std::ostringstream reason;
  if (limits.max_duration_s > 0.0 && report.duration_s > limits.max_duration_s) {
    reason << "预计时长 " << report.duration_s << "s > " <<
      limits.max_duration_s << "s";
    report.reason = reason.str();
    return report;
  }
  if (report.total_joint_travel_rad > limits.max_total_joint_travel_rad) {
    reason << "累计关节行程 " << report.total_joint_travel_rad << "rad > " <<
      limits.max_total_joint_travel_rad << "rad";
    report.reason = reason.str();
    return report;
  }
  if (
    report.max_single_joint_travel_rad > limits.max_single_joint_travel_rad)
  {
    reason << "单轴累计行程 " << report.max_single_joint_travel_rad << "rad > " <<
      limits.max_single_joint_travel_rad << "rad";
    report.reason = reason.str();
    return report;
  }
  report.allowed = true;
  reason << "短路径门通过";
  report.reason = reason.str();
  return report;
}

// 单段便捷封装，语义同 inspectApproachTrajectories。
inline TrajectoryGuardReport inspectApproachTrajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const TrajectoryGuardLimits & limits)
{
  return inspectApproachTrajectories({trajectory}, limits);
}

// 欧氏距离（米）。笛卡尔绕行三项审查的底层原语。
inline double cartesianDist(
  const CartesianWaypoint & a, const CartesianWaypoint & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// 点到线段 [start,end] 的距离（米）；线段退化成点时退化为两点距离。
inline double pointToSegmentM(
  const CartesianWaypoint & point,
  const CartesianWaypoint & start,
  const CartesianWaypoint & end)
{
  const double abx = end.x - start.x;
  const double aby = end.y - start.y;
  const double abz = end.z - start.z;
  const double length2 = abx * abx + aby * aby + abz * abz;
  if (length2 < 1.0e-16) {
    return cartesianDist(point, start);
  }
  double t = (
    (point.x - start.x) * abx +
    (point.y - start.y) * aby +
    (point.z - start.z) * abz) / length2;
  t = std::max(0.0, std::min(1.0, t));
  const CartesianWaypoint closest{
    start.x + t * abx, start.y + t * aby, start.z + t * abz};
  return cartesianDist(point, closest);
}

// 规划 TCP 相对起止弦：绕行比、最大偏离、相对终点回退（比起点更远离目标）。
inline CartesianDetourReport inspectCartesianDetour(
  const std::vector<CartesianWaypoint> & points,
  const CartesianDetourLimits & limits)
{
  CartesianDetourReport report;
  if (points.size() < 2U) {
    report.allowed = true;
    report.reason = "笛卡尔点列不足，跳过绕行审查";
    return report;
  }
  const CartesianWaypoint & start = points.front();
  const CartesianWaypoint & goal = points.back();
  report.chord_m = cartesianDist(start, goal);
  const double start_to_goal = report.chord_m;
  for (std::size_t i = 1; i < points.size(); ++i) {
    report.path_m += cartesianDist(points[i - 1], points[i]);
    report.max_dev_m = std::max(
      report.max_dev_m, pointToSegmentM(points[i], start, goal));
    const double to_goal = cartesianDist(points[i], goal);
    report.max_recede_m = std::max(
      report.max_recede_m, std::max(0.0, to_goal - start_to_goal));
  }
  if (report.chord_m >= 0.02) {
    report.detour_ratio = report.path_m / report.chord_m;
  }
  std::ostringstream reason;
  if (limits.max_recede_m > 0.0 &&
    report.max_recede_m > limits.max_recede_m)
  {
    reason << "TCP 回退 " << report.max_recede_m << "m > " <<
      limits.max_recede_m << "m";
    report.reason = reason.str();
    return report;
  }
  if (limits.max_chord_deviation_m > 0.0 &&
    report.max_dev_m > limits.max_chord_deviation_m)
  {
    reason << "TCP 相对弦偏离 " << report.max_dev_m << "m > " <<
      limits.max_chord_deviation_m << "m";
    report.reason = reason.str();
    return report;
  }
  if (limits.max_detour_ratio > 0.0 && report.chord_m >= 0.02 &&
    report.detour_ratio > limits.max_detour_ratio)
  {
    reason << "TCP 绕行比 " << report.detour_ratio << " > " <<
      limits.max_detour_ratio;
    report.reason = reason.str();
    return report;
  }
  report.allowed = true;
  reason << "笛卡尔短路径门通过";
  report.reason = reason.str();
  return report;
}

// 两单位四元数测地线夹角（度）。dot 取绝对值，q 与 −q 同一姿态。
inline double quatGeodesicDeg(
  double ax, double ay, double az, double aw,
  double bx, double by, double bz, double bw)
{
  const double na = std::sqrt(ax * ax + ay * ay + az * az + aw * aw);
  const double nb = std::sqrt(bx * bx + by * by + bz * bz + bw * bw);
  if (na < 1.0e-9 || nb < 1.0e-9) {
    return 0.0;
  }
  double dot = std::abs((ax * bx + ay * by + az * bz + aw * bw) / (na * nb));
  if (dot > 1.0) {
    dot = 1.0;
  }
  return 2.0 * std::acos(dot) * 180.0 / 3.14159265358979323846;
}

struct OrientationTravelReport
{
  bool allowed{true};
  double max_from_start_deg{0.0};
  std::string reason{"TCP 姿态行程通过"};
};

// 接近段 TCP 姿态测地线。max_rotation_deg<=0 跳过。
// 绝对上限拦水平袋上再叠大滚转 / 180° 拧腕；slack_deg>0 时路径还不得
// 超过「起止测地线 + 余量」（拦对轴只需 20° 却中途拧到 90°+ 的 PTP）。
inline OrientationTravelReport inspectTcpOrientationTravel(
  const std::vector<CartesianWaypoint> & points, double max_rotation_deg,
  double slack_deg = 0.0)
{
  OrientationTravelReport report;
  if (max_rotation_deg <= 0.0) {
    report.reason = "TCP 姿态行程关闭";
    return report;
  }
  if (points.size() < 2U) {
    report.reason = "TCP 点列不足，跳过姿态审查";
    return report;
  }
  const CartesianWaypoint & start = points.front();
  const CartesianWaypoint & goal = points.back();
  const double goal_deg = quatGeodesicDeg(
    start.qx, start.qy, start.qz, start.qw,
    goal.qx, goal.qy, goal.qz, goal.qw);
  double cap = max_rotation_deg;
  if (slack_deg > 0.0) {
    cap = std::min(cap, goal_deg + slack_deg);
  }
  for (const auto & point : points) {
    const double deg = quatGeodesicDeg(
      start.qx, start.qy, start.qz, start.qw,
      point.qx, point.qy, point.qz, point.qw);
    report.max_from_start_deg = std::max(report.max_from_start_deg, deg);
    if (deg > cap) {
      std::ostringstream reason;
      reason << "TCP 姿态行程 " << deg << "deg > " << cap << "deg";
      report.reason = reason.str();
      report.allowed = false;
      return report;
    }
  }
  return report;
}

}  // namespace peach_manipulation

#endif  // PEACH_MANIPULATION__TRAJECTORY_GUARD_HPP_
