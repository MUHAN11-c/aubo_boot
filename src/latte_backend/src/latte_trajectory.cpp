// 心形拉花轨迹生成器实现 — 源自 aubo_boot latte_backend（2026-09-30 移植）。
// spout_marker 是 tool_tcp 的固定子 link（无旋转）：
//   spout_world = TCP_world + R_TCP * spout_offset
// MoveIt 的 IK/Jacobian 可能不处理固定偏移，因此手动将 spout 路点转为
// tool_tcp 路点，用默认 EE 执行。
#include "latte_backend/latte_trajectory.hpp"

#include <cmath>

namespace latte_backend
{

namespace
{

geometry_msgs::msg::Quaternion quat_from_x_rotation(double angle_deg)
{
  double half = angle_deg * M_PI / 360.0;
  geometry_msgs::msg::Quaternion q;
  q.x = std::sin(half);
  q.y = 0.0;
  q.z = 0.0;
  q.w = std::cos(half);
  return q;
}

/// 四元数 Hamilton 乘法：q1 * q0（先 q0 后 q1）；左乘 = 世界系全局旋转
geometry_msgs::msg::Quaternion quat_mul(
  const geometry_msgs::msg::Quaternion & q1,
  const geometry_msgs::msg::Quaternion & q0)
{
  geometry_msgs::msg::Quaternion r;
  r.w = q1.w * q0.w - q1.x * q0.x - q1.y * q0.y - q1.z * q0.z;
  r.x = q1.w * q0.x + q1.x * q0.w + q1.y * q0.z - q1.z * q0.y;
  r.y = q1.w * q0.y - q1.x * q0.z + q1.y * q0.w + q1.z * q0.x;
  r.z = q1.w * q0.z + q1.x * q0.y - q1.y * q0.x + q1.z * q0.w;
  return r;
}

// v_rot = q * (0, v) * q_conj — 将 TCP 局部偏移转到世界系
geometry_msgs::msg::Pose convertSpoutToTcp(
  const geometry_msgs::msg::Pose & spout_pose,
  double ox, double oy, double oz)
{
  const auto & q = spout_pose.orientation;
  double qw = q.w, qx = q.x, qy = q.y, qz = q.z;

  // q * (0, ox, oy, oz)
  double t1w = -qx * ox - qy * oy - qz * oz;
  double t1x = qw * ox + qy * oz - qz * oy;
  double t1y = qw * oy + qz * ox - qx * oz;
  double t1z = qw * oz + qx * oy - qy * ox;

  // (q*v) * q_conj（q_conj = w, -x, -y, -z）
  double rx = t1w * (-qx) + t1x * qw + t1y * (-qz) - t1z * (-qy);
  double ry = t1w * (-qy) + t1x * qz + t1y * qw - t1z * (-qx);
  double rz = t1w * (-qz) + t1x * (-qy) + t1y * qx + t1z * qw;

  geometry_msgs::msg::Pose tcp;
  tcp.position.x = spout_pose.position.x - rx;
  tcp.position.y = spout_pose.position.y - ry;
  tcp.position.z = spout_pose.position.z - rz;
  tcp.orientation = spout_pose.orientation;
  return tcp;
}

}  // namespace

LatteTrajectoryGenerator::LatteTrajectoryGenerator(const LatteTrajectoryParams & p)
: p_(p)
{
  // spout 原点 = 纸杯杯口世界坐标（奶缸嘴对准杯口中心）
  origin_.position.x = p_.cup_x;
  origin_.position.y = p_.cup_y;
  origin_.position.z = p_.cup_z;
  origin_.orientation = p_.tcp_orientation;
}

geometry_msgs::msg::Pose LatteTrajectoryGenerator::spoutToTcp(
  const geometry_msgs::msg::Pose & sp) const
{
  return convertSpoutToTcp(sp, p_.spout_offset_x, p_.spout_offset_y, p_.spout_offset_z);
}

StagePlan LatteTrajectoryGenerator::spoutToTcp(const StagePlan & sp) const
{
  StagePlan tcp;
  tcp.stage_id = sp.stage_id;
  tcp.name = sp.name;
  tcp.transition_target = spoutToTcp(sp.transition_target);
  tcp.waypoints.reserve(sp.waypoints.size());
  for (const auto & w : sp.waypoints) {
    tcp.waypoints.push_back(spoutToTcp(w));
  }
  return tcp;
}

geometry_msgs::msg::Pose LatteTrajectoryGenerator::makeStagePose(
  const geometry_msgs::msg::Pose & origin,
  double z_offset, double roll_deg, double sway_y_offset)
{
  // roll 从水平（0°）起算，直接使用绝对倾角
  auto q_roll = quat_from_x_rotation(roll_deg);

  geometry_msgs::msg::Pose p;
  p.position.x = origin.position.x;
  p.position.y = origin.position.y + sway_y_offset;
  p.position.z = origin.position.z + z_offset;
  p.orientation = quat_mul(q_roll, origin.orientation);  // 左乘 = 全局旋转
  return p;
}

// 生成单阶段路点（spout 坐标系）。阶段分配（MSLA 推荐）：
//   stage=1 融合画圈 25%  stage=2 成形注入 55%  stage=3 划穿收尾 20%
std::vector<geometry_msgs::msg::Pose>
LatteTrajectoryGenerator::generateHeartStageWaypoints(
  const geometry_msgs::msg::Pose & origin,
  const HeartParams & hp,
  double fixed_z,
  double fixed_roll_deg,
  int stage)
{
  const double cup_x = origin.position.x;
  const double cup_y = origin.position.y;
  const double sway_y = cup_y + hp.sway_offset_y;  // 杯前偏移 10mm，留划穿空间

  // 阶段2允许动态 Roll 渐变，其他阶段固定
  bool dynamic = (stage == 2 && hp.roll_draw_dynamic);
  geometry_msgs::msg::Quaternion fixed_ori;
  if (!dynamic) {
    auto q_roll = quat_from_x_rotation(fixed_roll_deg);
    fixed_ori = quat_mul(q_roll, origin.orientation);
  }

  int N = 0;
  if (stage == 1) {
    N = static_cast<int>(hp.total_points * 0.25);  // 融合 25%
  } else if (stage == 2) {
    N = static_cast<int>(hp.total_points * 0.55);  // 成形 55%
  } else {
    N = static_cast<int>(hp.total_points * 0.20);  // 收尾 20%
  }
  if (N < 1) {N = 1;}

  auto calc_progress = [](int i, int n) -> double {
      return n > 1 ? static_cast<double>(i) / (n - 1) : 0.0;
    };

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.reserve(N);

  for (int i = 0; i < N; ++i) {
    double t = calc_progress(i, N);  // t ∈ [0, 1]
    geometry_msgs::msg::Pose wp;
    wp.position.z = fixed_z;

    // ── XY 运动（阶段决定）──
    if (stage == 1) {
      // [5a] 融合画圈：r=10mm × 2圈，高位 Pin-Drop 细流穿透 crema 建白圆基底
      double angle = 2.0 * M_PI * hp.mix_circles * t;
      wp.position.x = cup_x + hp.mix_circle_r * std::cos(angle);
      wp.position.y = sway_y + hp.mix_circle_r * std::sin(angle);
    } else if (stage == 2) {
      // [5b] 成形注入：壶嘴定点不动（MSLA B1 5.04-5.05），白圆靠奶泡浮面扩散
      wp.position.x = cup_x;
      wp.position.y = sway_y;
    } else {
      // [5c] 划穿收尾：沿 Y 轴穿过圆心 15mm，中速细流产生心形尖部
      wp.position.x = cup_x;
      wp.position.y = sway_y + t * hp.push_y;
    }

    // ── 朝向（roll 绕世界 X 轴）──
    if (!dynamic) {
      wp.orientation = fixed_ori;
    } else {
      // 动态 Roll：60°→45° 线性渐变（高流量建底 → 降流量防溢出，MSLA 4.04）
      double dy_roll = hp.roll_draw_start + t * (hp.roll_draw_end - hp.roll_draw_start);
      auto q_dyn = quat_from_x_rotation(dy_roll);
      wp.orientation = quat_mul(q_dyn, origin.orientation);
    }
    waypoints.push_back(wp);
  }
  return waypoints;
}

StagePlan LatteTrajectoryGenerator::stageApproach()
{
  const auto & hp = p_.heart;
  StagePlan s;
  s.stage_id = 0;
  s.name = "靠近杯口";
  // 从远处把 spout 平移到杯口上方 80mm，保持水平（roll=0°），无连续轨迹
  s.transition_target = makeStagePose(origin_, hp.mix_height, 0.0, hp.sway_offset_y);
  s.waypoints.clear();
  return s;
}

StagePlan LatteTrajectoryGenerator::stageMix()
{
  const auto & hp = p_.heart;
  StagePlan s;
  s.stage_id = 1;
  s.name = "融合画圈";
  // 过渡：spout → 杯口上方 80mm + 前偏 10mm + roll=45°；执行：画 r=10mm 圆 ×2 圈
  s.transition_target = makeStagePose(origin_, hp.mix_height, hp.roll_mix, hp.sway_offset_y);
  s.waypoints = generateHeartStageWaypoints(
    origin_, hp, p_.cup_z + hp.mix_height, hp.roll_mix, 1);
  return s;
}

StagePlan LatteTrajectoryGenerator::stageDraw()
{
  const auto & hp = p_.heart;
  StagePlan s;
  s.stage_id = 2;
  s.name = "成形注入";
  // 过渡：spout → 杯口上方 5mm + 前偏 10mm + roll=60°；执行：定点不动 roll 渐变
  s.transition_target = makeStagePose(
    origin_, hp.draw_height, hp.roll_draw_start, hp.sway_offset_y);
  s.waypoints = generateHeartStageWaypoints(
    origin_, hp, p_.cup_z + hp.draw_height, hp.roll_draw_start, 2);
  return s;
}

StagePlan LatteTrajectoryGenerator::stageFinish()
{
  const auto & hp = p_.heart;
  StagePlan s;
  s.stage_id = 3;
  s.name = "划穿收尾";
  // 过渡：spout → 杯口上方 80mm + 前偏 10mm + roll=50°；执行：Y 轴推进 15mm
  s.transition_target = makeStagePose(origin_, hp.mix_height, hp.roll_finish, hp.sway_offset_y);
  s.waypoints = generateHeartStageWaypoints(
    origin_, hp, p_.cup_z + hp.mix_height, hp.roll_finish, 3);
  return s;
}

StagePlan LatteTrajectoryGenerator::stageHome()
{
  StagePlan s;
  s.stage_id = 4;
  s.name = "恢复水平";
  // 回到杯口上方 80mm 处 roll=0°，防止奶缸嘴保持倾斜
  s.transition_target = makeStagePose(
    origin_, p_.heart.mix_height, 0.0, p_.heart.sway_offset_y);
  s.waypoints.clear();  // 纯过渡，无连续轨迹
  return s;
}

}  // namespace latte_backend
