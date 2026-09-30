// 心形拉花轨迹生成器 — 纯计算，零 ROS 图依赖。
// 输出 spout 坐标系 StagePlan，经 spoutToTcp() 转为 tool_tcp 坐标喂给 MoveIt。
// 源自 aubo_boot latte_backend/latte_trajectory（2026-09-30 移植；
// HeartParams 拆到 latte_heart.hpp 消除与节点头的循环依赖）。
#ifndef LATTE_BACKEND__LATTE_TRAJECTORY_HPP_
#define LATTE_BACKEND__LATTE_TRAJECTORY_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include "latte_backend/latte_heart.hpp"

namespace latte_backend
{

/// 拉花轨迹生成参数（一次性传入，无状态）
struct LatteTrajectoryParams
{
  double cup_x, cup_y, cup_z;                            // 纸杯杯口世界坐标
  double spout_offset_x, spout_offset_y, spout_offset_z;  // TCP→奶缸嘴（TCP 局部坐标）
  HeartParams heart;                                     // 心形轨迹参数
  geometry_msgs::msg::Quaternion tcp_orientation;        // 当前 TCP 姿态（初始朝向）
};

/// 单阶段规划结果 — spout 坐标系路点，使用时需经 spoutToTcp() 转 TCP 坐标
struct StagePlan
{
  std::vector<geometry_msgs::msg::Pose> waypoints;  // spout 坐标路点
  geometry_msgs::msg::Pose transition_target;       // moveCartesianStraight 目标
  int stage_id;
  const char * name;
};

class LatteTrajectoryGenerator
{
public:
  explicit LatteTrajectoryGenerator(const LatteTrajectoryParams & p);

  StagePlan stageApproach();  // [5]  靠近杯口：Z=80mm roll=0° 从远处带至杯口上方
  StagePlan stageMix();       // [5a] 融合画圈：Z=80mm roll=45° r=10mm×2圈
  StagePlan stageDraw();      // [5b] 成形注入：Z=5mm roll=60°→45° 定点
  StagePlan stageFinish();    // [5c] 划穿收尾：Z=80mm roll=50° Y推15mm
  StagePlan stageHome();      // [5d] 恢复水平：Z=80mm roll=0° 收尾归位

  /// 将 spout 坐标系 StagePlan 转换为 tool_tcp 坐标系（MoveIt 直接可用）
  StagePlan spoutToTcp(const StagePlan & sp) const;

private:
  LatteTrajectoryParams p_;
  geometry_msgs::msg::Pose origin_;  // spout 空间原点（=纸杯杯口）

  geometry_msgs::msg::Pose spoutToTcp(const geometry_msgs::msg::Pose & spout_pose) const;

  static geometry_msgs::msg::Pose makeStagePose(
    const geometry_msgs::msg::Pose & origin,
    double z_offset, double roll_deg, double sway_y_offset);
  static std::vector<geometry_msgs::msg::Pose> generateHeartStageWaypoints(
    const geometry_msgs::msg::Pose & origin,
    const HeartParams & hp,
    double fixed_z, double fixed_roll_deg, int stage);
};

}  // namespace latte_backend

#endif  // LATTE_BACKEND__LATTE_TRAJECTORY_HPP_
