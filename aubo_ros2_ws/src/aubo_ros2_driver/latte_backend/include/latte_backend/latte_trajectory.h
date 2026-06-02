#ifndef LATTE_BACKEND_LATTE_TRAJECTORY_H_
#define LATTE_BACKEND_LATTE_TRAJECTORY_H_

#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include "latte_backend/latte_workflow_node.h"  // HeartParams

namespace latte_backend
{

/// 拉花轨迹生成参数 (一次性传入, 无状态)
struct LatteTrajectoryParams {
    double cup_x, cup_y, cup_z;                 // 纸杯杯口世界坐标
    double spout_offset_x, spout_offset_y, spout_offset_z;  // TCP→奶缸嘴 (TCP局部坐标)
    HeartParams heart;                           // 心形轨迹参数
    geometry_msgs::msg::Quaternion tcp_orientation;  // 当前 TCP 姿态 (用于初始朝向)
};

/// 单阶段规划结果 — spout 坐标系路点, 使用时需通过 spoutToTcp() 转换为 TCP 坐标
struct StagePlan {
    std::vector<geometry_msgs::msg::Pose> waypoints;   // spout 坐标路点
    geometry_msgs::msg::Pose transition_target;         // moveCartesianStraight 目标
    int  stage_id;
    const char* name;
};

/// 心形拉花轨迹生成器 — 纯计算, 无 ROS 依赖
/// 输出 spout 坐标系 StagePlan, 通过 spoutToTcp() 转为 tool_tcp 坐标后喂给 MoveIt
class LatteTrajectoryGenerator
{
public:
    explicit LatteTrajectoryGenerator(const LatteTrajectoryParams& p);

    StagePlan stageApproach(); // [5]  靠近杯口: Z=80mm roll=0° 无倾角, 从远处带至杯口上方
    StagePlan stageMix();     // [5a] 融合画圈: Z=80mm roll=45° r=10mm×2圈
    StagePlan stageDraw();    // [5b] 成形注入: Z=5mm roll=60°→45° 定点
    StagePlan stageFinish();  // [5c] 划穿收尾: Z=80mm roll=50° Y推15mm
    StagePlan stageHome();    // [5d] 恢复水平: Z=80mm roll=0° 收尾后归位

    /// 将 spout 坐标系 StagePlan 转换为 tool_tcp 坐标系 (MoveIt 直接可用)
    StagePlan spoutToTcp(const StagePlan& sp) const;

private:
    LatteTrajectoryParams p_;
    geometry_msgs::msg::Pose origin_;    // spout 空间原点 (=纸杯杯口)

    geometry_msgs::msg::Pose spoutToTcp(const geometry_msgs::msg::Pose& spout_pose) const;

    static geometry_msgs::msg::Pose makeStagePose(
        const geometry_msgs::msg::Pose& origin,
        double z_offset, double roll_deg, double sway_y_offset);
    static std::vector<geometry_msgs::msg::Pose> generateHeartStageWaypoints(
        const geometry_msgs::msg::Pose& origin,
        const HeartParams& hp,
        double fixed_z, double fixed_roll_deg, int stage);
};

}  // namespace latte_backend

#endif  // LATTE_BACKEND_LATTE_TRAJECTORY_H_
