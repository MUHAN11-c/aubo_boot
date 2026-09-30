// IVG 演示栈运动纯核：零 MoveIt 运行时依赖的几何/路点工具。
// 源自 aubo_boot demo_driver 的 RobotController 静态方法与
// ExecuteGraspPoseWorker 的路点构造，抽出为可 gtest 的自由函数。
#ifndef IVG_DEMO_SERVICES__MOTION_UTILS_HPP_
#define IVG_DEMO_SERVICES__MOTION_UTILS_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <vector>

namespace ivg_demo_services
{

/// 笛卡尔路径段：沿指定轴移动 offset 米
struct CartesianSegment
{
  char axis;      // 'x'/'y'/'z'
  double offset;  // 偏移量 (m)
};

/// 四元数球面最短路径插值，t∈[0,1]
geometry_msgs::msg::Quaternion slerp(
  const geometry_msgs::msg::Quaternion & q0,
  const geometry_msgs::msg::Quaternion & q1, double t);

/// 生成 from→to 的笛卡尔 waypoints（位置线性 + 朝向 slerp），不含起点
std::vector<geometry_msgs::msg::Pose> interpolateCartesian(
  const geometry_msgs::msg::Pose & from,
  const geometry_msgs::msg::Pose & to, int steps);

/// 保证 q 与 q_ref 同号半球（q 与 -q 同旋转，选短弧）
geometry_msgs::msg::Quaternion quatSameHemisphere(
  const geometry_msgs::msg::Quaternion & q_ref,
  const geometry_msgs::msg::Quaternion & q);

/// 抓取位姿上方偏移（gripper_tip→end_effector 简化变换：沿世界 Z 抬高 offset）
geometry_msgs::msg::Pose applyGraspZOffset(
  const geometry_msgs::msg::Pose & grasp, double offset);

/// 抓取接近路点序列（与 aubo_boot publish_grasps_client_worker 对齐）：
/// 当前→X→Y→抬升→绕 Z 最短角旋转→下降。返回不含起点的 waypoints。
std::vector<geometry_msgs::msg::Pose> buildApproachWaypoints(
  const geometry_msgs::msg::Pose & current,
  const geometry_msgs::msg::Pose & target,
  double height_above, double z_min_limit);

/// 从 segments 生成相对 start 的笛卡尔 waypoints（Z 不低于 z_min_limit）
std::vector<geometry_msgs::msg::Pose> buildSegmentWaypoints(
  const geometry_msgs::msg::Pose & start,
  const std::vector<CartesianSegment> & segments, double z_min_limit);

}  // namespace ivg_demo_services

#endif  // IVG_DEMO_SERVICES__MOTION_UTILS_HPP_
