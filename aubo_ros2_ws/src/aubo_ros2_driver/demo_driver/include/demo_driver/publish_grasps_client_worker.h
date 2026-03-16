/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_PUBLISH_GRASPS_CLIENT_WORKER_H_
#define DEMO_DRIVER_PUBLISH_GRASPS_CLIENT_WORKER_H_

#include "demo_driver/moveit_gripper_io_base.h"

#include <array>
#include <atomic>
#include <deque>
#include <mutex>
#include <optional>
#include <string>

#include <Eigen/Core>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace demo_driver
{

/**
 * @brief GraspNet 抓取放置 Worker 节点
 *
 * 继承 MoveitGripperIoBase，订阅 grasp_poses_base（PoseArray），循环执行：
 * 清理窗口 -> 等待新数据 -> 选优 -> gripper_tip 补偿 -> 抓取接近 -> 闭夹爪 -> 抬起 ->
 * 移动到放置位 -> 开夹爪 -> 回安全位。
 */
class PublishGraspsClientWorker : public MoveitGripperIoBase
{
public:
  explicit PublishGraspsClientWorker(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~PublishGraspsClientWorker() override = default;

  static std::shared_ptr<PublishGraspsClientWorker> create(const rclcpp::NodeOptions& options);

  bool run() override;

  /** 单次抓取放置周期，任一步失败返回 false */
  bool runOneCycle();

  /** 加锁清空窗口与 _latest_grasp_poses */
  void clearGraspWindow();

  /** 阻塞等待窗口达到 min_groups_before_pick，超时返回 false */
  bool waitForGraspWindowReady();

  /** 从窗口选优，prefer_vertical 时取 verticalityScore 最高；返回 (tag, pose) 或 nullopt */
  std::optional<std::pair<std::string, geometry_msgs::msg::Pose>> selectBestFromWindow();

  /**
   * 将 gripper_tip 位姿变换为 end_effector 目标位姿
   * @param pose gripper_tip 在 base_link 下的抓取位姿
   * @param transform_local 抓取局部坐标系下的 4x4 变换，沿 z 轴 -grasp_z_offset
   * @return end_effector 在 base_link 下的目标位姿
   */
  geometry_msgs::msg::Pose applyTransformationToPose(const geometry_msgs::msg::Pose& pose,
                                                     const Eigen::Matrix4d& transform_local);

  /** 构建 gripper_tip -> end_effector 局部变换（沿 z 轴 -grasp_z_offset） */
  Eigen::Matrix4d buildGraspToEndEffectorTransform();

  /** 对 end_effector 目标位姿执行抓取接近（4 点笛卡尔） */
  bool runGraspMotion(const geometry_msgs::msg::Pose& target_pose);

  /** 优雅退出：回安全位、开夹爪 */
  void onShutdown();

  /** 请求退出：SIGINT 时调用，设置 shutdown_requested_ 使主循环尽快退出 */
  void requestShutdown();
  /** 是否已请求退出 */
  bool isShutdownRequested() const { return shutdown_requested_.load(); }

private:
  /**
   * 4 点笛卡尔抓取接近，fraction<1 或点数>60 直接返回 false，不回退
   * @param pose_ee end_effector 在 base_link 下的目标位姿（gripper_tip 补偿后）
   * @param height_above 抓取点上方安全高度 (m)
   * @param vel 笛卡尔轨迹速度缩放 [0~1]
   * @param acc 加速度缩放 [0~1]
   */
  bool runGraspApproach(const geometry_msgs::msg::Pose& pose_ee, double height_above, float vel, float acc);

  /**
   * 四元数同半球，避免笛卡尔插值走 180° 长路径
   * @param q_ref 参考四元数（通常为当前末端姿态）
   * @param q 待选四元数（抓取姿态），若 dot<0 返回 -q
   */
  geometry_msgs::msg::Quaternion quatSameHemisphere(const geometry_msgs::msg::Quaternion& q_ref,
                                                     const geometry_msgs::msg::Quaternion& q);

  /** GraspNet Z 轴 180° 修正：ori * _QUAT_Z_180，位置不变 */
  geometry_msgs::msg::Pose applyGraspZFlip180(const geometry_msgs::msg::Pose& pose);

  /** 垂直度得分 [0,1]：抓取 Z 轴与 [0,0,-1] 点积绝对值，越大越垂直 */
  double verticalityScore(const geometry_msgs::msg::Pose& pose);

  void graspPosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  /**
   * 发布周期状态到 status_topic（JSON 格式）
   * @param cycle_count 当前周期数
   * @param success_count 成功次数
   * @param fail_count 失败次数
   * @param last_step 上次失败步骤 1~10，成功为 0
   * @param is_running 主循环是否在运行
   */
  void publishStatus(int cycle_count, int success_count, int fail_count, int last_step, bool is_running);

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr grasp_poses_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  std::deque<geometry_msgs::msg::PoseArray> grasp_groups_window_;
  geometry_msgs::msg::PoseArray latest_grasp_poses_;
  std::mutex window_mutex_;

  /** 选优策略：true 取垂直度最高抓取，false 取最新组第一个 */
  bool prefer_vertical_;
  /** gripper_tip 相对 end_effector 的 z 轴补偿 (m)，沿 approach 方向反向平移 */
  double grasp_z_offset_;
  /** 抓取点上方安全高度 (m)，笛卡尔路径先到该高度再垂直下降 */
  double height_above_;
  /** 关节/笛卡尔速度缩放 [0~1]，用于 scaleTrajectoryTime */
  float joint_velocity_scaling_;
  /** 关节/笛卡尔加速度缩放 [0~1] */
  float joint_acceleration_scaling_;
  /** 抓取位姿话题名，与 graspnet_demo_points_node 发布一致 */
  std::string grasp_poses_topic_;
  /** 等待窗口就绪超时 (s)，超时返回 false */
  double wait_poses_timeout_sec_;
  /** 滑动窗口大小：缓存最近 N 组 PoseArray */
  size_t grasp_window_size_;
  /** 至少积累 M 组后再选优，避免单次识别噪声 */
  size_t min_groups_before_pick_;
  /** Aubo 夹爪 IO pin 号，与 GripperSwapWorker 一致 */
  int32_t gripper_io_index_;
  /** 抓取后沿 Z 轴抬起高度 (m)，正为向上 */
  double lift_offset_;
  /** 放置模式："pose" 用 place_pose，"joints" 用 place_joints，"home_offset" 用安全位+偏移 */
  std::string place_mode_;
  /** 放置位姿 (x,y,z,qx,qy,qz,qw)，place_mode=pose 时使用，end_effector 在 base_link 下 */
  std::array<double, 7> place_pose_;
  /** 放置关节角 6×rad，place_mode=joints 时使用 */
  std::array<double, 6> place_joints_;
  /** 安全位偏移 y (m)，place_mode=home_offset 时使用 */
  double place_offset_y_;
  /** 安全位偏移 z (m)，place_mode=home_offset 时使用 */
  double place_offset_z_;
  /** 每周期结束后的等待时间 (s)，成功与失败后均执行 */
  double cycle_delay_sec_;
  /** 失败后额外等待 (s)，避免连续失败时过于频繁重试 */
  double fail_retry_delay_sec_;
  /** 最大周期数，-1 表示无限循环 */
  int max_cycles_;
  /** 状态监控话题，发布 cycle_count、success_count、fail_count 等 JSON */
  std::string status_topic_;

  std::atomic<bool> shutdown_requested_{ false };
  int cycle_count_{ 0 };
  int success_count_{ 0 };
  int fail_count_{ 0 };
};

}  // namespace demo_driver

#endif  // DEMO_DRIVER_PUBLISH_GRASPS_CLIENT_WORKER_H_
