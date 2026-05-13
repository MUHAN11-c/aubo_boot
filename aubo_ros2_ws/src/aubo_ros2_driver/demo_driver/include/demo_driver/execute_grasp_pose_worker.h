/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_EXECUTE_GRASP_POSE_WORKER_H_
#define DEMO_DRIVER_EXECUTE_GRASP_POSE_WORKER_H_

#include "demo_driver/moveit_gripper_io_base.h"

#include <array>
#include <atomic>
#include <string>
#include <thread>

#include <Eigen/Core>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <ivg_interfaces/srv/execute_grasp_pose.hpp>
#include <ivg_interfaces/srv/estimate_pose.hpp>

namespace demo_driver
{

/**
 * @brief 执行抓取位姿 Worker 节点（服务驱动模式）
 *
 * 继承 MoveitGripperIoBase，提供服务接口触发抓取任务。
 * 特点：
 * - 支持视觉估计或参数常量两种模式
 * - 提供单次抓取和循环抓取服务
 * - 不进行 Z 轴 180° 修正
 * - 只允许 Z 轴旋转（姿态由 Z 轴旋转角度指定）
 * - 执行流程：回安全位 -> 视觉估计(可选) -> 抓取接近 -> 闭夹爪 -> 抬起 -> 放置 -> 开夹爪 -> 回安全位
 */
class ExecuteGraspPoseWorker : public MoveitGripperIoBase
{
public:
  explicit ExecuteGraspPoseWorker(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~ExecuteGraspPoseWorker() override;

  static std::shared_ptr<ExecuteGraspPoseWorker> create(const rclcpp::NodeOptions& options);

  bool run() override;

  /** 单次抓取放置周期，任一步失败返回 false */
  bool runOneCycle();

private:
  /**
   * 4 点笛卡尔抓取接近
   * @param pose_ee end_effector 在 base_link 下的目标位姿
   * @param height_above 抓取点上方安全高度 (m)
   * @param vel 笛卡尔轨迹速度缩放 [0~1]
   * @param acc 加速度缩放 [0~1]
   */
  bool runGraspApproach(const geometry_msgs::msg::Pose& pose_ee, double height_above, float vel, float acc);

  /**
   * 将 gripper_tip 位姿变换为 end_effector 目标位姿
   * @param pose gripper_tip 在 base_link 下的抓取位姿
   * @param transform_local 抓取局部坐标系下的 4x4 变换
   * @return end_effector 在 base_link 下的目标位姿
   */
  geometry_msgs::msg::Pose applyTransformationToPose(const geometry_msgs::msg::Pose& pose,
                                                     const Eigen::Matrix4d& transform_local);

  /** 构建 gripper_tip -> end_effector 局部变换（沿 z 轴 -grasp_z_offset） */
  Eigen::Matrix4d buildGraspToEndEffectorTransform();

  /**
   * 四元数同半球，避免笛卡尔插值走 180° 长路径
   * @param q_ref 参考四元数（通常为当前末端姿态）
   * @param q 待选四元数（抓取姿态），若 dot<0 返回 -q
   */
  geometry_msgs::msg::Quaternion quatSameHemisphere(const geometry_msgs::msg::Quaternion& q_ref,
                                                     const geometry_msgs::msg::Quaternion& q);

  /**
   * 根据 Z 轴旋转角度创建姿态四元数
   * @param z_rotation_rad Z 轴旋转角度 (弧度)
   * @return 四元数表示的姿态
   */
  geometry_msgs::msg::Quaternion createOrientationFromZRotation(double z_rotation_rad);

  /**
   * 调用视觉位姿估计服务
   * @param object_id 工件ID
   * @return 是否成功获取并更新抓取位姿
   */
  bool estimatePoseFromVision(const std::string& object_id);

  /**
   * 单次抓取服务回调
   */
  void handleExecuteSingleGrasp(
      const std::shared_ptr<ivg_interfaces::srv::ExecuteGraspPose::Request> request,
      std::shared_ptr<ivg_interfaces::srv::ExecuteGraspPose::Response> response);

  /**
   * 循环抓取控制服务回调
   */
  void handleLoopGraspControl(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  /**
   * 启动循环抓取
   */
  void startLoopGrasp();

  /**
   * 停止循环抓取
   */
  void stopLoopGrasp();

  /**
   * 循环抓取线程函数
   */
  void loopGraspThread();

  /** 关节↔笛卡尔切换衔接延时；与 publish_grasps_client_worker 语义一致，<=0 跳过 */
  bool sleepJointCartesianSwitchDelay(const char* where);

  /** gripper_tip 相对 end_effector 的 z 轴补偿 (m) */
  double grasp_z_offset_;
  /** 抓取点上方安全高度 (m) */
  double height_above_;
  /** 关节、回安全位与笛卡尔共用速度缩放 [0~1]（与 publish_grasps_client_worker 一致） */
  float joint_velocity_scaling_;
  /** 关节、回安全位与笛卡尔共用加速度缩放 [0~1] */
  float joint_acceleration_scaling_;
  /** Aubo 夹爪 IO pin 号 */
  int32_t gripper_io_index_;
  /** 抓取后沿 Z 轴抬起高度 (m) */
  double lift_offset_;
  /** 放置：先回安全位再沿 y/x/z 做笛卡尔偏移 (m) */
  double place_offset_y_;
  double place_offset_z_;
  /** 关节↔笛卡尔切换衔接延时 (s)，与 egp_joint_cartesian_switch_delay_sec 对应；0 关闭 */
  double joint_cartesian_switch_delay_sec_{ 0.2 };
  /** 抓取接近笛卡尔轨迹点数上限 */
  int cartesian_max_points_;

  /** 抓取位置 (x, y, z) */
  std::array<double, 3> grasp_position_;
  /** 抓取姿态四元数（直接作为目标姿态） */
  geometry_msgs::msg::Quaternion grasp_orientation_;
  /** 抓取 Z 轴旋转角度 (弧度) */
  double grasp_z_rotation_;
  /** 工件ID（用于视觉估计） */
  std::string object_id_;

  // 服务
  rclcpp::Service<ivg_interfaces::srv::ExecuteGraspPose>::SharedPtr execute_single_grasp_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr loop_grasp_control_service_;
  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  rclcpp::Client<ivg_interfaces::srv::EstimatePose>::SharedPtr estimate_pose_client_;
  /** 独立 Reentrant 组：在 /execute_single_grasp 回调内 async 调用 /estimate_pose 时，响应可在另一线程完成 future */
  rclcpp::CallbackGroup::SharedPtr estimate_pose_client_cb_group_;

  // 循环抓取状态
  std::atomic<bool> loop_grasp_running_{false};
  std::atomic<bool> stop_loop_flag_{false};
  std::atomic<bool> stop_after_cycle_{false};
  std::atomic<bool> cycle_in_progress_{false};
  std::thread loop_thread_;
};

}  // namespace demo_driver

#endif  // DEMO_DRIVER_EXECUTE_GRASP_POSE_WORKER_H_
