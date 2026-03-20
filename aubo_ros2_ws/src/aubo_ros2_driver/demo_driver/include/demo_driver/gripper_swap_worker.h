/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_GRIPPER_SWAP_WORKER_H_
#define DEMO_DRIVER_GRIPPER_SWAP_WORKER_H_

#include "demo_driver/moveit_gripper_io_base.h"

#include <demo_interface/srv/run_gripper_swap.hpp>
#include <atomic>
#include <cstdint>

namespace demo_driver
{

/**
 * @brief 夹爪更换 Worker 节点
 *
 * 继承 MoveitGripperIoBase，实现夹爪快换（gripper0 <-> gripper2）及夹爪 IO 控制。
 */
class GripperSwapWorker : public MoveitGripperIoBase
{
public:
  explicit GripperSwapWorker(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~GripperSwapWorker() override = default;

  static std::shared_ptr<GripperSwapWorker> create(const rclcpp::NodeOptions& options);

  bool run() override;
  /** 从 gripper2 更换到 gripper0 */
  bool swapToGripper0();
  /** 从 gripper0 更换到 gripper2 */
  bool swapToGripper2();
  /** 切换到 gripper2 位姿 */
  bool switchToGripper2();

private:
  // 重写同名笛卡尔接口：夹爪更换流程不做 Z 轴安全下限裁剪
  bool runArcPath(double z_offset, float velocity_factor = 0.5f, float acceleration_factor = 0.1f);
  bool runArcPath(char axis, double offset, float velocity_factor = 0.5f, float acceleration_factor = 0.1f);
  bool runArcPathSequence(const std::vector<CartesianSegment>& segments, float velocity_factor = 0.5f,
                          float acceleration_factor = 0.1f);

  void onGripperSwapRequest(const std::shared_ptr<demo_interface::srv::RunGripperSwap::Request> request,
                            std::shared_ptr<demo_interface::srv::RunGripperSwap::Response> response);

  double joint_velocity_scaling_{ 1.0 };
  double joint_acceleration_scaling_{ 0.1 };
  double cartesian_velocity_scaling_{ 0.5 };
  double cartesian_acceleration_scaling_{ 0.1 };
  double home_velocity_scaling_{ 0.7 };
  double home_acceleration_scaling_{ 0.45 };
  int32_t gripper_io_index_{ kQuickSwapIoIndex };
  std::atomic<bool> swap_in_progress_{ false };

  // 将换爪服务放入独立回调组，避免长耗时服务回调阻塞 MoveIt 状态更新回调。
  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  rclcpp::Service<demo_interface::srv::RunGripperSwap>::SharedPtr gripper_swap_srv_;
};

}  // namespace demo_driver

#endif  // DEMO_DRIVER_GRIPPER_SWAP_WORKER_H_
