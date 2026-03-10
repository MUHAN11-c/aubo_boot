/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_MOVEIT_ARCLINE_GRIPPER_WORKER_H_
#define DEMO_DRIVER_MOVEIT_ARCLINE_GRIPPER_WORKER_H_

#include "demo_driver/moveit_arcline_demo.h"

#include <demo_interface/srv/run_gripper_swap.hpp>

namespace demo_driver
{

/**
 * @brief 夹爪快换 Worker：继承 MoveitArclineDemo，重写 run() 实现夹爪 2 换下 + 夹爪 0 换上流程。
 * 提供 ROS2 服务，收到请求时执行 run()。
 */
class MoveitArclineGripperWorker : public MoveitArclineDemo
{
public:
  explicit MoveitArclineGripperWorker(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~MoveitArclineGripperWorker() override = default;

  /** 工厂函数：构造子类节点并调用 initMoveGroup() */
  static std::shared_ptr<MoveitArclineGripperWorker> create(const rclcpp::NodeOptions& options);

  /** 执行夹爪快换流程（夹爪 2 换下 -> 夹爪 0 换上），任一步失败返回 false */
  bool run() override;
  bool gripper2_to_gripper0();
  bool gripper0_to_gripper2();

private:
  /** 服务回调：根据 request->direction 执行 gripper0_to_gripper2 或 gripper2_to_gripper0，将结果填入 response */
  void runGripperSwapCallback(
    const std::shared_ptr<demo_interface::srv::RunGripperSwap::Request> request,
    std::shared_ptr<demo_interface::srv::RunGripperSwap::Response> response);

  rclcpp::Service<demo_interface::srv::RunGripperSwap>::SharedPtr run_gripper_swap_srv_;
};

}  // namespace demo_driver

#endif  // DEMO_DRIVER_MOVEIT_ARCLINE_GRIPPER_WORKER_H_
