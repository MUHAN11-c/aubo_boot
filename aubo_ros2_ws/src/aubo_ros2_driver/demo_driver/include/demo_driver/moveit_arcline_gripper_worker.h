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

class MoveitArclineGripperWorker : public MoveitArclineDemo
{
public:
  explicit MoveitArclineGripperWorker(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~MoveitArclineGripperWorker() override = default;

  static std::shared_ptr<MoveitArclineGripperWorker> create(const rclcpp::NodeOptions& options);

  bool run() override;
  bool gripper2_to_gripper0();
  bool gripper0_to_gripper2();

private:
  void runGripperSwapCallback(const std::shared_ptr<demo_interface::srv::RunGripperSwap::Request> request,
                              std::shared_ptr<demo_interface::srv::RunGripperSwap::Response> response);

  rclcpp::Service<demo_interface::srv::RunGripperSwap>::SharedPtr run_gripper_swap_srv_;
};

}  // namespace demo_driver

#endif  // DEMO_DRIVER_MOVEIT_ARCLINE_GRIPPER_WORKER_H_
