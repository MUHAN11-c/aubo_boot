/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_MOVE_TO_POSE_SERVER_H_
#define DEMO_DRIVER_MOVE_TO_POSE_SERVER_H_

#include "demo_driver/moveit_gripper_io_base.h"

#include <ivg_interfaces/srv/move_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <memory>
#include <mutex>
#include <string>

namespace demo_driver
{

/**
 * @brief /move_to_pose 服务节点
 *
 * 继承 MoveitGripperIoBase，复用其 MoveGroup 初始化、位姿运动与 IO 依赖等待能力。
 * 服务模式参考 GripperSwapWorker：构造时创建服务，create() 中调用 initMoveGroup()。
 */
class MoveToPoseServer : public MoveitGripperIoBase
{
public:
  explicit MoveToPoseServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~MoveToPoseServer() override = default;

  static std::shared_ptr<MoveToPoseServer> create(const rclcpp::NodeOptions& options);
  bool run() override;

private:
  void onMoveToPoseRequest(const std::shared_ptr<ivg_interfaces::srv::MoveToPose::Request> request,
                           std::shared_ptr<ivg_interfaces::srv::MoveToPose::Response> response);

  rclcpp::Service<ivg_interfaces::srv::MoveToPose>::SharedPtr move_to_pose_service_;
  std::string service_name_;
  std::mutex service_mutex_;
};

}  // namespace demo_driver

#endif  // DEMO_DRIVER_MOVE_TO_POSE_SERVER_H_
