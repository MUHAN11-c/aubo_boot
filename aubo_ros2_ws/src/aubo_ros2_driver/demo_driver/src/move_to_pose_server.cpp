/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#include "demo_driver/move_to_pose_server.h"

#include <chrono>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <thread>

namespace demo_driver
{

MoveToPoseServer::MoveToPoseServer(const rclcpp::NodeOptions& options) : MoveitGripperIoBase(options)
{
  declare_parameter("move_to_pose_service_name", std::string("/move_to_pose"));
  service_name_ = get_parameter("move_to_pose_service_name").as_string();

  move_to_pose_service_ = create_service<ivg_interfaces::srv::MoveToPose>(
      service_name_,
      std::bind(&MoveToPoseServer::onMoveToPoseRequest, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "服务 %s 已创建（继承 MoveitGripperIoBase）", service_name_.c_str());
}

std::shared_ptr<MoveToPoseServer> MoveToPoseServer::create(const rclcpp::NodeOptions& options)
{
  auto node = std::make_shared<MoveToPoseServer>(options);
  node->initMoveGroup();
  return node;
}

bool MoveToPoseServer::run()
{
  return true;
}

void MoveToPoseServer::onMoveToPoseRequest(
    const std::shared_ptr<ivg_interfaces::srv::MoveToPose::Request> request,
    std::shared_ptr<ivg_interfaces::srv::MoveToPose::Response> response)
{
  std::lock_guard<std::mutex> lock(service_mutex_);

  if (!move_group_)
  {
    response->success = false;
    response->error_code = -100;
    response->message = "MoveGroup 未初始化";
    RCLCPP_ERROR(get_logger(), "[MoveToPose] %s", response->message.c_str());
    return;
  }

  if (request->velocity_factor < 0.0f || request->velocity_factor > 1.0f ||
      request->acceleration_factor < 0.0f || request->acceleration_factor > 1.0f)
  {
    response->success = false;
    response->error_code = -1;
    response->message = "velocity_factor 与 acceleration_factor 必须在 [0,1]";
    RCLCPP_WARN(get_logger(), "[MoveToPose] %s", response->message.c_str());
    return;
  }

  const auto& p = request->target_pose.position;
  const auto& q = request->target_pose.orientation;
  RCLCPP_INFO(get_logger(),
              "[MoveToPose] 收到请求 target=[%.3f, %.3f, %.3f, %.4f, %.4f, %.4f, %.4f], use_joints=%s, vel=%.2f, acc=%.2f",
              p.x, p.y, p.z, q.x, q.y, q.z, q.w, request->use_joints ? "true" : "false",
              request->velocity_factor, request->acceleration_factor);

  bool ok = false;
  if (request->use_joints)
  {
    std::array<double, 6> target_joints = { 0, 0, 0, 0, 0, 0 };
    for (size_t i = 0; i < 6; ++i)
      target_joints[i] = request->target_joints[i];
    RCLCPP_INFO(get_logger(),
                "[MoveToPose] use_joints=true，使用 target_joints=[%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                target_joints[0], target_joints[1], target_joints[2],
                target_joints[3], target_joints[4], target_joints[5]);
    ok = moveToJoints(target_joints, request->velocity_factor, request->acceleration_factor);
  }
  else
  {
    ok = moveToPose(p.x, p.y, p.z, q.x, q.y, q.z, q.w, false, request->velocity_factor,
                    request->acceleration_factor);
  }

  response->success = ok;
  response->error_code = ok ? 0 : -2;
  response->message = ok ? (request->use_joints ? "moveToJoints 执行成功" : "moveToPose 执行成功")
                         : (request->use_joints ? "moveToJoints 执行失败" : "moveToPose 执行失败");
  if (ok)
    RCLCPP_INFO(get_logger(), "[MoveToPose] 响应成功");
  else
    RCLCPP_WARN(get_logger(), "[MoveToPose] 响应失败");
}

}  // namespace demo_driver

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = demo_driver::MoveToPoseServer::create(options);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  const int kServiceWaitSec = 10;
  if (!node->waitForServices(std::chrono::seconds(kServiceWaitSec)))
  {
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "MoveToPoseServer 已就绪，调用服务 %s 触发运动",
              node->get_parameter("move_to_pose_service_name").as_string().c_str());
  spinner.join();
  rclcpp::shutdown();
  return 0;
}