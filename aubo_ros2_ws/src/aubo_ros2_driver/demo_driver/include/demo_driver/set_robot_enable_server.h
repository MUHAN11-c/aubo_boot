/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_SET_ROBOT_ENABLE_SERVER_H_
#define DEMO_DRIVER_SET_ROBOT_ENABLE_SERVER_H_

#include <rclcpp/rclcpp.hpp>
#include <ivg_interfaces/srv/set_robot_enable.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <memory>

namespace demo_driver
{

/**
 * @brief 设置机器人使能状态服务 — 代理到仪表盘 startup/shutdown
 *
 * enable=true  → 调用 /aubo/startup (rootServiceRobotStartup: 上电+松刹车+碰撞等级)
 * enable=false → 调用 /aubo/shutdown (robotServiceRobotShutdown: 下电)
 *
 * 不直接调用 SDK，而是复用 AuboDashboardNode 的已有服务，避免重复持有
 * ServiceInterface 连接和 sdk_mutex_。
 *
 * 线程安全:
 *   客户端在独立 MutuallyExclusive 回调组, 服务在默认组。
 *   MultiThreadedExecutor(2) 并发调度两组, future.wait_for 不阻塞客户端响应。
 */
class SetRobotEnableServer : public rclcpp::Node
{
public:
    explicit SetRobotEnableServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~SetRobotEnableServer();

    void spin();

private:
    // 独立回调组 — 客户端响应回调不阻塞服务默认组
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;

    // 仪表盘服务客户端 — enable=true 时调用 startup, false 时调用 shutdown
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr startup_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr shutdown_client_;

    // 对外暴露的 set_robot_enable 服务
    rclcpp::Service<ivg_interfaces::srv::SetRobotEnable>::SharedPtr set_robot_enable_service_;

    // 服务回调
    void setRobotEnableCallback(
        const std::shared_ptr<ivg_interfaces::srv::SetRobotEnable::Request> req,
        std::shared_ptr<ivg_interfaces::srv::SetRobotEnable::Response> res);
};

} // namespace demo_driver

#endif // DEMO_DRIVER_SET_ROBOT_ENABLE_SERVER_H_
