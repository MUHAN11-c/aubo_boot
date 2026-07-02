/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_READ_ROBOT_IO_SERVER_H_
#define DEMO_DRIVER_READ_ROBOT_IO_SERVER_H_

#include <rclcpp/rclcpp.hpp>
#include <ivg_interfaces/msg/robot_io_status.hpp>
#include <ivg_interfaces/srv/read_robot_io.hpp>
#include <string>
#include <mutex>
#include <memory>

namespace demo_driver
{

/**
 * @brief 读取机器人IO服务服务器类
 * 订阅 AuboStateBroadcaster 发布的 /robot_io_status 话题，缓存最新 IO 状态，
 * 通过 /read_robot_io 服务按索引返回指定 IO 点的值。
 *
 * 线程安全: io_state_mutex_ 保护订阅者回调（写入）和服务回调（读取）之间的并发访问。
 * MultiThreadedExecutor(2) 提供两个线程：订阅者 + 服务可并发处理。
 */
class ReadRobotIOServer : public rclcpp::Node
{
public:
    explicit ReadRobotIOServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ReadRobotIOServer();

    void spin();

private:
    // 订阅 AuboStateBroadcaster 发布的 RobotIOStatus（~50Hz）
    rclcpp::Subscription<ivg_interfaces::msg::RobotIOStatus>::SharedPtr robot_io_status_sub_;

    // 读取 IO 服务
    rclcpp::Service<ivg_interfaces::srv::ReadRobotIO>::SharedPtr read_robot_io_service_;

    // 订阅者回调 — 缓存最新 IO 状态
    void robotIOStatusCallback(const ivg_interfaces::msg::RobotIOStatus::SharedPtr msg);

    // 服务回调 — 按 io_type + io_index 返回对应值
    void readRobotIOCallback(
        const std::shared_ptr<ivg_interfaces::srv::ReadRobotIO::Request> req,
        std::shared_ptr<ivg_interfaces::srv::ReadRobotIO::Response> res);

    // 缓存的最新 IO 状态
    ivg_interfaces::msg::RobotIOStatus current_io_state_;
    std::atomic<bool> io_state_received_{false};
    std::mutex io_state_mutex_;

    // 参数 — 可配置的 IO 状态话题名
    std::string robot_io_status_topic_;
};

} // namespace demo_driver

#endif // DEMO_DRIVER_READ_ROBOT_IO_SERVER_H_
