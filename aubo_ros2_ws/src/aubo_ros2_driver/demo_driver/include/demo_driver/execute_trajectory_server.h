/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_EXECUTE_TRAJECTORY_SERVER_H_
#define DEMO_DRIVER_EXECUTE_TRAJECTORY_SERVER_H_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <demo_interface/srv/execute_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <string>
#include <memory>

namespace demo_driver
{

/**
 * @brief 执行轨迹服务服务器类
 * 提供轨迹执行服务，接收轨迹并执行
 */
class ExecuteTrajectoryServer : public rclcpp::Node
{
public:
    explicit ExecuteTrajectoryServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());  // 构造函数
    ~ExecuteTrajectoryServer(); // 析构函数

    bool initialize(int max_retries = 10, int retry_delay_seconds = 2);  // 初始化 MoveIt 接口（在节点创建为 shared_ptr 后调用，带重试机制）
    void spin();  // 主循环，保持节点运行

private:
    bool wait_for_robot_description(int timeout_seconds = 30);  // 等待并从其他节点复制 robot_description 参数
    // MoveIt 接口
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;  // MoveIt 运动组接口
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;  // 规划场景接口

    // 服务服务器
    rclcpp::Service<demo_interface::srv::ExecuteTrajectory>::SharedPtr execute_trajectory_service_;  // 执行轨迹服务

    // 服务回调函数
    void executeTrajectoryCallback(
        const std::shared_ptr<demo_interface::srv::ExecuteTrajectory::Request> req,
        std::shared_ptr<demo_interface::srv::ExecuteTrajectory::Response> res);

    // 辅助函数
    bool executeTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory,
                          int32_t& error_code,
                          std::string& message);

    // 参数
    std::string planning_group_name_;  // 规划组名称
    std::string base_frame_;           // 基础坐标系名称
};

} // namespace demo_driver

#endif // DEMO_DRIVER_EXECUTE_TRAJECTORY_SERVER_H_
