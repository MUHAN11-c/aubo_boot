/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_MOVE_TO_POSE_SERVER_H_
#define DEMO_DRIVER_MOVE_TO_POSE_SERVER_H_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <demo_interface/srv/move_to_pose.hpp>
#include <string>
#include <memory>

namespace demo_driver
{

/**
 * @brief 移动到目标位姿服务服务器类
 * 提供 MoveIt 运动规划服务，支持关节空间和笛卡尔空间规划
 */
class MoveToPoseServer : public rclcpp::Node
{
public:
    explicit MoveToPoseServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());  // 构造函数
    ~MoveToPoseServer(); // 析构函数

    bool initialize(int max_retries = 10, int retry_delay_seconds = 2);  // 初始化 MoveIt 接口（在节点创建为 shared_ptr 后调用，带重试机制）
    void spin();  // 主循环，保持节点运行

private:
    bool wait_for_robot_description(int timeout_seconds = 30);  // 等待并从其他节点复制 robot_description 参数
    // MoveIt 接口
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;  // MoveIt 运动组接口
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;  // 规划场景接口

    // 服务服务器
    rclcpp::Service<demo_interface::srv::MoveToPose>::SharedPtr move_to_pose_service_;  // 移动到目标位姿服务

    // 服务回调函数
    void moveToPoseCallback(
        const std::shared_ptr<demo_interface::srv::MoveToPose::Request> req,
        std::shared_ptr<demo_interface::srv::MoveToPose::Response> res);

    // 辅助函数
    bool moveToPose(const geometry_msgs::msg::Pose& target_pose, 
                    bool use_joints,
                    float velocity_factor,
                    float acceleration_factor,
                    int32_t& error_code,
                    std::string& message);

    // 参数
    std::string planning_group_name_;  // 规划组名称
    std::string base_frame_;           // 基础坐标系名称
    std::string end_effector_link_;    // 末端执行器链接名称
};

} // namespace demo_driver

#endif // DEMO_DRIVER_MOVE_TO_POSE_SERVER_H_
