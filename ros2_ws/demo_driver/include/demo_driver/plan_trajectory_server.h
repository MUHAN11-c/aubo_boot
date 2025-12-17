/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_PLAN_TRAJECTORY_SERVER_H_
#define DEMO_DRIVER_PLAN_TRAJECTORY_SERVER_H_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <demo_interface/srv/plan_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <string>
#include <memory>

namespace demo_driver
{

/**
 * @brief 规划轨迹服务服务器类
 * 提供 MoveIt 轨迹规划服务，只规划不执行，支持关节空间和笛卡尔空间规划
 */
class PlanTrajectoryServer : public rclcpp::Node
{
public:
    PlanTrajectoryServer();  // 构造函数
    ~PlanTrajectoryServer(); // 析构函数

    void spin();  // 主循环，保持节点运行

private:
    // MoveIt 接口
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;  // MoveIt 运动组接口
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;  // 规划场景接口

    // 服务服务器
    rclcpp::Service<demo_interface::srv::PlanTrajectory>::SharedPtr plan_trajectory_service_;  // 规划轨迹服务

    // 服务回调函数
    void planTrajectoryCallback(
        const std::shared_ptr<demo_interface::srv::PlanTrajectory::Request> req,
        std::shared_ptr<demo_interface::srv::PlanTrajectory::Response> res);

    // 辅助函数
    bool planTrajectory(const geometry_msgs::msg::Pose& target_pose,
                        bool use_joints,
                        trajectory_msgs::msg::JointTrajectory& trajectory,
                        float& planning_time,
                        std::string& message);

    // 参数
    std::string planning_group_name_;  // 规划组名称
    std::string base_frame_;           // 基础坐标系名称
    std::string end_effector_link_;    // 末端执行器链接名称
};

} // namespace demo_driver

#endif // DEMO_DRIVER_PLAN_TRAJECTORY_SERVER_H_
