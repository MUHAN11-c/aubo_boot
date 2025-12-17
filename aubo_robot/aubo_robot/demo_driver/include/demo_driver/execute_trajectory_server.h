/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_EXECUTE_TRAJECTORY_SERVER_H_
#define DEMO_DRIVER_EXECUTE_TRAJECTORY_SERVER_H_

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <demo_interface/ExecuteTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <string>

namespace demo_driver
{

/**
 * @brief 执行轨迹服务服务器类
 * 提供轨迹执行服务，接收轨迹并执行
 */
class ExecuteTrajectoryServer
{
public:
    ExecuteTrajectoryServer();  // 构造函数
    ~ExecuteTrajectoryServer(); // 析构函数

    void spin();  // 主循环，保持节点运行

private:
    // ROS 节点句柄
    ros::NodeHandle nh_;           // 公共节点句柄
    ros::NodeHandle private_nh_;  // 私有节点句柄（用于参数）

    // MoveIt 接口
    moveit::planning_interface::MoveGroupInterface* move_group_;  // MoveIt 运动组接口
    moveit::planning_interface::PlanningSceneInterface* planning_scene_interface_;  // 规划场景接口

    // 服务服务器
    ros::ServiceServer execute_trajectory_service_;  // 执行轨迹服务

    // 服务回调函数
    bool executeTrajectoryCallback(demo_interface::ExecuteTrajectory::Request& req,
                                   demo_interface::ExecuteTrajectory::Response& res);

    // 辅助函数
    bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory,
                          int32_t& error_code,
                          std::string& message);

    // 参数
    std::string planning_group_name_;  // 规划组名称
    std::string base_frame_;           // 基础坐标系名称
};

} // namespace demo_driver

#endif // DEMO_DRIVER_EXECUTE_TRAJECTORY_SERVER_H_

