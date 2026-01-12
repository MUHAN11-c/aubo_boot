/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_SET_SPEED_FACTOR_SERVER_H_
#define DEMO_DRIVER_SET_SPEED_FACTOR_SERVER_H_

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <demo_interface/SetSpeedFactor.h>
#include <string>

namespace demo_driver
{

/**
 * @brief 设置速度因子服务服务器类
 * 提供设置 MoveIt 速度缩放因子的服务
 */
class SetSpeedFactorServer
{
public:
    SetSpeedFactorServer();  // 构造函数
    ~SetSpeedFactorServer(); // 析构函数

    void spin();  // 主循环，保持节点运行

private:
    // ROS 节点句柄
    ros::NodeHandle nh_;           // 公共节点句柄
    ros::NodeHandle private_nh_;  // 私有节点句柄（用于参数）

    // MoveIt 接口
    moveit::planning_interface::MoveGroupInterface* move_group_;  // MoveIt 运动组接口
    moveit::planning_interface::PlanningSceneInterface* planning_scene_interface_;  // 规划场景接口

    // 服务服务器
    ros::ServiceServer set_speed_factor_service_;  // 设置速度因子服务

    // 服务回调函数
    bool setSpeedFactorCallback(demo_interface::SetSpeedFactor::Request& req,
                                demo_interface::SetSpeedFactor::Response& res);

    // 辅助函数
    bool setSpeedFactor(float velocity_factor, std::string& message);

    // 参数
    std::string planning_group_name_;  // 规划组名称
    std::string base_frame_;           // 基础坐标系名称
};

} // namespace demo_driver

#endif // DEMO_DRIVER_SET_SPEED_FACTOR_SERVER_H_

