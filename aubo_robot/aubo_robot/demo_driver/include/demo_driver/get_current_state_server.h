/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_GET_CURRENT_STATE_SERVER_H_
#define DEMO_DRIVER_GET_CURRENT_STATE_SERVER_H_

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <demo_interface/GetCurrentState.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <vector>

namespace demo_driver
{

/**
 * @brief 获取当前状态服务服务器类
 * 提供获取机器人当前状态的服务，包括关节位置、笛卡尔位置和速度
 */
class GetCurrentStateServer
{
public:
    GetCurrentStateServer();  // 构造函数
    ~GetCurrentStateServer(); // 析构函数

    void spin();  // 主循环，保持节点运行

private:
    // ROS 节点句柄
    ros::NodeHandle nh_;           // 公共节点句柄
    ros::NodeHandle private_nh_;  // 私有节点句柄（用于参数）

    // MoveIt 接口
    moveit::planning_interface::MoveGroupInterface* move_group_;  // MoveIt 运动组接口
    moveit::planning_interface::PlanningSceneInterface* planning_scene_interface_;  // 规划场景接口

    // 订阅器
    ros::Subscriber joint_states_sub_;  // 订阅关节状态

    // 服务客户端
    ros::ServiceClient fk_client_;  // 正运动学计算服务客户端

    // 服务服务器
    ros::ServiceServer get_current_state_service_;  // 获取当前状态服务

    // 回调函数
    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);  // 关节状态回调

    // 服务回调函数
    bool getCurrentStateCallback(demo_interface::GetCurrentState::Request& req,
                                 demo_interface::GetCurrentState::Response& res);

    // 辅助函数
    bool getCurrentState(std::vector<double>& joint_position_rad,
                        geometry_msgs::Pose& cartesian_position,
                        std::vector<double>& velocity,
                        std::string& message);

    bool getForwardKinematics(const std::vector<double>& joint_positions,
                              geometry_msgs::Pose& cartesian_pose);

    // 成员变量
    sensor_msgs::JointState current_joint_states_;  // 当前关节状态
    bool joint_states_received_;                    // 是否已收到关节状态

    // 参数
    std::string planning_group_name_;  // 规划组名称
    std::string base_frame_;           // 基础坐标系名称
    std::string end_effector_link_;    // 末端执行器链接名称
};

} // namespace demo_driver

#endif // DEMO_DRIVER_GET_CURRENT_STATE_SERVER_H_

