/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_GET_CURRENT_STATE_SERVER_H_
#define DEMO_DRIVER_GET_CURRENT_STATE_SERVER_H_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <demo_interface/srv/get_current_state.hpp>
#include <aubo_msgs/srv/get_fk.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <string>
#include <vector>
#include <memory>

namespace demo_driver
{

/**
 * @brief 获取当前状态服务服务器类
 * 提供获取机器人当前状态的服务，包括关节位置、笛卡尔位置和速度
 */
class GetCurrentStateServer : public rclcpp::Node
{
public:
    GetCurrentStateServer();  // 构造函数
    ~GetCurrentStateServer(); // 析构函数

    void spin();  // 主循环，保持节点运行

private:
    // MoveIt 接口
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;  // MoveIt 运动组接口
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;  // 规划场景接口

    // 订阅器
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;  // 订阅关节状态

    // 服务客户端
    rclcpp::Client<aubo_msgs::srv::GetFK>::SharedPtr fk_client_;  // 正运动学计算服务客户端

    // 服务服务器
    rclcpp::Service<demo_interface::srv::GetCurrentState>::SharedPtr get_current_state_service_;  // 获取当前状态服务

    // 回调函数
    void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);  // 关节状态回调

    // 服务回调函数
    void getCurrentStateCallback(
        const std::shared_ptr<demo_interface::srv::GetCurrentState::Request> req,
        std::shared_ptr<demo_interface::srv::GetCurrentState::Response> res);

    // 辅助函数
    bool getCurrentState(std::vector<double>& joint_position_rad,
                        geometry_msgs::msg::Pose& cartesian_position,
                        std::vector<double>& velocity,
                        std::string& message);

    bool getForwardKinematics(const std::vector<double>& joint_positions,
                              geometry_msgs::msg::Pose& cartesian_pose);

    // 成员变量
    sensor_msgs::msg::JointState current_joint_states_;  // 当前关节状态
    bool joint_states_received_;                    // 是否已收到关节状态

    // 参数
    std::string planning_group_name_;  // 规划组名称
    std::string base_frame_;           // 基础坐标系名称
    std::string end_effector_link_;    // 末端执行器链接名称
};

} // namespace demo_driver

#endif // DEMO_DRIVER_GET_CURRENT_STATE_SERVER_H_
