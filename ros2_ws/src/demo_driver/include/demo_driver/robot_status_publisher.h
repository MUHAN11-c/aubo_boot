/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_ROBOT_STATUS_PUBLISHER_H_
#define DEMO_DRIVER_ROBOT_STATUS_PUBLISHER_H_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <demo_interface/msg/robot_status.hpp>
#include <aubo_msgs/srv/get_fk.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <string>
#include <vector>
#include <memory>

namespace demo_driver
{

/**
 * @brief 机器人状态发布器类
 * 用于收集机器人状态信息并发布到 /robot_status 话题
 */
class RobotStatusPublisher : public rclcpp::Node
{
public:
    RobotStatusPublisher();  // 构造函数
    ~RobotStatusPublisher(); // 析构函数

    void spin();  // 主循环，保持节点运行

private:
    // 发布器
    rclcpp::Publisher<demo_interface::msg::RobotStatus>::SharedPtr robot_status_pub_;  // 发布机器人状态消息

    // 订阅器
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;              // 订阅关节状态
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trajectory_execution_sub_;      // 订阅轨迹执行事件

    // 服务客户端
    rclcpp::Client<aubo_msgs::srv::GetFK>::SharedPtr fk_client_;  // 正运动学计算服务客户端

    // 回调函数
    void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);  // 关节状态回调
    void trajectoryExecutionCallback(const std_msgs::msg::String::SharedPtr msg);  // 轨迹执行事件回调

    // 辅助函数
    bool getForwardKinematics(const std::vector<double>& joint_positions, 
                              geometry_msgs::msg::Pose& cartesian_pose);  // 计算正运动学（获取笛卡尔位置）
    std::string getPlanningStatus();  // 获取规划状态
    void publishRobotStatus();  // 发布机器人状态消息

    // 成员变量
    sensor_msgs::msg::JointState current_joint_states_;           // 当前关节状态
    bool joint_states_received_;      // 是否已收到关节状态
    std::string planning_status_;     // 规划状态（idle/planning/executing/error）
    rclcpp::TimerBase::SharedPtr status_timer_;         // 状态发布定时器
    
    // 定时器回调
    void statusTimerCallback();  // 定时器回调，定期发布状态

    // 参数
    double publish_rate_;              // 发布频率（Hz）
    std::string base_frame_;           // 基础坐标系名称
    std::string planning_group_name_;  // 规划组名称
};

} // namespace demo_driver

#endif // DEMO_DRIVER_ROBOT_STATUS_PUBLISHER_H_
