/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#include "demo_driver/robot_status_publisher.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <array>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace demo_driver
{

/**
 * @brief 构造函数，初始化发布器、订阅器和服务客户端
 */
RobotStatusPublisher::RobotStatusPublisher()
    : Node("robot_status_publisher_node")
    , joint_states_received_(false)
    , planning_status_("idle")
    , publish_rate_(10.0)
    , base_frame_("base_link")
    , planning_group_name_("manipulator_e5")
{
    // 获取参数
    this->declare_parameter("publish_rate", 10.0);
    this->declare_parameter("base_frame", std::string("base_link"));
    this->declare_parameter("planning_group_name", std::string("manipulator_e5"));
    
    this->get_parameter("publish_rate", publish_rate_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("planning_group_name", planning_group_name_);

    // 初始化发布器
    robot_status_pub_ = this->create_publisher<demo_interface::msg::RobotStatus>("/robot_status", 10);

    // 初始化订阅器
    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, 
        std::bind(&RobotStatusPublisher::jointStatesCallback, this, std::placeholders::_1));
    
    trajectory_execution_sub_ = this->create_subscription<std_msgs::msg::String>(
        "trajectory_execution_event", 10,
        std::bind(&RobotStatusPublisher::trajectoryExecutionCallback, this, std::placeholders::_1));

    // 初始化服务客户端
    fk_client_ = this->create_client<aubo_msgs::srv::GetFK>("/aubo_driver/get_fk");

    // 等待服务可用
    if (!fk_client_->wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_WARN(this->get_logger(), "FK service not available, cartesian position may not be accurate");
    }

    // 初始化定时器，用于定期发布状态
    status_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
        std::bind(&RobotStatusPublisher::statusTimerCallback, this));

    RCLCPP_INFO(this->get_logger(), "RobotStatusPublisher initialized");
}

RobotStatusPublisher::~RobotStatusPublisher()
{
}

/**
 * @brief 关节状态回调函数
 * @param msg 关节状态消息
 */
void RobotStatusPublisher::jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    current_joint_states_ = *msg;  // 保存当前关节状态
    joint_states_received_ = true;  // 标记已收到关节状态
}

/**
 * @brief 轨迹执行事件回调函数
 * @param msg 轨迹执行事件消息
 */
void RobotStatusPublisher::trajectoryExecutionCallback(const std_msgs::msg::String::SharedPtr msg)
{
    // 根据轨迹执行事件更新规划状态
    std::string event = msg->data;
    if (event == "stop" || event == "cancel")
    {
        planning_status_ = "idle";  // 停止或取消
    }
    else if (event == "planning")
    {
        planning_status_ = "planning";  // 规划中
    }
    else if (event == "executing" || event == "execute")
    {
        planning_status_ = "executing";  // 执行中
    }
    // 其他事件可以在这里处理
}

/**
 * @brief 计算正运动学，获取笛卡尔位置
 * @param joint_positions 关节位置（弧度）
 * @param cartesian_pose 输出的笛卡尔位姿
 * @return 成功返回true，失败返回false
 */
bool RobotStatusPublisher::getForwardKinematics(const std::vector<double>& joint_positions,
                                                 geometry_msgs::msg::Pose& cartesian_pose)
{
    if (joint_positions.size() < 6)
    {
        RCLCPP_WARN(this->get_logger(), "Insufficient joint positions for FK calculation");
        return false;
    }

    // 准备服务请求
    auto request = std::make_shared<aubo_msgs::srv::GetFK::Request>();
    for (size_t i = 0; i < 6 && i < joint_positions.size(); ++i)
    {
        request->joint[i] = static_cast<float>(joint_positions[i]);  // 转换为float
    }
    // 如果关节数少于6，填充剩余位置为0
    for (size_t i = joint_positions.size(); i < 6; ++i)
    {
        request->joint[i] = 0.0f;
    }

    // 调用正运动学服务
    auto result = fk_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) == 
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = result.get();
        if (response->pos.size() >= 3 && response->ori.size() >= 4)
        {
            // 提取位置（x, y, z）
            cartesian_pose.position.x = response->pos[0];
            cartesian_pose.position.y = response->pos[1];
            cartesian_pose.position.z = response->pos[2];
            // 提取姿态四元数（w, x, y, z）
            cartesian_pose.orientation.w = response->ori[0];
            cartesian_pose.orientation.x = response->ori[1];
            cartesian_pose.orientation.y = response->ori[2];
            cartesian_pose.orientation.z = response->ori[3];
            return true;
        }
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(), "Failed to call FK service");
    }

    return false;
}

/**
 * @brief 获取当前规划状态
 * @return 规划状态字符串（idle/planning/executing/error）
 */
std::string RobotStatusPublisher::getPlanningStatus()
{
    return planning_status_;
}

/**
 * @brief 发布机器人状态消息
 * 收集所有状态信息并发布到 /robot_status 话题
 */
void RobotStatusPublisher::publishRobotStatus()
{
    // 检查是否已收到关节状态
    if (!joint_states_received_)
    {
        RCLCPP_DEBUG(this->get_logger(), "Joint states not received yet");
        return;
    }

    auto status_msg = std::make_shared<demo_interface::msg::RobotStatus>();

    // 设置消息头
    status_msg->header.stamp = this->now();  // 当前时间戳
    status_msg->header.frame_id = base_frame_;    // 基础坐标系

    // 获取关节位置
    std::vector<double> joint_positions_rad;  // 弧度
    std::vector<double> joint_positions_deg;   // 度

    // 从关节状态中查找关节位置
    // 假设关节名称：shoulder_joint, upperArm_joint, foreArm_joint, 
    // wrist1_joint, wrist2_joint, wrist3_joint
    std::vector<std::string> joint_names = {
        "shoulder_joint", "upperArm_joint", "foreArm_joint",
        "wrist1_joint", "wrist2_joint", "wrist3_joint"
    };

    joint_positions_rad.resize(6, 0.0);
    joint_positions_deg.resize(6, 0.0);

    // 根据关节名称匹配并提取位置
    for (size_t i = 0; i < joint_names.size() && i < 6; ++i)
    {
        for (size_t j = 0; j < current_joint_states_.name.size(); ++j)
        {
            if (current_joint_states_.name[j] == joint_names[i])
            {
                if (j < current_joint_states_.position.size())
                {
                    joint_positions_rad[i] = current_joint_states_.position[j];  // 弧度
                    joint_positions_deg[i] = joint_positions_rad[i] * 180.0 / M_PI;  // 转换为度
                }
                break;
            }
        }
    }

    // 如果关节名称不匹配，尝试使用前6个位置
    if (joint_positions_rad[0] == 0.0 && joint_positions_rad[1] == 0.0 && 
        current_joint_states_.position.size() >= 6)
    {
        for (size_t i = 0; i < 6; ++i)
        {
            joint_positions_rad[i] = current_joint_states_.position[i];
            joint_positions_deg[i] = joint_positions_rad[i] * 180.0 / M_PI;
        }
    }

    // 设置关节位置（弧度和度）- 将 vector 转换为 array
    std::array<double, 6> joint_rad_array;
    std::array<double, 6> joint_deg_array;
    for (size_t i = 0; i < 6; ++i)
    {
        joint_rad_array[i] = (i < joint_positions_rad.size()) ? joint_positions_rad[i] : 0.0;
        joint_deg_array[i] = (i < joint_positions_deg.size()) ? joint_positions_deg[i] : 0.0;
    }
    status_msg->joint_position_rad = joint_rad_array;
    status_msg->joint_position_deg = joint_deg_array;

    // 设置默认状态（ROS2中没有industrial_msgs，使用默认值）
    status_msg->is_online = true;   // 假设在线
    status_msg->enable = true;      // 假设已使能
    status_msg->in_motion = (planning_status_ == "executing");  // 根据规划状态判断

    // 获取规划状态
    status_msg->planning_status = getPlanningStatus();

    // 使用正运动学服务获取笛卡尔位置
    if (getForwardKinematics(joint_positions_rad, status_msg->cartesian_position))
    {
        // 成功获取笛卡尔位置
    }
    else
    {
        // 如果正运动学计算失败，设置默认位姿
        status_msg->cartesian_position.position.x = 0.0;
        status_msg->cartesian_position.position.y = 0.0;
        status_msg->cartesian_position.position.z = 0.0;
        status_msg->cartesian_position.orientation.w = 1.0;  // 单位四元数
        status_msg->cartesian_position.orientation.x = 0.0;
        status_msg->cartesian_position.orientation.y = 0.0;
        status_msg->cartesian_position.orientation.z = 0.0;
    }

    // 发布消息
    robot_status_pub_->publish(*status_msg);
}

/**
 * @brief 定时器回调函数
 */
void RobotStatusPublisher::statusTimerCallback()
{
    publishRobotStatus();  // 定期发布机器人状态
}

/**
 * @brief 主循环，保持节点运行
 */
void RobotStatusPublisher::spin()
{
    rclcpp::spin(this->shared_from_this());  // ROS 自旋，处理回调
}

} // namespace demo_driver
