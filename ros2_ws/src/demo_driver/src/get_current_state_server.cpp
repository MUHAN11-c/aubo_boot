/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#include "demo_driver/get_current_state_server.h"
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <aubo_msgs/srv/get_fk.hpp>
#include <Eigen/Geometry>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace demo_driver
{

/**
 * @brief 构造函数，初始化 MoveIt 接口和服务服务器
 */
GetCurrentStateServer::GetCurrentStateServer()
    : Node("get_current_state_server_node")
    , joint_states_received_(false)
    , planning_group_name_("manipulator_e5")
    , base_frame_("base_link")
    , end_effector_link_("")
{
    // 获取参数
    this->declare_parameter("planning_group_name", std::string("manipulator_e5"));
    this->declare_parameter("base_frame", std::string("base_link"));
    
    this->get_parameter("planning_group_name", planning_group_name_);
    this->get_parameter("base_frame", base_frame_);

    // 初始化 MoveIt 接口
    RCLCPP_INFO(this->get_logger(), "Initializing MoveIt interfaces for planning group: %s", planning_group_name_.c_str());
    
    try
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), planning_group_name_);
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
        
        // 设置参考坐标系
        move_group_->setPoseReferenceFrame(base_frame_);
        
        // 获取末端执行器链接名称
        end_effector_link_ = move_group_->getEndEffectorLink();
        RCLCPP_INFO(this->get_logger(), "End effector link: %s", end_effector_link_.c_str());
        
        RCLCPP_INFO(this->get_logger(), "MoveIt interfaces initialized successfully");
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveIt interfaces: %s", e.what());
        throw;
    }

    // 初始化订阅器
    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        std::bind(&GetCurrentStateServer::jointStatesCallback, this, std::placeholders::_1));

    // 初始化服务客户端
    fk_client_ = this->create_client<aubo_msgs::srv::GetFK>("/aubo_driver/get_fk");

    // 等待服务可用
    if (!fk_client_->wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_WARN(this->get_logger(), "FK service not available, will try to use MoveIt to get cartesian position");
    }

    // 初始化服务服务器
    get_current_state_service_ = this->create_service<demo_interface::srv::GetCurrentState>(
        "/get_current_state",
        std::bind(&GetCurrentStateServer::getCurrentStateCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "GetCurrentStateServer initialized, service '/get_current_state' is ready");
}

GetCurrentStateServer::~GetCurrentStateServer()
{
}

/**
 * @brief 关节状态回调函数
 */
void GetCurrentStateServer::jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    current_joint_states_ = *msg;  // 保存当前关节状态
    joint_states_received_ = true;  // 标记已收到关节状态
}

/**
 * @brief 服务回调函数
 */
void GetCurrentStateServer::getCurrentStateCallback(
    const std::shared_ptr<demo_interface::srv::GetCurrentState::Request> /* req */,
    std::shared_ptr<demo_interface::srv::GetCurrentState::Response> res)
{
    RCLCPP_DEBUG(this->get_logger(), "Received get_current_state request");

    // 获取当前状态
    std::string message;
    bool success = getCurrentState(res->joint_position_rad,
                                   res->cartesian_position,
                                   res->velocity,
                                   message);

    res->success = success;
    res->message = message;

    if (success)
    {
        RCLCPP_DEBUG(this->get_logger(), "Get current state succeeded: %s", message.c_str());
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Get current state failed: %s", message.c_str());
    }
}

/**
 * @brief 获取当前状态
 */
bool GetCurrentStateServer::getCurrentState(std::vector<double>& joint_position_rad,
                                            geometry_msgs::msg::Pose& cartesian_position,
                                            std::vector<double>& velocity,
                                            std::string& message)
{
    // 方法1：从订阅的 joint_states 获取
    if (joint_states_received_ && !current_joint_states_.position.empty())
    {
        // 获取关节位置
        joint_position_rad.clear();
        velocity.clear();

        // 查找关节名称（假设是6个关节）
        std::vector<std::string> joint_names = {
            "shoulder_joint", "upperArm_joint", "foreArm_joint",
            "wrist1_joint", "wrist2_joint", "wrist3_joint"
        };

        joint_position_rad.resize(6, 0.0);
        velocity.resize(6, 0.0);

        for (size_t i = 0; i < joint_names.size() && i < 6; ++i)
        {
            for (size_t j = 0; j < current_joint_states_.name.size(); ++j)
            {
                if (current_joint_states_.name[j] == joint_names[i])
                {
                    if (j < current_joint_states_.position.size())
                    {
                        joint_position_rad[i] = current_joint_states_.position[j];
                    }
                    if (j < current_joint_states_.velocity.size())
                    {
                        velocity[i] = current_joint_states_.velocity[j];
                    }
                    break;
                }
            }
        }

        // 如果关节名称不匹配，尝试使用前6个位置
        if (joint_position_rad[0] == 0.0 && joint_position_rad[1] == 0.0 &&
            current_joint_states_.position.size() >= 6)
        {
            for (size_t i = 0; i < 6; ++i)
            {
                joint_position_rad[i] = current_joint_states_.position[i];
                if (i < current_joint_states_.velocity.size())
                {
                    velocity[i] = current_joint_states_.velocity[i];
                }
            }
        }

        // 获取笛卡尔位置
        if (getForwardKinematics(joint_position_rad, cartesian_position))
        {
            message = "Successfully retrieved current state from joint_states";
            return true;
        }
    }

    // 方法2：从 MoveIt 获取当前状态
    if (move_group_)
    {
        try
        {
            // 获取当前关节值
            moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();
            const moveit::core::JointModelGroup* joint_model_group =
                current_state->getJointModelGroup(planning_group_name_);

            std::vector<double> joint_values;
            current_state->copyJointGroupPositions(joint_model_group, joint_values);

            if (joint_values.size() >= 6)
            {
                joint_position_rad.clear();
                for (size_t i = 0; i < 6 && i < joint_values.size(); ++i)
                {
                    joint_position_rad.push_back(joint_values[i]);
                }

                // 获取当前位姿
                geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose(end_effector_link_);
                cartesian_position = current_pose.pose;

                // 速度信息从 MoveIt 较难获取，设为0
                velocity.clear();
                velocity.resize(6, 0.0);

                message = "Successfully retrieved current state from MoveIt";
                return true;
            }
        }
        catch (const std::exception& e)
        {
            message = std::string("Failed to get state from MoveIt: ") + e.what();
            RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
        }
    }

    // 如果都失败了
    message = "Failed to retrieve current state: no valid data source available";
    return false;
}

/**
 * @brief 计算正运动学，获取笛卡尔位置
 */
bool GetCurrentStateServer::getForwardKinematics(const std::vector<double>& joint_positions,
                                                 geometry_msgs::msg::Pose& cartesian_pose)
{
    if (joint_positions.size() < 6)
    {
        return false;
    }

    // 尝试使用 FK 服务
    if (fk_client_->service_is_ready())
    {
        auto request = std::make_shared<aubo_msgs::srv::GetFK::Request>();
        for (size_t i = 0; i < 6 && i < joint_positions.size(); ++i)
        {
            request->joint[i] = static_cast<float>(joint_positions[i]);
        }
        // 如果关节数少于6，填充剩余位置为0
        for (size_t i = joint_positions.size(); i < 6; ++i)
        {
            request->joint[i] = 0.0f;
        }

        auto result = fk_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = result.get();
            if (response->pos.size() >= 3 && response->ori.size() >= 4)
            {
                cartesian_pose.position.x = response->pos[0];
                cartesian_pose.position.y = response->pos[1];
                cartesian_pose.position.z = response->pos[2];
                cartesian_pose.orientation.w = response->ori[0];
                cartesian_pose.orientation.x = response->ori[1];
                cartesian_pose.orientation.y = response->ori[2];
                cartesian_pose.orientation.z = response->ori[3];
                return true;
            }
        }
    }

    // 如果 FK 服务不可用，尝试使用 MoveIt
    if (move_group_)
    {
        try
        {
            // 设置关节值
            moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();
            const moveit::core::JointModelGroup* joint_model_group =
                current_state->getJointModelGroup(planning_group_name_);

            std::vector<double> joint_values;
            for (size_t i = 0; i < 6 && i < joint_positions.size(); ++i)
            {
                joint_values.push_back(joint_positions[i]);
            }

            current_state->setJointGroupPositions(joint_model_group, joint_values);
            current_state->update();

            // 获取末端执行器位姿
            const Eigen::Isometry3d& end_effector_state =
                current_state->getGlobalLinkTransform(end_effector_link_);

            // 转换为 geometry_msgs::Pose
            Eigen::Vector3d position = end_effector_state.translation();
            Eigen::Quaterniond orientation(end_effector_state.rotation());

            cartesian_pose.position.x = position.x();
            cartesian_pose.position.y = position.y();
            cartesian_pose.position.z = position.z();
            cartesian_pose.orientation.w = orientation.w();
            cartesian_pose.orientation.x = orientation.x();
            cartesian_pose.orientation.y = orientation.y();
            cartesian_pose.orientation.z = orientation.z();

            return true;
        }
        catch (const std::exception& e)
        {
            RCLCPP_DEBUG(this->get_logger(), "Failed to compute FK using MoveIt: %s", e.what());
        }
    }

    return false;
}

/**
 * @brief 主循环，保持节点运行
 */
void GetCurrentStateServer::spin()
{
    rclcpp::spin(this->shared_from_this());
}

} // namespace demo_driver
