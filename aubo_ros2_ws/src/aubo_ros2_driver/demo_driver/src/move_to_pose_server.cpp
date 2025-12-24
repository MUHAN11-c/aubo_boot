/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#include "demo_driver/move_to_pose_server.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <aubo_msgs/srv/get_ik.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <thread>
#include <chrono>
#include <rclcpp/parameter_client.hpp>

namespace demo_driver
{

/**
 * @brief 构造函数，初始化 MoveIt 接口和服务服务器
 */
MoveToPoseServer::MoveToPoseServer()
    : Node("move_to_pose_server_node")
    , planning_group_name_("manipulator")
    , base_frame_("base_link")
    , end_effector_link_("")
{
    // 获取参数
    this->declare_parameter("planning_group_name", std::string("manipulator"));
    this->declare_parameter("base_frame", std::string("base_link"));
    
    this->get_parameter("planning_group_name", planning_group_name_);
    this->get_parameter("base_frame", base_frame_);

    // 初始化服务服务器
    move_to_pose_service_ = this->create_service<demo_interface::srv::MoveToPose>(
        "/move_to_pose",
        std::bind(&MoveToPoseServer::moveToPoseCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "MoveToPoseServer node created, ready to initialize MoveIt");
}

bool MoveToPoseServer::wait_for_robot_description(int timeout_seconds)
{
    RCLCPP_INFO(this->get_logger(), "Waiting for robot_description parameter and MoveIt2 services...");
    
    auto start_time = std::chrono::steady_clock::now();
    int check_count = 0;
    
    std::vector<std::string> source_nodes = {"/move_group", "/robot_state_publisher"};
    bool param_copied = false;
    
    for (const auto& source_node : source_nodes) {
        try {
            auto remote_param_client = std::make_shared<rclcpp::SyncParametersClient>(
                shared_from_this(), source_node);
            
            if (remote_param_client->wait_for_service(std::chrono::seconds(2))) {
                std::vector<rclcpp::Parameter> params = remote_param_client->get_parameters(
                    {"robot_description", "robot_description_semantic"});
                
                if (!params.empty()) {
                    for (const auto& param : params) {
                        this->declare_parameter(param.get_name(), param.get_parameter_value());
                        this->set_parameter(param);
                    }
                    param_copied = true;
                    RCLCPP_INFO(this->get_logger(), "Successfully copied robot_description parameter from %s node", source_node.c_str());
                    break;
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_DEBUG(this->get_logger(), "Failed to get param from source %s: %s", source_node.c_str(), e.what());
        }
    }
    
    if (!param_copied) {
        RCLCPP_WARN(this->get_logger(), "Could not get robot_description parameter from other nodes, will try to use MoveGroupInterface directly");
    }
    
    while (rclcpp::ok())
    {
        try
        {
            auto test_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(), planning_group_name_);
            
            std::string test_frame = test_move_group->getPlanningFrame();
            
            if (!test_frame.empty())
            {
                RCLCPP_INFO(this->get_logger(), "Robot description parameter and MoveIt2 services are ready");
                RCLCPP_INFO(this->get_logger(), "Planning frame: %s", test_frame.c_str());
                return true;
            }
        }
        catch (const std::exception& e)
        {
            check_count++;
            if (check_count % 10 == 0)
            {
                auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::steady_clock::now() - start_time).count();
                RCLCPP_INFO(this->get_logger(), "Still waiting... (elapsed %ld seconds, error: %s)",
                           elapsed_seconds, e.what());
            }
        }
        
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() >= timeout_seconds)
        {
            RCLCPP_ERROR(this->get_logger(), "Timeout waiting (%d seconds)", timeout_seconds);
            return false;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    return false;
}

bool MoveToPoseServer::initialize(int max_retries, int retry_delay_seconds)
{
    RCLCPP_INFO(this->get_logger(), "Initializing MoveIt interfaces for planning group: %s", planning_group_name_.c_str());
    
    // 等待机器人描述参数
    if (!wait_for_robot_description(30))
    {
        return false;
    }
    
    // 等待一段时间，确保参数完全加载
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    for (int attempt = 1; attempt <= max_retries; ++attempt)
    {
        try
        {
            RCLCPP_INFO(this->get_logger(), "Attempting to initialize MoveIt interfaces (attempt %d/%d)...", attempt, max_retries);
            
            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(), planning_group_name_);
            planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
            
            // 设置参考坐标系
            move_group_->setPoseReferenceFrame(base_frame_);
            
            // 获取末端执行器链接名称
            end_effector_link_ = move_group_->getEndEffectorLink();
            RCLCPP_INFO(this->get_logger(), "End effector link: %s", end_effector_link_.c_str());
            
            // 设置规划时间
            move_group_->setPlanningTime(10.0);
            
            RCLCPP_INFO(this->get_logger(), "MoveIt interfaces initialized successfully");
            RCLCPP_INFO(this->get_logger(), "MoveToPoseServer initialized, service '/move_to_pose' is ready");
            return true;
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to initialize MoveIt interfaces (attempt %d/%d): %s", 
                       attempt, max_retries, e.what());
            
            if (attempt < max_retries)
            {
                RCLCPP_INFO(this->get_logger(), "Waiting %d seconds before retry...", retry_delay_seconds);
                std::this_thread::sleep_for(std::chrono::seconds(retry_delay_seconds));
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveIt interfaces after %d attempts", max_retries);
                return false;
            }
        }
    }
    
    return false;
}

MoveToPoseServer::~MoveToPoseServer()
{
}

/**
 * @brief 服务回调函数
 */
void MoveToPoseServer::moveToPoseCallback(
    const std::shared_ptr<demo_interface::srv::MoveToPose::Request> req,
    std::shared_ptr<demo_interface::srv::MoveToPose::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "Received move_to_pose request");
    RCLCPP_INFO(this->get_logger(), "Target pose: x=%.3f, y=%.3f, z=%.3f", 
             req->target_pose.position.x, req->target_pose.position.y, req->target_pose.position.z);
    RCLCPP_INFO(this->get_logger(), "Use joints: %s", req->use_joints ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Velocity factor: %.2f, Acceleration factor: %.2f", 
             req->velocity_factor, req->acceleration_factor);

    // 验证输入参数
    if (req->velocity_factor < 0.0 || req->velocity_factor > 1.0)
    {
        res->success = false;
        res->error_code = -1;
        res->message = "Invalid velocity_factor, must be between 0.0 and 1.0";
        RCLCPP_WARN(this->get_logger(), "%s", res->message.c_str());
        return;
    }

    if (req->acceleration_factor < 0.0 || req->acceleration_factor > 1.0)
    {
        res->success = false;
        res->error_code = -2;
        res->message = "Invalid acceleration_factor, must be between 0.0 and 1.0";
        RCLCPP_WARN(this->get_logger(), "%s", res->message.c_str());
        return;
    }

    // 执行运动
    int32_t error_code = 0;
    std::string message;
    bool success = moveToPose(req->target_pose, req->use_joints, 
                              req->velocity_factor, req->acceleration_factor,
                              error_code, message);

    res->success = success;
    res->error_code = error_code;
    res->message = message;

    if (success)
    {
        RCLCPP_INFO(this->get_logger(), "Move to pose succeeded: %s", message.c_str());
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Move to pose failed: %s (error_code: %d)", message.c_str(), error_code);
    }
}

/**
 * @brief 执行移动到目标位姿
 */
bool MoveToPoseServer::moveToPose(const geometry_msgs::msg::Pose& target_pose,
                                  bool use_joints,
                                  float velocity_factor,
                                  float acceleration_factor,
                                  int32_t& error_code,
                                  std::string& message)
{
    if (!move_group_)
    {
        error_code = -100;
        message = "MoveIt interface not initialized";
        return false;
    }

    try
    {
        // 设置速度和加速度缩放因子
        move_group_->setMaxVelocityScalingFactor(velocity_factor);
        move_group_->setMaxAccelerationScalingFactor(acceleration_factor);

        if (use_joints)
        {
            // 关节空间规划：需要先将目标位姿转换为关节角度
            RCLCPP_INFO(this->get_logger(), "Planning in joint space");
            
            // 获取当前关节状态
            moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();
            std::vector<double> joint_group_positions;
            const moveit::core::JointModelGroup* joint_model_group = 
                current_state->getJointModelGroup(planning_group_name_);
            current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

            // 使用 IK 服务计算目标关节角度
            auto ik_client = this->create_client<aubo_msgs::srv::GetIK>("/aubo_driver/get_ik");
            
            if (ik_client->wait_for_service(std::chrono::seconds(1)))
            {
                auto request = std::make_shared<aubo_msgs::srv::GetIK::Request>();
                // 使用当前关节位置作为参考
                for (size_t i = 0; i < 6 && i < joint_group_positions.size(); ++i)
                {
                    request->ref_joint[i] = static_cast<float>(joint_group_positions[i]);
                }
                // 如果关节数少于6，填充剩余位置为0
                for (size_t i = joint_group_positions.size(); i < 6; ++i)
                {
                    request->ref_joint[i] = 0.0f;
                }
                request->pos[0] = target_pose.position.x;
                request->pos[1] = target_pose.position.y;
                request->pos[2] = target_pose.position.z;
                request->ori[0] = target_pose.orientation.w;
                request->ori[1] = target_pose.orientation.x;
                request->ori[2] = target_pose.orientation.y;
                request->ori[3] = target_pose.orientation.z;

                auto result = ik_client->async_send_request(request);
                if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) == 
                    rclcpp::FutureReturnCode::SUCCESS)
                {
                    auto response = result.get();
                    if (response->joint.size() >= 6)
                    {
                        // 使用 IK 计算得到的关节角度
                        std::vector<double> target_joints;
                        for (size_t i = 0; i < 6; ++i)
                        {
                            target_joints.push_back(response->joint[i]);
                        }
                        move_group_->setJointValueTarget(target_joints);
                    }
                    else
                    {
                        error_code = -101;
                        message = "Failed to compute IK for target pose";
                        return false;
                    }
                }
                else
                {
                    error_code = -101;
                    message = "Failed to call IK service";
                    return false;
                }
            }
            else
            {
                // 如果没有 IK 服务，使用 MoveIt 内置的 IK 求解器
                // 设置目标位姿，MoveIt 会自动计算 IK 并转换为关节空间
                move_group_->setPoseTarget(target_pose);
                RCLCPP_INFO(this->get_logger(), "IK service not available, using MoveIt's built-in IK solver");
            }
        }
        else
        {
            // 笛卡尔空间规划：直接设置目标位姿
            RCLCPP_INFO(this->get_logger(), "Planning in Cartesian space");
            move_group_->setPoseTarget(target_pose);
        }

        // 执行规划
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        moveit::planning_interface::MoveItErrorCode plan_result = move_group_->plan(my_plan);

        if (plan_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            error_code = static_cast<int32_t>(plan_result.val);
            message = "Planning failed with error code: " + std::to_string(plan_result.val);
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Planning succeeded, executing trajectory");

        // 执行运动
        moveit::planning_interface::MoveItErrorCode execute_result = move_group_->execute(my_plan);

        if (execute_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            error_code = static_cast<int32_t>(execute_result.val);
            message = "Execution failed with error code: " + std::to_string(execute_result.val);
            return false;
        }

        // 成功
        error_code = 0;
        message = "Successfully moved to target pose";
        return true;
    }
    catch (const std::exception& e)
    {
        error_code = -200;
        message = std::string("Exception occurred: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
        return false;
    }
}

/**
 * @brief 主循环，保持节点运行
 */
void MoveToPoseServer::spin()
{
    rclcpp::spin(this->shared_from_this());
}

} // namespace demo_driver

/**
 * @brief 主函数
 * 初始化 ROS 节点并启动移动到目标位姿服务服务器
 */
int main(int argc, char** argv)
{
    // 初始化 ROS 节点
    rclcpp::init(argc, argv);
    
    // 创建执行器（MoveIt 需要多线程执行器）
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);

    try
    {
        // 创建移动到目标位姿服务服务器对象
        auto server = std::make_shared<demo_driver::MoveToPoseServer>();
        RCLCPP_INFO(server->get_logger(), "Move To Pose Server node created");
        
        // 初始化 MoveIt 接口（在节点创建为 shared_ptr 后）
        if (!server->initialize(10, 2))
        {
            RCLCPP_ERROR(rclcpp::get_logger("move_to_pose_server_node"), 
                        "Failed to initialize MoveIt interfaces");
            rclcpp::shutdown();
            return 1;
        }
        
        executor.add_node(server);
        RCLCPP_INFO(server->get_logger(), "Move To Pose Server node started");
        // 进入主循环
        executor.spin();
    }
    catch (const std::exception& e)
    {
        // 捕获异常并输出错误信息
        RCLCPP_ERROR(rclcpp::get_logger("move_to_pose_server_node"), 
                    "Exception in move_to_pose_server_node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}