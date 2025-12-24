/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#include "demo_driver/set_speed_factor_server.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <cmath>

namespace demo_driver
{

/**
 * @brief 构造函数，初始化 MoveIt 接口和服务服务器
 */
SetSpeedFactorServer::SetSpeedFactorServer()
    : Node("set_speed_factor_server_node")
    , planning_group_name_("manipulator")
    , base_frame_("base_link")
{
    // 获取参数
    this->declare_parameter("planning_group_name", std::string("manipulator"));
    this->declare_parameter("base_frame", std::string("base_link"));
    
    this->get_parameter("planning_group_name", planning_group_name_);
    this->get_parameter("base_frame", base_frame_);

    // 初始化服务服务器
    set_speed_factor_service_ = this->create_service<demo_interface::srv::SetSpeedFactor>(
        "/set_speed_factor",
        std::bind(&SetSpeedFactorServer::setSpeedFactorCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "SetSpeedFactorServer node created, ready to initialize MoveIt");
}

bool SetSpeedFactorServer::wait_for_robot_description(int timeout_seconds)
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

bool SetSpeedFactorServer::initialize(int max_retries, int retry_delay_seconds)
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
            
            RCLCPP_INFO(this->get_logger(), "MoveIt interfaces initialized successfully");
            RCLCPP_INFO(this->get_logger(), "SetSpeedFactorServer initialized, service '/set_speed_factor' is ready");
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

SetSpeedFactorServer::~SetSpeedFactorServer()
{
}

/**
 * @brief 服务回调函数
 */
void SetSpeedFactorServer::setSpeedFactorCallback(
    const std::shared_ptr<demo_interface::srv::SetSpeedFactor::Request> req,
    std::shared_ptr<demo_interface::srv::SetSpeedFactor::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "Received set_speed_factor request: velocity_factor = %.2f", req->velocity_factor);

    // 验证输入参数
    if (req->velocity_factor < 0.0 || req->velocity_factor > 1.0)
    {
        res->success = false;
        res->message = "Invalid velocity_factor, must be between 0.0 and 1.0";
        RCLCPP_WARN(this->get_logger(), "%s", res->message.c_str());
        return;
    }

    // 设置速度因子
    std::string message;
    bool success = setSpeedFactor(req->velocity_factor, message);

    res->success = success;
    res->message = message;

    if (success)
    {
        RCLCPP_INFO(this->get_logger(), "Set speed factor succeeded: %s", message.c_str());
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Set speed factor failed: %s", message.c_str());
    }
}

/**
 * @brief 设置速度因子
 */
bool SetSpeedFactorServer::setSpeedFactor(float velocity_factor, std::string& message)
{
    if (!move_group_)
    {
        message = "MoveIt interface not initialized";
        return false;
    }

    try
    {
        // 设置速度缩放因子
        move_group_->setMaxVelocityScalingFactor(velocity_factor);
        
        message = "Successfully set velocity factor to " + std::to_string(velocity_factor);
        RCLCPP_INFO(this->get_logger(), "Velocity factor set to %.2f", velocity_factor);
        return true;
    }
    catch (const std::exception& e)
    {
        message = std::string("Exception occurred: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
        return false;
    }
}

/**
 * @brief 主循环，保持节点运行
 */
void SetSpeedFactorServer::spin()
{
    rclcpp::spin(this->shared_from_this());
}

} // namespace demo_driver

/**
 * @brief 主函数
 * 初始化 ROS 节点并启动设置速度因子服务服务器
 */
int main(int argc, char** argv)
{
    // 初始化 ROS 节点
    rclcpp::init(argc, argv);
    
    // 创建执行器（MoveIt 需要多线程执行器）
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);

    try
    {
        // 创建设置速度因子服务服务器对象
        auto server = std::make_shared<demo_driver::SetSpeedFactorServer>();
        RCLCPP_INFO(server->get_logger(), "Set Speed Factor Server node created");
        
        // 初始化 MoveIt 接口（在节点创建为 shared_ptr 后）
        if (!server->initialize(10, 2))
        {
            RCLCPP_ERROR(rclcpp::get_logger("set_speed_factor_server_node"), 
                        "Failed to initialize MoveIt interfaces");
            rclcpp::shutdown();
            return 1;
        }
        
        executor.add_node(server);
        RCLCPP_INFO(server->get_logger(), "Set Speed Factor Server node started");
        // 进入主循环
        executor.spin();
    }
    catch (const std::exception& e)
    {
        // 捕获异常并输出错误信息
        RCLCPP_ERROR(rclcpp::get_logger("set_speed_factor_server_node"), 
                    "Exception in set_speed_factor_server_node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}