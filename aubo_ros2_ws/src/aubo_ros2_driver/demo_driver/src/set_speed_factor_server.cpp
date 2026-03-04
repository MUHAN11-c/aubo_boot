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
#include <cstdlib>
#include <string>
#include <thread>
#include <chrono>

namespace demo_driver
{



/**
 * @brief 构造函数，初始化 MoveIt 接口和服务服务器
 */
SetSpeedFactorServer::SetSpeedFactorServer(const rclcpp::NodeOptions& options)
    : Node("set_speed_factor_server_node", options)
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
}

bool SetSpeedFactorServer::wait_for_robot_description(int timeout_seconds)
{
    static bool info_logged = false;
    static bool warned_once = false;
    
    if (!info_logged) {
        RCLCPP_INFO(this->get_logger(), "Waiting for robot_description parameter and MoveIt2 services...");
        info_logged = true;
    }
    
    if (!this->has_parameter("robot_description") || !this->has_parameter("robot_description_semantic")) {
        if (!warned_once) {
            RCLCPP_WARN(this->get_logger(), "robot_description or robot_description_semantic parameter not found");
            warned_once = true;
        }
    }
    
    auto start_time = std::chrono::steady_clock::now();
    int check_count = 0;
    
    while (rclcpp::ok())
    {
        try
        {
            auto test_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(), planning_group_name_);
            
            std::string test_frame = test_move_group->getPlanningFrame();
            
            if (!test_frame.empty())
            {
                static bool ready_logged = false;
                if (!ready_logged) {
                    RCLCPP_INFO(this->get_logger(), "Robot description ready, planning frame: %s", test_frame.c_str());
                    ready_logged = true;
                }
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
    static bool init_logged = false;
    if (!init_logged) {
        RCLCPP_INFO(this->get_logger(), "Initializing MoveIt interfaces for planning group: %s", planning_group_name_.c_str());
        init_logged = true;
    }
    
    if (!wait_for_robot_description(30))
    {
        return false;
    }
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    for (int attempt = 1; attempt <= max_retries; ++attempt)
    {
        try
        {
            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(), planning_group_name_);
            planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
            move_group_->setPoseReferenceFrame(base_frame_);
            
            static bool success_logged = false;
            if (!success_logged) {
                RCLCPP_INFO(this->get_logger(), "SetSpeedFactorServer initialized, service '/set_speed_factor' ready");
                success_logged = true;
            }
            return true;
        }
        catch (const std::exception& e)
        {
            if (attempt < max_retries)
            {
                static int last_attempt_logged = 0;
                if (attempt != last_attempt_logged) {
                    RCLCPP_WARN(this->get_logger(), "Initialization attempt %d/%d failed: %s, retrying...", 
                               attempt, max_retries, e.what());
                    last_attempt_logged = attempt;
                }
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
    static bool first_request = true;
    if (first_request) {
        RCLCPP_INFO(this->get_logger(), "SetSpeedFactor service ready");
        first_request = false;
    }
    
    RCLCPP_INFO(this->get_logger(), "SetSpeedFactor request: velocity_factor=%.2f", req->velocity_factor);

    if (req->velocity_factor < 0.0 || req->velocity_factor > 1.0)
    {
        res->success = false;
        res->message = "Invalid velocity_factor, must be between 0.0 and 1.0";
        RCLCPP_WARN(this->get_logger(), "SetSpeedFactor response: %s", res->message.c_str());
        return;
    }

    std::string message;
    bool success = setSpeedFactor(req->velocity_factor, message);

    res->success = success;
    res->message = message;

    if (success)
    {
        RCLCPP_INFO(this->get_logger(), "SetSpeedFactor response: success (velocity_factor=%.2f)", req->velocity_factor);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "SetSpeedFactor response: failed (message: %s)", message.c_str());
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
        move_group_->setMaxVelocityScalingFactor(velocity_factor);
        this->set_parameter(rclcpp::Parameter("moveit_velocity_scaling_factor", velocity_factor));
        
        std::vector<rclcpp::Parameter> params_to_set = {
            rclcpp::Parameter("moveit_velocity_scaling_factor", velocity_factor)
        };
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        auto param_client = std::make_shared<rclcpp::SyncParametersClient>(shared_from_this(), "plan_trajectory_server");
        if (param_client->wait_for_service(std::chrono::milliseconds(1000)))
        {
            try
            {
                param_client->set_parameters(params_to_set);
            }
            catch (...)
            {
            }
        }
        
        param_client = std::make_shared<rclcpp::SyncParametersClient>(shared_from_this(), "execute_trajectory_server");
        if (param_client->wait_for_service(std::chrono::milliseconds(1000)))
        {
            try
            {
                param_client->set_parameters(params_to_set);
            }
            catch (...)
            {
            }
        }
        
        message = "Successfully set velocity factor to " + std::to_string(velocity_factor);
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
        // 创建设置速度因子服务服务器对象，使用 automatically_declare_parameters_from_overrides 自动声明参数
        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);
        auto server = std::make_shared<demo_driver::SetSpeedFactorServer>(node_options);
        
        if (!server->initialize(10, 2))
        {
            RCLCPP_ERROR(rclcpp::get_logger("set_speed_factor_server_node"), 
                        "Failed to initialize MoveIt interfaces");
            rclcpp::shutdown();
            return 1;
        }
        
        executor.add_node(server);
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