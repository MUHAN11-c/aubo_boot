/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#include "demo_driver/execute_trajectory_server.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <thread>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <cstdlib>
#include <string>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/exceptions.hpp>
#include <fstream>

namespace demo_driver
{



/**
 * @brief 构造函数，初始化 MoveIt 接口和服务服务器
 */
ExecuteTrajectoryServer::ExecuteTrajectoryServer(const rclcpp::NodeOptions& options)
    : Node("execute_trajectory_server_node", options)
    , planning_group_name_("manipulator")
    , base_frame_("base_link")
{
    // 获取参数
    this->declare_parameter("planning_group_name", std::string("manipulator"));
    this->declare_parameter("base_frame", std::string("base_link"));
    
    this->get_parameter("planning_group_name", planning_group_name_);
    this->get_parameter("base_frame", base_frame_);

    execute_trajectory_service_ = this->create_service<demo_interface::srv::ExecuteTrajectory>(
        "/execute_trajectory",
        std::bind(&ExecuteTrajectoryServer::executeTrajectoryCallback, this, std::placeholders::_1, std::placeholders::_2));
}

bool ExecuteTrajectoryServer::wait_for_robot_description(int timeout_seconds)
{
    static bool warned_once = false;
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

bool ExecuteTrajectoryServer::initialize(int max_retries, int retry_delay_seconds)
{
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
            return true;
        }
        catch (const std::exception& e)
        {
            if (attempt < max_retries)
            {
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

ExecuteTrajectoryServer::~ExecuteTrajectoryServer()
{
}

void ExecuteTrajectoryServer::executeTrajectoryCallback(
    const std::shared_ptr<demo_interface::srv::ExecuteTrajectory::Request> req,
    std::shared_ptr<demo_interface::srv::ExecuteTrajectory::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "ExecuteTrajectory request: %zu waypoints", req->trajectory.points.size());
    
    if (req->trajectory.points.empty())
    {
        res->success = false;
        res->error_code = -1;
        res->message = "Trajectory is empty";
        RCLCPP_WARN(this->get_logger(), "ExecuteTrajectory response: %s", res->message.c_str());
        return;
    }

    if (req->trajectory.joint_names.empty())
    {
        res->success = false;
        res->error_code = -2;
        res->message = "Trajectory joint names are empty";
        RCLCPP_WARN(this->get_logger(), "ExecuteTrajectory response: %s", res->message.c_str());
        return;
    }

    int32_t error_code = 0;
    std::string message;
    bool success = executeTrajectory(req->trajectory, error_code, message);

    res->success = success;
    res->error_code = error_code;
    res->message = message;
    
    if (success)
    {
        RCLCPP_INFO(this->get_logger(), "ExecuteTrajectory response: success");
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "ExecuteTrajectory response: failed (error_code: %d, message: %s)", 
                   error_code, message.c_str());
    }
}

bool ExecuteTrajectoryServer::executeTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory,
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
        moveit_msgs::msg::RobotTrajectory robot_trajectory_msg;
        robot_trajectory_msg.joint_trajectory = trajectory;

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = robot_trajectory_msg;

        if (plan.trajectory_.joint_trajectory.points.empty())
        {
            error_code = -101;
            message = "Converted trajectory is empty";
            return false;
        }

        double trajectory_duration = 0.0;
        if (!plan.trajectory_.joint_trajectory.points.empty())
        {
            auto& last_point = plan.trajectory_.joint_trajectory.points.back();
            trajectory_duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9;
        }
        
        auto start_time = std::chrono::steady_clock::now();
        
        double velocity_scaling = 0.1;
        double acceleration_scaling = 0.1;
        
        if (this->has_parameter("moveit_velocity_scaling_factor"))
        {
            velocity_scaling = this->get_parameter("moveit_velocity_scaling_factor").as_double();
            move_group_->setMaxVelocityScalingFactor(velocity_scaling);
        }
        else
        {
            velocity_scaling = 0.1;
        }
        
        try
        {
            auto param_client = std::make_shared<rclcpp::SyncParametersClient>(shared_from_this(), "move_group");
            if (param_client->wait_for_service(std::chrono::milliseconds(200)))
            {
                try
                {
                    auto params = param_client->get_parameters({"robot_description_planning.default_acceleration_scaling_factor"});
                    if (!params.empty())
                    {
                        acceleration_scaling = params[0].as_double();
                    }
                }
                catch (...)
                {
                }
            }
        }
        catch (...)
        {
        }
        
        RCLCPP_INFO(this->get_logger(), "ExecuteTrajectory: %zu waypoints, %.3f s, vel=%.2f, acc=%.2f",
                   plan.trajectory_.joint_trajectory.points.size(), trajectory_duration,
                   velocity_scaling, acceleration_scaling);
        
        moveit::planning_interface::MoveItErrorCode execute_result = move_group_->execute(plan);
        
        auto end_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() / 1000.0;
        RCLCPP_INFO(this->get_logger(), "ExecuteTrajectory: MoveIt execute() returned after %.3f s (error_code: %d)", 
                   elapsed, execute_result.val);

        if (execute_result.val == 0)
        {
            error_code = -1000;
            message = "Execution returned unknown error code 0. This may indicate MoveIt timed out, but the robot may still be executing the trajectory. Check robot position to confirm.";
            RCLCPP_WARN(this->get_logger(), "ExecuteTrajectory: MoveIt reported timeout (error_code=0), but robot may have reached target. Elapsed time: %.3f s, trajectory duration: %.3f s", 
                      elapsed, trajectory_duration);
            return false;
        }
        else if (execute_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            error_code = static_cast<int32_t>(execute_result.val);
            message = "Execution failed with error code: " + std::to_string(execute_result.val);
            RCLCPP_WARN(this->get_logger(), "ExecuteTrajectory: MoveIt reported failure (error_code=%d), but robot may have reached target. Elapsed time: %.3f s, trajectory duration: %.3f s", 
                      execute_result.val, elapsed, trajectory_duration);
            return false;
        }

        error_code = 0;
        message = "Successfully executed trajectory";
        RCLCPP_INFO(this->get_logger(), "ExecuteTrajectory: Execution completed, duration=%.3f s", trajectory_duration);
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


void ExecuteTrajectoryServer::spin()
{
    rclcpp::spin(this->shared_from_this());
}

} // namespace demo_driver

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);

    try
    {
        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);
        auto server = std::make_shared<demo_driver::ExecuteTrajectoryServer>(node_options);
        
        if (!server->initialize(10, 2))
        {
            RCLCPP_ERROR(rclcpp::get_logger("execute_trajectory_server_node"), 
                        "Failed to initialize MoveIt interfaces");
            rclcpp::shutdown();
            return 1;
        }
        
        executor.add_node(server);
        executor.spin();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("execute_trajectory_server_node"), 
                    "Exception in execute_trajectory_server_node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}