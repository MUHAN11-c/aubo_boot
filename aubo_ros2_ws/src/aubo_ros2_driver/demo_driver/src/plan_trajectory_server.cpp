/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#include "demo_driver/plan_trajectory_server.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <aubo_msgs/srv/get_ik.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <chrono>

namespace demo_driver
{

/**
 * @brief 构造函数，初始化 MoveIt 接口和服务服务器
 */
PlanTrajectoryServer::PlanTrajectoryServer()
    : Node("plan_trajectory_server_node")
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
    plan_trajectory_service_ = this->create_service<demo_interface::srv::PlanTrajectory>(
        "/plan_trajectory",
        std::bind(&PlanTrajectoryServer::planTrajectoryCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "PlanTrajectoryServer node created, ready to initialize MoveIt");
}

bool PlanTrajectoryServer::wait_for_robot_description(int timeout_seconds)
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

bool PlanTrajectoryServer::initialize(int max_retries, int retry_delay_seconds)
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
            RCLCPP_INFO(this->get_logger(), "PlanTrajectoryServer initialized, service '/plan_trajectory' is ready");
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

PlanTrajectoryServer::~PlanTrajectoryServer()
{
}

/**
 * @brief 服务回调函数
 */
void PlanTrajectoryServer::planTrajectoryCallback(
    const std::shared_ptr<demo_interface::srv::PlanTrajectory::Request> req,
    std::shared_ptr<demo_interface::srv::PlanTrajectory::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "Received plan_trajectory request");
    RCLCPP_INFO(this->get_logger(), "Target pose: x=%.3f, y=%.3f, z=%.3f", 
             req->target_pose.position.x, req->target_pose.position.y, req->target_pose.position.z);
    RCLCPP_INFO(this->get_logger(), "Use joints: %s", req->use_joints ? "true" : "false");

    // 执行规划
    float planning_time = 0.0;
    std::string message;
    bool success = planTrajectory(req->target_pose, req->use_joints,
                                   res->trajectory, planning_time, message);

    res->success = success;
    res->planning_time = planning_time;
    res->message = message;

    if (success)
    {
        RCLCPP_INFO(this->get_logger(), "Trajectory planning succeeded: %s (time: %.3f s)", message.c_str(), planning_time);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Trajectory planning failed: %s", message.c_str());
    }
}

/**
 * @brief 执行轨迹规划
 */
bool PlanTrajectoryServer::planTrajectory(const geometry_msgs::msg::Pose& target_pose,
                                         bool use_joints,
                                         trajectory_msgs::msg::JointTrajectory& trajectory,
                                         float& planning_time,
                                         std::string& message)
{
    if (!move_group_)
    {
        message = "MoveIt interface not initialized";
        return false;
    }

    try
    {
        // 记录规划开始时间
        auto start_time = std::chrono::high_resolution_clock::now();

        if (use_joints)
        {
            // 关节空间规划：需要先将目标位姿转换为关节角度
            RCLCPP_INFO(this->get_logger(), "Planning in joint space");
            
            // 获取当前关节状态
            moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();
            
            // 如果 getCurrentState() 返回 null，使用机器人模型创建默认状态
            moveit::core::RobotStatePtr robot_state;
            if (!current_state)
            {
                RCLCPP_WARN(this->get_logger(), "getCurrentState() returned null (likely no joint_states available), creating default state from robot model");
                const moveit::core::RobotModelConstPtr& robot_model = move_group_->getRobotModel();
                if (!robot_model)
                {
                    message = "Failed to get robot model from MoveIt";
                    return false;
                }
                robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
                robot_state->setToDefaultValues();
            }
            else
            {
                robot_state = current_state;
            }
            
            std::vector<double> joint_group_positions;
            const moveit::core::JointModelGroup* joint_model_group = 
                robot_state->getJointModelGroup(planning_group_name_);
            
            if (!joint_model_group)
            {
                message = "Failed to get joint model group from MoveIt";
                return false;
            }
            
            robot_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

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
                        message = "Failed to compute IK for target pose";
                        return false;
                    }
                }
                else
                {
                    message = "Failed to call IK service";
                    return false;
                }
            }
            else
            {
                // 如果没有 IK 服务，使用 MoveIt 内置的 IK 求解器
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

        // 计算规划时间
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        planning_time = duration.count() / 1000.0f;

        if (plan_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            message = "Planning failed with error code: " + std::to_string(plan_result.val);
            return false;
        }

        // 将 MoveIt 轨迹转换为 trajectory_msgs::JointTrajectory
        if (my_plan.trajectory_.joint_trajectory.joint_names.empty())
        {
            message = "Planned trajectory is empty";
            return false;
        }

        // 复制轨迹信息
        trajectory = my_plan.trajectory_.joint_trajectory;

        // 成功
        message = "Successfully planned trajectory with " + 
                  std::to_string(trajectory.points.size()) + " waypoints";
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
void PlanTrajectoryServer::spin()
{
    rclcpp::spin(this->shared_from_this());
}

} // namespace demo_driver

/**
 * @brief 主函数
 * 初始化 ROS 节点并启动规划轨迹服务服务器
 */
int main(int argc, char** argv)
{
    // 初始化 ROS 节点
    rclcpp::init(argc, argv);
    
    // 创建执行器（MoveIt 需要多线程执行器）
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);

    try
    {
        // 创建规划轨迹服务服务器对象
        auto server = std::make_shared<demo_driver::PlanTrajectoryServer>();
        RCLCPP_INFO(server->get_logger(), "Plan Trajectory Server node created");
        
        // 初始化 MoveIt 接口（在节点创建为 shared_ptr 后）
        if (!server->initialize(10, 2))
        {
            RCLCPP_ERROR(rclcpp::get_logger("plan_trajectory_server_node"), 
                        "Failed to initialize MoveIt interfaces");
            rclcpp::shutdown();
            return 1;
        }
        
        executor.add_node(server);
        RCLCPP_INFO(server->get_logger(), "Plan Trajectory Server node started");
        // 进入主循环
        executor.spin();
    }
    catch (const std::exception& e)
    {
        // 捕获异常并输出错误信息
        RCLCPP_ERROR(rclcpp::get_logger("plan_trajectory_server_node"), 
                    "Exception in plan_trajectory_server_node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}