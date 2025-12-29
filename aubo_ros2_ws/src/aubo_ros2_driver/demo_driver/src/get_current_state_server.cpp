/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#include "demo_driver/get_current_state_server.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <aubo_msgs/srv/get_fk.hpp>
#include <Eigen/Geometry>
#include <cmath>
#include <thread>
#include <chrono>
#include <rclcpp/parameter_client.hpp>
#include <cstdlib>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace demo_driver
{



/**
 * @brief 构造函数，初始化 MoveIt 接口和服务服务器
 */
GetCurrentStateServer::GetCurrentStateServer(const rclcpp::NodeOptions& options)
    : Node("get_current_state_server_node", options)
    , joint_states_received_(false)
    , planning_group_name_("manipulator")
    , base_frame_("base_link")
    , end_effector_link_("")
{
    // 获取参数
    this->declare_parameter("planning_group_name", std::string("manipulator"));
    this->declare_parameter("base_frame", std::string("base_link"));
    
    this->get_parameter("planning_group_name", planning_group_name_);
    this->get_parameter("base_frame", base_frame_);

    // 初始化订阅器
    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        std::bind(&GetCurrentStateServer::jointStatesCallback, this, std::placeholders::_1));

    // 初始化服务客户端
    fk_client_ = this->create_client<aubo_msgs::srv::GetFK>("/aubo_driver/get_fk");

    // 初始化服务服务器
    get_current_state_service_ = this->create_service<demo_interface::srv::GetCurrentState>(
        "/get_current_state",
        std::bind(&GetCurrentStateServer::getCurrentStateCallback, this, std::placeholders::_1, std::placeholders::_2));
}

bool GetCurrentStateServer::wait_for_robot_description(int timeout_seconds)
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
            RCLCPP_ERROR(this->get_logger(), "Please ensure:");
            RCLCPP_ERROR(this->get_logger(), "  1. MoveIt2 is running: ros2 launch aubo_moveit_config aubo_moveit_bridge_ros1.launch.py");
            RCLCPP_ERROR(this->get_logger(), "  2. Planning group name is correct: currently using '%s'", planning_group_name_.c_str());
            RCLCPP_ERROR(this->get_logger(), "  3. move_group node is running");
            return false;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    return false;
}

bool GetCurrentStateServer::initialize(int max_retries, int retry_delay_seconds)
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
            end_effector_link_ = move_group_->getEndEffectorLink();
            
            static bool fk_warned = false;
            if (!fk_client_->wait_for_service(std::chrono::seconds(5)))
            {
                if (!fk_warned) {
                    RCLCPP_WARN(this->get_logger(), "FK service not available, will use MoveIt for cartesian position");
                    fk_warned = true;
                }
            }
            
            static bool success_logged = false;
            if (!success_logged) {
                RCLCPP_INFO(this->get_logger(), "GetCurrentStateServer initialized, end effector: %s, service '/get_current_state' ready", 
                           end_effector_link_.c_str());
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
                RCLCPP_ERROR(this->get_logger(), "Please ensure MoveIt2 is running: ros2 launch aubo_moveit_config aubo_moveit_bridge_ros1.launch.py");
                return false;
            }
        }
    }
    
    return false;
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
    RCLCPP_INFO(this->get_logger(), "GetCurrentState request");
    
    std::string message;
    bool success = getCurrentState(res->joint_position_rad,
                                   res->cartesian_position,
                                   res->velocity,
                                   message);

    res->success = success;
    res->message = message;

    if (success)
    {
        RCLCPP_INFO(this->get_logger(), "GetCurrentState response: success, cartesian=(%.3f, %.3f, %.3f)",
                   res->cartesian_position.position.x, res->cartesian_position.position.y, 
                   res->cartesian_position.position.z);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "GetCurrentState response: failed (message: %s)", message.c_str());
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
            
            // 如果 getCurrentState() 返回 null（通常是因为没有 joint_states），使用机器人模型创建默认状态
            moveit::core::RobotStatePtr robot_state;
            if (!current_state)
            {
                RCLCPP_WARN(this->get_logger(), "getCurrentState() returned null (likely no joint_states available), creating default state from robot model");
                
                // 从机器人模型创建默认状态
                const moveit::core::RobotModelConstPtr& robot_model = move_group_->getRobotModel();
                if (!robot_model)
                {
                    message = "Failed to get robot model from MoveIt";
                    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
                    return false;
                }
                
                robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
                robot_state->setToDefaultValues();  // 设置为默认值（通常是零位）
            }
            else
            {
                robot_state = current_state;
            }
            
            const moveit::core::JointModelGroup* joint_model_group =
                robot_state->getJointModelGroup(planning_group_name_);
            
            if (!joint_model_group)
            {
                message = "Failed to get joint model group from MoveIt: joint_model_group is null";
                RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
                return false;
            }

            std::vector<double> joint_values;
            robot_state->copyJointGroupPositions(joint_model_group, joint_values);

            if (joint_values.size() >= 6)
            {
                joint_position_rad.clear();
                for (size_t i = 0; i < 6 && i < joint_values.size(); ++i)
                {
                    joint_position_rad.push_back(joint_values[i]);
                }

                // 获取当前位姿
                if (end_effector_link_.empty())
                {
                    message = "End effector link is empty, cannot get current pose";
                    RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
                    return false;
                }
                
                // 获取当前位姿
                // 注意：getCurrentPose 返回的位姿在规划框架中（可能是 world）
                // 为了确保返回 base_link 坐标系下的位姿，我们使用 robot_state 计算
                // robot_state->getGlobalLinkTransform 返回的位姿在机器人模型的根坐标系中（base_link）
                robot_state->update();
                const Eigen::Isometry3d& end_effector_transform = robot_state->getGlobalLinkTransform(end_effector_link_);
                
                Eigen::Vector3d position = end_effector_transform.translation();
                Eigen::Quaterniond orientation(end_effector_transform.rotation());
                
                cartesian_position.position.x = position.x();
                cartesian_position.position.y = position.y();
                cartesian_position.position.z = position.z();
                cartesian_position.orientation.w = orientation.w();
                cartesian_position.orientation.x = orientation.x();
                cartesian_position.orientation.y = orientation.y();
                cartesian_position.orientation.z = orientation.z();

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
            
            if (!current_state)
            {
                RCLCPP_DEBUG(this->get_logger(), "Failed to get current state from MoveIt: current_state is null");
                return false;
            }
            
            const moveit::core::JointModelGroup* joint_model_group =
                current_state->getJointModelGroup(planning_group_name_);
            
            if (!joint_model_group)
            {
                RCLCPP_DEBUG(this->get_logger(), "Failed to get joint model group from MoveIt: joint_model_group is null");
                return false;
            }
            
            if (end_effector_link_.empty())
            {
                RCLCPP_DEBUG(this->get_logger(), "End effector link is empty, cannot compute FK");
                return false;
            }

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

/**
 * @brief 主函数
 * 初始化 ROS 节点并启动获取当前状态服务服务器
 */
int main(int argc, char** argv)
{
    // 初始化 ROS 节点
    rclcpp::init(argc, argv);
    
    // 创建执行器（MoveIt 需要多线程执行器）
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);

    try
    {
        // 创建获取当前状态服务服务器对象，使用 automatically_declare_parameters_from_overrides 自动声明参数
        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);
        auto server = std::make_shared<demo_driver::GetCurrentStateServer>(node_options);
        
        if (!server->initialize(10, 2))
        {
            RCLCPP_ERROR(rclcpp::get_logger("get_current_state_server_node"), 
                        "Failed to initialize MoveIt interfaces after retries");
            rclcpp::shutdown();
            return 1;
        }
        
        executor.add_node(server);
        executor.spin();
    }
    catch (const std::exception& e)
    {
        // 捕获异常并输出错误信息
        RCLCPP_ERROR(rclcpp::get_logger("get_current_state_server_node"), 
                    "Exception in get_current_state_server_node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}