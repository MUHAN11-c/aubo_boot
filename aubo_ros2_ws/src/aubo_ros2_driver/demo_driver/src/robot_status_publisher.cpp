#include "demo_driver/robot_status_publisher.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <thread>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace demo_driver
{

RobotStatusPublisher::RobotStatusPublisher()
    : Node("robot_status_publisher_node")
    , joint_states_received_(false)
    , industrial_status_received_(false)
    , planning_status_("idle")
    , is_online_from_industrial_(false)
    , enable_from_industrial_(false)
    , in_motion_from_industrial_(false)
    , end_effector_link_("")
    , moveit_initialized_(false)
    , publish_rate_(10.0)
    , base_frame_("base_link")
    , planning_group_name_("manipulator_e5")
    , robot_status_topic_("/demo_robot_status_ros2")
    , subscribe_industrial_status_(true)
{
    this->declare_parameter("publish_rate", 10.0);
    this->declare_parameter("base_frame", std::string("base_link"));
    this->declare_parameter("planning_group_name", std::string("manipulator_e5"));
    this->declare_parameter("robot_status_topic", std::string("/demo_robot_status_ros2"));
    this->declare_parameter("subscribe_industrial_status", true);
    
    this->get_parameter("publish_rate", publish_rate_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("planning_group_name", planning_group_name_);
    this->get_parameter("robot_status_topic", robot_status_topic_);
    this->get_parameter("subscribe_industrial_status", subscribe_industrial_status_);

    robot_status_pub_ = this->create_publisher<demo_interface::msg::RobotStatus>(robot_status_topic_, 10);

    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        std::bind(&RobotStatusPublisher::jointStatesCallback, this, std::placeholders::_1));
    
    trajectory_execution_sub_ = this->create_subscription<std_msgs::msg::String>(
        "trajectory_execution_event", 10,
        std::bind(&RobotStatusPublisher::trajectoryExecutionCallback, this, std::placeholders::_1));
    
    if (subscribe_industrial_status_)
    {
        industrial_robot_status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "robot_status", 10,
            std::bind(&RobotStatusPublisher::industrialRobotStatusCallback, this, std::placeholders::_1));
    }

    status_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
        std::bind(&RobotStatusPublisher::statusTimerCallback, this));
}

RobotStatusPublisher::~RobotStatusPublisher()
{
}

bool RobotStatusPublisher::wait_for_robot_description(int timeout_seconds)
{
    auto start_time = std::chrono::steady_clock::now();
    
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
        }
        
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() >= timeout_seconds)
        {
            return false;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    return false;
}

bool RobotStatusPublisher::initialize(int max_retries, int retry_delay_seconds)
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
            
            end_effector_link_ = move_group_->getEndEffectorLink();
            moveit_initialized_ = true;
            return true;
        }
        catch (const std::exception& e)
        {
            if (attempt < max_retries)
            {
                std::this_thread::sleep_for(std::chrono::seconds(retry_delay_seconds));
            }
        }
    }
    
    return false;
}

void RobotStatusPublisher::jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    current_joint_states_ = *msg;
    if (!joint_states_received_)
    {
        joint_states_received_ = true;
    }
}

void RobotStatusPublisher::trajectoryExecutionCallback(const std_msgs::msg::String::SharedPtr msg)
{
    std::string event = msg->data;
    if (event == "stop" || event == "cancel")
    {
        planning_status_ = "idle";
    }
    else if (event == "planning")
    {
        planning_status_ = "planning";
    }
    else if (event == "executing" || event == "execute")
    {
        planning_status_ = "executing";
    }
}

void RobotStatusPublisher::industrialRobotStatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
    industrial_status_received_ = true;
    std::string status_str = msg->data;
    
    if (planning_status_ != "planning" && planning_status_ != "executing")
    {
        if (status_str.find("in_motion") != std::string::npos || 
            status_str.find("executing") != std::string::npos)
        {
            planning_status_ = "executing";
            in_motion_from_industrial_ = true;
        }
        else if (status_str.find("error") != std::string::npos || 
                 status_str.find("in_error") != std::string::npos)
        {
            planning_status_ = "error";
        }
        else
        {
            planning_status_ = "idle";
            in_motion_from_industrial_ = false;
        }
    }
    
    if (status_str.find("drives_powered") != std::string::npos || 
        status_str.find("online") != std::string::npos)
    {
        is_online_from_industrial_ = true;
    }
    else
    {
        is_online_from_industrial_ = false;
    }
    
    if (status_str.find("motion_possible") != std::string::npos || 
        status_str.find("enable") != std::string::npos)
    {
        enable_from_industrial_ = true;
    }
    else
    {
        enable_from_industrial_ = false;
    }
}

std::string RobotStatusPublisher::getPlanningStatus()
{
    return planning_status_;
}

bool RobotStatusPublisher::getForwardKinematics(const std::vector<double>& joint_positions,
                                                 geometry_msgs::msg::Pose& cartesian_pose)
{
    if (joint_positions.size() < 6)
    {
        return false;
    }

    if (moveit_initialized_ && move_group_)
    {
        try
        {
            const moveit::core::RobotModelConstPtr& robot_model = move_group_->getRobotModel();
            if (!robot_model)
            {
                return false;
            }
            
            moveit::core::RobotStatePtr robot_state = 
                std::make_shared<moveit::core::RobotState>(robot_model);
            
            const moveit::core::JointModelGroup* joint_model_group =
                robot_state->getJointModelGroup(planning_group_name_);
            
            if (joint_model_group && !end_effector_link_.empty())
            {
                std::vector<double> joint_values;
                for (size_t i = 0; i < 6 && i < joint_positions.size(); ++i)
                {
                    joint_values.push_back(joint_positions[i]);
                }
                for (size_t i = joint_positions.size(); i < 6; ++i)
                {
                    joint_values.push_back(0.0);
                }
                
                robot_state->setJointGroupPositions(joint_model_group, joint_values);
                robot_state->update();
                
                const Eigen::Isometry3d& end_effector_transform =
                    robot_state->getGlobalLinkTransform(end_effector_link_);
                
                Eigen::Vector3d position = end_effector_transform.translation();
                Eigen::Quaterniond orientation(end_effector_transform.rotation());
                
                cartesian_pose.position.x = position.x();
                cartesian_pose.position.y = position.y();
                cartesian_pose.position.z = position.z();
                cartesian_pose.orientation.w = orientation.w();
                cartesian_pose.orientation.x = orientation.x();
                cartesian_pose.orientation.y = orientation.y();
                cartesian_pose.orientation.z = orientation.z();
                
                return true;
            }
        }
        catch (const std::exception& e)
        {
        }
    }

    return false;
}

void RobotStatusPublisher::publishRobotStatus()
{
    if (!joint_states_received_)
    {
        return;
    }

    if (!moveit_initialized_)
    {
        static bool init_attempted = false;
        if (!init_attempted)
        {
            init_attempted = true;
            std::thread init_thread([this]() {
                this->initialize(3, 2);
            });
            init_thread.detach();
        }
    }

    demo_interface::msg::RobotStatus status_msg;

    status_msg.header.stamp = this->now();
    status_msg.header.frame_id = base_frame_;

    std::vector<double> joint_positions_rad;
    std::vector<double> joint_positions_deg;

    std::vector<std::string> joint_names = {
        "shoulder_joint", "upperArm_joint", "foreArm_joint",
        "wrist1_joint", "wrist2_joint", "wrist3_joint"
    };

    joint_positions_rad.resize(6, 0.0);
    joint_positions_deg.resize(6, 0.0);

    for (size_t i = 0; i < joint_names.size() && i < 6; ++i)
    {
        for (size_t j = 0; j < current_joint_states_.name.size(); ++j)
        {
            if (current_joint_states_.name[j] == joint_names[i])
            {
                if (j < current_joint_states_.position.size())
                {
                    joint_positions_rad[i] = current_joint_states_.position[j];
                    joint_positions_deg[i] = joint_positions_rad[i] * 180.0 / M_PI;
                }
                break;
            }
        }
    }

    if (joint_positions_rad[0] == 0.0 && joint_positions_rad[1] == 0.0 && 
        current_joint_states_.position.size() >= 6)
    {
        for (size_t i = 0; i < 6; ++i)
        {
            joint_positions_rad[i] = current_joint_states_.position[i];
            joint_positions_deg[i] = joint_positions_rad[i] * 180.0 / M_PI;
        }
    }

    for (size_t i = 0; i < 6 && i < joint_positions_rad.size(); ++i)
    {
        status_msg.joint_position_rad[i] = joint_positions_rad[i];
    }
    for (size_t i = joint_positions_rad.size(); i < 6; ++i)
    {
        status_msg.joint_position_rad[i] = 0.0;
    }
    for (size_t i = 0; i < 6 && i < joint_positions_deg.size(); ++i)
    {
        status_msg.joint_position_deg[i] = joint_positions_deg[i];
    }
    for (size_t i = joint_positions_deg.size(); i < 6; ++i)
    {
        status_msg.joint_position_deg[i] = 0.0;
    }

    if (industrial_status_received_)
    {
        status_msg.is_online = is_online_from_industrial_;
        status_msg.enable = enable_from_industrial_;
        status_msg.in_motion = in_motion_from_industrial_;
    }
    else
    {
        status_msg.is_online = joint_states_received_;
        status_msg.enable = moveit_initialized_;
        status_msg.in_motion = (planning_status_ == "executing");
    }

    status_msg.planning_status = getPlanningStatus();

    bool fk_success = getForwardKinematics(joint_positions_rad, status_msg.cartesian_position);
    
    if (!fk_success)
    {
        status_msg.cartesian_position.position.x = 0.0;
        status_msg.cartesian_position.position.y = 0.0;
        status_msg.cartesian_position.position.z = 0.0;
        status_msg.cartesian_position.orientation.w = 1.0;
        status_msg.cartesian_position.orientation.x = 0.0;
        status_msg.cartesian_position.orientation.y = 0.0;
        status_msg.cartesian_position.orientation.z = 0.0;
    }
    
    robot_status_pub_->publish(status_msg);
}

void RobotStatusPublisher::statusTimerCallback()
{
    publishRobotStatus();
}

void RobotStatusPublisher::spin()
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
        auto publisher = std::make_shared<demo_driver::RobotStatusPublisher>();
        executor.add_node(publisher);
        
        std::thread init_thread([publisher]() {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            publisher->initialize(10, 2);
        });
        init_thread.detach();
        
        executor.spin();
    }
    catch (const std::exception& e)
    {
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
