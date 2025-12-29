#ifndef DEMO_DRIVER_ROBOT_STATUS_PUBLISHER_H_
#define DEMO_DRIVER_ROBOT_STATUS_PUBLISHER_H_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <demo_interface/msg/robot_status.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <string>
#include <vector>
#include <memory>

namespace demo_driver
{

class RobotStatusPublisher : public rclcpp::Node
{
public:
    RobotStatusPublisher();
    ~RobotStatusPublisher();

    void spin();
    bool initialize(int max_retries = 10, int retry_delay_seconds = 2);

private:
    rclcpp::Publisher<demo_interface::msg::RobotStatus>::SharedPtr robot_status_pub_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trajectory_execution_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr industrial_robot_status_sub_;

    void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void trajectoryExecutionCallback(const std_msgs::msg::String::SharedPtr msg);
    void industrialRobotStatusCallback(const std_msgs::msg::String::SharedPtr msg);

    bool wait_for_robot_description(int timeout_seconds = 30);
    bool getForwardKinematics(const std::vector<double>& joint_positions, 
                              geometry_msgs::msg::Pose& cartesian_pose);
    std::string getPlanningStatus();
    void publishRobotStatus();

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

    sensor_msgs::msg::JointState current_joint_states_;
    bool joint_states_received_;
    bool industrial_status_received_;
    std::string planning_status_;
    
    bool is_online_from_industrial_;
    bool enable_from_industrial_;
    bool in_motion_from_industrial_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    std::string end_effector_link_;
    bool moveit_initialized_;
    
    void statusTimerCallback();

    double publish_rate_;
    std::string base_frame_;
    std::string planning_group_name_;
    std::string robot_status_topic_;
    bool subscribe_industrial_status_;
};

} // namespace demo_driver

#endif // DEMO_DRIVER_ROBOT_STATUS_PUBLISHER_H_
