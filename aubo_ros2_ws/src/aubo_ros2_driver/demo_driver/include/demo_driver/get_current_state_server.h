#ifndef DEMO_DRIVER_GET_CURRENT_STATE_SERVER_H_
#define DEMO_DRIVER_GET_CURRENT_STATE_SERVER_H_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ivg_interfaces/srv/get_current_state.hpp>
#include <ivg_interfaces/srv/get_fk.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <string>
#include <memory>

namespace demo_driver
{

class GetCurrentStateServer : public rclcpp::Node
{
public:
    explicit GetCurrentStateServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    void init();

private:
    // 数据源
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    sensor_msgs::msg::JointState current_joint_states_;
    bool joint_states_received_{false};

    // MoveIt
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    // FK 客户端 (Reentrant 组)
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::Client<ivg_interfaces::srv::GetFK>::SharedPtr fk_client_;

    // 服务
    rclcpp::Service<ivg_interfaces::srv::GetCurrentState>::SharedPtr service_;

    void getCurrentStateCallback(std::shared_ptr<ivg_interfaces::srv::GetCurrentState::Response> res);
    bool getCurrentState(std::vector<double>& joint_rad, geometry_msgs::msg::Pose& cartesian,
                         std::vector<double>& vel, std::string& msg);
    bool computeFK(const std::vector<double>& joints, geometry_msgs::msg::Pose& pose);

    std::string planning_group_name_;
    std::string base_frame_;
    std::string end_effector_link_;
};

} // namespace demo_driver
#endif
