#ifndef DEMO_DRIVER_SET_SPEED_FACTOR_SERVER_H_
#define DEMO_DRIVER_SET_SPEED_FACTOR_SERVER_H_

#include <rclcpp/rclcpp.hpp>
#include <ivg_interfaces/srv/set_speed_factor.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>
#include <string>

namespace demo_driver
{

class SetSpeedFactorServer : public rclcpp::Node
{
public:
    explicit SetSpeedFactorServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    rclcpp::Service<ivg_interfaces::srv::SetSpeedFactor>::SharedPtr service_;

    rclcpp::CallbackGroup::SharedPtr param_cb_group_;
    std::shared_ptr<rclcpp::AsyncParametersClient> plan_traj_client_;
    std::shared_ptr<rclcpp::AsyncParametersClient> exec_traj_client_;

    void setSpeedFactorCallback(
        const std::shared_ptr<ivg_interfaces::srv::SetSpeedFactor::Request> req,
        std::shared_ptr<ivg_interfaces::srv::SetSpeedFactor::Response> res);

    bool setSpeedFactor(float velocity_factor, std::string& message);

    std::string planning_group_name_;
    std::string base_frame_;
};

} // namespace demo_driver
#endif
