#ifndef DEMO_DRIVER_EXECUTE_TRAJECTORY_SERVER_H_
#define DEMO_DRIVER_EXECUTE_TRAJECTORY_SERVER_H_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ivg_interfaces/srv/execute_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <string>
#include <memory>

namespace demo_driver
{

class ExecuteTrajectoryServer : public rclcpp::Node
{
public:
    explicit ExecuteTrajectoryServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    rclcpp::Service<ivg_interfaces::srv::ExecuteTrajectory>::SharedPtr service_;

    void executeTrajectoryCallback(
        const std::shared_ptr<ivg_interfaces::srv::ExecuteTrajectory::Request> req,
        std::shared_ptr<ivg_interfaces::srv::ExecuteTrajectory::Response> res);

    bool executeTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory,
                           int32_t& error_code, std::string& message);

    std::string planning_group_name_;
    std::string base_frame_;
};

} // namespace demo_driver
#endif
