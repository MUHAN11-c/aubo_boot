#ifndef DEMO_DRIVER_MOVE_TO_POSE_SERVER_H_
#define DEMO_DRIVER_MOVE_TO_POSE_SERVER_H_

#include <rclcpp/rclcpp.hpp>
#include <ivg_interfaces/srv/move_to_pose.hpp>
#include <memory>
#include <mutex>

namespace demo_driver
{

class RobotController;

class MoveToPoseServer : public rclcpp::Node
{
public:
    explicit MoveToPoseServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    void init();

private:
    std::unique_ptr<RobotController> robot_controller_;

    rclcpp::Service<ivg_interfaces::srv::MoveToPose>::SharedPtr service_;
    std::mutex service_mutex_;

    void onMoveToPoseRequest(const std::shared_ptr<ivg_interfaces::srv::MoveToPose::Request> req,
                             std::shared_ptr<ivg_interfaces::srv::MoveToPose::Response> res);
};

}  // namespace demo_driver
#endif
