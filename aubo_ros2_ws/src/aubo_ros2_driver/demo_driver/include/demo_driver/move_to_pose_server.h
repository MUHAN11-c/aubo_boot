#ifndef DEMO_DRIVER_MOVE_TO_POSE_SERVER_H_
#define DEMO_DRIVER_MOVE_TO_POSE_SERVER_H_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ivg_interfaces/srv/move_to_pose.hpp>
#include <array>
#include <memory>
#include <mutex>

namespace demo_driver
{

class MoveToPoseServer : public rclcpp::Node
{
public:
    explicit MoveToPoseServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    void init();

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    rclcpp::Service<ivg_interfaces::srv::MoveToPose>::SharedPtr service_;
    std::mutex service_mutex_;

    void onMoveToPoseRequest(const std::shared_ptr<ivg_interfaces::srv::MoveToPose::Request> req,
                             std::shared_ptr<ivg_interfaces::srv::MoveToPose::Response> res);

    bool moveToJoints(const double* joints, float vel, float acc);
    bool moveToPose(double x, double y, double z, double qx, double qy, double qz, double qw,
                    float vel, float acc);
};

}  // namespace demo_driver
#endif
