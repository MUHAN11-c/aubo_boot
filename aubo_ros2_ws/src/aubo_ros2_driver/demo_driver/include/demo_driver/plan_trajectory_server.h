#ifndef DEMO_DRIVER_PLAN_TRAJECTORY_SERVER_H_
#define DEMO_DRIVER_PLAN_TRAJECTORY_SERVER_H_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ivg_interfaces/srv/plan_trajectory.hpp>
#include <ivg_interfaces/srv/get_ik.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <string>
#include <memory>

namespace demo_driver
{

class PlanTrajectoryServer : public rclcpp::Node
{
public:
    explicit PlanTrajectoryServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /** MoveGroupInterface 依赖 Node 已被 shared_ptr 持有，不得在构造函数内 shared_from_this() 喵~ */
    void init();

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    rclcpp::Service<ivg_interfaces::srv::PlanTrajectory>::SharedPtr service_;
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::Client<ivg_interfaces::srv::GetIK>::SharedPtr ik_client_;

    void planTrajectoryCallback(
        const std::shared_ptr<ivg_interfaces::srv::PlanTrajectory::Request> req,
        std::shared_ptr<ivg_interfaces::srv::PlanTrajectory::Response> res);

    bool planTrajectory(const geometry_msgs::msg::Pose& target_pose, bool use_joints,
                        trajectory_msgs::msg::JointTrajectory& trajectory,
                        float& planning_time, std::string& message);

    std::string planning_group_name_;
    std::string base_frame_;
    std::string end_effector_link_;
};

} // namespace demo_driver
#endif
