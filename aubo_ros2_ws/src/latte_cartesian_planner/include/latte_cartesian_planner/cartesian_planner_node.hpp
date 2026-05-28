#ifndef LATTE_CARTESIAN_PLANNER_CARTESIAN_PLANNER_NODE_HPP_
#define LATTE_CARTESIAN_PLANNER_CARTESIAN_PLANNER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ivg_interfaces/srv/latte_cartesian_plan.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <mutex>
#include <memory>
#include <string>
#include <vector>

namespace latte_cartesian_planner
{

class CartesianPlannerNode : public rclcpp::Node
{
public:
  explicit CartesianPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  void init();

private:
  std::string planning_group_;
  std::string base_frame_;
  std::string ee_link_;

  rclcpp::Service<ivg_interfaces::srv::LatteCartesianPlan>::SharedPtr service_;
  std::mutex service_mutex_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;

  void planAndExecuteCallback(
      const std::shared_ptr<ivg_interfaces::srv::LatteCartesianPlan::Request> req,
      std::shared_ptr<ivg_interfaces::srv::LatteCartesianPlan::Response> res);

  bool planAndExecute(
      const std::vector<geometry_msgs::msg::Pose>& waypoints,
      double max_step, double jump_threshold, bool avoid_collisions,
      double velocity_scaling, double acceleration_scaling,
      double fraction_threshold, double dt,
      double& out_fraction, int32_t& out_trajectory_points,
      int32_t& out_error_code, std::string& out_message);

  static void scaleTrajectoryTiming(
      trajectory_msgs::msg::JointTrajectory& joint_traj, double dt);
};

}  // namespace latte_cartesian_planner

#endif
