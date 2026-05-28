#include "latte_cartesian_planner/cartesian_planner_node.hpp"
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <builtin_interfaces/msg/duration.hpp>

namespace latte_cartesian_planner
{

CartesianPlannerNode::CartesianPlannerNode(const rclcpp::NodeOptions& options)
  : Node("latte_cartesian_planner", options)
{
  if (!has_parameter("planning_group")) declare_parameter("planning_group", "manipulator");
  if (!has_parameter("base_frame")) declare_parameter("base_frame", "base_link");
  planning_group_ = get_parameter("planning_group").as_string();
  base_frame_ = get_parameter("base_frame").as_string();

  service_ = create_service<ivg_interfaces::srv::LatteCartesianPlan>(
      "/latte/plan_and_execute",
      [this](const std::shared_ptr<ivg_interfaces::srv::LatteCartesianPlan::Request> req,
             std::shared_ptr<ivg_interfaces::srv::LatteCartesianPlan::Response> res) {
        planAndExecuteCallback(req, res);
      });

  RCLCPP_INFO(get_logger(), "CartesianPlannerNode awaiting init()...");
}

void CartesianPlannerNode::init()
{
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), planning_group_);
  move_group_->setPoseReferenceFrame(base_frame_);
  move_group_->allowReplanning(true);
  ee_link_ = move_group_->getEndEffectorLink();

  visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
      shared_from_this(), base_frame_, rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_->getRobotModel());
  visual_tools_->loadMarkerPub();
  visual_tools_->deleteAllMarkers();
  visual_tools_->trigger();

  RCLCPP_INFO(get_logger(), "CartesianPlannerNode ready (ee_link=%s)", ee_link_.c_str());
}

void CartesianPlannerNode::planAndExecuteCallback(
    const std::shared_ptr<ivg_interfaces::srv::LatteCartesianPlan::Request> req,
    std::shared_ptr<ivg_interfaces::srv::LatteCartesianPlan::Response> res)
{
  std::lock_guard<std::mutex> lock(service_mutex_);

  if (!move_group_) {
    res->success = false;
    res->message = "not initialized";
    res->fraction = 0.0;
    res->trajectory_points = 0;
    res->error_code_val = -100;
    return;
  }

  if (req->waypoints.empty()) {
    res->success = false;
    res->message = "empty waypoints";
    res->fraction = 0.0;
    res->trajectory_points = 0;
    res->error_code_val = -1;
    return;
  }

  double fraction = 0.0;
  int32_t traj_pts = 0, error_code = 0;
  std::string msg;

  bool ok = planAndExecute(
      req->waypoints, req->max_step, req->jump_threshold, req->avoid_collisions,
      req->velocity_scaling, req->acceleration_scaling,
      req->fraction_threshold, req->dt,
      fraction, traj_pts, error_code, msg);

  res->success           = ok;
  res->message           = msg;
  res->fraction          = fraction;
  res->trajectory_points = traj_pts;
  res->error_code_val    = error_code;
}

bool CartesianPlannerNode::planAndExecute(
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    double max_step, double jump_threshold, bool avoid_collisions,
    double velocity_scaling, double acceleration_scaling,
    double fraction_threshold, double dt,
    double& out_fraction, int32_t& out_trajectory_points,
    int32_t& out_error_code, std::string& out_message)
{
  move_group_->setMaxVelocityScalingFactor(velocity_scaling);
  move_group_->setMaxAccelerationScalingFactor(acceleration_scaling);

  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = 0.0;
  try {
    fraction = move_group_->computeCartesianPath(
        waypoints, max_step, jump_threshold, trajectory, avoid_collisions);
  } catch (const std::exception& e) {
    out_fraction = 0.0;
    out_trajectory_points = 0;
    out_error_code = -200;
    out_message = std::string("computeCartesianPath exception: ") + e.what();
    return false;
  }

  out_fraction = fraction;

  if (trajectory.joint_trajectory.points.empty()) {
    out_trajectory_points = 0;
    out_error_code = -2;
    out_message = "computeCartesianPath returned empty trajectory";
    return false;
  }

  out_trajectory_points = static_cast<int32_t>(trajectory.joint_trajectory.points.size());

  if (fraction < fraction_threshold) {
    RCLCPP_WARN(get_logger(),
        "fraction %.3f < threshold %.3f, retrying with avoid_collisions=false",
        fraction, fraction_threshold);
    try {
      fraction = move_group_->computeCartesianPath(
          waypoints, max_step, jump_threshold, trajectory, false);
    } catch (const std::exception& e) {
      out_fraction = fraction;
      out_trajectory_points = static_cast<int32_t>(trajectory.joint_trajectory.points.size());
      out_error_code = -200;
      out_message = std::string("computeCartesianPath retry exception: ") + e.what();
      return false;
    }
    out_fraction = fraction;
    out_trajectory_points = static_cast<int32_t>(trajectory.joint_trajectory.points.size());
    if (fraction < fraction_threshold) {
      out_error_code = -3;
      out_message = "fraction " + std::to_string(fraction) +
                    " < threshold " + std::to_string(fraction_threshold) +
                    " (retry without collision also failed)";
      return false;
    }
  }

  if (dt > 0.0) {
    scaleTrajectoryTiming(trajectory.joint_trajectory, dt);
  }

  try {
    moveit::core::MoveItErrorCode result = move_group_->execute(trajectory);
    out_error_code = static_cast<int32_t>(result.val);

    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      out_message = "execute failed, error_code=" + std::to_string(result.val);
      return false;
    }
  } catch (const std::exception& e) {
    out_error_code = -200;
    out_message = std::string("execute exception: ") + e.what();
    return false;
  }

  out_message = "plan_and_execute succeeded";

  // 发布 EE 末端轨迹线到 RViz2
  auto ee_link = move_group_->getRobotModel()->getLinkModel(ee_link_);
  auto jmg = move_group_->getRobotModel()->getJointModelGroup(planning_group_);
  if (ee_link && jmg) {
    visual_tools_->deleteAllMarkers();
    visual_tools_->publishTrajectoryLine(trajectory, ee_link, jmg,
                                         rviz_visual_tools::LIME_GREEN);
    visual_tools_->trigger();
  }

  return true;
}

void CartesianPlannerNode::scaleTrajectoryTiming(
    trajectory_msgs::msg::JointTrajectory& joint_traj, double dt)
{
  for (size_t i = 0; i < joint_traj.points.size(); ++i) {
    double t = static_cast<double>(i) * dt;
    builtin_interfaces::msg::Duration dur;
    dur.sec = static_cast<int32_t>(t);
    dur.nanosec = static_cast<uint32_t>((t - static_cast<double>(dur.sec)) * 1e9);
    joint_traj.points[i].time_from_start = dur;
  }
}

}  // namespace latte_cartesian_planner
