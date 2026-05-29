#include "latte_cartesian_planner/cartesian_planner_node.hpp"
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <sstream>
#include <iomanip>

namespace latte_cartesian_planner
{

CartesianPlannerNode::CartesianPlannerNode(const rclcpp::NodeOptions& options)
  : Node("latte_cartesian_planner", options)
{
  if (!has_parameter("planning_group")) declare_parameter("planning_group", "manipulator");
  if (!has_parameter("base_frame")) declare_parameter("base_frame", "base_link");
  planning_group_ = get_parameter("planning_group").as_string();
  base_frame_ = get_parameter("base_frame").as_string();

  // 独立回调组 — 避免服务回调阻塞 MoveIt 内部的 CurrentStateMonitor 订阅
  service_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  service_ = create_service<ivg_interfaces::srv::LatteCartesianPlan>(
      "/latte/plan_and_execute",
      [this](const std::shared_ptr<ivg_interfaces::srv::LatteCartesianPlan::Request> req,
             std::shared_ptr<ivg_interfaces::srv::LatteCartesianPlan::Response> res) {
        planAndExecuteCallback(req, res);
      },
      rmw_qos_profile_services_default, service_cb_group_);

  RCLCPP_INFO(get_logger(), "CartesianPlannerNode awaiting init()...");
}

void CartesianPlannerNode::init()
{
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), planning_group_);
  move_group_->setPoseReferenceFrame(base_frame_);
  move_group_->allowReplanning(true);
  ee_link_ = move_group_->getEndEffectorLink();

  // TF2 — 用于获取当前 TCP 位姿 (不依赖 CurrentStateMonitor)
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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

  // ── 通过 TF2 获取当前 TCP 位姿 (不依赖 CurrentStateMonitor) ──
  geometry_msgs::msg::Pose current_tcp;
  {
    try {
      auto tf = tf_buffer_->lookupTransform(
          base_frame_, ee_link_, tf2::TimePointZero, tf2::durationFromSec(3.0));
      current_tcp.position.x = tf.transform.translation.x;
      current_tcp.position.y = tf.transform.translation.y;
      current_tcp.position.z = tf.transform.translation.z;
      current_tcp.orientation = tf.transform.rotation;
    } catch (const tf2::TransformException& e) {
      res->success = false;
      res->message = std::string("TF2 查 TCP 失败: ") + e.what();
      res->fraction = 0.0;
      res->trajectory_points = 0;
      res->error_code_val = -400;
      RCLCPP_ERROR(get_logger(), "TF2 lookup failed: %s", e.what());
      return;
    }
  }

  // ── 打印当前 TCP 位姿 + waypoints 首尾 ──
  {
    RCLCPP_INFO(get_logger(),
        "当前 TCP (TF2): pos(%.4f %.4f %.4f) orient(%.4f %.4f %.4f %.4f)",
        current_tcp.position.x, current_tcp.position.y, current_tcp.position.z,
        current_tcp.orientation.x, current_tcp.orientation.y,
        current_tcp.orientation.z, current_tcp.orientation.w);

    // 首尾 waypoint 距离诊断
    const auto& wp0 = req->waypoints[0];
    double dx = wp0.position.x - current_tcp.position.x;
    double dy = wp0.position.y - current_tcp.position.y;
    double dz = wp0.position.z - current_tcp.position.z;
    double dist_first = std::sqrt(dx*dx + dy*dy + dz*dz);
    RCLCPP_INFO(get_logger(),
        "wp[0] vs TCP 偏移: d(%.3f %.3f %.3f) dist=%.3fm",
        dx, dy, dz, dist_first);
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
        "fraction %.3f < threshold %.3f — 笛卡尔路径不可达, 不执行 (碰撞检测保持启用)",
        fraction, fraction_threshold);
    out_error_code = -3;
    out_message = "fraction " + std::to_string(fraction) +
                  " < threshold " + std::to_string(fraction_threshold) +
                  " (笛卡尔路径不可达, 请检查起点位姿与第1个waypoint的对齐)";
    return false;
  }

  if (dt > 0.0) {
    scaleTrajectoryTiming(trajectory.joint_trajectory, dt);
  }

  // 打印轨迹概要 (用于诊断执行失败)
  {
    auto& jt = trajectory.joint_trajectory;
    RCLCPP_INFO(get_logger(),
        "轨迹概要: %zu joint_names, %zu points, time [%.2fs → %.2fs]",
        jt.joint_names.size(), jt.points.size(),
        jt.points.empty() ? 0.0 :
            rclcpp::Duration(jt.points[0].time_from_start).seconds(),
        jt.points.empty() ? 0.0 :
            rclcpp::Duration(jt.points.back().time_from_start).seconds());
    if (!jt.joint_names.empty() && !jt.points.empty()) {
      auto& first_pt = jt.points[0];
      auto& last_pt = jt.points.back();
      std::stringstream ss_first, ss_last;
      for (size_t i = 0; i < std::min(jt.joint_names.size(), size_t(3)); ++i)
        ss_first << std::fixed << std::setprecision(3) << first_pt.positions[i] << " ";
      for (size_t i = 0; i < std::min(jt.joint_names.size(), size_t(3)); ++i)
        ss_last  << std::fixed << std::setprecision(3) << last_pt.positions[i] << " ";
      RCLCPP_INFO(get_logger(), "  joints[0]: %s", ss_first.str().c_str());
      RCLCPP_INFO(get_logger(), "  joints[-1]: %s",  ss_last.str().c_str());
    }
  }

  // MoveIt2 Humble 标准: RobotTrajectory 必须包装在 Plan 结构体中执行
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;
  plan.planning_time_ = 0.0;  // planning time placeholder

  try {
    moveit::core::MoveItErrorCode result = move_group_->execute(plan);
    out_error_code = static_cast<int32_t>(result.val);

    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      const char* err_name = "UNKNOWN";
      switch (result.val) {
        case moveit_msgs::msg::MoveItErrorCodes::SUCCESS: err_name = "SUCCESS"; break;
        case moveit_msgs::msg::MoveItErrorCodes::FAILURE: err_name = "FAILURE"; break;
        case moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED: err_name = "PLANNING_FAILED"; break;
        case moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN: err_name = "INVALID_MOTION_PLAN"; break;
        case moveit_msgs::msg::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: err_name = "ENVIRONMENT_CHANGE"; break;
        case moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED: err_name = "CONTROL_FAILED"; break;
        case moveit_msgs::msg::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA: err_name = "SENSOR_DATA"; break;
        case moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT: err_name = "TIMED_OUT"; break;
        case moveit_msgs::msg::MoveItErrorCodes::PREEMPTED: err_name = "PREEMPTED"; break;
      }
      out_message = std::string("execute: ") + err_name +
                    " (code=" + std::to_string(result.val) + ")";
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
