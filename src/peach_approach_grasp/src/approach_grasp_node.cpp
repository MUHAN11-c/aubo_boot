// Copyright 2026, aubo_e5_ros2_ws authors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// 主动视觉靠近与抓取编排节点：只通过 MoveIt 和现有 ROS 接口工作，不直接访问 SDK。
#include <Eigen/Geometry>
#include <behaviortree_cpp/bt_factory.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <aubo_msgs/msg/robot_status.hpp>
#include <aubo_msgs/srv/set_io.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <nlohmann/json.hpp>
#include <peach_pose_msgs/msg/bag_fitting_array.hpp>
#include <peach_pose_msgs/msg/bag_grasp_candidate_array.hpp>
#include <peach_pose_msgs/msg/peach_target_observation_array.hpp>
#include <peach_harvest_msgs/action/run_target_cycle.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "peach_approach_grasp/quality_gate.hpp"
#include "peach_approach_grasp/action_contract.hpp"
#include "peach_approach_grasp/grasp_task.hpp"
#include "peach_approach_grasp/view_planner.hpp"

using namespace std::chrono_literals;

namespace peach_approach_grasp
{
namespace
{
using Trigger = std_srvs::srv::Trigger;
using SetBool = std_srvs::srv::SetBool;
using json = nlohmann::json;
using RunTargetCycle = peach_harvest_msgs::action::RunTargetCycle;
using RunTargetGoalHandle = rclcpp_action::ServerGoalHandle<RunTargetCycle>;

Eigen::Isometry3d poseToEigen(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond quaternion(
    pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z);
  if (!quaternion.coeffs().allFinite() || quaternion.norm() < 1.0e-9) {
    quaternion = Eigen::Quaterniond::Identity();
  } else {
    quaternion.normalize();
  }
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = quaternion.toRotationMatrix();
  transform.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  return transform;
}

geometry_msgs::msg::Pose eigenToPose(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  const Eigen::Quaterniond quaternion(transform.linear());
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

Eigen::Vector3d pointToEigen(const geometry_msgs::msg::Point & point)
{
  return {point.x, point.y, point.z};
}

Eigen::Vector3d vectorToEigen(const geometry_msgs::msg::Vector3 & vector)
{
  return {vector.x, vector.y, vector.z};
}

bool nonzeroFinite(const Eigen::Vector3d & value)
{
  return value.allFinite() && value.norm() > 1.0e-6;
}
}  // namespace

class ApproachGraspNode : public rclcpp::Node
{
public:
  ApproachGraspNode()
  : Node("peach_approach_grasp_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declareParameters();
    loadParameters();
    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&ApproachGraspNode::onParameters, this, std::placeholders::_1));
    createInterfaces();
    setState("IDLE", "等待 start_cycle；默认只规划，不执行运动");
  }

  ~ApproachGraspNode() override
  {
    cancel_requested_.store(true);
    if (move_group_) {
      move_group_->stop();
    }
    if (grasp_task_) {
      grasp_task_->cancel();
    }
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  void initializeMoveIt()
  {
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), planning_group_);
    move_group_->setPoseReferenceFrame(base_frame_);
    move_group_->setPlanningTime(planning_time_s_);
    move_group_->setNumPlanningAttempts(planning_attempts_);
    move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);
    move_group_->allowReplanning(true);
    GraspTaskConfig task_config;
    task_config.planning_group = planning_group_;
    task_config.wrist_frame = wrist_frame_;
    task_config.base_frame = base_frame_;
    task_config.free_space_pipeline = fallback_pipeline_;
    task_config.free_space_planner = mtc_free_space_planner_;
    task_config.planning_time_s = planning_time_s_;
    task_config.velocity_scaling = velocity_scaling_;
    task_config.acceleration_scaling = acceleration_scaling_;
    task_config.cartesian_step_m = mtc_cartesian_step_m_;
    task_config.cartesian_precision_m = mtc_cartesian_precision_m_;
    task_config.max_solutions = static_cast<std::size_t>(mtc_max_solutions_);
    task_config.approach_execution_gate = [this](std::string & reason) {
        return safetyReady(reason) && cycleTargetReady(cycle_target_id_, reason) &&
               !cancel_requested_.load();
      };
    // 插入后目标常被工具遮挡，撤离不能依赖视觉可见性，否则会把正常遮挡误判为禁止撤离。
    task_config.retreat_execution_gate = [this](std::string & reason) {
        return safetyReady(reason) && !cancel_requested_.load();
      };
    grasp_task_ = std::make_unique<GraspTask>(shared_from_this(), task_config);
    registerBehaviorTreeNodes();
    RCLCPP_INFO(
      get_logger(),
      "主动视觉靠近节点 ready: group=%s base=%s wrist=%s camera=%s "
      "execution=%s grasp=%s",
      planning_group_.c_str(), base_frame_.c_str(), wrist_frame_.c_str(),
      camera_frame_.c_str(), execution_enabled_ ? "enabled" : "plan_only",
      grasp_enabled_ ? "enabled" : "disabled");
  }

private:
  struct TargetData
  {
    std::string id;
    std::string harvest_run_id;
    Eigen::Vector3d center{Eigen::Vector3d::Zero()};
    Eigen::Isometry3d initial_pose{Eigen::Isometry3d::Identity()};
    Eigen::Vector3d initial_axis{Eigen::Vector3d::UnitZ()};
    double suggested_travel_m{0.0};
    rclcpp::Time received{0, 0, RCL_ROS_TIME};
    bool valid{false};
  };

  struct RefinedData
  {
    std::string id;
    Eigen::Vector3d entry{Eigen::Vector3d::Zero()};
    Eigen::Vector3d bottom{Eigen::Vector3d::Zero()};
    Eigen::Vector3d neck{Eigen::Vector3d::Zero()};
    Eigen::Vector3d axis{Eigen::Vector3d::UnitZ()};
    double suggested_travel_m{0.0};
    bool valid{false};
  };


  double insertionTravel(const RefinedData & refined) const
  {
    // suggested_travel_m 由感知/重建端结合工具几何给出，是跨包行程契约。
    // 仅为兼容旧记录或异常消息，字段无效时才按几何距离与颈部余量回退。
    double travel = refined.suggested_travel_m;
    if (!std::isfinite(travel) || travel <= 0.0) {
      travel = (refined.neck - refined.entry).norm() - neck_margin_m_;
    }
    return std::clamp(travel, minimum_travel_m_, maximum_travel_m_);
  }
  void declareParameters()
  {
    declare_parameter("frames.base", "base_link");
    declare_parameter("frames.wrist", "wrist3_Link");
    declare_parameter("frames.camera", "camera_depth_optical_frame");
    declare_parameter("frames.tool", "tcp");
    declare_parameter("moveit.planning_group", "manipulator_e5");
    declare_parameter("moveit.planning_time_s", 5.0);
    declare_parameter("moveit.planning_attempts", 5);
    declare_parameter("moveit.velocity_scaling", 0.05);
    declare_parameter("moveit.acceleration_scaling", 0.05);
    declare_parameter("moveit.pilz_pipeline", "pilz_industrial_motion_planner");
    declare_parameter("moveit.fallback_pipeline", "ompl");
    declare_parameter("moveit.mtc_free_space_planner", "RRTConnectkConfigDefault");
    declare_parameter("moveit.mtc_cartesian_step_m", 0.005);
    declare_parameter("moveit.mtc_cartesian_precision_m", 0.001);
    declare_parameter("moveit.mtc_max_solutions", 5);
    declare_parameter("behavior_tree.xml", "");
    declare_parameter("scan.observation_radius_m", 0.28);
    declare_parameter("scan.minimum_radius_m", 0.20);
    declare_parameter("scan.azimuth_step_deg", 12.0);
    declare_parameter("scan.azimuth_limit_deg", 36.0);
    declare_parameter("scan.elevation_step_deg", 8.0);
    declare_parameter("scan.elevation_limit_deg", 16.0);
    declare_parameter("scan.preferred_baseline_deg", 15.0);
    declare_parameter("scan.radial_step_m", 0.015);
    declare_parameter("scan.candidate_layers", 3);
    declare_parameter("scan.views_to_minimum_radius", 5);
    declare_parameter("scan.maximum_moves", 8);
    declare_parameter("scan.frame_wait_s", 3.0);
    declare_parameter("quality.minimum_views", 5);
    declare_parameter("quality.minimum_baseline_deg", 22.0);
    declare_parameter("quality.minimum_mean_nearest_baseline_deg", 8.0);
    declare_parameter("quality.minimum_mean_depth_ratio", 0.40);
    declare_parameter("quality.maximum_refined_rmse_m", 0.005);
    declare_parameter("quality.minimum_refined_inlier_ratio", 0.35);
    declare_parameter("quality.maximum_data_age_s", 2.0);
    declare_parameter("execution.enabled", false);
    declare_parameter("execution.require_robot_status", true);
    declare_parameter("execution.robot_status_max_age_s", 1.0);
    declare_parameter("execution.target_observation_max_age_s", 1.0);
    declare_parameter("execution.reset_reconstruction_on_start", false);
    declare_parameter("grasp.enabled", false);
    declare_parameter("grasp.neck_margin_m", 0.015);
    declare_parameter("grasp.minimum_travel_m", 0.02);
    declare_parameter("grasp.maximum_travel_m", 0.20);
    declare_parameter("grasp.complete_target_after_retreat", true);
    declare_parameter("tool.enabled", false);
    declare_parameter("tool.io_fun", 3);
    declare_parameter("tool.io_pin", 0);
    declare_parameter("tool.close_state", 1.0);
    declare_parameter("timeouts.service_s", 3.0);
    declare_parameter("timeouts.refined_s", 10.0);
  }

  void loadParameters()
  {
    base_frame_ = get_parameter("frames.base").as_string();
    wrist_frame_ = get_parameter("frames.wrist").as_string();
    camera_frame_ = get_parameter("frames.camera").as_string();
    tool_frame_ = get_parameter("frames.tool").as_string();
    planning_group_ = get_parameter("moveit.planning_group").as_string();
    planning_time_s_ = get_parameter("moveit.planning_time_s").as_double();
    planning_attempts_ = get_parameter("moveit.planning_attempts").as_int();
    velocity_scaling_ = get_parameter("moveit.velocity_scaling").as_double();
    acceleration_scaling_ = get_parameter("moveit.acceleration_scaling").as_double();
    pilz_pipeline_ = get_parameter("moveit.pilz_pipeline").as_string();
    fallback_pipeline_ = get_parameter("moveit.fallback_pipeline").as_string();
    mtc_free_space_planner_ = get_parameter("moveit.mtc_free_space_planner").as_string();
    mtc_cartesian_step_m_ = get_parameter("moveit.mtc_cartesian_step_m").as_double();
    mtc_cartesian_precision_m_ =
      get_parameter("moveit.mtc_cartesian_precision_m").as_double();
    mtc_max_solutions_ = get_parameter("moveit.mtc_max_solutions").as_int();
    behavior_tree_xml_ = get_parameter("behavior_tree.xml").as_string();

    ViewPlannerConfig view_config;
    view_config.observation_radius_m = get_parameter("scan.observation_radius_m").as_double();
    view_config.minimum_radius_m = get_parameter("scan.minimum_radius_m").as_double();
    view_config.azimuth_step_deg = get_parameter("scan.azimuth_step_deg").as_double();
    view_config.azimuth_limit_deg = get_parameter("scan.azimuth_limit_deg").as_double();
    view_config.elevation_step_deg = get_parameter("scan.elevation_step_deg").as_double();
    view_config.elevation_limit_deg = get_parameter("scan.elevation_limit_deg").as_double();
    view_config.preferred_baseline_deg =
      get_parameter("scan.preferred_baseline_deg").as_double();
    view_config.radial_step_m = get_parameter("scan.radial_step_m").as_double();
    view_config.candidate_layers = get_parameter("scan.candidate_layers").as_int();
    view_config.views_to_minimum_radius =
      get_parameter("scan.views_to_minimum_radius").as_int();
    view_planner_ = std::make_unique<ViewPlanner>(view_config);
    maximum_scan_moves_ = get_parameter("scan.maximum_moves").as_int();
    frame_wait_s_ = get_parameter("scan.frame_wait_s").as_double();

    QualityGateConfig gate_config;
    gate_config.minimum_views = static_cast<std::size_t>(
      get_parameter("quality.minimum_views").as_int());
    gate_config.minimum_baseline_deg =
      get_parameter("quality.minimum_baseline_deg").as_double();
    gate_config.minimum_mean_nearest_baseline_deg =
      get_parameter("quality.minimum_mean_nearest_baseline_deg").as_double();
    gate_config.minimum_mean_depth_ratio =
      get_parameter("quality.minimum_mean_depth_ratio").as_double();
    gate_config.maximum_refined_rmse_m =
      get_parameter("quality.maximum_refined_rmse_m").as_double();
    gate_config.minimum_refined_inlier_ratio =
      get_parameter("quality.minimum_refined_inlier_ratio").as_double();
    gate_config.maximum_data_age_s =
      get_parameter("quality.maximum_data_age_s").as_double();
    quality_gate_ = std::make_unique<QualityGate>(gate_config);

    execution_enabled_ = get_parameter("execution.enabled").as_bool();
    require_robot_status_ = get_parameter("execution.require_robot_status").as_bool();
    robot_status_max_age_s_ =
      get_parameter("execution.robot_status_max_age_s").as_double();
    target_observation_max_age_s_ =
      get_parameter("execution.target_observation_max_age_s").as_double();
    reset_reconstruction_on_start_ =
      get_parameter("execution.reset_reconstruction_on_start").as_bool();
    grasp_enabled_ = get_parameter("grasp.enabled").as_bool();
    neck_margin_m_ = get_parameter("grasp.neck_margin_m").as_double();
    minimum_travel_m_ = get_parameter("grasp.minimum_travel_m").as_double();
    maximum_travel_m_ = get_parameter("grasp.maximum_travel_m").as_double();
    complete_target_after_retreat_ =
      get_parameter("grasp.complete_target_after_retreat").as_bool();
    tool_enabled_ = get_parameter("tool.enabled").as_bool();
    tool_io_fun_ = get_parameter("tool.io_fun").as_int();
    tool_io_pin_ = get_parameter("tool.io_pin").as_int();
    tool_close_state_ = get_parameter("tool.close_state").as_double();
    service_timeout_s_ = get_parameter("timeouts.service_s").as_double();
    refined_timeout_s_ = get_parameter("timeouts.refined_s").as_double();
  }

  rcl_interfaces::msg::SetParametersResult onParameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    if (running_.load()) {
      result.reason = "周期运行中不能修改运动策略";
      return result;
    }
    bool execution = execution_enabled_;
    bool grasp = grasp_enabled_;
    bool tool = tool_enabled_;
    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "execution.enabled") {
        execution = parameter.as_bool();
      } else if (parameter.get_name() == "grasp.enabled") {
        grasp = parameter.as_bool();
      } else if (parameter.get_name() == "tool.enabled") {
        tool = parameter.as_bool();
      }
    }
    if ((grasp && !execution) || (tool && !grasp)) {
      result.reason = "使能依赖必须满足 execution→grasp→tool";
      return result;
    }
    execution_enabled_ = execution;
    grasp_enabled_ = grasp;
    tool_enabled_ = tool;
    if (!execution_enabled_) {execution_armed_.store(false);}
    result.successful = true;
    result.reason = "运动策略已动态更新";
    publishState();
    return result;
  }

  void createInterfaces()
  {
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    target_sub_ = create_subscription<peach_pose_msgs::msg::PeachTargetObservationArray>(
      "/peach/perception/target_observations", 10,
      std::bind(&ApproachGraspNode::onTargets, this, std::placeholders::_1));
    diagnostics_sub_ = create_subscription<std_msgs::msg::String>(
      "/peach/reconstruction/diagnostics", latched,
      std::bind(&ApproachGraspNode::onDiagnostics, this, std::placeholders::_1));
    decision_sub_ = create_subscription<std_msgs::msg::String>(
      "/peach/reconstruction/grasp_decision", latched,
      std::bind(&ApproachGraspNode::onDecision, this, std::placeholders::_1));
    refined_pose_sub_ =
      create_subscription<peach_pose_msgs::msg::BagGraspCandidateArray>(
      "/peach/reconstruction/refined_pose", latched,
      std::bind(&ApproachGraspNode::onRefinedPose, this, std::placeholders::_1));
    refined_diag_sub_ = create_subscription<peach_pose_msgs::msg::BagFittingArray>(
      "/peach/reconstruction/refined_diagnostics", latched,
      std::bind(&ApproachGraspNode::onRefinedDiagnostics, this, std::placeholders::_1));
    robot_status_sub_ = create_subscription<aubo_msgs::msg::RobotStatus>(
      "/aubo_io_controller/robot_status", 10,
      std::bind(&ApproachGraspNode::onRobotStatus, this, std::placeholders::_1));

    status_pub_ = create_publisher<std_msgs::msg::String>("~/status", latched);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/planned_views", latched);
    start_service_ = create_service<Trigger>(
      "~/start_cycle",
      std::bind(
        &ApproachGraspNode::onStart, this,
        std::placeholders::_1, std::placeholders::_2));
    preview_approach_service_ = create_service<Trigger>(
      "~/preview_approach_insert",
      std::bind(
        &ApproachGraspNode::onPreviewApproachInsert, this,
        std::placeholders::_1, std::placeholders::_2));
    preview_full_contact_service_ = create_service<Trigger>(
      "~/preview_full_contact",
      std::bind(
        &ApproachGraspNode::onPreviewFullContact, this,
        std::placeholders::_1, std::placeholders::_2));
    cancel_service_ = create_service<Trigger>(
      "~/cancel_cycle",
      std::bind(
        &ApproachGraspNode::onCancel, this,
        std::placeholders::_1, std::placeholders::_2));
    recovery_service_ = create_service<Trigger>(
      "~/acknowledge_recovery",
      std::bind(
        &ApproachGraspNode::onAcknowledgeRecovery, this,
        std::placeholders::_1, std::placeholders::_2));
    query_service_ = create_service<Trigger>(
      "~/query_state",
      std::bind(
        &ApproachGraspNode::onQuery, this,
        std::placeholders::_1, std::placeholders::_2));
    arm_service_ = create_service<SetBool>(
      "~/set_execution_armed",
      std::bind(
        &ApproachGraspNode::onArm, this,
        std::placeholders::_1, std::placeholders::_2));
    cycle_action_server_ = rclcpp_action::create_server<RunTargetCycle>(
      this, "~/run_target_cycle",
      std::bind(
        &ApproachGraspNode::onActionGoal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(
        &ApproachGraspNode::onActionCancel, this, std::placeholders::_1),
      std::bind(
        &ApproachGraspNode::onActionAccepted, this, std::placeholders::_1));

    reset_client_ = create_client<Trigger>(
      "/peach_reconstruction_node/reset_reconstruction");
    finalize_client_ = create_client<Trigger>(
      "/peach_reconstruction_node/finalize_reconstruction");
    save_client_ = create_client<Trigger>(
      "/peach_reconstruction_node/save_session");
    complete_client_ = create_client<Trigger>(
      "/peach_pose_node/complete_selected_target");
    tool_io_client_ = create_client<aubo_msgs::srv::SetIO>(
      "/aubo_io_controller/set_io");
  }

  void onTargets(
    const peach_pose_msgs::msg::PeachTargetObservationArray::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto selected = std::find_if(
      message->observations.begin(), message->observations.end(),
      [&message](const auto & item) {
        return item.target_id == message->selected_target_id;
      });
    if (selected == message->observations.end()) {
      return;
    }
    // transient_local 的精化结果可能早于 volatile 的目标观测到达。首次获知目标时，
    // 若已缓存结果属于同一 target_id，必须保留；仅在确认 ID 冲突时清理旧目标数据。
    const bool target_changed =
      !target_.id.empty() && target_.id != message->selected_target_id;
    const bool refined_changed =
      (!refined_.id.empty() && refined_.id != message->selected_target_id) ||
      (!quality_.refined_target_id.empty() &&
      quality_.refined_target_id != message->selected_target_id);
    const bool decision_changed =
      !grasp_decision_target_id_.empty() &&
      grasp_decision_target_id_ != message->selected_target_id;
    if (target_changed || refined_changed || decision_changed) {
      target_.valid = false;
      refined_ = RefinedData();
      quality_.refined_target_id.clear();
      quality_.refined_rmse_m = -1.0;
      quality_.refined_inlier_ratio = -1.0;
      quality_.refined_accept = false;
      quality_.grasp_allowed = false;
      grasp_decision_target_id_.clear();
    }
    target_.id = message->selected_target_id;
    target_.valid = false;
    target_.harvest_run_id = message->harvest_run_id;
    target_.received = now();
    const Eigen::Vector3d bottom = pointToEigen(selected->candidate.bag_bottom);
    const Eigen::Vector3d neck = pointToEigen(selected->candidate.bag_neck);
    const Eigen::Vector3d axis = vectorToEigen(
      selected->candidate.translation_direction);
    if (selected->tracking_status ==
      peach_pose_msgs::msg::PeachTargetObservation::OBSERVED &&
      nonzeroFinite(bottom) && nonzeroFinite(neck) && nonzeroFinite(axis) &&
      selected->candidate.status !=
      peach_pose_msgs::msg::BagGraspCandidate::REJECT)
    {
      target_.center = 0.5 * (bottom + neck);
      target_.initial_pose = poseToEigen(selected->candidate.entry_pose);
      target_.initial_axis = axis.normalized();
      target_.suggested_travel_m = selected->candidate.suggested_travel_m;
      target_.valid = true;
    }
    quality_.selected_target_id = target_.id;
    data_cv_.notify_all();
  }

  void onDiagnostics(const std_msgs::msg::String::SharedPtr message)
  {
    try {
      const json value = json::parse(message->data);
      std::lock_guard<std::mutex> lock(data_mutex_);
      quality_.reconstruction_target_id = value.value("target_id", "");
      quality_.reconstruction_state = value.value("state", "IDLE");
      quality_.captured_views = value.value("captured_views", 0U);
      observed_directions_.clear();
      const auto coverage = value.value("view_coverage", json::object());
      quality_.max_baseline_deg = coverage.value("max_baseline_deg", 0.0);
      quality_.mean_nearest_baseline_deg =
        coverage.value("mean_nearest_baseline_deg", 0.0);
      quality_.mean_depth_ratio = coverage.value("valid_depth_ratio_mean", 0.0);
      for (const auto & view : coverage.value("views", json::array())) {
        const auto direction = view.value(
          "direction_target_to_camera", std::vector<double>{});
        if (direction.size() == 3U) {
          observed_directions_.emplace_back(direction[0], direction[1], direction[2]);
        }
      }
      diagnostics_received_ = now();
      quality_.data_age_s = 0.0;
      data_cv_.notify_all();
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "无法解析 reconstruction diagnostics: %s", error.what());
    }
  }

  void onDecision(const std_msgs::msg::String::SharedPtr message)
  {
    try {
      const json value = json::parse(message->data);
      std::lock_guard<std::mutex> lock(data_mutex_);
      const std::string decision_id = value.value("target_id", "");
      if (!target_.id.empty() && decision_id != target_.id) {
        RCLCPP_WARN(
          get_logger(), "忽略非当前目标的 grasp_decision: expected=%s actual=%s",
          target_.id.c_str(), decision_id.c_str());
        return;
      }
      grasp_decision_target_id_ = decision_id;
      quality_.grasp_allowed = value.value("allowed", false);
      data_cv_.notify_all();
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "无法解析 reconstruction grasp_decision: %s", error.what());
    }
  }

  void onRefinedPose(
    const peach_pose_msgs::msg::BagGraspCandidateArray::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (message->candidates.empty()) {
      refined_ = RefinedData();
      quality_.refined_target_id.clear();
      return;
    }
    const auto & candidate = message->candidates.front();
    if (!target_.id.empty() && candidate.target_id != target_.id) {
      RCLCPP_WARN(
        get_logger(), "忽略非当前目标的 refined pose: expected=%s actual=%s",
        target_.id.c_str(), candidate.target_id.c_str());
      return;
    }
    refined_ = RefinedData();
    refined_.id = candidate.target_id;
    refined_.entry = pointToEigen(candidate.entry_pose.position);
    refined_.bottom = pointToEigen(candidate.bag_bottom);
    refined_.neck = pointToEigen(candidate.bag_neck);
    refined_.axis = vectorToEigen(candidate.translation_direction);
    refined_.suggested_travel_m = candidate.suggested_travel_m;
    refined_.valid = nonzeroFinite(refined_.axis) && refined_.entry.allFinite();
    quality_.refined_target_id = candidate.target_id;
    quality_.refined_accept =
      candidate.status == peach_pose_msgs::msg::BagGraspCandidate::ACCEPT;
    data_cv_.notify_all();
  }

  void onRefinedDiagnostics(
    const peach_pose_msgs::msg::BagFittingArray::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (message->fittings.empty()) {
      return;
    }
    const auto & fitting = message->fittings.front();
    const std::string expected_id = refined_.id.empty() ? target_.id : refined_.id;
    if (!expected_id.empty() && fitting.target_id != expected_id) {
      RCLCPP_WARN(
        get_logger(), "忽略非当前目标的 refined diagnostics: expected=%s actual=%s",
        expected_id.c_str(), fitting.target_id.c_str());
      return;
    }
    // 节点刚启动时可能尚未收到 volatile 目标观测。先按 target_id 缓存锁存的
    // 拟合指标，后续 onTargets() 会保留同 ID 数据或清除冲突数据。
    quality_.refined_target_id = fitting.target_id;
    if (fitting.target_kind == "fruit") {
      quality_.refined_rmse_m = fitting.sphere_rms_m;
      quality_.refined_inlier_ratio = fitting.sphere_inlier_ratio;
    } else {
      quality_.refined_rmse_m = fitting.cylinder_rms_m;
      quality_.refined_inlier_ratio = fitting.cylinder_inlier_ratio;
    }
    quality_.refined_accept =
      fitting.status == peach_pose_msgs::msg::BagFitting::ACCEPT;
    data_cv_.notify_all();
  }

  void onRobotStatus(const aubo_msgs::msg::RobotStatus::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    robot_status_ = *message;
    robot_status_received_ = now();
    robot_status_valid_ = true;
  }

  rclcpp_action::GoalResponse onActionGoal(
    const rclcpp_action::GoalUUID &,
    const std::shared_ptr<const RunTargetCycle::Goal> goal)
  {
    if (goal->target_id.empty() || goal->mode == RunTargetCycle::Goal::OBSERVE_ONLY ||
      running_.load() || contact_recovery_required_.load())
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
    const auto target = targetSnapshot();
    if (!target || target->id != goal->target_id) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse onActionCancel(
    const std::shared_ptr<RunTargetGoalHandle>)
  {
    cancel_requested_.store(true);
    if (move_group_) {move_group_->stop();}
    if (grasp_task_) {grasp_task_->cancel();}
    data_cv_.notify_all();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void onActionAccepted(const std::shared_ptr<RunTargetGoalHandle> goal_handle)
  {
    std::thread([this, goal_handle]() {executeAction(goal_handle);}).detach();
  }

  void executeAction(const std::shared_ptr<RunTargetGoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto trigger_response = std::make_shared<Trigger::Response>();
    if (goal->mode == RunTargetCycle::Goal::PREVIEW) {
      previewContact(false, trigger_response);
    } else {
      // Action 是自动编排专用入口；手动 Trigger 仍要求每周期单独 arm。
      if (execution_enabled_) {execution_armed_.store(true);}
      onStart(std::make_shared<Trigger::Request>(), trigger_response);
    }
    if (!trigger_response->success) {
      auto result = std::make_shared<RunTargetCycle::Result>();
      result->outcome = RunTargetCycle::Result::FAILED;
      result->reason = trigger_response->message;
      result->recovery_required = contact_recovery_required_.load();
      goal_handle->abort(result);
      return;
    }

    while (rclcpp::ok() && running_.load()) {
      if (goal_handle->is_canceling()) {
        cancel_requested_.store(true);
        if (move_group_) {move_group_->stop();}
        if (grasp_task_) {grasp_task_->cancel();}
        data_cv_.notify_all();
      }
      auto feedback = std::make_shared<RunTargetCycle::Feedback>();
      feedback->state.target_id = goal->target_id;
      feedback->state.action_active = running_.load();
      feedback->state.execution_enabled = execution_enabled_;
      feedback->state.grasp_enabled = grasp_enabled_;
      feedback->state.tool_enabled = tool_enabled_;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        feedback->state.message = state_json_.value("message", std::string());
      }
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(200ms);
    }

    std::string terminal_state;
    std::string terminal_message;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      terminal_state = state_json_.value("state", std::string("FAILED"));
      terminal_message = state_json_.value("message", std::string());
    }
    const CycleOutcome outcome = classifyTerminalState(terminal_state);
    auto result = std::make_shared<RunTargetCycle::Result>();
    result->reason = terminal_message;
    result->recovery_required = outcome == CycleOutcome::RECOVERY_REQUIRED;
    if (outcome == CycleOutcome::SUCCEEDED) {
      result->outcome = RunTargetCycle::Result::SUCCEEDED;
      goal_handle->succeed(result);
    } else if (outcome == CycleOutcome::CANCELED || goal_handle->is_canceling()) {
      result->outcome = RunTargetCycle::Result::FAILED;
      goal_handle->canceled(result);
    } else {
      result->outcome = RunTargetCycle::Result::FAILED;
      goal_handle->abort(result);
    }
  }

  void onStart(const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
  {
    if (!move_group_) {
      response->success = false;
      response->message = "MoveIt 尚未初始化";
      return;
    }
    if (contact_recovery_required_.load()) {
      response->success = false;
      response->message =
        "上一周期可能停在接触区；现场人工撤离并确认后调用 acknowledge_recovery";
      return;
    }
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
      response->success = false;
      response->message = "已有靠近/抓取周期正在运行";
      return;
    }
    if (execution_enabled_ && !execution_armed_.load()) {
      running_.store(false);
      response->success = false;
      response->message = "execution.enabled=true 但尚未人工 arm";
      return;
    }
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!target_.valid || target_.id.empty()) {
        running_.store(false);
        response->success = false;
        response->message = "没有可用的 selected_target 初始几何";
        return;
      }
    }
    if (worker_.joinable()) {
      worker_.join();
    }
    cancel_requested_.store(false);
    worker_ = std::thread(&ApproachGraspNode::runCycle, this);
    response->success = true;
    response->message = execution_enabled_ ? "已启动主动视觉靠近周期" :
      "已启动只规划预览（不会发送运动）";
  }

  void onPreviewApproachInsert(
    const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
  {
    previewContact(false, response);
  }

  void onPreviewFullContact(
    const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
  {
    previewContact(true, response);
  }

  void previewContact(bool include_retreat, Trigger::Response::SharedPtr response)
  {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
      response->success = false;
      response->message = "已有规划或执行周期正在运行";
      return;
    }
    const auto finish = [this, &response](
      bool success, const std::string & state, const std::string & message)
      {
        running_.store(false);
        response->success = success;
        response->message = message;
        setState(state, message);
      };

    const GateResult gate = quality_gate_->readyToPreviewContact(qualitySnapshot());
    const auto target = targetSnapshot();
    const auto refined = refinedSnapshot();
    if (!gate.allowed) {
      finish(false, "PREVIEW_FAILED", "接触轨迹预览质量门失败: " + gate.reason);
      return;
    }
    if (!target || !refined || target->id != refined->id ||
      graspDecisionTargetSnapshot() != target->id)
    {
      finish(false, "PREVIEW_FAILED", "接触轨迹预览目标 ID 或精化几何不一致");
      return;
    }

    cycle_target_id_ = target->id;
    Eigen::Isometry3d entry_tool_pose = Eigen::Isometry3d::Identity();
    entry_tool_pose.translation() = refined->entry;
    entry_tool_pose.linear() = ViewPlanner::toolOrientation(
      refined->axis, target->initial_pose.linear().col(0));
    const auto wrist_from_tool = lookupTransform(wrist_frame_, tool_frame_);
    if (!wrist_from_tool) {
      finish(false, "PREVIEW_FAILED", "无法取得 wrist 到 tool 的变换");
      return;
    }
    const Eigen::Isometry3d entry_wrist_pose =
      entry_tool_pose * wrist_from_tool->inverse();
    const double travel = insertionTravel(*refined);

    setState(
      "PREVIEW_CONTACT_PLANNING",
      include_retreat ? "MTC 只规划：到入口、直线插入、同轴撤离" :
      "MTC 只规划：到入口、直线插入");
    const GraspTaskResult result = include_retreat ?
      grasp_task_->previewFullContact(entry_wrist_pose, refined->axis, travel) :
      grasp_task_->approachAndInsert(entry_wrist_pose, refined->axis, travel, false);
    if (!result.success) {
      finish(false, "PREVIEW_FAILED", "MTC 接触轨迹预览失败: " + result.reason);
      return;
    }
    finish(
      true, "PREVIEW_READY",
      include_retreat ?
      "完整接触轨迹已发布到 RViz；仅规划，未发送任何运动" :
      "入口与插入轨迹已发布到 RViz；仅规划，未发送任何运动");
  }

  void onCancel(const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
  {
    cancel_requested_.store(true);
    if (move_group_) {
      move_group_->stop();
    }
    if (grasp_task_) {
      grasp_task_->cancel();
    }
    data_cv_.notify_all();
    response->success = true;
    response->message = "已请求取消；当前 MoveIt 执行将停止";
  }

  void onAcknowledgeRecovery(
    const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
  {
    if (running_.load()) {
      response->success = false;
      response->message = "周期运行中不能确认恢复";
      return;
    }
    contact_recovery_required_.store(false);
    response->success = true;
    response->message = "已记录现场人工撤离确认；本服务不发送任何运动命令";
    setState("IDLE", response->message);
  }

  void onQuery(const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    response->success = true;
    response->message = state_json_.dump();
  }

  void onArm(const SetBool::Request::SharedPtr request, SetBool::Response::SharedPtr response)
  {
    if (running_.load()) {
      response->success = false;
      response->message = "周期运行中不能改变 arm 状态";
      return;
    }
    execution_armed_.store(request->data);
    response->success = true;
    response->message = request->data ?
      "已为下一次周期一次性 arm；周期结束自动解除" : "已解除执行 arm";
    publishState();
  }

  QualitySnapshot qualitySnapshot()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    QualitySnapshot snapshot = quality_;
    if (diagnostics_received_.nanoseconds() > 0) {
      snapshot.data_age_s = std::max(0.0, (now() - diagnostics_received_).seconds());
    }
    return snapshot;
  }

  std::optional<TargetData> targetSnapshot()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!target_.valid) {
      return std::nullopt;
    }
    return target_;
  }

  std::vector<Eigen::Vector3d> observedDirectionsSnapshot()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return observed_directions_;
  }

  std::optional<RefinedData> refinedSnapshot()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!refined_.valid) {
      return std::nullopt;
    }
    return refined_;
  }

  std::string graspDecisionTargetSnapshot()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return grasp_decision_target_id_;
  }

  bool cycleTargetReady(const std::string & target_id, std::string & reason)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (target_.id != target_id) {
      reason = "selected_target_changed";
      return false;
    }
    if (!target_.valid) {
      reason = "selected_target_not_observed";
      return false;
    }
    if ((now() - target_.received).seconds() > target_observation_max_age_s_) {
      reason = "selected_target_stale";
      return false;
    }
    return true;
  }

  bool safetyReady(std::string & reason)
  {
    if (!require_robot_status_) {
      return true;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!robot_status_valid_) {
      reason = "robot_status_missing";
      return false;
    }
    if ((now() - robot_status_received_).seconds() > robot_status_max_age_s_) {
      reason = "robot_status_stale";
      return false;
    }
    if (robot_status_.e_stopped != 0 || robot_status_.in_error != 0 ||
      robot_status_.drives_powered == 0 || robot_status_.motion_possible == 0)
    {
      reason = "robot_status_not_motion_ready";
      return false;
    }
    return true;
  }

  std::optional<Eigen::Isometry3d> lookupTransform(
    const std::string & target, const std::string & source)
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(
        target, source, tf2::TimePointZero, 1s);
      return tf2::transformToEigen(transform.transform);
    } catch (const tf2::TransformException & error) {
      RCLCPP_ERROR(
        get_logger(), "TF %s <- %s 不可用: %s",
        target.c_str(), source.c_str(), error.what());
      return std::nullopt;
    }
  }

  bool planOrMoveWrist(
    const Eigen::Isometry3d & wrist_pose,
    const std::string & planner_id,
    bool execute,
    const std::string & label,
    bool allow_fallback)
  {
    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = base_frame_;
    target.header.stamp = now();
    target.pose = eigenToPose(wrist_pose);
    move_group_->setStartStateToCurrentState();
    move_group_->setPlanningPipelineId(pilz_pipeline_);
    move_group_->setPlannerId(planner_id);
    move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);
    move_group_->setPoseTarget(target, wrist_frame_);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group_->plan(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS &&
      planner_id == "LIN" && allow_fallback)
    {
      RCLCPP_WARN(
        get_logger(), "%s 的 LIN 规划失败，回退 OMPL PTP", label.c_str());
      move_group_->setPlanningPipelineId(fallback_pipeline_);
      move_group_->setPlannerId("");
      result = move_group_->plan(plan);
    }
    move_group_->clearPoseTargets();
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      return false;
    }
    if (!execute) {
      return true;
    }
    std::string safety_reason;
    if (!safetyReady(safety_reason)) {
      setState("FAILED", "执行前安全门失败: " + safety_reason);
      return false;
    }
    if (!cycle_target_id_.empty() &&
      !cycleTargetReady(cycle_target_id_, safety_reason))
    {
      setState("FAILED", "执行前目标安全门失败: " + safety_reason);
      return false;
    }
    return move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool planOrMoveCamera(
    const Eigen::Isometry3d & camera_pose,
    const std::string & planner_id,
    bool execute,
    const std::string & label,
    bool allow_fallback = true)
  {
    const auto wrist_from_camera = lookupTransform(wrist_frame_, camera_frame_);
    if (!wrist_from_camera) {
      return false;
    }
    const Eigen::Isometry3d wrist_pose = camera_pose * wrist_from_camera->inverse();
    return planOrMoveWrist(
      wrist_pose, planner_id, execute, label, allow_fallback);
  }

  bool planOrMoveTool(
    const Eigen::Isometry3d & tool_pose,
    const std::string & planner_id,
    const std::string & label)
  {
    const auto wrist_from_tool = lookupTransform(wrist_frame_, tool_frame_);
    if (!wrist_from_tool) {
      return false;
    }
    return planOrMoveWrist(
      tool_pose * wrist_from_tool->inverse(), planner_id, true, label, false);
  }

  bool callTrigger(
    const rclcpp::Client<Trigger>::SharedPtr & client,
    const std::string & label,
    bool required = true)
  {
    if (!client->wait_for_service(std::chrono::duration<double>(service_timeout_s_))) {
      if (required) {
        setState("FAILED", label + " 服务不可用");
      }
      return false;
    }
    auto future = client->async_send_request(std::make_shared<Trigger::Request>());
    if (future.wait_for(std::chrono::duration<double>(service_timeout_s_)) !=
      std::future_status::ready)
    {
      if (required) {
        setState("FAILED", label + " 服务超时");
      }
      return false;
    }
    if (!future.get()->success) {
      if (required) {
        setState("FAILED", label + " 拒绝请求");
      }
      return false;
    }
    return true;
  }

  bool commandToolClose()
  {
    if (!tool_enabled_) {
      setState("FAILED", "grasp.enabled=true 但 tool.enabled=false");
      return false;
    }
    if (!tool_io_client_->wait_for_service(
        std::chrono::duration<double>(service_timeout_s_)))
    {
      setState("FAILED", "末端工具 set_io 服务不可用");
      return false;
    }
    auto request = std::make_shared<aubo_msgs::srv::SetIO::Request>();
    request->fun = static_cast<int8_t>(tool_io_fun_);
    request->pin = static_cast<int8_t>(tool_io_pin_);
    request->state = static_cast<float>(tool_close_state_);
    auto future = tool_io_client_->async_send_request(request);
    if (future.wait_for(std::chrono::duration<double>(service_timeout_s_)) !=
      std::future_status::ready || !future.get()->success)
    {
      setState("FAILED", "末端工具关闭命令失败");
      return false;
    }
    return true;
  }

  bool waitForNewView(std::size_t previous_views)
  {
    std::unique_lock<std::mutex> lock(data_mutex_);
    return data_cv_.wait_for(
      lock, std::chrono::duration<double>(frame_wait_s_),
      [this, previous_views]() {
        return cancel_requested_.load() || quality_.captured_views > previous_views;
      }) && !cancel_requested_.load();
  }

  bool waitForRefined(const std::string & target_id)
  {
    std::unique_lock<std::mutex> lock(data_mutex_);
    return data_cv_.wait_for(
      lock, std::chrono::duration<double>(refined_timeout_s_),
      [this, &target_id]() {
        return cancel_requested_.load() ||
               (quality_.reconstruction_state == "READY" &&
               quality_.refined_target_id == target_id);
      }) && !cancel_requested_.load();
  }

  void registerBehaviorTreeNodes()
  {
    bt_factory_.registerSimpleAction(
      "PrepareCycle", [this](BT::TreeNode &) {return btPrepareCycle();});
    bt_factory_.registerSimpleCondition(
      "IsPlanOnly", [this](BT::TreeNode &) {
        return execution_enabled_ ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
      });
    bt_factory_.registerSimpleAction(
      "PlanObservationPreview", [this](BT::TreeNode &) {return btPlanPreview();});
    bt_factory_.registerSimpleAction(
      "AcquireReconstructionViews", [this](BT::TreeNode &) {return btAcquireViews();});
    bt_factory_.registerSimpleAction(
      "FinalizeAndValidate", [this](BT::TreeNode &) {return btFinalizeAndValidate();});
    bt_factory_.registerSimpleCondition(
      "IsGraspDisabled", [this](BT::TreeNode &) {
        return grasp_enabled_ ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
      });
    bt_factory_.registerSimpleAction(
      "ReportReadyForGrasp", [this](BT::TreeNode &) {return btReportReady();});
    bt_factory_.registerSimpleAction(
      "MTCApproachAndInsert", [this](BT::TreeNode &) {return btMtcApproachAndInsert();});
    bt_factory_.registerSimpleAction(
      "ActuateTool", [this](BT::TreeNode &) {return btActuateTool();});
    bt_factory_.registerSimpleAction(
      "MTCRetreat", [this](BT::TreeNode &) {return btMtcRetreat();});
    bt_factory_.registerSimpleAction(
      "CompleteTarget", [this](BT::TreeNode &) {return btCompleteTarget();});
  }

  BT::NodeStatus btFailure(const std::string & reason)
  {
    bt_failure_reason_ = reason;
    setState("FAILED", reason);
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus btPrepareCycle()
  {
    cycle_target_ = targetSnapshot();
    cycle_refined_.reset();
    cycle_candidates_.clear();
    if (!cycle_target_) {
      return btFailure("selected_target 在启动后失效");
    }
    cycle_target_id_ = cycle_target_->id;
    setState("PLAN_OBSERVATION", "行为树生成目标导向主动视点");
    const auto base_from_camera = lookupTransform(base_frame_, camera_frame_);
    if (!base_from_camera) {
      return btFailure("无法取得当前相机位姿");
    }
    cycle_candidates_ = view_planner_->generate(
      cycle_target_->center, base_from_camera->translation(), observedDirectionsSnapshot());
    publishViewMarkers(cycle_target_->center, cycle_candidates_);
    if (cycle_candidates_.empty()) {
      return btFailure("没有生成可用观察视点");
    }
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus btPlanPreview()
  {
    for (const auto & candidate : cycle_candidates_) {
      if (planOrMoveCamera(candidate.camera_pose, "PTP", false, candidate.label)) {
        cycle_terminal_state_ = "PLAN_READY";
        cycle_terminal_message_ = "行为树只规划预览成功；未发送任何运动";
        return BT::NodeStatus::SUCCESS;
      }
    }
    return btFailure("所有候选观察位姿均不可规划");
  }

  BT::NodeStatus btAcquireViews()
  {
    if (reset_reconstruction_on_start_ &&
      !callTrigger(reset_client_, "reset_reconstruction"))
    {
      return btFailure("重建重置失败");
    }
    int moves = 0;
    std::vector<std::string> attempted;
    while (!cancel_requested_.load() && moves < maximum_scan_moves_) {
      const GateResult finalize_gate = quality_gate_->readyToFinalize(qualitySnapshot());
      if (finalize_gate.allowed) {
        break;
      }
      const auto current_camera = lookupTransform(base_frame_, camera_frame_);
      if (!current_camera) {
        return btFailure("扫描中无法取得相机位姿");
      }
      cycle_candidates_ = view_planner_->generate(
        cycle_target_->center, current_camera->translation(), observedDirectionsSnapshot());
      publishViewMarkers(cycle_target_->center, cycle_candidates_);
      bool moved = false;
      for (const auto & candidate : cycle_candidates_) {
        if (std::find(attempted.begin(), attempted.end(), candidate.label) != attempted.end()) {
          continue;
        }
        attempted.push_back(candidate.label);
        std::string target_reason;
        if (!cycleTargetReady(cycle_target_id_, target_reason)) {
          return btFailure("目标身份/可见性安全门失败: " + target_reason);
        }
        const std::size_t before = qualitySnapshot().captured_views;
        setState("MOVE_TO_VIEW", candidate.label + " score=" + std::to_string(candidate.score));
        const std::string planner = moves == 0 ? "PTP" : "LIN";
        if (!planOrMoveCamera(candidate.camera_pose, planner, true, candidate.label)) {
          continue;
        }
        ++moves;
        moved = true;
        setState("WAIT_FRAME", "等待重建接收新的精确时刻 RGB-D 帧");
        if (!waitForNewView(before)) {
          RCLCPP_WARN(get_logger(), "视点到达但等待新重建帧超时");
        }
        break;
      }
      if (!moved) {
        return btFailure("剩余候选视点均不可达或规划失败");
      }
    }
    if (cancel_requested_.load()) {
      return BT::NodeStatus::FAILURE;
    }
    const GateResult gate = quality_gate_->readyToFinalize(qualitySnapshot());
    if (!gate.allowed) {
      return btFailure("达到扫描上限仍未收敛: " + gate.reason);
    }
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus btFinalizeAndValidate()
  {
    setState("FINALIZE", "视角覆盖达标，提取 TSDF 与精化几何");
    if (!callTrigger(finalize_client_, "finalize_reconstruction") ||
      !waitForRefined(cycle_target_id_))
    {
      return btFailure("finalize 后未收到同 ID 精化结果");
    }
    const GateResult gate = quality_gate_->readyToGrasp(qualitySnapshot());
    if (!gate.allowed || graspDecisionTargetSnapshot() != cycle_target_id_) {
      return btFailure("最终抓取质量门失败: " + gate.reason);
    }
    callTrigger(save_client_, "save_session", false);
    cycle_refined_ = refinedSnapshot();
    if (!cycle_refined_) {
      return btFailure("精化位姿数据不存在");
    }
    Eigen::Isometry3d entry_tool_pose = Eigen::Isometry3d::Identity();
    entry_tool_pose.translation() = cycle_refined_->entry;
    entry_tool_pose.linear() = ViewPlanner::toolOrientation(
      cycle_refined_->axis, cycle_target_->initial_pose.linear().col(0));
    const auto wrist_from_tool = lookupTransform(wrist_frame_, tool_frame_);
    if (!wrist_from_tool) {
      return btFailure("无法取得 wrist 到 tool 的变换");
    }
    cycle_entry_wrist_pose_ = entry_tool_pose * wrist_from_tool->inverse();
    cycle_travel_m_ = insertionTravel(*cycle_refined_);
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus btReportReady()
  {
    cycle_terminal_state_ = "READY_FOR_GRASP";
    cycle_terminal_message_ = "精化质量通过；grasp.enabled=false，未执行接触动作";
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus btMtcApproachAndInsert()
  {
    std::string reason;
    if (!safetyReady(reason) || !cycleTargetReady(cycle_target_id_, reason)) {
      return btFailure("MTC 执行前安全门失败: " + reason);
    }
    setState("MTC_APPROACH_INSERT", "MTC: OMPL 避障到入口，再沿精化轴直线插入");
    const auto result = grasp_task_->approachAndInsert(
      cycle_entry_wrist_pose_, cycle_refined_->axis, cycle_travel_m_, true);
    if (result.execution_started) {
      contact_recovery_required_.store(true);
    }
    if (!result.success) {
      return btFailure("MTC 接近/插入失败: " + result.reason);
    }
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus btActuateTool()
  {
    setState("ACTUATE_TOOL", "触发末端工具抓取/切割");
    if (!commandToolClose()) {
      const auto retreat = grasp_task_->retreat(
        cycle_refined_->axis, cycle_travel_m_, true);
      if (retreat.success) {
        contact_recovery_required_.store(false);
      }
      return btFailure(
        "末端工具失败；MTC 紧急撤离结果: " + retreat.reason);
    }
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus btMtcRetreat()
  {
    setState("MTC_RETREAT", "MTC 沿插入反方向保持直线撤离");
    const auto result = grasp_task_->retreat(
      cycle_refined_->axis, cycle_travel_m_, true);
    if (!result.success) {
      return btFailure("MTC 抓取后撤离失败，需要人工处理: " + result.reason);
    }
    contact_recovery_required_.store(false);
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus btCompleteTarget()
  {
    if (complete_target_after_retreat_ &&
      !callTrigger(complete_client_, "complete_selected_target"))
    {
      return btFailure("抓取已完成但目标计划推进失败");
    }
    cycle_terminal_state_ = "SUCCEEDED";
    cycle_terminal_message_ = "行为树完成主动视觉、MTC 抓取和撤离";
    return BT::NodeStatus::SUCCESS;
  }

  void runCycle()
  {
    const auto finish = [this](const std::string & state, const std::string & message) {
        execution_armed_.store(false);
        running_.store(false);
        setState(state, message);
      };
    bt_failure_reason_.clear();
    cycle_terminal_state_ = "SUCCEEDED";
    cycle_terminal_message_ = "行为树周期完成";
    try {
      if (behavior_tree_xml_.empty()) {
        finish("FAILED", "behavior_tree.xml 参数为空");
        return;
      }
      BT::Tree tree = bt_factory_.createTreeFromFile(behavior_tree_xml_);
      const BT::NodeStatus result = tree.tickOnce();
      if (contact_recovery_required_.load()) {
        finish(
          "RECOVERY_REQUIRED",
          "接触阶段取消或撤离未确认；保持停止，须现场人工撤离后确认恢复");
      } else if (cancel_requested_.load()) {
        finish("CANCELED", "用户取消主动视觉/MTC 周期");
      } else if (result == BT::NodeStatus::SUCCESS) {
        finish(cycle_terminal_state_, cycle_terminal_message_);
      } else {
        finish(
          "FAILED", bt_failure_reason_.empty() ? "行为树返回 FAILURE" : bt_failure_reason_);
      }
    } catch (const std::exception & error) {
      if (contact_recovery_required_.load()) {
        finish(
          "RECOVERY_REQUIRED",
          std::string("接触阶段异常，须现场人工撤离: ") + error.what());
      } else {
        finish("FAILED", std::string("行为树加载或执行异常: ") + error.what());
      }
    }
  }

  void publishViewMarkers(
    const Eigen::Vector3d & target,
    const std::vector<ViewCandidate> & candidates)
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);
    const std::size_t limit = std::min<std::size_t>(candidates.size(), 24U);
    for (std::size_t index = 0; index < limit; ++index) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = base_frame_;
      marker.header.stamp = now();
      marker.ns = "candidate_views";
      marker.id = static_cast<int>(index);
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;
      geometry_msgs::msg::Point start;
      start.x = candidates[index].camera_pose.translation().x();
      start.y = candidates[index].camera_pose.translation().y();
      start.z = candidates[index].camera_pose.translation().z();
      geometry_msgs::msg::Point end;
      end.x = target.x();
      end.y = target.y();
      end.z = target.z();
      marker.points = {start, end};
      marker.scale.x = 0.003;
      marker.scale.y = 0.007;
      marker.scale.z = 0.010;
      marker.color.r = static_cast<float>(1.0 - candidates[index].score);
      marker.color.g = static_cast<float>(candidates[index].score);
      marker.color.b = 0.25F;
      marker.color.a = 0.75F;
      array.markers.push_back(marker);
    }
    marker_pub_->publish(array);
  }

  void setState(const std::string & state, const std::string & message)
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state_json_ = {
        {"state", state},
        {"message", message},
        {"running", running_.load()},
        {"execution_enabled", execution_enabled_},
        {"execution_armed", execution_armed_.load()},
        {"grasp_enabled", grasp_enabled_},
        {"contact_recovery_required", contact_recovery_required_.load()},
        {"target_id", cycle_target_id_},
      };
      const QualitySnapshot snapshot = qualitySnapshot();
      state_json_["quality"] = {
        {"selected_target_id", snapshot.selected_target_id},
        {"reconstruction_target_id", snapshot.reconstruction_target_id},
        {"refined_target_id", snapshot.refined_target_id},
        {"captured_views", snapshot.captured_views},
        {"max_baseline_deg", snapshot.max_baseline_deg},
        {"mean_nearest_baseline_deg", snapshot.mean_nearest_baseline_deg},
        {"mean_depth_ratio", snapshot.mean_depth_ratio},
        {"refined_rmse_m", snapshot.refined_rmse_m},
        {"refined_inlier_ratio", snapshot.refined_inlier_ratio},
        {"grasp_allowed", snapshot.grasp_allowed},
      };
    }
    publishState();
    RCLCPP_INFO(get_logger(), "[%s] %s", state.c_str(), message.c_str());
  }

  void publishState()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_json_["running"] = running_.load();
    state_json_["execution_armed"] = execution_armed_.load();
    state_json_["contact_recovery_required"] = contact_recovery_required_.load();
    std_msgs::msg::String message;
    message.data = state_json_.dump();
    status_pub_->publish(message);
  }

  std::string base_frame_;
  std::string wrist_frame_;
  std::string camera_frame_;
  std::string tool_frame_;
  std::string planning_group_;
  std::string pilz_pipeline_;
  std::string fallback_pipeline_;
  std::string mtc_free_space_planner_;
  std::string behavior_tree_xml_;
  double planning_time_s_{5.0};
  int planning_attempts_{5};
  double velocity_scaling_{0.05};
  double acceleration_scaling_{0.05};
  double mtc_cartesian_step_m_{0.005};
  double mtc_cartesian_precision_m_{0.001};
  int mtc_max_solutions_{5};
  int maximum_scan_moves_{8};
  double frame_wait_s_{3.0};
  bool execution_enabled_{false};
  bool require_robot_status_{true};
  double robot_status_max_age_s_{1.0};
  double target_observation_max_age_s_{1.0};
  bool reset_reconstruction_on_start_{false};
  bool grasp_enabled_{false};
  double neck_margin_m_{0.015};
  double minimum_travel_m_{0.02};
  double maximum_travel_m_{0.20};
  bool complete_target_after_retreat_{true};
  bool tool_enabled_{false};
  int tool_io_fun_{3};
  int tool_io_pin_{0};
  double tool_close_state_{1.0};
  double service_timeout_s_{3.0};
  double refined_timeout_s_{10.0};

  std::unique_ptr<ViewPlanner> view_planner_;
  std::unique_ptr<QualityGate> quality_gate_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::unique_ptr<GraspTask> grasp_task_;
  BT::BehaviorTreeFactory bt_factory_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex data_mutex_;
  std::condition_variable data_cv_;
  TargetData target_;
  RefinedData refined_;
  QualitySnapshot quality_;
  std::vector<Eigen::Vector3d> observed_directions_;
  std::string grasp_decision_target_id_;
  rclcpp::Time diagnostics_received_{0, 0, RCL_ROS_TIME};
  aubo_msgs::msg::RobotStatus robot_status_;
  rclcpp::Time robot_status_received_{0, 0, RCL_ROS_TIME};
  bool robot_status_valid_{false};

  std::mutex state_mutex_;
  json state_json_;
  std::string cycle_target_id_;
  std::string bt_failure_reason_;
  std::string cycle_terminal_state_;
  std::string cycle_terminal_message_;
  std::optional<TargetData> cycle_target_;
  std::optional<RefinedData> cycle_refined_;
  std::vector<ViewCandidate> cycle_candidates_;
  Eigen::Isometry3d cycle_entry_wrist_pose_{Eigen::Isometry3d::Identity()};
  double cycle_travel_m_{0.0};
  std::atomic_bool running_{false};
  std::atomic_bool cancel_requested_{false};
  std::atomic_bool execution_armed_{false};
  std::atomic_bool contact_recovery_required_{false};
  std::thread worker_;

  rclcpp::Subscription<peach_pose_msgs::msg::PeachTargetObservationArray>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr diagnostics_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr decision_sub_;
  rclcpp::Subscription<peach_pose_msgs::msg::BagGraspCandidateArray>::SharedPtr
    refined_pose_sub_;
  rclcpp::Subscription<peach_pose_msgs::msg::BagFittingArray>::SharedPtr refined_diag_sub_;
  rclcpp::Subscription<aubo_msgs::msg::RobotStatus>::SharedPtr robot_status_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Service<Trigger>::SharedPtr start_service_;
  rclcpp::Service<Trigger>::SharedPtr preview_approach_service_;
  rclcpp::Service<Trigger>::SharedPtr preview_full_contact_service_;
  rclcpp::Service<Trigger>::SharedPtr cancel_service_;
  rclcpp::Service<Trigger>::SharedPtr recovery_service_;
  rclcpp::Service<Trigger>::SharedPtr query_service_;
  rclcpp::Service<SetBool>::SharedPtr arm_service_;
  rclcpp_action::Server<RunTargetCycle>::SharedPtr cycle_action_server_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rclcpp::Client<Trigger>::SharedPtr reset_client_;
  rclcpp::Client<Trigger>::SharedPtr finalize_client_;
  rclcpp::Client<Trigger>::SharedPtr save_client_;
  rclcpp::Client<Trigger>::SharedPtr complete_client_;
  rclcpp::Client<aubo_msgs::srv::SetIO>::SharedPtr tool_io_client_;
};

}  // namespace peach_approach_grasp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<peach_approach_grasp::ApproachGraspNode>();
  try {
    node->initializeMoveIt();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      node->get_logger(), "MoveIt 初始化失败: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
