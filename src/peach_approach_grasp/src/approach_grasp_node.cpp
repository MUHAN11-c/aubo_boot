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
// 节点本体：参数、接口创建、订阅回调薄壳、runCycle、状态发布与进程入口。
// 运动接口见 motion_interface.cpp，行为树节点见 bt_nodes.cpp，
// action 服务端与周期控制服务见 cycle_action.cpp。
#include <algorithm>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.hpp>

#include "approach_grasp_node_impl.hpp"
#include "eigen_conversions.hpp"

namespace peach_approach_grasp
{

ApproachGraspNode::ApproachGraspNode()
: Node("peach_approach_grasp_node"),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  cache_([this]() {return now().seconds();})
{
  declareParameters();
  loadParameters();
  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&ApproachGraspNode::onParameters, this, std::placeholders::_1));
  createInterfaces();
  setState(CycleState::IDLE, "等待 start_cycle；默认只规划，不执行运动");
}

ApproachGraspNode::~ApproachGraspNode()
{
  // 先置取消标志并唤醒等待，再回收 worker 与 action 线程，避免 shutdown 后
  // 线程仍访问已销毁成员（use-after-free）。
  cancel_requested_.store(true);
  if (move_group_) {
    move_group_->stop();
  }
  if (grasp_task_) {
    grasp_task_->cancel();
  }
  cache_.notifyAll();
  if (worker_.joinable()) {
    worker_.join();
  }
  if (action_thread_.joinable()) {
    action_thread_.join();
  }
}

void ApproachGraspNode::initializeMoveIt()
{
  move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), planning_group_);
  move_group_->setPoseReferenceFrame(base_frame_);
  move_group_->setPlanningTime(planning_time_s_);
  move_group_->setNumPlanningAttempts(planning_attempts_);
  move_group_->setMaxVelocityScalingFactor(transit_velocity_scaling_);
  move_group_->setMaxAccelerationScalingFactor(transit_acceleration_scaling_);
  move_group_->allowReplanning(true);
  GraspTaskConfig task_config;
  task_config.planning_group = planning_group_;
  task_config.tip_frame = tip_frame_;
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
    "主动视觉靠近节点 ready: group=%s base=%s tip=%s camera=%s "
    "execution=%s grasp=%s",
    planning_group_.c_str(), base_frame_.c_str(), tip_frame_.c_str(),
    camera_frame_.c_str(), execution_enabled_.load() ? "enabled" : "plan_only",
    grasp_enabled_.load() ? "enabled" : "disabled");
}

void ApproachGraspNode::declareParameters()
{
  declare_parameter("frames.base", "base_link");
  // 规划/IK 末端连杆：MoveIt 组 tip_link（URDF 内建 tcp，见 aubo_description tcp.xacro）。
  declare_parameter("frames.tip", "tcp");
  declare_parameter("frames.camera", "camera_depth_optical_frame");
  declare_parameter("frames.tool", "tcp");
  declare_parameter("moveit.planning_group", "manipulator_e5");
  declare_parameter("moveit.planning_time_s", 5.0);
  declare_parameter("moveit.planning_attempts", 5);
  declare_parameter("moveit.velocity_scaling", 0.05);
  declare_parameter("moveit.acceleration_scaling", 0.05);
  declare_parameter("moveit.transit_velocity_scaling", 0.05);
  declare_parameter("moveit.transit_acceleration_scaling", 0.05);
  declare_parameter("moveit.pilz_pipeline", "pilz_industrial_motion_planner");
  declare_parameter("moveit.fallback_pipeline", "ompl");
  declare_parameter("moveit.mtc_free_space_planner", "RRTConnectkConfigDefault");
  declare_parameter("moveit.mtc_cartesian_step_m", 0.005);
  declare_parameter("moveit.mtc_cartesian_precision_m", 0.001);
  declare_parameter("moveit.mtc_max_solutions", 5);
  // 全局拍照位姿的 SRDF 命名状态（go_to_photo_pose 服务的目标）。
  declare_parameter("photo_pose_named_target", "global_photo_pose");
  declare_parameter("behavior_tree.xml", "");
  declare_parameter("scan.observation_radius_m", 0.40);
  declare_parameter("scan.minimum_radius_m", 0.32);
  declare_parameter("scan.azimuth_step_deg", 12.0);
  declare_parameter("scan.azimuth_limit_deg", 36.0);
  declare_parameter("scan.elevation_step_deg", 8.0);
  declare_parameter("scan.elevation_limit_deg", 16.0);
  declare_parameter("scan.preferred_baseline_deg", 15.0);
  declare_parameter("scan.radial_step_m", 0.015);
  declare_parameter("scan.candidate_layers", 3);
  declare_parameter("scan.views_to_minimum_radius", 5);
  declare_parameter("scan.maximum_moves", 8);
  declare_parameter("scan.frame_wait_s", 6.0);
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
  declare_parameter("execution.target_observation_max_age_s", 3.0);
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

void ApproachGraspNode::loadParameters()
{
  base_frame_ = get_parameter("frames.base").as_string();
  tip_frame_ = get_parameter("frames.tip").as_string();
  camera_frame_ = get_parameter("frames.camera").as_string();
  tool_frame_ = get_parameter("frames.tool").as_string();
  planning_group_ = get_parameter("moveit.planning_group").as_string();
  planning_time_s_ = get_parameter("moveit.planning_time_s").as_double();
  planning_attempts_ = get_parameter("moveit.planning_attempts").as_int();
  velocity_scaling_ = get_parameter("moveit.velocity_scaling").as_double();
  acceleration_scaling_ = get_parameter("moveit.acceleration_scaling").as_double();
  transit_velocity_scaling_ =
    get_parameter("moveit.transit_velocity_scaling").as_double();
  transit_acceleration_scaling_ =
    get_parameter("moveit.transit_acceleration_scaling").as_double();
  pilz_pipeline_ = get_parameter("moveit.pilz_pipeline").as_string();
  fallback_pipeline_ = get_parameter("moveit.fallback_pipeline").as_string();
  mtc_free_space_planner_ = get_parameter("moveit.mtc_free_space_planner").as_string();
  mtc_cartesian_step_m_ = get_parameter("moveit.mtc_cartesian_step_m").as_double();
  mtc_cartesian_precision_m_ =
    get_parameter("moveit.mtc_cartesian_precision_m").as_double();
  mtc_max_solutions_ = get_parameter("moveit.mtc_max_solutions").as_int();
  photo_pose_named_target_ = get_parameter("photo_pose_named_target").as_string();
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

  SafetyGateConfig safety_config;
  safety_config.require_robot_status =
    get_parameter("execution.require_robot_status").as_bool();
  safety_config.robot_status_max_age_s =
    get_parameter("execution.robot_status_max_age_s").as_double();
  safety_config.target_observation_max_age_s =
    get_parameter("execution.target_observation_max_age_s").as_double();
  safety_gate_ = std::make_unique<SafetyGate>(
    safety_config, [this]() {return now().seconds();});

  execution_enabled_.store(get_parameter("execution.enabled").as_bool());
  reset_reconstruction_on_start_ =
    get_parameter("execution.reset_reconstruction_on_start").as_bool();
  grasp_enabled_.store(get_parameter("grasp.enabled").as_bool());
  neck_margin_m_ = get_parameter("grasp.neck_margin_m").as_double();
  minimum_travel_m_ = get_parameter("grasp.minimum_travel_m").as_double();
  maximum_travel_m_ = get_parameter("grasp.maximum_travel_m").as_double();
  complete_target_after_retreat_ =
    get_parameter("grasp.complete_target_after_retreat").as_bool();
  tool_enabled_.store(get_parameter("tool.enabled").as_bool());
  tool_io_fun_ = get_parameter("tool.io_fun").as_int();
  tool_io_pin_ = get_parameter("tool.io_pin").as_int();
  tool_close_state_ = get_parameter("tool.close_state").as_double();
  service_timeout_s_ = get_parameter("timeouts.service_s").as_double();
  refined_timeout_s_ = get_parameter("timeouts.refined_s").as_double();
}

rcl_interfaces::msg::SetParametersResult ApproachGraspNode::onParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  if (running_.load()) {
    result.reason = "周期运行中不能修改运动策略";
    return result;
  }
  bool execution = execution_enabled_.load();
  bool grasp = grasp_enabled_.load();
  bool tool = tool_enabled_.load();
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
  execution_enabled_.store(execution);
  grasp_enabled_.store(grasp);
  tool_enabled_.store(tool);
  if (!execution_enabled_.load()) {execution_armed_.store(false);}
  result.successful = true;
  result.reason = "运动策略已动态更新";
  publishState();
  return result;
}

void ApproachGraspNode::createInterfaces()
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
      std::placeholders::_1, std::placeholders::_2, false));
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
  photo_pose_service_ = create_service<Trigger>(
    "~/go_to_photo_pose",
    std::bind(
      &ApproachGraspNode::onGoToPhotoPose, this,
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

void ApproachGraspNode::onTargets(
  const peach_pose_msgs::msg::PeachTargetObservationArray::SharedPtr message)
{
  auto selected = std::find_if(
    message->observations.begin(), message->observations.end(),
    [&message](const auto & item) {
      return item.target_id == message->selected_target_id;
    });
  if (selected == message->observations.end()) {
    return;
  }
  // 消息字段提取（含 ROS 类型）留在节点薄壳，ID 一致性调和委托 cache_ 纯核。
  SelectedTargetUpdate update;
  update.selected_id = message->selected_target_id;
  update.harvest_run_id = message->harvest_run_id;
  update.observed =
    selected->tracking_status == peach_pose_msgs::msg::PeachTargetObservation::OBSERVED &&
    selected->candidate.status != peach_pose_msgs::msg::BagGraspCandidate::REJECT;
  update.bottom = pointToEigen(selected->candidate.bag_bottom);
  update.neck = pointToEigen(selected->candidate.bag_neck);
  update.axis = vectorToEigen(selected->candidate.translation_direction);
  update.entry_pose = poseToEigen(selected->candidate.entry_pose);
  update.suggested_travel_m = selected->candidate.suggested_travel_m;
  cache_.updateSelectedTarget(update);
}

void ApproachGraspNode::onDiagnostics(const std_msgs::msg::String::SharedPtr message)
{
  try {
    const json value = json::parse(message->data);
    ReconstructionDiagnosticsUpdate update;
    update.target_id = value.value("target_id", "");
    update.state = value.value("state", "IDLE");
    update.captured_views = value.value("captured_views", 0U);
    const auto coverage = value.value("view_coverage", json::object());
    update.max_baseline_deg = coverage.value("max_baseline_deg", 0.0);
    update.mean_nearest_baseline_deg =
      coverage.value("mean_nearest_baseline_deg", 0.0);
    update.mean_depth_ratio = coverage.value("valid_depth_ratio_mean", 0.0);
    for (const auto & view : coverage.value("views", json::array())) {
      const auto direction = view.value(
        "direction_target_to_camera", std::vector<double>{});
      if (direction.size() == 3U) {
        update.view_directions.emplace_back(direction[0], direction[1], direction[2]);
      }
    }
    cache_.updateReconstructionDiagnostics(update);
  } catch (const std::exception & error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "无法解析 reconstruction diagnostics: %s", error.what());
  }
}

void ApproachGraspNode::onDecision(const std_msgs::msg::String::SharedPtr message)
{
  try {
    const json value = json::parse(message->data);
    const std::string decision_id = value.value("target_id", "");
    if (!cache_.updateGraspDecision(decision_id, value.value("allowed", false))) {
      RCLCPP_WARN(
        get_logger(), "忽略非当前目标的 grasp_decision: expected=%s actual=%s",
        cache_.targetGateSample().id.c_str(), decision_id.c_str());
    }
  } catch (const std::exception & error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "无法解析 reconstruction grasp_decision: %s", error.what());
  }
}

void ApproachGraspNode::onRefinedPose(
  const peach_pose_msgs::msg::BagGraspCandidateArray::SharedPtr message)
{
  RefinedPoseUpdate update;
  if (message->candidates.empty()) {
    update.clear = true;
    cache_.updateRefinedPose(update);
    return;
  }
  const auto & candidate = message->candidates.front();
  update.target_id = candidate.target_id;
  update.entry = pointToEigen(candidate.entry_pose.position);
  update.bottom = pointToEigen(candidate.bag_bottom);
  update.neck = pointToEigen(candidate.bag_neck);
  update.axis = vectorToEigen(candidate.translation_direction);
  update.suggested_travel_m = candidate.suggested_travel_m;
  update.accepted = candidate.status == peach_pose_msgs::msg::BagGraspCandidate::ACCEPT;
  if (!cache_.updateRefinedPose(update)) {
    RCLCPP_WARN(
      get_logger(), "忽略非当前目标的 refined pose: expected=%s actual=%s",
      cache_.targetGateSample().id.c_str(), candidate.target_id.c_str());
  }
}

void ApproachGraspNode::onRefinedDiagnostics(
  const peach_pose_msgs::msg::BagFittingArray::SharedPtr message)
{
  if (message->fittings.empty()) {
    return;
  }
  const auto & fitting = message->fittings.front();
  RefinedFittingUpdate update;
  update.target_id = fitting.target_id;
  update.is_fruit = fitting.target_kind == "fruit";
  update.sphere_rms_m = fitting.sphere_rms_m;
  update.sphere_inlier_ratio = fitting.sphere_inlier_ratio;
  update.cylinder_rms_m = fitting.cylinder_rms_m;
  update.cylinder_inlier_ratio = fitting.cylinder_inlier_ratio;
  update.accepted = fitting.status == peach_pose_msgs::msg::BagFitting::ACCEPT;
  if (!cache_.updateRefinedFitting(update)) {
    RCLCPP_WARN(
      get_logger(), "忽略非当前目标的 refined diagnostics: expected=%s actual=%s",
      cache_.expectedFittingTargetId().c_str(), fitting.target_id.c_str());
  }
}

void ApproachGraspNode::runCycle()
{
  const auto finish = [this](CycleState state, const std::string & message) {
      execution_armed_.store(false);
      // 先落终态再解除 running：executeAction 以 running==false 作为周期结束信号，
      // 这样它读到的状态一定是终态而不是上一个中间态。
      setState(state, message);
      running_.store(false);
      publishState();
    };
  bt_failure_reason_.clear();
  cycle_terminal_state_ = CycleState::SUCCEEDED;
  cycle_terminal_message_ = "行为树周期完成";
  try {
    if (behavior_tree_xml_.empty()) {
      finish(CycleState::FAILED, "behavior_tree.xml 参数为空");
      return;
    }
    BT::Tree tree = bt_factory_.createTreeFromFile(behavior_tree_xml_);
    const BT::NodeStatus result = tree.tickOnce();
    if (contact_recovery_required_.load()) {
      finish(
        CycleState::RECOVERY_REQUIRED,
        "接触阶段取消或撤离未确认；保持停止，须现场人工撤离后确认恢复");
    } else if (cancel_requested_.load()) {
      finish(CycleState::CANCELED, "用户取消主动视觉/MTC 周期");
    } else if (result == BT::NodeStatus::SUCCESS) {
      finish(cycle_terminal_state_, cycle_terminal_message_);
    } else {
      finish(
        CycleState::FAILED,
        bt_failure_reason_.empty() ? "行为树返回 FAILURE" : bt_failure_reason_);
    }
  } catch (const std::exception & error) {
    if (contact_recovery_required_.load()) {
      finish(
        CycleState::RECOVERY_REQUIRED,
        std::string("接触阶段异常，须现场人工撤离: ") + error.what());
    } else {
      finish(CycleState::FAILED, std::string("行为树加载或执行异常: ") + error.what());
    }
  }
}

void ApproachGraspNode::setState(CycleState state, const std::string & message)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    // 枚举是唯一权威状态；字符串仅投影进 JSON 供 dashboard/web 只读消费。
    current_state_ = state;
    state_json_ = {
      {"state", toString(state)},
      {"message", message},
      {"running", running_.load()},
      {"execution_enabled", execution_enabled_.load()},
      {"execution_armed", execution_armed_.load()},
      {"grasp_enabled", grasp_enabled_.load()},
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
  RCLCPP_INFO(get_logger(), "[%s] %s", toString(state).c_str(), message.c_str());
}

void ApproachGraspNode::publishState()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_json_["running"] = running_.load();
  state_json_["execution_armed"] = execution_armed_.load();
  state_json_["contact_recovery_required"] = contact_recovery_required_.load();
  std_msgs::msg::String message;
  message.data = state_json_.dump();
  status_pub_->publish(message);
}

}  // namespace peach_approach_grasp
