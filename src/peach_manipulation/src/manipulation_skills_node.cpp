// 功能：节点外壳。Lifecycle、参数、订阅/服务/动作创建、状态发布。
// 运动入口见 motion.cpp；动作周期与授权矩阵见 cycle.cpp；阶段执行器见 stages.cpp。
// 生命周期回调职责表（A8 / Robotics_Tutorial 2.16-1）：
//   on_configure  ：依赖链复核（启动覆盖值不走 on-set 钩子）→ loadParameters
//     实现装配 → createInterfaces（订阅/服务/action/client）→ initializeMoveIt
//     （MoveIt/MTC 资源）；任一步抛异常即回滚并 FAILURE，不进 Active。
//   on_activate   ：只做快速可预测切换——开放运动输出权限（原子标志）。
//   on_deactivate ：先关输出权限（撤 arm、拒新 goal），再按 CANCEL_NOW 等价路径
//     真取消活动周期（置取消标志、stop MoveIt/MTC、唤醒等待、回收线程，action
//     侧按 outcome=CANCELED 终局上报）后落 Inactive；接触段 recovery 锁
//     （contact_recovery_required_）跨停用保持，语义不变。
//   on_cleanup    ：释放 MoveIt/MTC/订阅/服务/action/client 全部资源，
//     回 Unconfigured（参数声明与验证钩子保留，可再次 configure）。
//   on_shutdown/on_error：关输出权限 + 取消活动周期 + 释放资源。
#include "peach_manipulation/manipulation_skills_node.hpp"
#include <algorithm>
#include <chrono>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include "peach_manipulation/eigen_conversions.hpp"

namespace peach_manipulation
{

ManipulationSkillsNode::ManipulationSkillsNode(const rclcpp::NodeOptions & options)
: LifecycleNode("peach_manipulation_node", options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  cache_([this]() {return now().seconds();})
{
  // MoveIt/MTC 伴随节点：同名（launch 参数文件按节点名匹配，同名即继承同一套
  // robot_description/管线参数；进程内 __node 重映射对两者一致生效）；
  // 覆盖参数自动声明（robot_description 等非本节点参数由 MoveIt/MTC 直接读）；
  // 关闭参数服务/参数事件，避免与本节点的参数服务撞名抢路由。
  rclcpp::NodeOptions moveit_options;
  moveit_options.use_global_arguments(options.use_global_arguments());
  moveit_options.parameter_overrides(options.parameter_overrides());
  moveit_options.automatically_declare_parameters_from_overrides(true);
  moveit_options.start_parameter_services(false);
  moveit_options.start_parameter_event_publisher(false);
  moveit_node_ = std::make_shared<rclcpp::Node>(
    "peach_manipulation_moveit", moveit_options);
  // ParamListener 构造即声明全部参数并做启动校验（yaml 覆盖值非法时抛
  // InvalidParameterValueException 直接启动失败），内置范围校验随每次 set 生效；
  // 声明/默认值/范围的单一事实源为 config/manipulation_parameters.yaml。
  param_listener_ = std::make_shared<peach_manipulation_node::ParamListener>(
    get_node_parameters_interface(), get_logger());
  loadParameters();
  // on-set 验证钩子（无副作用，见 onParameters）：运行中拒改 +
  // execution→grasp→tool 依赖链。rclcpp 的 on-set 回调按注册逆序调用，
  // 本钩子后注册先执行，拒绝时监听器的快照更新不会触发。
  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&ManipulationSkillsNode::onParameters, this, std::placeholders::_1));
  // 全量生效放 post-set（参数实际写入之后；P0-6：此前被静默吞掉）。
  post_parameter_callback_handle_ = add_post_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> &) {
      // loadParameters 会重建 view_planner_/quality_gate_/safety_gate_（以及
      // motion_，若 MoveIt 已初始化）（非线程安全），仅空闲时重载；运行中的
      // 改参已被 onParameters 前置拒绝，此处 !running_ 为兜底（默认互斥回调组，
      // 不与订阅回调并发）。
      if (!running_.load()) {
        loadParameters();
      }
      // execution 关闭时解除人工 arm（原 onParameters 成功路径语义，移到
      // post-set 后读到的必为写入后的值）。
      if (!execution_enabled_.load()) {execution_armed_.store(false);}
      publishState();
    });
}

CallbackReturn ManipulationSkillsNode::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    // 使能依赖链复核：启动覆盖值不经 on-set 钩子，违链（grasp/tool 越级开启）
    // 必须拦在 Inactive 之前；运行期动态改参仍由 onParameters 逐批把关。
    // 与后续步骤同 try：get_params 对非法覆盖值抛异常时也走 FAILURE 回滚，
    // 不让异常逸出生命周期转换回调。
    const auto params = param_listener_->get_params();
    if ((params.grasp.enabled && !params.execution.enabled) ||
      (params.tool.enabled && !params.grasp.enabled))
    {
      RCLCPP_ERROR(
        get_logger(),
        "configure 失败：使能依赖必须满足 execution→grasp→tool"
        "（execution=%d grasp=%d tool=%d）",
        params.execution.enabled, params.grasp.enabled, params.tool.enabled);
      return CallbackReturn::FAILURE;
    }
    loadParameters();
    createInterfaces();
    // MoveIt/MTC 资源分配放 configure（activate 只做快速切换）；
    // 机器人模型缺失/规划组不存在等在此抛异常 → FAILURE。
    initializeMoveIt();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "configure 失败: %s", error.what());
    releaseResources();
    return CallbackReturn::FAILURE;
  }
  setState(CycleState::IDLE, "已配置；activate 后才开放运动输出权限");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ManipulationSkillsNode::on_activate(const rclcpp_lifecycle::State &)
{
  // 快速可预测切换：开放运动输出权限（原子标志）+ 激活状态发布者，
  // 不做任何资源分配/等待。
  motion_output_permitted_.store(true);
  status_pub_->on_activate();
  marker_pub_->on_activate();
  grasp_hyp_pub_->on_activate();
  RCLCPP_INFO(get_logger(), "节点已激活：运动输出权限开放");
  publishState();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ManipulationSkillsNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  // 先关输出权限（此后一切运动类入口拒绝），再按 CANCEL_NOW 等价路径取消
  // 活动周期并回收线程；contact_recovery_required_ 不随停用清除。
  closeMotionOutputAndCancel();
  setState(CycleState::IDLE, "节点已停用：运动输出权限关闭，活动周期已按取消路径终止");
  status_pub_->on_deactivate();
  marker_pub_->on_deactivate();
  grasp_hyp_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ManipulationSkillsNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  releaseResources();
  RCLCPP_INFO(get_logger(), "已清理：MoveIt/MTC/订阅/服务资源全部释放，回 Unconfigured");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ManipulationSkillsNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  closeMotionOutputAndCancel();
  releaseResources();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ManipulationSkillsNode::on_error(const rclcpp_lifecycle::State &)
{
  // 进入 ErrorProcessing 即关闭运动输出（motionOutputAllowed 随之拒绝一切
  // 运动入口），随后按 shutdown 同路径释放资源。
  closeMotionOutputAndCancel();
  releaseResources();
  return CallbackReturn::SUCCESS;
}

bool ManipulationSkillsNode::motionOutputAllowed(std::string & why) const
{
  if (!motion_output_permitted_.load()) {
    why = "节点未处于 Active 态：运动输出权限仅在 Active 开放";
    return false;
  }
  return true;
}

void ManipulationSkillsNode::requestCancelAll()
{
  cancel_requested_.store(true);
  if (move_group_) {
    move_group_->stop();
  }
  if (grasp_task_) {
    grasp_task_->cancel();
  }
  cache_.notifyAll();
}

void ManipulationSkillsNode::closeMotionOutputAndCancel()
{
  // 顺序即语义：先关权限/撤 arm（拒新入口），再取消活动周期并唤醒所有等待。
  motion_output_permitted_.store(false);
  execution_armed_.store(false);
  requestCancelAll();
  // 先回收 worker（executeCycle 落定终态），再回收 action 线程——
  // executeAction 以 running==false 为周期结束信号读终态上报，反向回收会让
  // 它读到覆盖后的状态。
  if (worker_.joinable()) {
    worker_.join();
  }
  if (action_thread_.joinable()) {
    action_thread_.join();
  }
  if (survey_thread_.joinable()) {
    survey_thread_.join();
  }
}

void ManipulationSkillsNode::releaseResources()
{
  // 与 createInterfaces/initializeMoveIt 对称；参数声明、验证钩子与
  // view_planner_/quality_gate_/safety_gate_ 纯核保留（再次 configure 时
  // loadParameters 重建），contact_recovery_required_ 跨清理保持。
  cycle_action_server_.reset();
  survey_action_server_.reset();
  preview_approach_service_.reset();
  preview_full_contact_service_.reset();
  cancel_service_.reset();
  recovery_service_.reset();
  photo_pose_service_.reset();
  arm_service_.reset();
  target_sub_.reset();
  diagnostics_sub_.reset();
  decision_sub_.reset();
  refined_pose_sub_.reset();
  refined_diag_sub_.reset();
  robot_status_sub_.reset();
  status_pub_.reset();
  marker_pub_.reset();
  grasp_hyp_pub_.reset();
  tool_io_client_.reset();
  grasp_task_.reset();
  motion_.reset();
  move_group_.reset();
}

ManipulationSkillsNode::~ManipulationSkillsNode()
{
  // 先置取消标志并唤醒等待，再回收 worker 与 action 线程，避免 shutdown 后
  // 线程仍访问已销毁成员（use-after-free）。
  closeMotionOutputAndCancel();
}

void ManipulationSkillsNode::initializeMoveIt()
{
  // wait_for_servers 有界化（A8）：默认 -1 为无限等待 move_group 服务器，
  // 会把 on_configure 卡死在生命周期转换回调里；有界等待只影响启动同步
  // （返回值被 MGI 忽略），规划/执行调用自身在服务器缺席时快速失败并报错。
  move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
    moveit_node_, planning_group_, std::shared_ptr<tf2_ros::Buffer>(),
    rclcpp::Duration::from_seconds(5.0));
  move_group_->setPoseReferenceFrame(base_frame_);
  move_group_->setPlanningTime(planning_time_s_);
  move_group_->setNumPlanningAttempts(planning_attempts_);
  move_group_->setMaxVelocityScalingFactor(transit_velocity_scaling_);
  move_group_->setMaxAccelerationScalingFactor(transit_acceleration_scaling_);
  move_group_->allowReplanning(true);
  rebuildMotionInterface();
  rebuildGraspTask();
  RCLCPP_INFO(
    get_logger(),
    "主动视觉靠近节点 ready: group=%s base=%s tip=%s camera=%s "
    "execution=%s grasp=%s",
    planning_group_.c_str(), base_frame_.c_str(), tip_frame_.c_str(),
    camera_frame_.c_str(), execution_enabled_.load() ? "enabled" : "plan_only",
    grasp_enabled_.load() ? "enabled" : "disabled");
}

void ManipulationSkillsNode::loadParameters()
{
  params_ = param_listener_->get_params();
  const auto & params = params_;
  base_frame_ = params.frames.base;
  tip_frame_ = params.frames.tip;
  camera_frame_ = params.frames.camera;
  tool_frame_ = params.frames.tool;
  planning_group_ = params.moveit.planning_group;
  planning_time_s_ = params.moveit.planning_time_s;
  planning_attempts_ = static_cast<int>(params.moveit.planning_attempts);
  transit_velocity_scaling_ = params.moveit.transit_velocity_scaling;
  transit_acceleration_scaling_ = params.moveit.transit_acceleration_scaling;
  photo_pose_named_target_ = params.photo_pose_named_target;
  harvest_stow_named_target_ = params.harvest_stow_named_target;

  ViewPlannerConfig view_config;
  view_config.observation_radius_m = params.scan.observation_radius_m;
  view_config.minimum_radius_m = params.scan.minimum_radius_m;
  view_config.azimuth_step_deg = params.scan.azimuth_step_deg;
  view_config.azimuth_limit_deg = params.scan.azimuth_limit_deg;
  view_config.elevation_step_deg = params.scan.elevation_step_deg;
  view_config.elevation_limit_deg = params.scan.elevation_limit_deg;
  view_config.preferred_baseline_deg = params.scan.preferred_baseline_deg;
  view_config.radial_step_m = params.scan.radial_step_m;
  view_config.candidate_layers = static_cast<int>(params.scan.candidate_layers);
  view_config.views_to_minimum_radius =
    static_cast<int>(params.scan.views_to_minimum_radius);
  view_config.max_camera_step_m = params.scan.max_camera_step_m;
  view_config.workspace_max_reach_m = params.scan.workspace_max_reach_m;
  view_config.min_camera_height_m = params.scan.min_camera_height_m;
  // 环境几何保护区（阶段 F1）：stride-6 扁平数组解析为轴对齐盒列表；
  // 畸形盒（残余组/非有限分量/min>=max）逐条 WARN 并丢弃，不炸节点。
  // 同一列表两处生效：视点生成剔除（view_config 副本）与 GraspTask
  // planning scene 碰撞盒（成员 protected_zones_）。
  const auto parsed_zones = parseProtectedZones(params.scan.protected_zones);
  for (const auto & issue : parsed_zones.issues) {
    RCLCPP_WARN(get_logger(), "scan.protected_zones 丢弃畸形盒: %s", issue.c_str());
  }
  protected_zones_ = parsed_zones.zones;
  view_config.protected_zones = parsed_zones.zones;
  if (!protected_zones_.empty()) {
    RCLCPP_INFO(
      get_logger(), "环境几何保护区已生效: %zu 个轴对齐盒（视点+MTC 碰撞盒）",
      protected_zones_.size());
  }
  // 职责实现直接构造（唯一实现，原 *.impl 工厂缝位已删除）。
  view_planner_ = std::make_unique<ViewPlanner>(view_config);
  assumed_frame_interval_s_ = params.scan.assumed_frame_interval_s;
  frame_wait_s_ = params.scan.frame_wait_s;

  QualityGateConfig gate_config;
  gate_config.minimum_views = static_cast<std::size_t>(params.quality.minimum_views);
  gate_config.minimum_baseline_deg = params.quality.minimum_baseline_deg;
  gate_config.minimum_mean_nearest_baseline_deg =
    params.quality.minimum_mean_nearest_baseline_deg;
  gate_config.minimum_mean_depth_ratio = params.quality.minimum_mean_depth_ratio;
  gate_config.maximum_data_age_s = params.quality.maximum_data_age_s;
  gate_config.maximum_axis_angle_deg = params.quality.maximum_axis_angle_deg;
  quality_gate_ = std::make_unique<QualityGate>(gate_config);

  SafetyGateConfig safety_config;
  safety_config.require_robot_status = params.execution.require_robot_status;
  safety_config.robot_status_max_age_s = params.execution.robot_status_max_age_s;
  safety_config.target_observation_max_age_s =
    params.execution.target_observation_max_age_s;
  target_observation_max_age_config_s_ = safety_config.target_observation_max_age_s;
  safety_gate_ = std::make_unique<SafetyGate>(
    safety_config, [this]() {return now().seconds();});

  execution_enabled_.store(params.execution.enabled);
  grasp_enabled_.store(params.grasp.enabled);
  neck_margin_m_ = params.grasp.neck_margin_m;
  minimum_travel_m_ = params.grasp.minimum_travel_m;
  maximum_travel_m_ = params.grasp.maximum_travel_m;
  reconfirm_wait_s_ = params.grasp.reconfirm_wait_s;
  reconfirm_tolerance_m_ = params.grasp.reconfirm_tolerance_m;
  reconfirm_max_attempts_ = static_cast<int>(params.grasp.reconfirm_max_attempts);
  allow_stale_anchor_ = params.grasp.allow_stale_anchor;
  tool_enabled_.store(params.tool.enabled);
  tool_io_fun_ = static_cast<int>(params.tool.io_fun);
  tool_io_pin_ = static_cast<int>(params.tool.io_pin);
  tool_close_state_ = params.tool.close_state;
  service_timeout_s_ = params.timeouts.service_s;
  refined_timeout_s_ = params.timeouts.refined_s;
  // 参数重载时同步重建运动接口实现（MoveIt 未初始化前为空操作，由
  // initializeMoveIt 首次装配）。GraspTask 同样按现行 yaml 重建，使接近/
  // 护栏改参在空闲时生效。
  rebuildMotionInterface();
  rebuildGraspTask();
}

void ManipulationSkillsNode::rebuildMotionInterface()
{
  if (!move_group_) {
    return;
  }
  MoveItMotionConfig motion_config;
  motion_config.base_frame = base_frame_;
  motion_config.tip_frame = tip_frame_;
  motion_config.camera_frame = camera_frame_;
  motion_config.tool_frame = tool_frame_;
  const auto & moveit = params_.moveit;
  motion_config.pilz_pipeline = moveit.pilz_pipeline;
  motion_config.fallback_pipeline = moveit.fallback_pipeline;
  motion_config.transit_velocity_scaling = transit_velocity_scaling_;
  motion_config.transit_acceleration_scaling = transit_acceleration_scaling_;
  motion_config.transit_max_duration_s = moveit.transit_max_duration_s;
  motion_config.transit_max_total_joint_travel_rad =
    moveit.transit_max_total_joint_travel_rad;
  motion_config.transit_max_single_joint_travel_rad =
    moveit.transit_max_single_joint_travel_rad;
  motion_config.observe_planning_time_s = moveit.observe_planning_time_s;
  motion_config.observe_planning_attempts =
    static_cast<int>(moveit.observe_planning_attempts);
  motion_config.observe_max_duration_s = moveit.observe_max_duration_s;
  motion_config.observe_max_total_joint_travel_rad =
    moveit.observe_max_total_joint_travel_rad;
  motion_config.observe_max_single_joint_travel_rad =
    moveit.observe_max_single_joint_travel_rad;
  motion_config.photo_planning_time_s = moveit.photo_planning_time_s;
  motion_config.default_planning_time_s = planning_time_s_;
  motion_config.default_planning_attempts = planning_attempts_;
  motion_config.photo_pose_joint_tolerance_rad =
    params_.photo_pose_joint_tolerance_rad;
  motion_config.photo_pose_max_joint_vel_rad_s =
    params_.photo_pose_max_joint_vel_rad_s;
  // 直接构造唯一实现。执行闸门（A8/I5）：运动输出权限（Active 态）叠加
  // 硬件安全门回调注入——即授权矩阵的 TRANSIT 级底座（任何 execute 路径
  // 不得旁路；plan-only 路径不经其执行段）；safety_block_hook 保持原
  // planOrMoveTip 被拦下时的 FAILED 状态投影。CONTACT/TOOL 级在阶段函数
  // （stages.cpp）与 GraspTask 门显式加查。
  motion_ = std::make_unique<MoveItMotionInterface>(
    move_group_.get(), &tf_buffer_, get_logger(), get_clock(), motion_config,
    [this](std::string & reason) {
      return motionOutputAllowed(reason) && safetyReady(reason);
    },
    [this](const std::string & message) {
      setState(CycleState::FAILED, message);
    });
}

void ManipulationSkillsNode::rebuildGraspTask()
{
  if (!moveit_node_ || !motion_) {
    return;
  }
  GraspTaskConfig task_config;
  const auto & moveit = params_.moveit;
  task_config.planning_group = planning_group_;
  task_config.tip_frame = tip_frame_;
  task_config.base_frame = base_frame_;
  task_config.free_space_pipeline = moveit.mtc_free_space_pipeline;
  task_config.free_space_planner = moveit.mtc_free_space_planner;
  task_config.planning_time_s = planning_time_s_;
  task_config.velocity_scaling = moveit.velocity_scaling;
  task_config.acceleration_scaling = moveit.acceleration_scaling;
  task_config.cartesian_step_m = moveit.mtc_cartesian_step_m;
  task_config.cartesian_min_fraction = moveit.mtc_cartesian_min_fraction;
  task_config.cartesian_precision_m = moveit.mtc_cartesian_precision_m;
  task_config.max_solutions = static_cast<std::size_t>(moveit.mtc_max_solutions);
  task_config.approach_max_duration_s = moveit.mtc_approach_max_duration_s;
  task_config.approach_max_total_joint_travel_rad =
    moveit.mtc_approach_max_total_joint_travel_rad;
  task_config.approach_max_single_joint_travel_rad =
    moveit.mtc_approach_max_single_joint_travel_rad;
  task_config.approach_max_detour_ratio = moveit.mtc_approach_max_detour_ratio;
  task_config.approach_max_chord_deviation_m =
    moveit.mtc_approach_max_chord_deviation_m;
  task_config.approach_max_recede_m = moveit.mtc_approach_max_recede_m;
  task_config.approach_cartesian_max_distance_m =
    moveit.mtc_approach_cartesian_max_distance_m;
  task_config.approach_along_axis_m = moveit.mtc_approach_along_axis_m;
  task_config.approach_max_lateral_m = moveit.mtc_approach_max_lateral_m;
  task_config.approach_max_align_deg = moveit.mtc_approach_max_align_deg;
  task_config.lookup_current_tip = [this]() {
      return motion_->lookupTransform(base_frame_, tip_frame_);
    };
  task_config.protected_zones = protected_zones_;
  // 执行边界 = 授权矩阵公共级 + execution + grasp（GraspTask 门内显式加查）；
  // 目标身份/新鲜度与 GraspDecision 复检由阶段执行器单点判定
  // （ExecuteTarget.goal.target_id 钉死；套入/剪切入口 requireStageAuthority）。
  // 撤离不依赖视觉：插入后目标常被工具遮挡、收割后决策可能翻转，撤退门
  // 不做决策复检（见 execution_authority.hpp 矩阵注释）。
  task_config.approach_execution_gate = [this](std::string & reason) {
      return motionOutputAllowed(reason) && safetyReady(reason) &&
             !cancel_requested_.load() && execution_enabled_.load() &&
             grasp_enabled_.load();
    };
  task_config.retreat_execution_gate = task_config.approach_execution_gate;
  grasp_task_ = std::make_unique<GraspTask>(moveit_node_, task_config);
}

rcl_interfaces::msg::SetParametersResult ManipulationSkillsNode::onParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  // 纯验证钩子（on-set 阶段，无副作用）：范围校验由生成的 ParamListener
  // 内置完成，本钩子只负责"运行中拒改"与 execution→grasp→tool 依赖链；
  // 使能原子与状态发布等副作用全在 post-set 回调（ctor 内）落地。
  rcl_interfaces::msg::SetParametersResult result;
  if (running_.load()) {
    result.successful = false;
    result.reason = "周期运行中不能修改运动策略";
    return result;
  }
  // 依赖链按"监听器现行值叠加本批改动"的合并结果判定。
  const auto & current = params_;
  bool execution = current.execution.enabled;
  bool grasp = current.grasp.enabled;
  bool tool = current.tool.enabled;
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
    result.successful = false;
    result.reason = "使能依赖必须满足 execution→grasp→tool";
    return result;
  }
  result.successful = true;
  return result;
}

void ManipulationSkillsNode::createInterfaces()
{
  planning_callback_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  createSubscriptions();
  createServices();
  createActions();
}

void ManipulationSkillsNode::createSubscriptions()
{
  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  target_sub_ = create_subscription<peach_interfaces::msg::PeachTargetObservationArray>(
    "/peach/perception/target_observations", 10,
    std::bind(&ManipulationSkillsNode::onTargets, this, std::placeholders::_1));
  diagnostics_sub_ = create_subscription<peach_interfaces::msg::ReconstructionStatus>(
    "/peach/reconstruction/diagnostics", latched,
    std::bind(&ManipulationSkillsNode::onDiagnostics, this, std::placeholders::_1));
  decision_sub_ = create_subscription<peach_interfaces::msg::GraspDecision>(
    "/peach/reconstruction/grasp_decision", latched,
    std::bind(&ManipulationSkillsNode::onDecision, this, std::placeholders::_1));
  refined_pose_sub_ =
    create_subscription<peach_interfaces::msg::BagGraspCandidateArray>(
    "/peach/reconstruction/refined_pose", latched,
    std::bind(&ManipulationSkillsNode::onRefinedPose, this, std::placeholders::_1));
  refined_diag_sub_ = create_subscription<peach_interfaces::msg::BagFittingArray>(
    "/peach/reconstruction/refined_diagnostics", latched,
    std::bind(&ManipulationSkillsNode::onRefinedDiagnostics, this, std::placeholders::_1));
  robot_status_sub_ = create_subscription<aubo_msgs::msg::RobotStatus>(
    "/aubo_io_controller/robot_status", 10,
    std::bind(&ManipulationSkillsNode::onRobotStatus, this, std::placeholders::_1));
  status_pub_ = create_publisher<std_msgs::msg::String>("~/status", latched);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/planned_views", latched);
  grasp_hyp_pub_ = create_publisher<peach_interfaces::msg::GraspHypothesis>(
    "/peach/manipulation/grasp_hypothesis", latched);
}

void ManipulationSkillsNode::createServices()
{
  preview_approach_service_ = create_service<Trigger>(
    "~/preview_approach_insert",
    std::bind(
      &ManipulationSkillsNode::onPreviewApproachInsert, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(), planning_callback_group_);
  preview_full_contact_service_ = create_service<Trigger>(
    "~/preview_full_contact",
    std::bind(
      &ManipulationSkillsNode::onPreviewFullContact, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(), planning_callback_group_);
  cancel_service_ = create_service<Trigger>(
    "~/cancel_cycle",
    std::bind(
      &ManipulationSkillsNode::onCancel, this,
      std::placeholders::_1, std::placeholders::_2));
  recovery_service_ = create_service<Trigger>(
    "~/acknowledge_recovery",
    std::bind(
      &ManipulationSkillsNode::onAcknowledgeRecovery, this,
      std::placeholders::_1, std::placeholders::_2));
  photo_pose_service_ = create_service<Trigger>(
    "~/go_to_photo_pose",
    std::bind(
      &ManipulationSkillsNode::onGoToPhotoPose, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(), planning_callback_group_);
  // 选果级 TCP IK 预检（executor SELECT 段批量查询）：只求解不动臂，
  // 种子=当前关节状态（解在当前构型邻域，与后续 MTC 规划口径一致）。
  reachability_service_ = create_service<CheckReachability>(
    "~/check_reachability",
    std::bind(
      &ManipulationSkillsNode::onCheckReachability, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(), planning_callback_group_);
  arm_service_ = create_service<SetBool>(
    "~/set_execution_armed",
    std::bind(
      &ManipulationSkillsNode::onArm, this,
      std::placeholders::_1, std::placeholders::_2));
  tool_io_client_ = create_client<aubo_msgs::srv::SetIO>(
    "/aubo_io_controller/set_io");
  tool_actuator_.setSendIo(
    [this](int, int, double, std::string & reason) {
      if (!commandToolClose()) {
        reason = "set_io_failed";
        return false;
      }
      return true;
    });
}

void ManipulationSkillsNode::createActions()
{
  cycle_action_server_ = rclcpp_action::create_server<ExecuteTarget>(
    this, "~/execute_target",
    std::bind(
      &ManipulationSkillsNode::onActionGoal, this,
      std::placeholders::_1, std::placeholders::_2),
    std::bind(
      &ManipulationSkillsNode::onActionCancel, this,
      std::placeholders::_1),
    std::bind(
      &ManipulationSkillsNode::onActionAccepted, this,
      std::placeholders::_1));
  survey_action_server_ = rclcpp_action::create_server<SurveyScene>(
    this, "~/survey_scene",
    std::bind(
      &ManipulationSkillsNode::onSurveyGoal, this,
      std::placeholders::_1, std::placeholders::_2),
    std::bind(
      &ManipulationSkillsNode::onSurveyCancel, this,
      std::placeholders::_1),
    std::bind(
      &ManipulationSkillsNode::onSurveyAccepted, this,
      std::placeholders::_1));
}

void ManipulationSkillsNode::onTargets(
  const peach_interfaces::msg::PeachTargetObservationArray::SharedPtr message)
{
  // 每帧测量观测到达间隔并刷新帧率自适应超时（帧率以运行状态为准）。
  trackFrameInterval();
  safety_gate_->set_target_observation_max_age_s(effectiveTargetMaxAgeS());
  // 单条观测的公共字段提取（消息 ROS 类型留在节点薄壳）：observed 判定与
  // 摆动/跟踪诊断透传语义对 selected 与锁定集两路完全一致——记忆锚点帧
  // （anchor_from_memory）不算新鲜观测，锚点可用于派发/规划但不刷新观测
  // 新鲜度，让安全门的 stale 判定继续以真实观测为准。
  const auto extract = [](const auto & item, auto & out) {
      const bool anchor_from_memory = std::find(
        item.diagnostic_flags.begin(), item.diagnostic_flags.end(),
        "anchor_from_memory") != item.diagnostic_flags.end();
      out.observed =
        item.tracking_status == peach_interfaces::msg::PeachTargetObservation::OBSERVED &&
        item.candidate.status != peach_interfaces::msg::BagGraspCandidate::REJECT &&
        !anchor_from_memory;
      // 再确认段诊断透传（2.7-RECONFIRM）：摆动旗标与跟踪状态原始枚举随帧进
      // 缓存，摆动等平息与失败原因文案（出视野/深度空洞/跟踪丢失）在
      // stageReconfirmTarget 消费。
      out.swinging = std::find(
        item.diagnostic_flags.begin(), item.diagnostic_flags.end(),
        "target_swinging") != item.diagnostic_flags.end();
      out.tracking_status = item.tracking_status;
      out.bottom = pointToEigen(item.candidate.bag_bottom);
      out.neck = pointToEigen(item.candidate.bag_neck);
      out.axis = vectorToEigen(item.candidate.translation_direction);
      out.entry_pose = poseToEigen(item.candidate.entry_pose);
      out.suggested_travel_m = item.candidate.suggested_travel_m;
      out.bbox_x = item.candidate_2d.bbox_x;
      out.bbox_y = item.candidate_2d.bbox_y;
      out.bbox_w = item.candidate_2d.bbox_w;
      out.bbox_h = item.candidate_2d.bbox_h;
      out.bbox_valid =
        item.candidate_2d.bbox_w > 0 && item.candidate_2d.bbox_h > 0;
      out.foreground_ratio = item.fitting.foreground_ratio;
      out.image_width = item.mask.width > 0 ?
        static_cast<int>(item.mask.width) : 640;
      out.image_height = item.mask.height > 0 ?
        static_cast<int>(item.mask.height) : 480;
    };
  // 锁定集锚点缓存刷新（阶段 E 残局抬质量能力端）：observations 携带锁定集
  // 全部 confirmed 目标（含非 selected），逐条提取批量委托 cache_。必须置于
  // 下方 selected 早退之前——残局期感知 selected 恒空（FULL 终局后目标已被
  // 计划 complete），若先早退，残局目标锚点永远进不了缓存，OBSERVE_ONLY
  // 受理门与执行体都无数据源。
  std::vector<LockedTargetUpdate> locked_updates;
  if (message->target_set_locked) {
    locked_updates.reserve(message->observations.size());
    for (const auto & item : message->observations) {
      // confirmed 过滤在薄壳完成（缓存只存 confirmed 目标）；未确认/空 ID
      // 记录不进锁定集缓存。
      if (!item.confirmed || item.target_id.empty()) {
        continue;
      }
      LockedTargetUpdate locked;
      locked.target_id = item.target_id;
      extract(item, locked);
      locked_updates.push_back(std::move(locked));
    }
  }
  cache_.updateLockedTargets(
    message->target_set_locked, message->harvest_run_id, locked_updates);
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    last_snapshot_id_ = std::to_string(message->snapshot_id);
    last_observation_count_ = static_cast<uint32_t>(message->observations.size());
    last_target_set_locked_ = message->target_set_locked;
  }
  // selected 绑定以感知消息为准（不读周期上下文：周期身份钉在 ctx，由
  // stagePrepareCycle 的 goal 钉死校验单点把关；锁定集缓存是受理/执行的
  // 独立数据源）。感知切换 selected 时缓存跟随，周期侧身份不一致即失败，
  // 由编排按新 selected 重新派发。
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
  extract(*selected, update);
  cache_.updateSelectedTarget(update);
}

void ManipulationSkillsNode::trackFrameInterval()
{
  const double arrival_s = now().seconds();
  const double last = last_targets_arrival_s_.load(std::memory_order_relaxed);
  if (last > 0.0) {
    const double dt = arrival_s - last;
    // 异常间隔（暂停后首帧/时钟跳变）不进 EMA，避免污染帧率估计
    if (dt > 1e-3 && dt < 30.0) {
      const double ema = frame_interval_ema_s_.load(std::memory_order_relaxed);
      frame_interval_ema_s_.store(
        ema > 0.0 ? 0.7 * ema + 0.3 * dt : dt, std::memory_order_relaxed);
    }
  }
  last_targets_arrival_s_.store(arrival_s, std::memory_order_relaxed);
}

double ManipulationSkillsNode::waitIntervalS() const
{
  const double ema = frame_interval_ema_s_.load(std::memory_order_relaxed);
  if (ema > 0.0) {
    return ema;
  }
  return assumed_frame_interval_s_;
}

double ManipulationSkillsNode::effectiveFrameWaitS() const
{
  const double interval = waitIntervalS();
  if (interval <= 0.0) {return frame_wait_s_;}
  // 视点到位后等 ~4 帧 + 1s 稳定余量；下限 2s，上限为配置值
  return adaptive_timeout_s(interval, 4.0, 1.0, 2.0, frame_wait_s_);
}

double ManipulationSkillsNode::effectiveTargetMaxAgeS() const
{
  const double fallback = target_observation_max_age_config_s_;
  const double ema = frame_interval_ema_s_.load(std::memory_order_relaxed);
  if (ema <= 0.0) {return fallback;}
  // 低帧率放宽；不得收得比 yaml 回退更紧（曾用 0.4s 预填 EMA，把 3s 收到 1.5s）。
  return std::max(
    fallback,
    adaptive_timeout_s(ema, 2.5, 0.5, 1.0, 10.0));
}

double ManipulationSkillsNode::effectiveReconfirmWaitS() const
{
  const double interval = waitIntervalS();
  if (interval <= 0.0) {return reconfirm_wait_s_;}
  // 与视点等帧同一形状：等 ~4 帧 + 1s 稳定余量，下限 2s，上限为配置值
  // （reconfirm_wait_s 同时承担回退值与自适应上限，摆动等平息也在本窗口预算内）。
  return adaptive_timeout_s(interval, 4.0, 1.0, 2.0, reconfirm_wait_s_);
}

double ManipulationSkillsNode::effectiveRefinedWaitS() const
{
  const double interval = waitIntervalS();
  if (interval <= 0.0) {return refined_timeout_s_;}
  // 协议 2.7-FINALIZE 的 T(refined)=clamp(下限, 3×实测refit耗时EMA, 上限)：
  // refit 耗时由重建节点持有、本包不可得（本阶段不动其他包接口），按观测帧
  // 间隔近似——finalize 触发后 refit 在后续约 3 帧内闩锁发布 refined，
  // 故 ≈3 帧 + 2s 余量；下限 2s（高帧率时 refit 仍有固定计算耗时），上限为
  // 配置值（同时是未测得帧率时的回退值）。
  return adaptive_timeout_s(
    interval, 3.0, 2.0, std::min(2.0, refined_timeout_s_),
    refined_timeout_s_);
}

void ManipulationSkillsNode::onDiagnostics(
  const peach_interfaces::msg::ReconstructionStatus::SharedPtr message)
{
  // 类型化诊断（2026-08 起替换裸 JSON）：字段直读，不再解析字符串。
  // 无效标量约定 -1：质量证据按 0 汇入缓存（无覆盖=基线/深度证据为零，
  // 质量门自然不通过），不把 -1 当真实测量值参与比较。
  ReconstructionDiagnosticsUpdate update;
  update.target_id = message->target_id;
  update.state = message->state.empty() ? "IDLE" : message->state;
  update.captured_views = static_cast<std::size_t>(
    std::max(message->captured_views, 0));
  update.max_baseline_deg = std::max(message->max_baseline_deg, 0.0);
  update.mean_nearest_baseline_deg = std::max(
    message->mean_nearest_baseline_deg, 0.0);
  update.mean_depth_ratio = std::max(message->valid_depth_ratio, 0.0);
  update.view_directions.reserve(message->view_directions.size());
  for (const auto & direction : message->view_directions) {
    update.view_directions.emplace_back(direction.x, direction.y, direction.z);
  }
  cache_.updateReconstructionDiagnostics(update);
}

void ManipulationSkillsNode::onDecision(
  const peach_interfaces::msg::GraspDecision::SharedPtr message)
{
  // 许可几何字段这里不消费（抓取几何走 refined_pose 通道）；只调和
  // allowed 与目标 ID，allowed=false 时 reason 由状态镜像透出。
  if (!cache_.updateGraspDecision(message->target_id, message->allowed)) {
    RCLCPP_WARN(
      get_logger(), "忽略非当前目标的 grasp_decision: expected=%s actual=%s",
      cache_.targetGateSample().id.c_str(), message->target_id.c_str());
  }
}

void ManipulationSkillsNode::onRefinedPose(
  const peach_interfaces::msg::BagGraspCandidateArray::SharedPtr message)
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
  update.accepted = candidate.status == peach_interfaces::msg::BagGraspCandidate::ACCEPT;
  if (!cache_.updateRefinedPose(update)) {
    RCLCPP_WARN(
      get_logger(), "忽略非当前目标的 refined pose: expected=%s actual=%s",
      cache_.targetGateSample().id.c_str(), candidate.target_id.c_str());
  }
}

void ManipulationSkillsNode::onRefinedDiagnostics(
  const peach_interfaces::msg::BagFittingArray::SharedPtr message)
{
  if (message->fittings.empty()) {
    RefinedFittingUpdate clear;
    clear.clear = true;
    cache_.updateRefinedFitting(clear);
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
  update.accepted = fitting.status == peach_interfaces::msg::BagFitting::ACCEPT;
  if (!cache_.updateRefinedFitting(update)) {
    RCLCPP_WARN(
      get_logger(), "忽略非当前目标的 refined diagnostics: expected=%s actual=%s",
      cache_.expectedFittingTargetId().c_str(), fitting.target_id.c_str());
  }
}

void ManipulationSkillsNode::setState(
  CycleState state, const std::string & message, const std::string & target_id)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    // 枚举是唯一权威状态；字符串仅投影进 JSON 供 dashboard/web 只读消费。
    current_state_ = state;
    // 阶段耗时埋点：cycle_state_ 变更点即计时切换点（周期未启动时为 no-op）。
    stage_timer_.onStateChange(state, StageTimer::Clock::now());
    state_json_ = {
      {"state", toString(state)},
      {"message", message},
      {"running", running_.load()},
      {"execution_enabled", execution_enabled_.load()},
      {"execution_armed", execution_armed_.load()},
      {"grasp_enabled", grasp_enabled_.load()},
      {"contact_recovery_required", contact_recovery_required_.load()},
      {"target_id", target_id},
    };
    const QualitySnapshot snapshot = qualitySnapshot();
    state_json_["quality"] = {
      {"selected_target_id", snapshot.selected_target_id},
      {"reconstruction_target_id", snapshot.reconstruction_target_id},
      {"refined_target_id", snapshot.refined_target_id},
      {"captured_views", snapshot.captured_views},
      {"station_count", snapshot.station_count},
      {"max_baseline_deg", snapshot.max_baseline_deg},
      {"mean_nearest_baseline_deg", snapshot.mean_nearest_baseline_deg},
      {"mean_depth_ratio", snapshot.mean_depth_ratio},
      {"refined_rmse_m", snapshot.refined_rmse_m},
      {"refined_inlier_ratio", snapshot.refined_inlier_ratio},
      {"axis_angle_deg", snapshot.axis_angle_deg},
      {"grasp_allowed", snapshot.grasp_allowed},
    };
  }
  publishState();
  RCLCPP_INFO(get_logger(), "[%s] %s", toString(state).c_str(), message.c_str());
}

void ManipulationSkillsNode::publishState()
{
  // Unconfigured/cleanup 后 status_pub_ 为空（接口尚未创建或已释放）、
  // Inactive 下发布者未激活：状态投影仅内存更新，跳过发布。
  if (!status_pub_ || !status_pub_->is_activated()) {
    return;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_json_["running"] = running_.load();
  state_json_["execution_armed"] = execution_armed_.load();
  state_json_["contact_recovery_required"] = contact_recovery_required_.load();
  // 回调耗时诊断投影（2.16-5）：复用 ~/status 既有通道，不新增话题。
  state_json_["callback_timing"] = callback_timing_.toJson();
  std_msgs::msg::String message;
  message.data = state_json_.dump();
  status_pub_->publish(message);
}

void ManipulationSkillsNode::startCycleTiming()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  stage_timer_.start(StageTimer::Clock::now());
}

void ManipulationSkillsNode::fillStageDurations(
  const std::shared_ptr<ExecuteTarget::Result> & result)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  // 兜底收口：终态 setState 正常已关闭计时；此处防止异常路径漏收。
  stage_timer_.close(StageTimer::Clock::now());
  for (const auto & entry : stage_timer_.entries()) {
    result->stage_names.push_back(entry.name);
    builtin_interfaces::msg::Duration duration;
    const auto nanos =
      std::chrono::duration_cast<std::chrono::nanoseconds>(entry.elapsed).count();
    duration.sec = static_cast<int32_t>(nanos / 1000000000LL);
    duration.nanosec = static_cast<uint32_t>(nanos % 1000000000LL);
    result->stage_durations.push_back(duration);
  }
}

void ManipulationSkillsNode::fillExecuteResults(
  const std::shared_ptr<ExecuteTarget::Result> & result,
  const CycleContext * ctx)
{
  const bool observe_only = ctx && ctx->observe_only;
  const bool pregrasp_only = ctx && ctx->pregrasp_only;
  const bool succeeded =
    result->outcome == ExecuteTarget::Result::SUCCEEDED;
  result->completion_level = ctx ? ctx->completion_level : 0;
  result->failure_code = succeeded || !ctx ? 0u : ctx->failure_code;
  result->cut_command_accepted = ctx && ctx->cut_command_accepted;
  result->cut_confirmed = ctx && ctx->cut_confirmed;
  result->retreat_confirmed = ctx && ctx->retreat_confirmed;
  result->harvest_confirmed =
    ctx && ctx->cut_confirmed && ctx->retreat_confirmed;
  result->pregrasp = ctx ? ctx->pregrasp_msg : peach_interfaces::msg::PregraspVerification{};
  result->harvest.completion_level = result->completion_level;
  result->harvest.commanded =
    grasp_enabled_.load() && !observe_only && !pregrasp_only;
  result->harvest.confirmed = result->harvest_confirmed;
  result->harvest.grasped = result->harvest_confirmed;
  result->harvest.reason = result->reason;
  if (!tool_enabled_.load() && result->harvest.commanded) {
    result->harvest.reason += "；tool.enabled=false，跳过末端 IO";
  }
  // DepositResult 预留：M8 卸果站未标定，固定按"跳过转移"语义填充。
  result->deposit.deposited = false;
  result->deposit.reason.clear();
  if (observe_only || pregrasp_only) {
    result->verification.passed = succeeded;
    result->verification.commanded = false;
    result->verification.confirmed =
      pregrasp_only && ctx->pregrasp_verified;
    result->verification.harvest_confirmed = false;
    result->verification.reason = result->reason;
    result->verification.failure_code = result->failure_code;
  } else {
    // FULL：deposit 恒跳过（M8 预留），验证随采摘确认走。
    result->verification.passed = result->harvest.grasped;
    result->verification.commanded = result->harvest.commanded;
    result->verification.confirmed = result->harvest.confirmed;
    result->verification.harvest_confirmed = result->harvest_confirmed;
    result->verification.reason = result->reason;
    result->verification.failure_code = result->failure_code;
  }
  result->outcome_record.target_id = ctx ? ctx->target_id : std::string();
  result->outcome_record.outcome = result->outcome;
  result->outcome_record.reason = result->reason;
}

}  // namespace peach_manipulation
