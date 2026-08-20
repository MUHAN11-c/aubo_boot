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
// 节点本体：生命周期回调、参数、接口创建、订阅回调薄壳、runCycle、状态发布。
// 运动接口见 motion_interface.cpp，行为树节点见 bt_nodes.cpp，
// action 服务端与周期控制服务见 cycle_action.cpp。
// 生命周期回调职责表（A8 / Robotics_Tutorial 2.16-1）：
//   on_configure  ：依赖链复核（启动覆盖值不走 on-set 钩子）→ loadParameters
//     工厂装配 → createInterfaces（订阅/服务/action/client）→ initializeMoveIt
//     （MoveIt/MTC/BT 资源）；任一步抛异常即回滚并 FAILURE，不进 Active。
//   on_activate   ：只做快速可预测切换——开放运动输出权限（原子标志）。
//   on_deactivate ：先关输出权限（撤 arm、拒新 goal），再按 CANCEL_NOW 等价路径
//     真取消活动周期（置取消标志、stop MoveIt/MTC、唤醒等待、回收线程，action
//     侧按 outcome=CANCELED 终局上报）后落 Inactive；接触段 recovery 锁
//     （contact_recovery_required_）跨停用保持，语义不变。
//   on_cleanup    ：释放 MoveIt/MTC/订阅/服务/action/client 全部资源，
//     回 Unconfigured（参数声明与验证钩子保留，可再次 configure）。
//   on_shutdown/on_error：关输出权限 + 取消活动周期 + 释放资源。
#include <algorithm>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.hpp>

#include "approach_grasp_node_impl.hpp"
#include "eigen_conversions.hpp"
#include "motion_factory.hpp"

namespace peach_manipulation_skills
{

ApproachGraspNode::ApproachGraspNode(const rclcpp::NodeOptions & options)
: LifecycleNode("peach_manipulation_skills_node", options),
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
    "peach_manipulation_skills_moveit", moveit_options);
  // ParamListener 构造即声明全部参数并做启动校验（yaml 覆盖值非法时抛
  // InvalidParameterValueException 直接启动失败），内置范围校验随每次 set 生效；
  // 声明/默认值/范围的单一事实源为 src/approach_grasp_node_parameters.yaml。
  param_listener_ = std::make_shared<peach_manipulation_skills_node::ParamListener>(
    get_node_parameters_interface(), get_logger());
  loadParameters();
  // on-set 验证钩子（无副作用，见 onParameters）：运行中拒改 +
  // execution→grasp→tool 依赖链。rclcpp 的 on-set 回调按注册逆序调用，
  // 本钩子后注册先执行，拒绝时监听器的快照更新不会触发。
  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&ApproachGraspNode::onParameters, this, std::placeholders::_1));
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

CallbackReturn ApproachGraspNode::on_configure(const rclcpp_lifecycle::State &)
{
  // 使能依赖链复核：启动覆盖值不经 on-set 钩子，违链（grasp/tool 越级开启）
  // 必须拦在 Inactive 之前；运行期动态改参仍由 onParameters 逐批把关。
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
  try {
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

CallbackReturn ApproachGraspNode::on_activate(const rclcpp_lifecycle::State &)
{
  // 快速可预测切换：开放运动输出权限（原子标志）+ 激活状态发布者，
  // 不做任何资源分配/等待。
  motion_output_permitted_.store(true);
  status_pub_->on_activate();
  marker_pub_->on_activate();
  RCLCPP_INFO(get_logger(), "节点已激活：运动输出权限开放");
  publishState();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ApproachGraspNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  // 先关输出权限（此后一切运动类入口拒绝），再按 CANCEL_NOW 等价路径取消
  // 活动周期并回收线程；contact_recovery_required_ 不随停用清除。
  closeMotionOutputAndCancel();
  setState(CycleState::IDLE, "节点已停用：运动输出权限关闭，活动周期已按取消路径终止");
  status_pub_->on_deactivate();
  marker_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ApproachGraspNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  releaseResources();
  RCLCPP_INFO(get_logger(), "已清理：MoveIt/MTC/订阅/服务资源全部释放，回 Unconfigured");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ApproachGraspNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  closeMotionOutputAndCancel();
  releaseResources();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ApproachGraspNode::on_error(const rclcpp_lifecycle::State &)
{
  // 进入 ErrorProcessing 即关闭运动输出（motionOutputAllowed 随之拒绝一切
  // 运动入口），随后按 shutdown 同路径释放资源。
  closeMotionOutputAndCancel();
  releaseResources();
  return CallbackReturn::SUCCESS;
}

bool ApproachGraspNode::motionOutputAllowed(std::string & why) const
{
  if (!motion_output_permitted_.load()) {
    why = "节点未处于 Active 态：运动输出权限仅在 Active 开放";
    return false;
  }
  return true;
}

void ApproachGraspNode::closeMotionOutputAndCancel()
{
  // 顺序即语义：先关权限/撤 arm（拒新入口），再取消活动周期并唤醒所有等待。
  motion_output_permitted_.store(false);
  execution_armed_.store(false);
  cancel_requested_.store(true);
  if (move_group_) {
    move_group_->stop();
  }
  // grasp_task_->cancel() 同时 preempt 预规划线程内未落定的 plan（预规划任务
  // 规划期间占用 active 槽），促其快速返回。
  if (grasp_task_) {
    grasp_task_->cancel();
  }
  cache_.notifyAll();
  // 先回收 worker（runCycle 落定终态，其 finish 已含 cleanupPreplan），再回收
  // action 线程——executeAction 以 running==false 为周期结束信号读终态上报，
  // 反向回收会让它读到覆盖后的状态。
  if (worker_.joinable()) {
    worker_.join();
  }
  // 兜底回收预规划线程（正常已被 runCycle finish 回收；此处覆盖 worker 未启动
  // 或异常路径）。
  cleanupPreplan();
  if (action_thread_.joinable()) {
    action_thread_.join();
  }
  if (survey_thread_.joinable()) {
    survey_thread_.join();
  }
}

// MTC 预规划三件套（2.13-E3）：
void ApproachGraspNode::launchPreplan()
{
  // 前置：btReconfirmTarget 已校验 cycle_refined_ 有效且入口几何齐备。
  // 运动输出权限（A8）兜底：预规划虽只规划不执行，也与预览服务同一纪律——
  // 非 Active 不启动（正常周期要求 Active 受理，此处防 deactivate 竞态）。
  std::string motion_reason;
  if (!mtc_preplan_enabled_ || !grasp_task_ ||
    !motionOutputAllowed(motion_reason))
  {
    return;
  }
  cleanupPreplan();
  if (!preplan_slot_.begin(cycle_reference_anchor_)) {
    return;
  }
  // 几何按值捕获（预规划期间 ReconfirmTarget 的 REFINED 分支可能重算周期
  // 黑板成员；重算后本槽会被 discard，迟到结果按 stale 忽略）。
  const Eigen::Isometry3d entry_tip_pose = cycle_entry_tip_pose_;
  const Eigen::Vector3d axis = cycle_refined_->axis;
  const double travel_m = cycle_travel_m_;
  preplan_thread_ = std::thread(
    [this, entry_tip_pose, axis, travel_m]() {
      const GraspTaskResult result = grasp_task_->preplanApproachAndInsert(
        entry_tip_pose, axis, travel_m);
      preplan_slot_.complete(result.success, result.reason);
    });
  RCLCPP_INFO(
    get_logger(),
    "MTC 预规划已启动（再确认等待窗口后台规划接近/插入任务，不执行）");
}

void ApproachGraspNode::settlePreplanBeforeReplan()
{
  // 内联重规划前清场：预规划线程未退出时其任务仍占用 GraspTask active 槽，
  // 直接内联规划会造成槽并发（preempt 标志串扰/悬垂指针）。先 preempt 促退
  // 再 join，保证进入 approachAndInsert 时 GraspTask 无并发使用者。
  if (preplan_slot_.state() == PreplanSlot::State::PLANNING && grasp_task_) {
    grasp_task_->cancel();
  }
  cleanupPreplan();
}

void ApproachGraspNode::cleanupPreplan()
{
  preplan_slot_.discard();
  if (preplan_thread_.joinable()) {
    preplan_thread_.join();
  }
  if (grasp_task_) {
    grasp_task_->discardPreplanned();
  }
}

void ApproachGraspNode::releaseResources()
{
  // 与 createInterfaces/initializeMoveIt 对称；参数声明、验证钩子与
  // view_planner_/quality_gate_/safety_gate_ 纯核保留（再次 configure 时
  // loadParameters 重建），contact_recovery_required_ 跨清理保持。
  cycle_action_server_.reset();
  survey_action_server_.reset();
  start_service_.reset();
  preview_approach_service_.reset();
  preview_full_contact_service_.reset();
  cancel_service_.reset();
  recovery_service_.reset();
  query_service_.reset();
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

ApproachGraspNode::~ApproachGraspNode()
{
  // 先置取消标志并唤醒等待，再回收 worker 与 action 线程，避免 shutdown 后
  // 线程仍访问已销毁成员（use-after-free）。
  closeMotionOutputAndCancel();
}

void ApproachGraspNode::initializeMoveIt()
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
  task_config.protected_zones = protected_zones_;
  // 执行边界 = 运动输出权限（Active 态）叠加硬件安全门（safetyReady 永不放宽）；
  // 目标身份/新鲜度策略由 BT 层单点决策（设计文档第 7 节），此处不再重复判定。
  task_config.approach_execution_gate = [this](std::string & reason) {
      return motionOutputAllowed(reason) && safetyReady(reason) && !cancel_requested_.load();
    };
  // 插入后目标常被工具遮挡，撤离不能依赖视觉可见性，否则会把正常遮挡误判为禁止撤离。
  task_config.retreat_execution_gate = [this](std::string & reason) {
      return motionOutputAllowed(reason) && safetyReady(reason) && !cancel_requested_.load();
    };
  grasp_task_ = std::make_unique<GraspTask>(moveit_node_, task_config);
  if (!bt_nodes_registered_) {
    registerBehaviorTreeNodes();
    bt_nodes_registered_ = true;
  }
  RCLCPP_INFO(
    get_logger(),
    "主动视觉靠近节点 ready: group=%s base=%s tip=%s camera=%s "
    "execution=%s grasp=%s",
    planning_group_.c_str(), base_frame_.c_str(), tip_frame_.c_str(),
    camera_frame_.c_str(), execution_enabled_.load() ? "enabled" : "plan_only",
    grasp_enabled_.load() ? "enabled" : "disabled");
}

void ApproachGraspNode::loadParameters()
{
  const auto params = param_listener_->get_params();
  base_frame_ = params.frames.base;
  tip_frame_ = params.frames.tip;
  camera_frame_ = params.frames.camera;
  tool_frame_ = params.frames.tool;
  planning_group_ = params.moveit.planning_group;
  planning_time_s_ = params.moveit.planning_time_s;
  planning_attempts_ = static_cast<int>(params.moveit.planning_attempts);
  velocity_scaling_ = params.moveit.velocity_scaling;
  acceleration_scaling_ = params.moveit.acceleration_scaling;
  transit_velocity_scaling_ = params.moveit.transit_velocity_scaling;
  transit_acceleration_scaling_ = params.moveit.transit_acceleration_scaling;
  pilz_pipeline_ = params.moveit.pilz_pipeline;
  fallback_pipeline_ = params.moveit.fallback_pipeline;
  mtc_free_space_planner_ = params.moveit.mtc_free_space_planner;
  mtc_cartesian_step_m_ = params.moveit.mtc_cartesian_step_m;
  mtc_cartesian_precision_m_ = params.moveit.mtc_cartesian_precision_m;
  mtc_max_solutions_ = static_cast<int>(params.moveit.mtc_max_solutions);
  photo_pose_named_target_ = params.photo_pose_named_target;
  deposit_pose_named_target_ = params.deposit_pose_named_target;
  behavior_tree_xml_ = params.behavior_tree.xml;

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
  view_config.min_camera_height_m = params.scan.min_camera_height_m;
  // 环境几何保护区（阶段 F1）：stride-6 扁平数组解析为轴对齐盒列表；
  // 畸形盒（残余组/非有限分量/min>=max）逐条 WARN 并丢弃，不炸节点。
  // 同一列表两处生效：视点生成剔除（view_config 副本）与 MTC 入口剔除
  // （成员 protected_zones_）。
  const auto parsed_zones = parseProtectedZones(params.scan.protected_zones);
  for (const auto & issue : parsed_zones.issues) {
    RCLCPP_WARN(get_logger(), "scan.protected_zones 丢弃畸形盒: %s", issue.c_str());
  }
  protected_zones_ = parsed_zones.zones;
  view_config.protected_zones = parsed_zones.zones;
  if (!protected_zones_.empty()) {
    RCLCPP_INFO(
      get_logger(), "环境几何保护区已生效: %zu 个轴对齐盒（视点+MTC 入口剔除）",
      protected_zones_.size());
  }
  // 工厂按 *.impl 装配（未知名抛 std::invalid_argument 且列出可用名）。
  view_planner_ = createViewPlanner(params.view_planner.impl, view_config);
  maximum_scan_moves_ = static_cast<int>(params.scan.maximum_moves);
  min_effective_views_ = static_cast<int>(params.scan.min_effective_views);
  scan_time_budget_s_ = params.scan.time_budget_s;
  frame_wait_s_ = params.scan.frame_wait_s;

  QualityGateConfig gate_config;
  gate_config.minimum_views = static_cast<std::size_t>(params.quality.minimum_views);
  gate_config.minimum_baseline_deg = params.quality.minimum_baseline_deg;
  gate_config.minimum_mean_nearest_baseline_deg =
    params.quality.minimum_mean_nearest_baseline_deg;
  gate_config.minimum_mean_depth_ratio = params.quality.minimum_mean_depth_ratio;
  gate_config.maximum_refined_rmse_m = params.quality.maximum_refined_rmse_m;
  gate_config.minimum_refined_inlier_ratio = params.quality.minimum_refined_inlier_ratio;
  gate_config.maximum_data_age_s = params.quality.maximum_data_age_s;
  quality_gate_ = createQualityGate(params.quality_gate.impl, gate_config);

  SafetyGateConfig safety_config;
  safety_config.require_robot_status = params.execution.require_robot_status;
  safety_config.robot_status_max_age_s = params.execution.robot_status_max_age_s;
  safety_config.target_observation_max_age_s =
    params.execution.target_observation_max_age_s;
  target_observation_max_age_config_s_ = safety_config.target_observation_max_age_s;
  safety_gate_ = createSafetyGate(
    params.safety_gate.impl, safety_config,
    [this]() {return now().seconds();});

  execution_enabled_.store(params.execution.enabled);
  grasp_enabled_.store(params.grasp.enabled);
  neck_margin_m_ = params.grasp.neck_margin_m;
  minimum_travel_m_ = params.grasp.minimum_travel_m;
  maximum_travel_m_ = params.grasp.maximum_travel_m;
  fallback_standoff_m_ = params.grasp.fallback_standoff_m;
  reconfirm_wait_s_ = params.grasp.reconfirm_wait_s;
  reconfirm_tolerance_m_ = params.grasp.reconfirm_tolerance_m;
  reconfirm_max_attempts_ = static_cast<int>(params.grasp.reconfirm_max_attempts);
  allow_stale_anchor_ = params.grasp.allow_stale_anchor;
  replan_threshold_m_ = params.grasp.replan_threshold_m;
  mtc_preplan_enabled_ = params.grasp.mtc_preplan;
  tool_enabled_.store(params.tool.enabled);
  tool_io_fun_ = static_cast<int>(params.tool.io_fun);
  tool_io_pin_ = static_cast<int>(params.tool.io_pin);
  tool_close_state_ = params.tool.close_state;
  service_timeout_s_ = params.timeouts.service_s;
  refined_timeout_s_ = params.timeouts.refined_s;
  // 参数重载时同步重建运动接口实现（MoveIt 未初始化前为空操作，由
  // initializeMoveIt 首次装配）。
  rebuildMotionInterface();
}

void ApproachGraspNode::rebuildMotionInterface()
{
  if (!move_group_) {
    return;
  }
  MoveItMotionConfig motion_config;
  motion_config.base_frame = base_frame_;
  motion_config.tip_frame = tip_frame_;
  motion_config.camera_frame = camera_frame_;
  motion_config.tool_frame = tool_frame_;
  motion_config.pilz_pipeline = pilz_pipeline_;
  motion_config.fallback_pipeline = fallback_pipeline_;
  motion_config.transit_velocity_scaling = transit_velocity_scaling_;
  motion_config.transit_acceleration_scaling = transit_acceleration_scaling_;
  // 执行闸门（A8）：运动输出权限（Active 态）叠加硬件安全门回调注入实现
  // （I5：任何 execute 路径不得旁路）；safety_block_hook 保持原 planOrMoveTip
  // 被拦下时的 FAILED 状态投影。
  motion_ = createMotionInterface(
    param_listener_->get_params().motion.impl,
    move_group_.get(), &tf_buffer_, get_logger(), get_clock(), motion_config,
    [this](std::string & reason) {
      return motionOutputAllowed(reason) && safetyReady(reason);
    },
    [this](const std::string & message) {
      setState(CycleState::FAILED, message);
    });
}

rcl_interfaces::msg::SetParametersResult ApproachGraspNode::onParameters(
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
  const auto current = param_listener_->get_params();
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

void ApproachGraspNode::createInterfaces()
{
  planning_callback_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  createSubscriptions();
  createServices();
  createActions();
}

void ApproachGraspNode::createSubscriptions()
{
  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  target_sub_ = create_subscription<peach_interfaces::msg::PeachTargetObservationArray>(
    "/peach/perception/target_observations", 10,
    std::bind(&ApproachGraspNode::onTargets, this, std::placeholders::_1));
  diagnostics_sub_ = create_subscription<peach_interfaces::msg::ReconstructionStatus>(
    "/peach/reconstruction/diagnostics", latched,
    std::bind(&ApproachGraspNode::onDiagnostics, this, std::placeholders::_1));
  decision_sub_ = create_subscription<peach_interfaces::msg::GraspDecision>(
    "/peach/reconstruction/grasp_decision", latched,
    std::bind(&ApproachGraspNode::onDecision, this, std::placeholders::_1));
  refined_pose_sub_ =
    create_subscription<peach_interfaces::msg::BagGraspCandidateArray>(
    "/peach/reconstruction/refined_pose", latched,
    std::bind(&ApproachGraspNode::onRefinedPose, this, std::placeholders::_1));
  refined_diag_sub_ = create_subscription<peach_interfaces::msg::BagFittingArray>(
    "/peach/reconstruction/refined_diagnostics", latched,
    std::bind(&ApproachGraspNode::onRefinedDiagnostics, this, std::placeholders::_1));
  robot_status_sub_ = create_subscription<aubo_msgs::msg::RobotStatus>(
    "/aubo_io_controller/robot_status", 10,
    std::bind(&ApproachGraspNode::onRobotStatus, this, std::placeholders::_1));
  status_pub_ = create_publisher<std_msgs::msg::String>("~/status", latched);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/planned_views", latched);
  grasp_hyp_pub_ = create_publisher<peach_interfaces::msg::GraspHypothesis>(
    "/peach/manipulation/grasp_hypothesis", latched);
}

void ApproachGraspNode::createServices()
{
  start_service_ = create_service<Trigger>(
    "~/start_cycle",
    std::bind(
      &ApproachGraspNode::onStart, this,
      std::placeholders::_1, std::placeholders::_2, false));
  preview_approach_service_ = create_service<Trigger>(
    "~/preview_approach_insert",
    std::bind(
      &ApproachGraspNode::onPreviewApproachInsert, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(), planning_callback_group_);
  preview_full_contact_service_ = create_service<Trigger>(
    "~/preview_full_contact",
    std::bind(
      &ApproachGraspNode::onPreviewFullContact, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(), planning_callback_group_);
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
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(), planning_callback_group_);
  arm_service_ = create_service<SetBool>(
    "~/set_execution_armed",
    std::bind(
      &ApproachGraspNode::onArm, this,
      std::placeholders::_1, std::placeholders::_2));
  tool_io_client_ = create_client<aubo_msgs::srv::SetIO>(
    "/aubo_io_controller/set_io");
}

void ApproachGraspNode::createActions()
{
  cycle_action_server_ = rclcpp_action::create_server<ExecuteTarget>(
    this, "~/execute_target",
    std::bind(
      &ApproachGraspNode::onActionGoal, this,
      std::placeholders::_1, std::placeholders::_2),
    std::bind(
      &ApproachGraspNode::onActionCancel, this, std::placeholders::_1),
    std::bind(
      &ApproachGraspNode::onActionAccepted, this, std::placeholders::_1));
  survey_action_server_ = rclcpp_action::create_server<SurveyScene>(
    this, "~/survey_scene",
    std::bind(
      &ApproachGraspNode::onSurveyGoal, this,
      std::placeholders::_1, std::placeholders::_2),
    std::bind(
      &ApproachGraspNode::onSurveyCancel, this, std::placeholders::_1),
    std::bind(
      &ApproachGraspNode::onSurveyAccepted, this, std::placeholders::_1));
}

void ApproachGraspNode::onTargets(
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
      // btReconfirmTarget 消费。
      out.swinging = std::find(
        item.diagnostic_flags.begin(), item.diagnostic_flags.end(),
        "target_swinging") != item.diagnostic_flags.end();
      out.tracking_status = item.tracking_status;
      out.bottom = pointToEigen(item.candidate.bag_bottom);
      out.neck = pointToEigen(item.candidate.bag_neck);
      out.axis = vectorToEigen(item.candidate.translation_direction);
      out.entry_pose = poseToEigen(item.candidate.entry_pose);
      out.suggested_travel_m = item.candidate.suggested_travel_m;
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
  last_snapshot_id_ = std::to_string(message->snapshot_id);
  last_observation_count_ = static_cast<uint32_t>(message->observations.size());
  last_target_set_locked_ = message->target_set_locked;
  std::string bind_id = message->selected_target_id;
  if (!cycle_target_id_.empty()) {
    bind_id = cycle_target_id_;
  }
  auto selected = std::find_if(
    message->observations.begin(), message->observations.end(),
    [&bind_id](const auto & item) {
      return item.target_id == bind_id;
    });
  if (selected == message->observations.end()) {
    return;
  }
  // 消息字段提取（含 ROS 类型）留在节点薄壳，ID 一致性调和委托 cache_ 纯核。
  SelectedTargetUpdate update;
  update.selected_id = bind_id;
  update.harvest_run_id = message->harvest_run_id;
  extract(*selected, update);
  cache_.updateSelectedTarget(update);
}

void ApproachGraspNode::trackFrameInterval()
{
  const double arrival_s = now().seconds();
  if (last_targets_arrival_s_ > 0.0) {
    const double dt = arrival_s - last_targets_arrival_s_;
    // 异常间隔（暂停后首帧/时钟跳变）不进 EMA，避免污染帧率估计
    if (dt > 1e-3 && dt < 30.0) {
      frame_interval_ema_s_ = frame_interval_ema_s_ > 0.0 ?
        0.7 * frame_interval_ema_s_ + 0.3 * dt : dt;
    }
  }
  last_targets_arrival_s_ = arrival_s;
}

double ApproachGraspNode::effectiveFrameWaitS() const
{
  if (frame_interval_ema_s_ <= 0.0) {return frame_wait_s_;}
  // 视点到位后等 ~4 帧 + 1s 稳定余量；下限 2s，上限为配置值
  return adaptive_timeout_s(frame_interval_ema_s_, 4.0, 1.0, 2.0, frame_wait_s_);
}

double ApproachGraspNode::effectiveTargetMaxAgeS() const
{
  if (frame_interval_ema_s_ <= 0.0) {return target_observation_max_age_config_s_;}
  // 新鲜度按 ~2.5 帧 + 0.5s 余量放宽：低帧率下丢 1 帧不误判 stale
  return adaptive_timeout_s(frame_interval_ema_s_, 2.5, 0.5, 1.0, 10.0);
}

double ApproachGraspNode::effectiveReconfirmWaitS() const
{
  if (frame_interval_ema_s_ <= 0.0) {return reconfirm_wait_s_;}
  // 与视点等帧同一形状：等 ~4 帧 + 1s 稳定余量，下限 2s，上限为配置值
  // （reconfirm_wait_s 同时承担回退值与自适应上限，摆动等平息也在本窗口预算内）。
  return adaptive_timeout_s(frame_interval_ema_s_, 4.0, 1.0, 2.0, reconfirm_wait_s_);
}

double ApproachGraspNode::effectiveRefinedWaitS() const
{
  if (frame_interval_ema_s_ <= 0.0) {return refined_timeout_s_;}
  // 协议 2.7-FINALIZE 的 T(refined)=clamp(下限, 3×实测refit耗时EMA, 上限)：
  // refit 耗时由重建节点持有、本包不可得（本阶段不动其他包接口），按观测帧
  // 间隔近似——finalize 触发后 refit 在后续约 3 帧内闩锁发布 refined，
  // 故 ≈3 帧 + 2s 余量；下限 2s（高帧率时 refit 仍有固定计算耗时），上限为
  // 配置值（同时是未测得帧率时的回退值）。
  return adaptive_timeout_s(
    frame_interval_ema_s_, 3.0, 2.0, std::min(2.0, refined_timeout_s_),
    refined_timeout_s_);
}

void ApproachGraspNode::onDiagnostics(
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

void ApproachGraspNode::onDecision(
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

void ApproachGraspNode::onRefinedPose(
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

void ApproachGraspNode::onRefinedDiagnostics(
  const peach_interfaces::msg::BagFittingArray::SharedPtr message)
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
  update.accepted = fitting.status == peach_interfaces::msg::BagFitting::ACCEPT;
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
      // 预规划清场（2.13-E3）：任何终局路径统一丢弃预规划槽/任务并回收线程
      // （取消路径下 plan 已被 cancel→preempt 促退，join 有界）。
      cleanupPreplan();
      // 降级抓取（精化未达标回退候选锚点）的终局 reason 一律带 degraded_anchor
      // 标记：编排器记账与批次统计据此识别降级抓占比（协议 2.7 降级链记账）。
      const std::string final_message = cycle_degraded_grasp_ ?
        message + "；降级抓取(degraded_anchor)" : message;
      // 先落终态再解除 running：executeAction 以 running==false 作为周期结束信号，
      // 这样它读到的状态一定是终态而不是上一个中间态。
      setState(state, final_message);
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

void ApproachGraspNode::startCycleTiming()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  stage_timer_.start(StageTimer::Clock::now());
}

void ApproachGraspNode::fillStageDurations(
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

void ApproachGraspNode::fillExecuteResults(
  const std::shared_ptr<ExecuteTarget::Result> & result)
{
  const bool observe_only = cycle_observe_only_.load();
  const bool succeeded =
    result->outcome == ExecuteTarget::Result::SUCCEEDED;
  result->harvest.grasped = succeeded && !observe_only;
  result->harvest.reason = result->reason;
  result->deposit.deposited = cycle_deposit_ok_;
  result->deposit.reason = cycle_deposit_reason_;
  if (observe_only) {
    result->verification.passed = succeeded;
    result->verification.reason = result->reason;
  } else {
    result->verification.passed =
      result->harvest.grasped &&
      (cycle_deposit_ok_ || cycle_deposit_skipped_m8_);
    result->verification.reason = cycle_deposit_reason_.empty() ?
      result->reason : cycle_deposit_reason_;
  }
  result->outcome_record.target_id = cycle_target_id_;
  result->outcome_record.outcome = result->outcome;
  result->outcome_record.reason = result->reason;
}

}  // namespace peach_manipulation_skills
