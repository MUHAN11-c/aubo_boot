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
#include "peach_arm/manipulation_skills_node.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <tf2/time.h>
#include <moveit/robot_state/cartesian_interpolator.hpp>
#include <moveit/collision_detection/collision_matrix.hpp>
#include <moveit/collision_detection_fcl/collision_env_fcl.hpp>
#include "peach_arm/eigen_conversions.hpp"
#include "peach_arm/grasp_geometry.hpp"
#include "peach_arm/model_contract.hpp"
#include "peach_arm/params_bridge.hpp"
#include "peach_arm/staging_selector.hpp"
#include <peach_arm/arm_parameters.hpp>

namespace peach_arm
{

// staging IK 自碰环境池（W5-2）：每 roll 任务一个 CollisionEnvFCL，从
// RobotModel + SRDF 相邻豁免 ACM 构造（无跨调用状态），首次调用构造后
// 跨调用复用；机器人模型变化时按模型指针重建。仅在周期规划路径串行访问。
struct ManipulationSkillsNode::StagingIkEnvironment
{
  moveit::core::RobotModelConstPtr model;
  collision_detection::AllowedCollisionMatrix acm;
  std::vector<std::shared_ptr<collision_detection::CollisionEnvFCL>> envs;

  void ensure(const moveit::core::RobotModelConstPtr & robot_model, std::size_t count)
  {
    if (model == robot_model && envs.size() == count && !envs.empty()) {
      return;
    }
    model = robot_model;
    acm = collision_detection::AllowedCollisionMatrix(*robot_model->getSRDF());
    envs.assign(count, {});
    for (auto & env : envs) {
      env = std::make_shared<collision_detection::CollisionEnvFCL>(robot_model);
    }
  }
};

ManipulationSkillsNode::ManipulationSkillsNode(const rclcpp::NodeOptions & options)
: LifecycleNode("peach_arm", options),
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
    "peach_arm_moveit", moveit_options);
  // ParamListener（generate_parameter_library，arm_parameters.yaml）构造即
  // 声明全部参数并做启动校验（yaml 覆盖值非法时抛异常直接启动失败），范围
  // 校验随每次 set 生效；部署值源为 config/peach_arm.yaml。
  param_listener_ = std::make_shared<peach_arm::ParamListener>(
    get_node_parameters_interface(), get_logger());
  robot_status_contract_timeout_s_ =
    param_listener_->get_params().execution_contract.robot_status_timeout_s;
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
  // 使能心跳看门狗（缺心跳=故障）：与 onEnables 同在默认互斥组，无锁安全。
  enables_watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&ManipulationSkillsNode::checkEnablesHeartbeat, this));
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
    // 诊断双轨（W5-10）：/diagnostics 1Hz（~/status 不动）。与生命周期实体
    // 同纪律——configure 创建、cleanup 释放；Unconfigured 期零 ROS 接口。
    diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this, 1.0);
    diagnostics_->setHardwareID("peach_arm");
    diagnostics_->add(
      "perception_stream",
      [this](diagnostic_updater::DiagnosticStatusWrapper & st) {
        reportStreamDiagnostics(st);
      });
    diagnostics_->add(
      "target_cache",
      [this](diagnostic_updater::DiagnosticStatusWrapper & st) {
        reportTargetCacheDiagnostics(st);
      });
    diagnostics_->add(
      "callback_timing",
      [this](diagnostic_updater::DiagnosticStatusWrapper & st) {
        reportCallbackTimingDiagnostics(st);
      });
    diagnostics_->add(
      "contact_monitor",
      [this](diagnostic_updater::DiagnosticStatusWrapper & st) {
        reportContactMonitorDiagnostics(st);
      });
    diagnostics_->add(
      "enables",
      [this](diagnostic_updater::DiagnosticStatusWrapper & st) {
        reportEnablesDiagnostics(st);
      });
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
  startBond();
  // ③层工具×octomap ACM 豁免：后台线程一次应用（含最多 4s 服务等待，
  // 不得占激活回调；static 入口无对象生命周期依赖）。Survey/观察/接近
  // 全程生效——09-17 真机实锤：不豁免则眼在手上 self-filter 漏收的工具
  // 点云会让臂停在任意视点位后所有规划自碰死锁。豁免清单=工具档案
  // tool.links（W5-6 参数化）。
  {
    const auto logger = get_logger();
    const std::vector<std::string> tool_links = params_.tool.links;
    std::thread([logger, tool_links]() {
        moveit::planning_interface::PlanningSceneInterface scene;
        GraspTask::applyWholeOctomapToolExemption(logger, scene, tool_links);
      }).detach();
  }
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
  stopBond();
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

void ManipulationSkillsNode::clearCancelFlagIfIdle()
{
  // M1：取消旗标此前唯一复位点是下一 ExecuteTarget 周期 onStart，一次单果
  // 取消/skip 后 sticky 旗标会把后续一切 MoveTo/观察拒之门外。三动作
  // （ExecuteTarget/Survey/MoveTo）终局各自调用本收口；只在周期 worker 已
  // 落终态（running_=false，取消不再向周期内传播）时清，避免取消传播中
  // 过早清（在途的其它取消经 requestCancelAll 已即时停运动，清旗不复活
  // 任何被停的运动；各动作自身终局另判 is_canceling）。
  if (!running_.load()) {
    cancel_requested_.store(false);
  }
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
  if (move_to_thread_.joinable()) {
    move_to_thread_.join();
  }
}

void ManipulationSkillsNode::startBond()
{
  if (bond_) {
    return;  // 重激活路径：心跳已在（on_deactivate 已断则此处为 null）
  }
  // W14：bondcpp 生命周期构造器——发布者走 LifecyclePublisher，Inactive 期
  // 自动静默，因此只在 on_activate 启动；bond_timeout=0 期无观察者亦无害。
  bond_ = std::make_unique<bond::Bond>("/bond", get_name(), shared_from_this());
  bond_->start();
}

void ManipulationSkillsNode::stopBond()
{
  if (bond_) {
    bond_->breakBond();
    bond_.reset();
  }
}

void ManipulationSkillsNode::releaseResources()
{
  // 与 createInterfaces/initializeMoveIt 对称；参数声明、验证钩子与
  // view_planner_/quality_gate_/safety_gate_ 纯核保留（再次 configure 时
  // loadParameters 重建），contact_recovery_required_ 跨清理保持。
  stopBond();
  cycle_action_server_.reset();
  survey_action_server_.reset();
  move_to_action_server_.reset();
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
  joint_status_sub_.reset();
  contact_guard_timer_.reset();
  diagnostics_.reset();
  status_pub_.reset();
  marker_pub_.reset();
  grasp_hyp_pub_.reset();
  tool_io_client_.reset();
  imu_follow_enable_client_.reset();
  imu_follow_disable_client_.reset();
  imu_follow_insert_start_client_.reset();
  imu_follow_insert_stop_client_.reset();
  imu_follow_insert_retract_client_.reset();
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
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    moveit_node_, params_.moveit.planning_group, std::shared_ptr<tf2_ros::Buffer>(),
    rclcpp::Duration::from_seconds(5.0));
  move_group_->setPoseReferenceFrame(params_.frames.base);
  move_group_->setPlanningTime(params_.moveit.planning_time_s);
  move_group_->setNumPlanningAttempts(
    static_cast<int>(params_.moveit.planning_attempts));
  move_group_->setMaxVelocityScalingFactor(params_.moveit.transit_velocity_scaling);
  move_group_->setMaxAccelerationScalingFactor(
    params_.moveit.transit_acceleration_scaling);
  move_group_->allowReplanning(true);
  rebuildMotionInterface();
  rebuildGraspTask();
  RCLCPP_INFO(
    get_logger(),
    "主动视觉靠近节点 ready: group=%s base=%s tip=%s camera=%s "
    "execution=%s grasp=%s",
    params_.moveit.planning_group.c_str(), params_.frames.base.c_str(),
    params_.frames.tip.c_str(), params_.frames.camera.c_str(),
    execution_enabled_.load() ? "enabled" : "plan_only",
    grasp_enabled_.load() ? "enabled" : "disabled");
}

void ManipulationSkillsNode::loadParameters()
{
  params_ = param_listener_->get_params();
  const auto & params = params_;
  // Config 值字段经 params_bridge 单点转换（W13-B）；保护区解析需逐盒
  // WARN 日志，留在节点装配。
  ViewPlannerConfig view_config = toViewPlannerConfig(params);
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

  quality_gate_ = std::make_unique<QualityGate>(toQualityGateConfig(params));
  safety_gate_ = std::make_unique<SafetyGate>(
    toSafetyGateConfig(params), [this]() {return now().seconds();});
  frame_timeouts_.updateConfig(toFrameRateTimeoutConfig(params_));

  // 广播源在权时本地参数不得覆盖使能（Enables.msg 契约：收到即覆盖；
  // 断流超时由 checkEnablesHeartbeat 回落后本地值才重新生效）。
  enables_heartbeat_timeout_s_ = params.execution.enables_heartbeat_timeout_s;
  if (!enables_external_) {
    execution_enabled_.store(params.execution.enabled);
    grasp_enabled_.store(params.grasp.enabled);
    tool_enabled_.store(params.tool.enabled);
  }
  // 接触止损配置经 params_bridge 单点转换（W13-B）；阈值默认关，
  // 须真机受控试验标定后启用。
  contact_detect_config_ = toContactDetectConfig(params);
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
  // Config 值字段经 params_bridge 单点转换（W5-1）；节点只装配安全门与
  // 状态投影回调。执行闸门（A8/I5）：运动输出权限（Active 态）叠加
  // 硬件安全门回调注入——即授权矩阵的 TRANSIT 级底座（任何 execute 路径
  // 不得旁路；plan-only 路径不经其执行段）；safety_block_hook 保持原
  // planOrMoveTip 被拦下时的 FAILED 状态投影。CONTACT/TOOL 级在阶段函数
  // （stages.cpp）与 GraspTask 门显式加查。
  if (motion_ && !motion_->executionIdle()) {
    // 运动在途（MoveTo 不置 running_）：销毁 motion_ 会 UAF 等待环（P1-1），
    // 放弃本次改参重建，下次空闲改参再生效。
    RCLCPP_WARN(get_logger(), "运动在途，跳过 motion 接口重建（改参待空闲生效）");
    return;
  }
  motion_ = std::make_unique<MoveItMotionInterface>(
    move_group_, &tf_buffer_, get_logger(), get_clock(),
    toMotionConfig(params_),
    [this](std::string & reason) {
      return motionOutputAllowed(reason) && safetyReady(reason);
    },
    [this](const std::string & message) {
      setState(CycleState::FAILED, message);
    },
    [this] {return cancel_requested_.load();});
}

void ManipulationSkillsNode::rebuildGraspTask()
{
  if (!moveit_node_ || !motion_) {
    return;
  }
  GraspTaskConfig task_config = toGraspTaskConfig(params_);
  // 滚转扫描 + 多 IK 种子：圆筒刀口滚转是自由参数，但只扫 keep-roll 及
  // ±30°/±60°。更大滚转会让 PTP 把 TCP 拧过 90°+。只把最近支位姿交给
  // 笛卡尔接近当目标姿态，不用关节目标下发 PTP（1740 跨构型绕行）。
  // 候选编排（权重/惩罚/top_n）在 StagingCandidateSelector 纯核
  // （staging_selector.hpp，W5-2）；本回调只供给 MoveIt 侧 IK/自碰探测：
  // KDL 互斥在回调内，每 roll 一个池内 CollisionEnvFCL（跨调用复用）。
  const StagingSelectorConfig staging_config = toStagingSelectorConfig(params_);
  task_config.select_goal_joints =
    [this, staging_config](const Eigen::Isometry3d & keep_roll_pose)
    -> std::vector<GraspTaskConfig::StagingCandidate>
    {
      if (!move_group_) {
        return {};
      }
      const auto base = move_group_->getCurrentState();
      const auto * group =
        base->getJointModelGroup(params_.moveit.planning_group);
      if (group == nullptr) {
        return {};
      }
      const auto names = group->getActiveJointModelNames();
      std::vector<double> current;
      base->copyJointGroupPositions(group, current);
      if (!staging_ik_env_) {
        staging_ik_env_ = std::make_unique<StagingIkEnvironment>();
      }
      const auto rolls = toolRollsRad();
      staging_ik_env_->ensure(base->getRobotModel(), rolls.size());
      StagingIkEnvironment & env_pool = *staging_ik_env_;
      std::mutex ik_mutex;
      const StagingIkSolve solve =
        [&](int roll_index, const Eigen::Isometry3d & pose, int attempt)
        -> std::optional<std::vector<double>>
        {
          moveit::core::RobotState probe = *base;
          if (attempt > 0) {
            probe.setToRandomPositions(group);
          }
          bool ik_ok = false;
          {
            std::lock_guard<std::mutex> lock(ik_mutex);
            // 深搜档单次超时（单源 staging_selector.hpp；选果预检的
            // kQuickIkProbeTimeoutS 减半预算）。
            ik_ok = probe.setFromIK(
              group, pose, params_.frames.tip, kStagingIkSolveTimeoutS);
          }
          if (!ik_ok) {
            return std::nullopt;
          }
          probe.update();
          if (!probe.satisfiesBounds(group)) {
            return std::nullopt;
          }
          collision_detection::CollisionRequest collision_request;
          collision_detection::CollisionResult collision_result;
          env_pool.envs[static_cast<std::size_t>(roll_index) %
            env_pool.envs.size()]->checkSelfCollision(
            collision_request, collision_result, probe, env_pool.acm);
          if (collision_result.collision) {
            return std::nullopt;
          }
          std::vector<double> sol;
          probe.copyJointGroupPositions(group, sol);
          return sol;
        };
      return selectStagingCandidates(
        staging_config, keep_roll_pose, current, names, rolls, solve);
    };
  task_config.lookup_current_tip = [this]() {
      return motion_->lookupTransform(params_.frames.base, params_.frames.tip);
    };
  task_config.protected_zones = protected_zones_;
  // 执行边界 = 授权矩阵公共级 + execution + grasp（GraspTask 门内显式加查）；
  // 目标身份/新鲜度与 GraspDecision 复检由阶段执行器单点判定
  // （ExecuteTarget.goal.target_id 钉死；套入/剪切入口 requireStageAuthority）。
  // 撤离不依赖视觉：插入后目标常被工具遮挡、收割后决策可能翻转，撤退门
  // 不做决策复检（见 cycle_support.hpp 矩阵注释）。
  task_config.approach_execution_gate = [this](std::string & reason) {
      if (!(motionOutputAllowed(reason) && safetyReady(reason) &&
        !cancel_requested_.load() && execution_enabled_.load() &&
        grasp_enabled_.load()))
      {
        return false;
      }
      const auto snap = cache_.modelSnapshot();
      if (identityComplete(snap.identity) &&
        !modelExecutable(snap, now().seconds(), false))
      {
        reason = "model_not_executable";
        return false;
      }
      return true;
    };
  task_config.retreat_execution_gate = task_config.approach_execution_gate;
  // MTC 执行超时兜底：MGI::stop 打 move_group 节点级停止服务，不依赖
  // MTC 自己的接口实例。
  task_config.execution_stop = [this] {motion_->stopExecution();};
  grasp_task_ = std::make_unique<GraspTask>(moveit_node_, task_config);
}

rcl_interfaces::msg::SetParametersResult ManipulationSkillsNode::onParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  // 纯验证钩子（on-set 阶段，无副作用）：范围校验由手写 ParamListener
  // （params.hpp）内置完成，本钩子只负责"运行中拒改"与 execution→grasp→tool 依赖链；
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
  joint_status_sub_ = create_subscription<aubo_msgs::msg::JointStatus>(
    "/aubo_io_controller/joint_status", 10,
    std::bind(&ManipulationSkillsNode::onJointStatus, this, std::placeholders::_1));
  status_pub_ = create_publisher<std_msgs::msg::String>("~/status", latched);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/planned_views", latched);
  grasp_hyp_pub_ = create_publisher<peach_interfaces::msg::GraspHypothesis>(
    "/peach/manipulation/grasp_hypothesis", latched);
  // 操作台使能广播（清洁重写轮）：无发布者时静默，本地参数保持权威。
  enables_sub_ = create_subscription<peach_interfaces::msg::Enables>(
    "/peach/batch/enables", latched,
    std::bind(&ManipulationSkillsNode::onEnables, this, std::placeholders::_1));
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
  // 仅自适应档案建客户端。空心也 create_client 会在 FastDDS 图上挂出
  // /imu_follow/* 名（无服务端），隔离验收会误判栈已起跟随。
  if (usesImuFollowContact()) {
    imu_follow_enable_client_ = create_client<Trigger>("/imu_follow/enable");
    imu_follow_disable_client_ = create_client<Trigger>("/imu_follow/disable");
    imu_follow_insert_start_client_ = create_client<Trigger>(
      "/imu_follow/insert_start");
    imu_follow_insert_stop_client_ = create_client<Trigger>(
      "/imu_follow/insert_stop");
    imu_follow_insert_retract_client_ = create_client<Trigger>(
      "/imu_follow/insert_retract");
  }
  tool_actuator_.setSendIo(
    [this](std::string & reason) {
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
  move_to_action_server_ = rclcpp_action::create_server<MoveToAction>(
    this, "~/move_to",
    std::bind(
      &ManipulationSkillsNode::onMoveToGoal, this,
      std::placeholders::_1, std::placeholders::_2),
    std::bind(
      &ManipulationSkillsNode::onMoveToCancel, this,
      std::placeholders::_1),
    std::bind(
      &ManipulationSkillsNode::onMoveToAccepted, this,
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
      out.bag_diameter_upper_m = item.candidate.bag_diameter_upper_m;
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

// 帧率自适应超时族转发（W5-3）：公式在 FrameRateTimeouts 纯核，此处只供
// 数据（EMA/缓存状态）；调用点与语义不变。
void ManipulationSkillsNode::trackFrameInterval()
{
  frame_timeouts_.onTargetFrame(now().seconds());
}

double ManipulationSkillsNode::effectiveFrameWaitS() const
{
  return frame_timeouts_.frameWaitS();
}

double ManipulationSkillsNode::effectiveTargetMaxAgeS() const
{
  return frame_timeouts_.targetMaxAgeS();
}

double ManipulationSkillsNode::effectiveReconfirmWaitS() const
{
  return frame_timeouts_.reconfirmWaitS();
}

double ManipulationSkillsNode::effectiveRefinedWaitS() const
{
  // 窗口态感知（2026-09-09 真机）：COLLECTING 用配置上限覆盖采集→finalize
  // →refit 全程（公式与分支见 frame_timeouts.hpp 注释）。
  return frame_timeouts_.refinedWaitS(
    cache_.qualitySnapshot().reconstruction_state == "COLLECTING");
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
  ModelIdentity identity;
  identity.run_id = message->harvest_run_id;
  identity.scene_epoch = message->scene_epoch;
  identity.target_id = message->target_id;
  identity.model_revision = message->model_revision;
  identity.tool_profile_id = message->tool_profile_id;
  identity.calibration_revision = message->calibration_revision;
  identity.config_revision = message->config_revision;
  const bool derived = allowedFromCapabilities(
    static_cast<Capability>(message->geometry_capability),
    static_cast<Capability>(message->sleeve_capability),
    static_cast<Capability>(message->cut_capability));
  const bool allowed = derived && identityComplete(identity);
  if (identityComplete(identity)) {
    const auto current = cache_.modelSnapshot();
    if (current.identity.model_revision != identity.model_revision ||
      current.identity.target_id != identity.target_id)
    {
      ModelSnapshot snap;
      snap.identity = identity;
      snap.generated_s = now().seconds();
      snap.valid_until_s = static_cast<double>(message->valid_until.sec) +
        1e-9 * static_cast<double>(message->valid_until.nanosec);
      // 心跳/缺字段不得续签；空有效期保持 generated==valid_until → 拒执行。
      snap.geometry = static_cast<Capability>(message->geometry_capability);
      snap.pregrasp = static_cast<Capability>(message->pregrasp_capability);
      snap.sleeve = static_cast<Capability>(message->sleeve_capability);
      snap.cut = static_cast<Capability>(message->cut_capability);
      cache_.replaceModelSnapshot(snap);
    }
  }
  if (!cache_.updateGraspDecision(identity, allowed)) {
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
  update.bag_diameter_upper_m = candidate.bag_diameter_upper_m;
  update.accepted = candidate.status == peach_interfaces::msg::BagGraspCandidate::ACCEPT;
  std::string reject_reason;
  if (!cache_.updateRefinedPose(update, &reject_reason)) {
    // unrefined_hold 是 skip_reconstruction 批的常态（latched 发布 0.5s
    // 心跳），不节流会把日志刷爆并淹没真信号（2026-09-23 单轮 8 万行）。
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "忽略 refined pose（%s）: gate=%s actual=%s", reject_reason.c_str(),
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
  std::string reject_reason;
  if (!cache_.updateRefinedFitting(update, &reject_reason)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "忽略 refined diagnostics（%s）: expected=%s actual=%s",
      reject_reason.c_str(), cache_.expectedFittingTargetId().c_str(),
      fitting.target_id.c_str());
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

// ---- 诊断双轨任务（W5-10）----
// 级别口径：OK=正常观测；WARN=数据陈旧/降级（周期仍可自行判停）；
// ERROR=通道断流/授权异常。只读投影，不做任何控制决策；不与 ~/status
// 互相替代（web 消费端不动）。

void ManipulationSkillsNode::reportStreamDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  using Status = diagnostic_msgs::msg::DiagnosticStatus;
  const double now_s = now().seconds();
  const double last_arrival = frame_timeouts_.lastArrivalS();
  const double arrival_age_s = last_arrival > 0.0 ? now_s - last_arrival : -1.0;
  const double ema_s = frame_timeouts_.frameIntervalEmaS();
  status.add("frame_interval_ema_s", ema_s);
  status.add("last_arrival_age_s", arrival_age_s);
  status.add("frame_wait_s", frame_timeouts_.frameWaitS());
  status.add("target_max_age_s", frame_timeouts_.targetMaxAgeS());
  status.add("refined_wait_s", frame_timeouts_.refinedWaitS(
      cache_.qualitySnapshot().reconstruction_state == "COLLECTING"));
  // base<-camera 最新 TF 可用性（0 超时探测，不阻塞）：视点生成/接近几何
  // 都依赖这条链，不可用即 ERROR。先核帧名是否已在树上（camera_enabled
  // =false 时相机帧不存在），避免 canTransform 对无效帧名刷 tf2 WARN。
  const auto frames = tf_buffer_.getAllFrameNames();
  const bool base_known =
    std::find(frames.begin(), frames.end(), params_.frames.base) != frames.end();
  const bool camera_known =
    std::find(frames.begin(), frames.end(), params_.frames.camera) != frames.end();
  const bool tf_ok = base_known && camera_known &&
    tf_buffer_.canTransform(
    params_.frames.base, params_.frames.camera, tf2::TimePointZero,
    std::chrono::milliseconds(0));
  status.add("tf_base_to_camera_ok", tf_ok ? 1 : 0);
  if (!camera_known) {
    status.summary(Status::WARN, "相机 TF 帧未发布（camera_enabled=false 或外参未起）");
    return;
  }
  if (!tf_ok) {
    status.summary(Status::ERROR, "base<-camera TF 不可用");
    return;
  }
  if (arrival_age_s < 0.0) {
    status.summary(Status::WARN, "尚未收到目标观测帧");
    return;
  }
  if (arrival_age_s > frame_timeouts_.targetMaxAgeS()) {
    status.summary(Status::WARN, "目标观测流陈旧");
    return;
  }
  status.summary(Status::OK, "观测流与 TF 正常");
}

void ManipulationSkillsNode::reportTargetCacheDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  using Status = diagnostic_msgs::msg::DiagnosticStatus;
  const QualitySnapshot snapshot = qualitySnapshot();
  status.add("reconstruction_state", snapshot.reconstruction_state);
  status.add("data_age_s", snapshot.data_age_s);
  status.add("captured_views",
    static_cast<int>(snapshot.captured_views));
  status.add("station_count", static_cast<int>(snapshot.station_count));
  status.add("selected_target_id", snapshot.selected_target_id);
  status.add("refined_target_id", snapshot.refined_target_id);
  const bool id_consistent =
    snapshot.selected_target_id.empty() ||
    snapshot.refined_target_id.empty() ||
    snapshot.selected_target_id == snapshot.refined_target_id;
  if (!id_consistent) {
    status.summary(Status::WARN, "selected 与精化目标 ID 不一致");
    return;
  }
  status.summary(Status::OK, "目标缓存正常");
}

void ManipulationSkillsNode::reportCallbackTimingDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  using Status = diagnostic_msgs::msg::DiagnosticStatus;
  // TopN 耗时（按 max_ms 降序取前 5；数值口径与 ~/status 的
  // callback_timing 投影一致——复用同一注册表，不新增计时点）。
  auto entries = callback_timing_.snapshot();
  std::vector<std::pair<std::string, CallbackTimingRegistry::Entry>> top(
    entries.begin(), entries.end());
  std::sort(
    top.begin(), top.end(),
    [](const auto & a, const auto & b) {
      return a.second.max_ms > b.second.max_ms;
    });
  std::size_t shown = 0;
  for (const auto & [label, entry] : top) {
    if (shown++ >= 5U) {
      break;
    }
    status.add(label + ".count", static_cast<int>(entry.count));
    status.add(label + ".last_ms", entry.last_ms);
    status.add(label + ".max_ms", entry.max_ms);
  }
  status.summary(Status::OK, "回调耗时 TopN");
}

void ManipulationSkillsNode::reportContactMonitorDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  using Status = diagnostic_msgs::msg::DiagnosticStatus;
  status.add("enabled", contact_detect_config_.enabled ? 1 : 0);
  status.add("baseline_s", contact_detect_config_.baseline_s);
  status.add("slope_threshold", contact_detect_config_.slope_threshold);
  status.add("spike_threshold", contact_detect_config_.spike_threshold);
  status.add("abort_suspected", contactAbortSuspected() ? 1 : 0);
  std::size_t sample_count = 0;
  {
    std::lock_guard<std::mutex> lock(joint_current_mutex_);
    sample_count = joint_current_samples_.size();
  }
  status.add("current_sample_count", static_cast<int>(sample_count));
  if (contactAbortSuspected()) {
    status.summary(Status::ERROR, "疑似硬接触（已取消执行，须现场确认后 ACK）");
    return;
  }
  status.summary(
    Status::OK, contact_detect_config_.enabled ? "接触检测运行中" : "接触检测关闭（仅缓存）");
}

void ManipulationSkillsNode::reportEnablesDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  using Status = diagnostic_msgs::msg::DiagnosticStatus;
  const char * source = enables_external_ ? "console_broadcast" : "local_params";
  double heartbeat_age_s = -1.0;
  if (enables_external_) {
    heartbeat_age_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - enables_last_beat_).count();
  }
  status.add("source", source);
  status.add("heartbeat_age_s", heartbeat_age_s);
  status.add("heartbeat_timeout_s", enables_heartbeat_timeout_s_);
  status.add("execution_enabled", execution_enabled_.load() ? 1 : 0);
  status.add("grasp_enabled", grasp_enabled_.load() ? 1 : 0);
  status.add("tool_enabled", tool_enabled_.load() ? 1 : 0);
  status.add("execution_armed", execution_armed_.load() ? 1 : 0);
  status.add("motion_output_permitted", motion_output_permitted_.load() ? 1 : 0);
  status.add("contact_recovery_required", contact_recovery_required_.load() ? 1 : 0);
  if (enables_external_ && enables_heartbeat_timeout_s_ > 0.0 &&
    heartbeat_age_s > enables_heartbeat_timeout_s_)
  {
    status.summary(Status::WARN, "操作台使能广播心跳超时（看门狗将回落本地参数）");
    return;
  }
  status.summary(Status::OK, std::string("使能正常（源=") + source + "）");
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
  // M3a：受理期拒单（plan mismatch）发生在 ctx 创建之前，读 pending 成员
  // 把码带出（每 goal 于 executeAction 入口复位）；PREVIEW 模式中断路径
  // 同为 !ctx，但该成员恒 0，行为与原状一致。
  result->failure_code = succeeded ?
    0u : (ctx != nullptr ? ctx->failure_code : pending_accept_failure_code_);
  // W7：顶层 bool 镜像已删；cut/retreat/harvest 证据单源 harvest/verification 块。
  const bool harvest_confirmed = harvestConfirmed(
    ctx && ctx->cut_confirmed, ctx && ctx->retreat_confirmed);
  result->pregrasp = ctx ? ctx->pregrasp_msg : peach_interfaces::msg::PregraspVerification{};
  result->harvest.completion_level = result->completion_level;
  result->harvest.commanded =
    grasp_enabled_.load() && !observe_only && !pregrasp_only;
  result->harvest.confirmed = harvest_confirmed;
  result->harvest.grasped = harvest_confirmed;
  result->harvest.reason = result->reason;
  if (!tool_enabled_.load() && result->harvest.commanded) {
    result->harvest.reason += "；tool.enabled=false，跳过末端 IO";
  }
  if (observe_only || pregrasp_only) {
    result->verification.passed = succeeded;
    result->verification.commanded = false;
    result->verification.confirmed =
      pregrasp_only && ctx->pregrasp_verified;
    result->verification.harvest_confirmed = false;
    result->verification.reason = result->reason;
    result->verification.failure_code = result->failure_code;
  } else {
    // FULL：验证随采摘确认走（M8 卸果预留不随 Result 携带）。
    result->verification.passed = result->harvest.grasped;
    result->verification.commanded = result->harvest.commanded;
    result->verification.confirmed = result->harvest.confirmed;
    result->verification.harvest_confirmed = harvest_confirmed;
    result->verification.reason = result->reason;
    result->verification.failure_code = result->failure_code;
  }
  result->outcome_record.target_id = ctx ? ctx->target_id : std::string();
  result->outcome_record.outcome = result->outcome;
  result->outcome_record.reason = result->reason;
}

}  // namespace peach_arm
