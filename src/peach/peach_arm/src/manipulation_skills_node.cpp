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
#include <moveit/robot_state/cartesian_interpolator.hpp>
#include <tf2/time.h>
#include <moveit/collision_detection/collision_matrix.hpp>
#include <moveit/collision_detection_fcl/collision_env_fcl.hpp>
#include "peach_arm/eigen_conversions.hpp"
#include "peach_arm/acm_policy.hpp"
#include "peach_arm/grasp_geometry.hpp"
#include "peach_arm/model_contract.hpp"
#include "peach_arm/params_bridge.hpp"
#include "peach_arm/thread_join.hpp"
#include <peach_arm/arm_parameters.hpp>


namespace peach_arm
{

// staging IK 自碰环境（v1 StagingIkEnvironment 精简复刻：单 FCL 环境 +
// SRDF 相邻豁免 ACM；模型变更重建）。随机重启/滚转候选先过自碰再择优
// （2026-09-23 v4 实测 10/20：0° 首解构型过不了 PTP ValidateSolution，
// v1 的择优机制是 86% 的真正来源）。
struct ManipulationSkillsNode::StagingIkEnvironment
{
  moveit::core::RobotModelConstPtr model;
  collision_detection::AllowedCollisionMatrix acm;
  std::shared_ptr<collision_detection::CollisionEnvFCL> env;

  void ensure(const moveit::core::RobotModelConstPtr & robot_model)
  {
    if (model == robot_model && env) {
      return;
    }
    model = robot_model;
    acm = collision_detection::AllowedCollisionMatrix(*robot_model->getSRDF());
    env = std::make_shared<collision_detection::CollisionEnvFCL>(robot_model);
  }
};

ManipulationSkillsNode::ManipulationSkillsNode(const rclcpp::NodeOptions & options)
: LifecycleNode("peach_arm", options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  cache_([this]() {return now().seconds();})
{
  cycle_logger_ = get_logger().get_child("cycle");
  contact_logger_ = get_logger().get_child("contact");
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
    diagnostics_->add(
      "robot_status",
      [this](diagnostic_updater::DiagnosticStatusWrapper & st) {
        reportRobotStatusDiagnostics(st);
      });
    diagnostics_->add(
      "tool_chain",
      [this](diagnostic_updater::DiagnosticStatusWrapper & st) {
        reportToolChainDiagnostics(st);
      });
    diagnostics_->add(
      "moveit_availability",
      [this](diagnostic_updater::DiagnosticStatusWrapper & st) {
        reportMoveItAvailabilityDiagnostics(st);
      });
    diagnostics_->add(
      "last_denial",
      [this](diagnostic_updater::DiagnosticStatusWrapper & st) {
        reportLastDenialDiagnostics(st);
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
  // ③层场景障碍 ACM 豁免：后台线程一次应用（含最多 4s 服务等待，
  // 不得占激活回调；static 入口无对象生命周期依赖）。Survey/观察/接近
  // 全程生效。2026-09-29 起避障只为保护相机：豁免清单=tool.links 部署值
  // （全机器人−camera_body_link），唯一受查对=guard_links×障碍对象；
  // 09-17 眼在手上 self-filter 漏收工具点云的死锁由豁免+快照节点自身
  // 滤除双保险。guard 开关（moveit.obstacle_guard_enabled）改参后空闲时
  // 经 rebuildGraspTask 生效、下次规划前重写 ACM。
  {
    const auto logger = get_logger();
    const std::vector<std::string> tool_links = params_.tool.links;
    const ObstacleExemptionSpec obstacle_spec{
      params_.tool.links, params_.moveit.obstacle_guard_links,
      params_.moveit.obstacle_object_ids,
      params_.moveit.obstacle_guard_enabled};
    std::thread([logger, tool_links, obstacle_spec]() {
        // 分离线程内逃逸异常 = std::terminate（cppreference std::thread）；rcl 关停期
        // 构造 PlanningSceneInterface 会抛 RCLError（2026-09-30 实锤的 exit -6 同款）。
        try {
          moveit::planning_interface::PlanningSceneInterface scene;
          // 09-29 真相机轮勘定：激活早期服务竞态会让 apply 静默落空（ACM
          // 无条目、无 WARN），观察/接近规划起点即撞。重试 3 次——每次都
          // 先读现行 ACM 再整表合并（幂等，不会冲掉别处已写的条目）。
          for (int attempt = 1; attempt <= 3; ++attempt) {
            GraspTask::applyWholeOctomapToolExemption(
              logger, scene, tool_links, obstacle_spec);
            if (attempt < 3) {
              std::this_thread::sleep_for(std::chrono::seconds(3));
            }
          }
        } catch (const std::exception & e) {
          RCLCPP_WARN(logger, "ACM 豁免线程异常退出（可能正在 shutdown）: %s", e.what());
        }
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

void ManipulationSkillsNode::requestCancelAll(const std::string & reason)
{
  // 取消发起必须留痕（P0 观测性）：此前只能从 CANCELED 终局反推谁触发
  RCLCPP_INFO(
    get_logger(), "requestCancelAll: 停止当前执行（来源：%s）",
    reason.empty() ? "未注明" : reason.c_str());
  cancel_requested_.store(true);
  // 防御式（E2E 首跑实锤 2026-09-30）：SIGINT 后 signal_handler 已调
  // rcl_shutdown，deactivate 收口路径里的 stop/cancel（内部经 rclcpp
  // 异步口创建 guard condition）会抛 RCLError → std::terminate（exit -6）。
  // 清理路径的失败只记录，不得终止进程。
  try {
    if (move_group_) {
      move_group_->stop();
    }
    if (grasp_task_) {
      grasp_task_->cancel();
    }
  } catch (const std::exception & error) {
    RCLCPP_DEBUG(
      get_logger(), "取消清理在关停期失败（可忽略）: %s", error.what());
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
  requestCancelAll("deactivate 收口");
  // 先回收 worker（executeCycle 落定终态），再回收 action 线程——
  // executeAction 以 running==false 为周期结束信号读终态上报，反向回收会让
  // 它读到覆盖后的状态。
  // 批次7：deactivate 收口的有界回收——2s 内不落即 detach（TEM 风暴等
  // 挂死场景不得卡死 lifecycle；与 cycle.cpp 入口线程同语义）。顺序保持：
  // worker → action → survey → move_to。
  auto bound_join = [](std::thread & thread, std::future<void> & done, const char * name) {
      if (joinFinishedThread(thread, done, std::chrono::seconds(2))) {
        return;
      }
      RCLCPP_WARN(
        rclcpp::get_logger("peach_arm"),
        "deactivate 收口：线程 %s 2s 未完成，保留取消并 detach；请检查阻塞依赖", name);
      thread.detach();
    };
  bound_join(worker_, worker_done_, "worker");
  bound_join(action_thread_, action_thread_done_, "action");
  bound_join(survey_thread_, survey_thread_done_, "survey");
  bound_join(move_to_thread_, move_to_thread_done_, "move_to");
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
  planning_scene_client_.reset();
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
  task_config.contact_logger = contact_logger_;
  task_config.lookup_current_tip = [this]() {
      return motion_->lookupTransform(params_.frames.base, params_.frames.tip);
    };
  // 工具 Z 滚转为工艺自由度。通过 IK/自碰/走廊检查后，先最小化额外滚转，
  // 同等滚转再选腕轴加权关节距离。完整 PTP 仍须规划器检查场景碰撞。
  // 批次4：unrefined 链袋径门（档案内径注入；0=门关）
  cache_.setToolInnerDiameter(params_.tool.d_inner_m);

  task_config.staging_ik = [this](
    const Eigen::Isometry3d & staging_pose, const std::vector<Eigen::Isometry3d> & corridor)
    -> std::vector<StagingCandidate>
    {
      std::string target_id;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (state_json_.contains("target_id")) {
          target_id = state_json_["target_id"].get<std::string>();
        }
      }
      const auto model = cache_.modelSnapshot();
      if (!move_group_) {
        RCLCPP_WARN(
          get_logger(), "staging IK 不可用 target=%s reason=moveit_unavailable",
          target_id.c_str());
        return {};
      }
      const auto base = move_group_->getCurrentState();
      if (!base) {
        // 状态监视器断流/启动早期 getCurrentState 可为空（审查 P1-1）
        RCLCPP_WARN(
          get_logger(), "staging IK 不可用 target=%s reason=robot_state_unavailable",
          target_id.c_str());
        return {};
      }
      const auto * group =
        base->getJointModelGroup(params_.moveit.planning_group);
      if (group == nullptr) {
        RCLCPP_WARN(
          get_logger(), "staging IK 不可用 target=%s reason=invalid_group group=%s",
          target_id.c_str(), params_.moveit.planning_group.c_str());
        return {};
      }
      const auto names = group->getActiveJointModelNames();
      const Eigen::Vector3d axis = staging_pose.linear().col(2);
      const Eigen::Matrix3d current_R =
        base->getGlobalLinkTransform(params_.frames.tip).linear();
      std::vector<double> current_joints;
      base->copyJointGroupPositions(group, current_joints);
      if (!staging_ik_env_) {
        staging_ik_env_ = std::make_unique<StagingIkEnvironment>();
      }
      staging_ik_env_->ensure(base->getRobotModel());
      // 每个滚转均检查当前种子与最多 3 次重启，不采用第一个 IK 解。
      // 滚转量优先级高于关节距离，避免较短关节路径导致无必要的拧腕。
      constexpr int kRandomRestarts = 3;
      constexpr double kWristWeight = 2.5;
      int attempts = 0;
      // 每次尝试只计首个拒绝门，四项之和恒等于 attempts；valid 不是择优次数。
      int ik_failed = 0;
      int bounds_failed = 0;
      int self_collision = 0;
      int valid_candidates = 0;
      int corridor_failed = 0;
      std::vector<StagingCandidate> candidates;
      for (const double roll : toolRollsRad()) {
        Eigen::Isometry3d rolled = staging_pose;
        rolled.linear() = alignFrameZRolled(current_R, axis, roll);
        for (int attempt = 0; attempt <= kRandomRestarts; ++attempt) {
          ++attempts;
          moveit::core::RobotState probe = *base;
          if (attempt > 0) {
            probe.setToRandomPositions(group);
          }
          if (!probe.setFromIK(
              group, rolled, params_.frames.tip, kStagingIkSolveTimeoutS))
          {
            ++ik_failed;
            continue;
          }
          probe.update();
          if (!probe.satisfiesBounds(group)) {
            ++bounds_failed;
            continue;
          }
          collision_detection::CollisionRequest collision_request;
          collision_detection::CollisionResult collision_result;
          staging_ik_env_->env->checkSelfCollision(
            collision_request, collision_result, probe, staging_ik_env_->acm);
          if (collision_result.collision) {
            ++self_collision;
            continue;
          }
          const auto * tip_link = base->getRobotModel()->getLinkModel(params_.frames.tip);
          EigenSTL::vector_Isometry3d waypoints;
          for (auto waypoint : corridor) {
            waypoint.linear() = rolled.linear();
            waypoints.push_back(waypoint);
          }
          if (tip_link == nullptr || waypoints.empty()) {
            ++corridor_failed;
            continue;
          }
          const moveit::core::GroupStateValidityCallbackFn valid_state =
            [this](moveit::core::RobotState * state,
            const moveit::core::JointModelGroup * checked_group, const double * positions) {
              state->setJointGroupPositions(checked_group, positions);
              state->update();
              if (!state->satisfiesBounds(checked_group)) {
                return false;
              }
              collision_detection::CollisionRequest request;
              collision_detection::CollisionResult result;
              staging_ik_env_->env->checkSelfCollision(
                request, result, *state, staging_ik_env_->acm);
              return !result.collision;
            };
          moveit::core::CartesianPrecision precision;
          precision.translational = params_.moveit.mtc_cartesian_precision_m > 0.0 ?
            params_.moveit.mtc_cartesian_precision_m : 0.001;
          std::vector<moveit::core::RobotStatePtr> path;
          const auto fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
            &probe, group, path, tip_link, waypoints, true,
            moveit::core::MaxEEFStep(params_.moveit.mtc_cartesian_step_m),
            precision, valid_state, kinematics::KinematicsQueryOptions());
          if (path.empty() || !stagingCorridorComplete(
              fraction.value, path.back()->getGlobalLinkTransform(tip_link), waypoints.back(),
              precision.translational, precision.rotational))
          {
            ++corridor_failed;
            continue;
          }
          std::vector<double> values;
          probe.copyJointGroupPositions(group, values);
          double score = 0.0;
          for (std::size_t i = 0; i < names.size(); ++i) {
            const double delta = values[i] - current_joints[i];
            const double weight =
              names[i].find("wrist") != std::string::npos ? kWristWeight : 1.0;
            score += weight * delta * delta;
          }
          if (!std::isfinite(score)) {
            ++bounds_failed;
            continue;
          }
          ++valid_candidates;
          StagingCandidate candidate;
          candidate.roll_rad = roll;
          candidate.joint_cost = score;
          for (std::size_t i = 0; i < names.size(); ++i) {
            candidate.joints[names[i]] = values[i];
          }
          candidates.push_back(std::move(candidate));
        }
      }
      const auto & position = staging_pose.translation();
      RCLCPP_INFO(
        get_logger(),
        "%s target=%s model_target=%s model_revision=%s frame=%s group=%s tip=%s "
        "position_m=[%.4f %.4f %.4f] axis=[%.4f %.4f %.4f] "
        "attempts=%d ik_failed=%d bounds_failed=%d self_collision=%d corridor_failed=%d "
        "valid_candidates=%d",
        !candidates.empty() ? "staging IK 候选筛选成功" : "staging IK 梯子全灭",
        target_id.c_str(), model.identity.target_id.c_str(),
        model.identity.model_revision.c_str(), params_.frames.base.c_str(),
        params_.moveit.planning_group.c_str(), params_.frames.tip.c_str(),
        position.x(), position.y(), position.z(), axis.x(), axis.y(), axis.z(),
        attempts, ik_failed, bounds_failed, self_collision, corridor_failed, valid_candidates);
      sortStagingCandidates(candidates);
      return candidates;
    };
  task_config.sleeve_roll_feasible = [this](
    double roll_rad, const Eigen::Isometry3d & entry, double travel_m) {
      double fraction = -1.0;
      const bool ok = probeSleeveInsert(roll_rad, entry, travel_m, &fraction);
      if (!ok) {
        RCLCPP_DEBUG(
          get_logger(),
          "套入直线滚转 %.0f° 未过门 fraction=%.3f",
          static_cast<double>(roll_rad * 180.0 / EIGEN_PI), fraction);
      }
      return ok;
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
  // 批次3：工具 DI（io_states.tool_io_states，pin0=刀闭合）。沿→三轴投影
  // →confirmFeedback（仅 CUT_COMMAND_SENT 态消费）；早已卡高不算新沿。
  io_state_sub_ = create_subscription<aubo_msgs::msg::IOState>(
    "/aubo_io_controller/io_states", 10,
    std::bind(&ManipulationSkillsNode::onIoState, this, std::placeholders::_1));
  tool_state_pub_ = create_publisher<peach_interfaces::msg::ToolState>(
    "/peach_arm/tool_state", latched);
  // move_group 可用性探针（诊断只读；不做规划请求）
  planning_scene_client_ = create_client<moveit_msgs::srv::GetPlanningScene>(
    "/get_planning_scene");
  // 批次5：imu_follow 插入进度（FK 不可用时的回退判据；目标积分）
  imu_progress_sub_ = create_subscription<std_msgs::msg::Float64>(
    "/imu_follow/insert_progress", 10,
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      imu_insert_travel_.store(msg->data);
    });
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

}  // namespace peach_arm
