// 功能：ManipulationSkillsNode 完整类声明。多编译单元共享
// （节点外壳 / 运动 / 动作周期 / 显式阶段执行器）。
#ifndef PEACH_MANIPULATION__MANIPULATION_SKILLS_NODE_HPP_
#define PEACH_MANIPULATION__MANIPULATION_SKILLS_NODE_HPP_

#include <Eigen/Geometry>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <aubo_msgs/msg/joint_status.hpp>
#include <aubo_msgs/msg/robot_status.hpp>
#include <aubo_msgs/srv/set_io.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <nlohmann/json.hpp>
#include <peach_interfaces/msg/bag_fitting_array.hpp>
#include <peach_interfaces/msg/bag_grasp_candidate_array.hpp>
#include <peach_interfaces/msg/enables.hpp>
#include <peach_interfaces/msg/peach_target_observation_array.hpp>
#include <peach_interfaces/action/execute_target.hpp>
#include <peach_interfaces/action/move_to.hpp>
#include <peach_interfaces/action/survey_scene.hpp>
#include <peach_interfaces/srv/check_reachability.hpp>
#include <peach_interfaces/msg/grasp_decision.hpp>
#include <peach_interfaces/msg/grasp_hypothesis.hpp>
#include <peach_interfaces/msg/pregrasp_verification.hpp>
#include <peach_interfaces/msg/reconstruction_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "peach_arm/contact_monitor.hpp"
#include "peach_arm/cycle_context.hpp"
#include "peach_arm/cycle_support.hpp"
#include "peach_arm/frame_timeouts.hpp"
#include "peach_arm/grasp_task.hpp"
#include "peach_arm/motion.hpp"
#include "peach_arm/quality_gate.hpp"
#include "peach_arm/safety_gate.hpp"
#include "peach_arm/target_cache.hpp"
#include "peach_arm/tool_actuator.hpp"
#include "peach_arm/view_planner.hpp"
// 参数声明/兜底默认/范围校验的单一事实源：generate_parameter_library
// （arm_parameters.yaml，清洁重写轮 2c）；部署值事实源为 config/peach_arm.yaml。
#include <peach_arm/arm_parameters.hpp>
#include "peach_arm/plan_contract.hpp"

namespace moveit::planning_interface
{
class MoveGroupInterface;
}  // namespace moveit::planning_interface

namespace peach_arm
{
using Trigger = std_srvs::srv::Trigger;
using SetBool = std_srvs::srv::SetBool;
using CheckReachability = peach_interfaces::srv::CheckReachability;
using json = nlohmann::json;
using ExecuteTarget = peach_interfaces::action::ExecuteTarget;
using SurveyScene = peach_interfaces::action::SurveyScene;
using MoveToAction = peach_interfaces::action::MoveTo;
using RunTargetGoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTarget>;
using SurveyGoalHandle = rclcpp_action::ServerGoalHandle<SurveyScene>;
using MoveToGoalHandle = rclcpp_action::ServerGoalHandle<MoveToAction>;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// 主动视觉靠近与抓取编排节点：只通过 MoveIt 和现有 ROS 接口工作，不直接访问 SDK。
// 生命周期协议（A8 / Robotics_Tutorial 2.16-1）：运动输出权限绑定 Active 态，
// Unconfigured/Inactive/ErrorProcessing 下一切运动类入口（ExecuteTarget action、
// start_cycle、go_to_photo_pose、preview_*、set_execution_armed、工具 IO、MTC
// 执行闸门）一律经 motionOutputAllowed 拒绝并给出明确原因；回调职责见实现文件。
class ManipulationSkillsNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ManipulationSkillsNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ManipulationSkillsNode() override;

  // MoveIt/MTC 专用伴随节点：MoveGroupInterface 与 MTC PipelinePlanner 只接受
  // rclcpp::Node（不支持 LifecycleNode），故组合一个同名普通节点承载全部
  // MoveIt 接口（launch 参数经全局 ros-args 同名匹配自动落到伴随节点；
  // 其参数服务已关闭，避免与本节点 ~/set_parameters 撞名）。executor 需一并
  // add_node（MoveIt 的 CurrentStateMonitor 订阅在默认回调组）。
  rclcpp::Node::SharedPtr moveit_node() const {return moveit_node_;}

  // 生命周期回调（职责表见 manipulation_skills_node.cpp 文件头注释）。
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

private:
  // 运动输出权限单点守卫（A8）：仅 Active 态放行；非 Active 时 why 给出原因。
  bool motionOutputAllowed(std::string & why) const;
  // 取消级联公共段：置取消标志 → 停 MoveIt 当前执行 → 取消 MTC → 唤醒
  // 缓存等待（各取消入口与 closeMotionOutputAndCancel 共用；后者在级联
  // 前先关权限、级联后回收线程，顺序即语义，不得并入本函数）。
  void requestCancelAll();
  // 关闭输出权限并按 CANCEL_NOW 等价路径取消活动周期（撤 arm、置取消标志、
  // stop MoveIt/MTC、唤醒等待、回收 worker/action 线程）。
  void closeMotionOutputAndCancel();
  // 释放 configure 期分配的全部 ROS/MoveIt 资源（cleanup/shutdown/error 共用）。
  void releaseResources();
  void initializeMoveIt();
  double insertionTravel(const CachedRefined & refined) const;
  // 参数声明已下沉到生成的 ParamListener（构造即声明+校验）；loadParameters
  // 写入 params_ 快照并重建可替换实现；Config 从快照直构。onParameters 为
  // on-set 验证钩子（运行中拒改 + execution→grasp→tool 依赖链），无副作用。
  void loadParameters();
  rcl_interfaces::msg::SetParametersResult onParameters(
    const std::vector<rclcpp::Parameter> & parameters);
  void createInterfaces();
  void createSubscriptions();
  void createServices();
  void createActions();

  // 订阅回调（薄壳）：抽消息字段后委托 cache_ 做四源一致性调和。
  // 映射表不写回本文件；热路径周期走 executeCycle(ctx)。
  void onTargets(
    const peach_interfaces::msg::PeachTargetObservationArray::SharedPtr message);
  void onDiagnostics(
    const peach_interfaces::msg::ReconstructionStatus::SharedPtr message);
  void onDecision(const peach_interfaces::msg::GraspDecision::SharedPtr message);
  void onRefinedPose(
    const peach_interfaces::msg::BagGraspCandidateArray::SharedPtr message);
  void onRefinedDiagnostics(
    const peach_interfaces::msg::BagFittingArray::SharedPtr message);
  void onRobotStatus(const aubo_msgs::msg::RobotStatus::SharedPtr message);
  void onJointStatus(const aubo_msgs::msg::JointStatus::SharedPtr message);

  // ExecuteTarget action 服务端与周期控制服务（cycle.cpp）。
  rclcpp_action::GoalResponse onActionGoal(
    const rclcpp_action::GoalUUID &,
    const std::shared_ptr<const ExecuteTarget::Goal> goal);
  rclcpp_action::CancelResponse onActionCancel(
    const std::shared_ptr<RunTargetGoalHandle>);
  void onActionAccepted(const std::shared_ptr<RunTargetGoalHandle> goal_handle);
  void executeAction(const std::shared_ptr<RunTargetGoalHandle> goal_handle);
  rclcpp_action::GoalResponse onSurveyGoal(
    const rclcpp_action::GoalUUID &,
    const std::shared_ptr<const SurveyScene::Goal> goal);
  rclcpp_action::CancelResponse onSurveyCancel(
    const std::shared_ptr<SurveyGoalHandle>);
  void onSurveyAccepted(const std::shared_ptr<SurveyGoalHandle> goal_handle);
  void executeSurvey(const std::shared_ptr<SurveyGoalHandle> goal_handle);
  // MoveTo 动作服务端（清洁重写轮 2b：视点/命名位/位姿移动；move_to.cpp）。
  rclcpp_action::GoalResponse onMoveToGoal(
    const rclcpp_action::GoalUUID &,
    const std::shared_ptr<const MoveToAction::Goal> goal);
  rclcpp_action::CancelResponse onMoveToCancel(
    const std::shared_ptr<MoveToGoalHandle>);
  void onMoveToAccepted(const std::shared_ptr<MoveToGoalHandle> goal_handle);
  void executeMoveTo(const std::shared_ptr<MoveToGoalHandle> goal_handle);
  // TRANSIT 级授权公共段（authorizeStage 的 TRANSIT/PREGRASP 分支与
  // MoveTo/Survey 共用：Active ∧ robotReady ∧ ¬cancel ∧ execution_enabled）。
  bool authorizeTransit(std::string & why);
  // 操作台使能广播订阅（清洁重写轮；无发布者时本地参数保持权威）。
  void onEnables(const peach_interfaces::msg::Enables::SharedPtr message);
  // 使能心跳看门狗（1Hz）：external 源超时未心跳即回落本地参数权威。
  void checkEnablesHeartbeat();
  // 阶段检查点（CK_*）：到达即记，反馈随行下发；0=未到首个检查点。
  void markCheckpoint(uint8_t checkpoint, const char * where);
  void onStart(const Trigger::Response::SharedPtr & response);
  void onCancel(const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response);
  void onAcknowledgeRecovery(
    const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response);
  void onArm(
    const SetBool::Request::SharedPtr request, SetBool::Response::SharedPtr response);

  // 运动相关（motion.cpp）：运动接口装配、工具 IO、接触轨迹预览与
  // go_to_photo_pose 服务回调。规划/执行/TF 查询在 MoveItMotionInterface
  // （阶段执行器经 motion_ 调用）。
  void rebuildMotionInterface();
  void rebuildGraspTask();
  bool commandToolClose();
  void onPreviewApproachInsert(
    const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response);
  void onPreviewFullContact(
    const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response);
  void previewContact(bool include_retreat, Trigger::Response::SharedPtr response);
  void onGoToPhotoPose(
    const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response);
  // 选果级 TCP IK 预检（motion.cpp）：入口→停位几何后 setFromIK，只答能否。
  void onCheckReachability(
    const CheckReachability::Request::SharedPtr request,
    CheckReachability::Response::SharedPtr response);

  // 数据快照与安全门薄壳：锁内组装纯值样本后委托 cache_/safety_gate_ 纯核。
  QualitySnapshot qualitySnapshot();
  std::optional<CachedTarget> targetSnapshot();
  // 周期生效目标快照（阶段 E 残局抬质量能力端）：OBSERVE_ONLY 周期按 goal
  // 钉入 ID 取锁定集锚点缓存，其余周期（FULL/PREVIEW/手动，target_id 空）
  // 恒等于感知 selected 缓存。周期执行体一律经本口取目标快照，不各自判模式。
  std::optional<CachedTarget> cycleTargetSnapshot(const std::string & target_id);
  std::vector<Eigen::Vector3d> observedDirectionsSnapshot();
  std::optional<CachedRefined> refinedSnapshot();
  std::string graspDecisionTargetSnapshot();
  bool cycleTargetReady(const std::string & target_id, std::string & reason);
  bool safetyReady(std::string & reason);
  bool waitForNewView(std::size_t previous_views);
  bool waitForNewStation(std::size_t previous_stations);
  // 等待一条晚于 after_s 的有效目标观测（视点到位后的新鲜帧）。
  // 数据源随周期生效目标走（OBSERVE_ONLY=goal 目标的锁定集条目，其余=
  // selected 缓存），窗口取帧率自适应值。
  bool waitForFreshTarget(const std::string & target_id, double after_s);
  // waitForFreshTarget 的显式窗口版：再确认段用自己的自适应窗口
  // （effectiveReconfirmWaitS，不走 effectiveFrameWaitS），故窗口由调用方给出。
  bool waitForFreshCycleTarget(
    const std::string & target_id, double after_s, double window_s,
    bool live_observation_required = true);
  bool waitForRefined(const std::string & target_id);
  // 帧率自适应超时族（W5-3）：公式内聚于 FrameRateTimeouts 纯核
  // （frame_timeouts.hpp），以下节点方法只做转发（保持既有调用点不变）。
  double effectiveFrameWaitS() const;
  double effectiveTargetMaxAgeS() const;
  // 再确认窗口（2.7-RECONFIRM）：实测帧间隔 EMA 自适应伸缩，未测得时回退
  // params_.grasp.reconfirm_wait_s（配置值同时是自适应上限）。
  double effectiveReconfirmWaitS() const;
  // 精化等待（2.7-FINALIZE 的 T(refined)）：refit 实测耗时本包不可得（不跨包
  // 改接口），按观测帧间隔 EMA 近似（finalize 后约 3 帧内闩锁发布 refined），
  // refined_timeout_s 为回退值与自适应上限。
  double effectiveRefinedWaitS() const;
  // 观测话题到达间隔 EMA 更新（onTargets 每帧调用）。
  void trackFrameInterval();

  // 运动阶段授权（cycle_support.hpp 矩阵的唯一实现，cycle.cpp）：
  // 一切运动执行入口最终收敛到本判定；why 给出拒绝原因（日志带 stage 名）。
  bool authorizeStage(const CycleContext & ctx, MotionStage stage, std::string & why);
  // authorizeStage 的失败包装（stages.cpp）：拒绝时按语义分级——GraspDecision
  // 复检未通过沿用 skipped_quality，其余（权限/安全/取消/使能）落 FAILED——
  // 并经 failStage 终结周期。
  bool requireStageAuthority(
    CycleContext & ctx, MotionStage stage, const std::string & label);

  // 显式模式 switch 执行器（stages.cpp）。阶段调用序列与原 behavior_tree.xml
  // 主树遍历严格同构：Prepare →（plan-only 预览 | 观察 → 精化验证 →
  // （observe_only 短路 | grasp 未使能短路 | 再确认 → 预抓取 → 验证 →
  // （PREGRASP_ONLY 停驻 | 套入 → 剪切 → 原路撤退 → 回 stow → 终验）→ Complete）。
  // 各阶段返回 false 即失败（failStage 已置 ctx.failure_reason/状态投影）。
  void executeCycle(CycleContext & ctx);
  // 周期失败单点：记 failure_reason 并投影 FAILED，返回 false 供阶段串联。
  bool failStage(CycleContext & ctx, const std::string & reason);
  // 阶段失败三分行合并写法：置终态 outcome + 失败码后走 failStage。
  bool failStage(
    CycleContext & ctx, uint8_t outcome, uint32_t failure_code,
    const std::string & reason);
  // 入口工具位姿（三处共用）：平移=精化入口；姿态优先沿当前工具姿态对轴
  // （alignFrameZ），TF 不可用退到 ViewPlanner::toolOrientation（preferred_x
  // 通常取目标 initial_pose 的 X 轴）；tip 位姿由调用方乘 (tip←tool)^-1。
  Eigen::Isometry3d entryToolPose(
    const Eigen::Vector3d & entry, const Eigen::Vector3d & axis,
    const Eigen::Vector3d & preferred_x);
  // 接触入口三元组（preview 与 VerifyPregrasp 修正两处同构收敛）：
  // (entry_tip_pose, travel_m)。tip←tool 变换缺失时 false 并置 error
  // （统一文案「无法取得 tip 到 tool 的变换」，调用方按各自路径分级）。
  bool contactEntryGeometry(
    const CachedRefined & refined, const Eigen::Isometry3d & initial_pose,
    Eigen::Isometry3d & entry_tip_pose, double & travel_m,
    std::string & error);
  // 果实胶囊（①层审查输入）：感知直径+膨胀；直径无效回退保守半径并 WARN。
  FruitCapsule fruitCapsuleFor(const CachedRefined & refined) const;
  // ④层接触止损：guarded 段（接近执行/套入）包裹调用；enabled=false 时
  // no-op。触发疑似硬接触→requestCancelAll + 旗标，阶段函数事后判旗标
  // 失败并给文案（CONTACT_ABORTED 枚举留待真机标定轮进 IDL）。
  void startContactGuard();
  void stopContactGuard();
  bool contactAbortSuspected() const;
  bool stagePrepareCycle(CycleContext & ctx);
  bool stagePlanPreview(CycleContext & ctx);
  bool stageAcquireViews(CycleContext & ctx);
  bool stageFinalizeAndValidate(CycleContext & ctx);
  // 抓取前再确认（2.7-RECONFIRM，阶段 E1）：FinalizeAndValidate 之后、接触段之前，
  // 等新鲜观测复核身份/锚点漂移/摆动平息；语义与判定委托 ReconfirmPolicy 纯核。
  bool stageReconfirmTarget(CycleContext & ctx);
  bool stageReportReady(CycleContext & ctx);
  bool stageReportObserveOnly(CycleContext & ctx);
  bool stageMovePregrasp(CycleContext & ctx);
  bool stageVerifyPregrasp(CycleContext & ctx);
  bool stageHoldPregrasp(CycleContext & ctx);
  bool stagePlanSleeveAndReverseRetreat(CycleContext & ctx);
  bool stageSleeveLinear(CycleContext & ctx);
  bool stageVerifyCutHold(CycleContext & ctx);
  bool stageActuateCutter(CycleContext & ctx);
  bool stageVerifyCut(CycleContext & ctx);
  bool stageExecuteReservedReverseRetreat(CycleContext & ctx);
  bool stageReturnHarvestStow(CycleContext & ctx);
  bool stageVerifyHarvestOutcome(CycleContext & ctx);
  bool stageCompleteTarget(CycleContext & ctx);

  void publishViewMarkers(
    const Eigen::Vector3d & target, const std::vector<ViewCandidate> & candidates);
  // 周期身份（target_id）经参数传入：预览/非周期调用不读周期上下文。
  void setState(
    CycleState state, const std::string & message,
    const std::string & target_id = std::string());
  void publishState();
  // 周期阶段耗时埋点（重构阶段 C）：startCycleTiming 在周期真正启动点
  // （onStart/previewContact 放行后）开始计时；setState 在 cycle_state_ 变更点
  // 喂入投影；fillStageDurations 终局收口并填充 Result 的并行数组。
  void startCycleTiming();
  void fillStageDurations(
    const std::shared_ptr<ExecuteTarget::Result> & result);
  // 终局字段填充：周期路径传 ctx；预览中断路径（无周期上下文）传 nullptr，
  // 按空上下文默认值填充（completion=0、无剪切/撤退确认）。
  void fillExecuteResults(
    const std::shared_ptr<ExecuteTarget::Result> & result,
    const CycleContext * ctx);

  // 运行配置不再镜像为扁平成员（W5-1）：值一律读 params_ 快照（GPL 单源，
  // 见 params_bridge.hpp 的 Config 单点转换）；此处只留运行期状态与纯核组件。
  /// 本目标内移动+等帧成本 [s]，≤0=未测得（观察段日志对照）。
  double scan_move_cost_ema_s_{0.0};
  std::atomic_bool execution_enabled_{false};  ///< 自由空间运动使能（默认关）。
  std::atomic_bool grasp_enabled_{false};      ///< 套入使能（默认关）。
  // 环境几何保护区（阶段 F1，scan.protected_zones 解析结果）：base 系轴对齐
  // 盒列表。视点剔除由 view_planner_ 持有的配置副本执行；GraspTask 把同一
  // 列表写入 planning scene 参与碰撞检查。与 view_planner_ 同一重载纪律：
  // 仅空闲时 loadParameters 重写（运行中改参已被 onParameters 前置拒绝），
  // 周期工作线程读取，无需额外锁。
  std::vector<ProtectedZone> protected_zones_;
  std::atomic_bool tool_enabled_{false};  ///< 刀具使能（默认关）。
  // 帧率自适应超时族（W5-3）：观测帧间隔 EMA 状态与六个 effective* 公式
  // 内聚于纯核；loadParameters 空闲期重写配置，节点方法只做转发。
  FrameRateTimeouts frame_timeouts_;
  // staging IK 自碰环境池（W5-2）：每 roll 任务一个 CollisionEnvFCL（SRDF
  // ACM 构造、无跨调用状态），首次调用构造、跨调用复用；定义在
  // manipulation_skills_node.cpp（避免节点头引入 MoveIt 碰撞检测头）。
  struct StagingIkEnvironment;
  std::unique_ptr<StagingIkEnvironment> staging_ik_env_;
  // 刀具 GPIO 状态机（SetIO ACK ≠ 切断确认；confirmFeedback 预留）。
  ToolActuator tool_actuator_{};

  // MoveIt/MTC 伴随节点（声明在所有 MoveIt 资源之前，保证析构时最后释放）。
  rclcpp::Node::SharedPtr moveit_node_;
  // 运动输出权限（A8）：on_activate 开放、on_deactivate/on_cleanup/on_shutdown/
  // on_error 关闭；一切运动类入口经 motionOutputAllowed 单点检查。
  std::atomic_bool motion_output_permitted_{false};

  // 职责实现直接持有具体类（原工厂缝位已删除）：参数重载时 loadParameters
  // 原地重建；motion_/grasp_task_ 需 MoveIt 初始化后由 rebuild* 装配
  // （MoveIt 未初始化前为空，调用端判空等价于原 !move_group_）。
  std::unique_ptr<ViewPlanner> view_planner_;
  std::unique_ptr<QualityGate> quality_gate_;
  std::unique_ptr<SafetyGate> safety_gate_;
  std::unique_ptr<MoveItMotionInterface> motion_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::unique_ptr<GraspTask> grasp_task_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // 目标/精化/质量/抓取决策四源缓存（纯核，注入时钟）；线程安全自给。
  TargetCache cache_;
  // robot_status 独立小锁：与目标缓存解耦，安全门样本在锁内组装。
  std::mutex robot_mutex_;
  aubo_msgs::msg::RobotStatus robot_status_;
  rclcpp::Time robot_status_received_{0, 0, RCL_ROS_TIME};
  double robot_status_mono_s_{0.0};
  bool robot_status_valid_{false};

  std::mutex state_mutex_;
  json state_json_;
  // 唯一权威周期状态（枚举）；state_json_["state"] 只是它的发布层投影。
  CycleState current_state_{CycleState::IDLE};
  // 周期阶段耗时计时器（重构阶段 C）：仅在本互斥锁内访问（setState 喂入、
  // startCycleTiming/fillStageDurations 起止与读取）。
  StageTimer stage_timer_;
  // 当前周期上下文：action 受理（executeAction）创建，
  // worker 线程启动时按 shared_ptr 持有；一切周期可变状态在 ctx 内
  // （见 cycle_context.hpp 线程规则）。下一周期创建即整体丢弃上一份。
  std::shared_ptr<CycleContext> cycle_;
  std::atomic_bool running_{false};
  std::atomic_bool cancel_requested_{false};
  std::atomic_bool execution_armed_{false};
  std::atomic_bool contact_recovery_required_{false};
  // abort 路径的终局分级（ExecuteTarget::Result 常量），由阶段失败点按需覆盖。
  std::atomic<uint8_t> pending_outcome_{ExecuteTarget::Result::FAILED};
  std::thread worker_;
  // action 执行线程保持可 join，析构时先取消再回收，避免 shutdown 后访问悬空 this。
  std::thread action_thread_;

  rclcpp::Subscription<peach_interfaces::msg::PeachTargetObservationArray>::SharedPtr target_sub_;
  rclcpp::Subscription<peach_interfaces::msg::ReconstructionStatus>::SharedPtr
    diagnostics_sub_;
  rclcpp::Subscription<peach_interfaces::msg::GraspDecision>::SharedPtr decision_sub_;
  rclcpp::Subscription<peach_interfaces::msg::BagGraspCandidateArray>::SharedPtr
    refined_pose_sub_;
  rclcpp::Subscription<peach_interfaces::msg::BagFittingArray>::SharedPtr refined_diag_sub_;
  rclcpp::Subscription<aubo_msgs::msg::RobotStatus>::SharedPtr robot_status_sub_;
  // ④层接触止损接线：joint_status 电流缓存（环形 64 样本，互斥保护），
  // guarded 段 timer 评估；默认 enabled=false 只缓存不判定。
  rclcpp::Subscription<aubo_msgs::msg::JointStatus>::SharedPtr joint_status_sub_;
  rclcpp::TimerBase::SharedPtr contact_guard_timer_;
  std::mutex joint_current_mutex_;
  std::vector<CurrentSample> joint_current_samples_;
  ContactDetectConfig contact_detect_config_;
  std::unique_ptr<ContactMonitor> contact_monitor_;
  std::atomic<bool> contact_abort_suspected_{false};
  // 生命周期发布者：on_activate/on_deactivate 切换激活态；publishState 在
  // 未激活/已清理时只更新内存投影不发布（~/status 仍随激活发布）。
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr status_pub_;
  // 回调耗时累计注册表（2.16-5）：关键回调入口的 ScopedTimer 析构时写入，
  // publishState 将其 JSON 投影随 ~/status 一起发布（不新增话题）。
  CallbackTimingRegistry callback_timing_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    marker_pub_;
  // 与 status/markers 一样走 LifecyclePublisher：Active 才发，Inactive 空操作。
  rclcpp_lifecycle::LifecyclePublisher<peach_interfaces::msg::GraspHypothesis>::SharedPtr
    grasp_hyp_pub_;
  rclcpp::Service<Trigger>::SharedPtr preview_approach_service_;
  rclcpp::Service<Trigger>::SharedPtr preview_full_contact_service_;
  rclcpp::Service<Trigger>::SharedPtr cancel_service_;
  rclcpp::Service<Trigger>::SharedPtr recovery_service_;
  rclcpp::Service<Trigger>::SharedPtr photo_pose_service_;
  rclcpp::Service<CheckReachability>::SharedPtr reachability_service_;
  rclcpp::Service<SetBool>::SharedPtr arm_service_;
  // 长规划类服务独立互斥回调组（preview_approach_insert / preview_full_contact /
  // go_to_photo_pose）：组内串行，数秒级规划不挡默认组的订阅与快捷服务。
  rclcpp::CallbackGroup::SharedPtr planning_callback_group_;
  rclcpp_action::Server<ExecuteTarget>::SharedPtr cycle_action_server_;
  rclcpp_action::Server<SurveyScene>::SharedPtr survey_action_server_;
  std::thread survey_thread_;
  // MoveTo 动作服务端与执行线程（与 survey 同纪律：可 join、互斥占用）。
  rclcpp_action::Server<MoveToAction>::SharedPtr move_to_action_server_;
  std::thread move_to_thread_;
  // 操作台使能广播（清洁重写轮）：收到过即 external 生效并覆盖本地参数；
  // 未收到过（旧栈/无大脑）本地参数保持唯一权威——行为零变化。
  rclcpp::Subscription<peach_interfaces::msg::Enables>::SharedPtr enables_sub_;
  bool enables_external_{false};
  // 使能心跳看门狗（缺心跳=故障）：steady 时钟记录最近一拍，超时回落
  // 本地参数权威。timeout<=0 时禁用（锁存兼容档）。
  rclcpp::TimerBase::SharedPtr enables_watchdog_timer_;
  std::chrono::steady_clock::time_point enables_last_beat_{};
  double enables_heartbeat_timeout_s_{5.0};
  // 当前周期最新检查点（ExecuteTarget::Goal::CK_*；0=未到）。
  std::atomic<uint8_t> last_checkpoint_{0};
  // 最近观测快照三元组（SurveyScene result 数据源）：onTargets（订阅线程）写、
  // executeSurvey（survey 线程）读，经 snapshot_mutex_ 互斥。
  std::mutex snapshot_mutex_;
  std::string last_snapshot_id_;
  uint32_t last_observation_count_{0};
  bool last_target_set_locked_{false};
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  PostSetParametersCallbackHandle::SharedPtr post_parameter_callback_handle_;
  // 参数监听器（构造即声明全部参数并做启动校验；运行期 set 经其内置范围
  // 校验 + onParameters 钩子，post-set 后 loadParameters 重载快照）。
  std::shared_ptr<peach_arm::ParamListener> param_listener_;
  peach_arm::Params params_;
  double robot_status_contract_timeout_s_{0.5};
  ContactPlan last_preview_plan_{};
  bool last_preview_valid_{false};
  rclcpp::Client<aubo_msgs::srv::SetIO>::SharedPtr tool_io_client_;
};

}  // namespace peach_arm

#endif  // PEACH_MANIPULATION__MANIPULATION_SKILLS_NODE_HPP_
