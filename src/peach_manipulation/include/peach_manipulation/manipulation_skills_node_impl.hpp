// 功能：ManipulationSkillsNode 完整类声明。多编译单元共享
// （节点外壳 / 运动 / 动作周期 / 显式阶段执行器）。
#ifndef PEACH_MANIPULATION__MANIPULATION_SKILLS_NODE_IMPL_HPP_
#define PEACH_MANIPULATION__MANIPULATION_SKILLS_NODE_IMPL_HPP_

#include <Eigen/Geometry>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <aubo_msgs/msg/robot_status.hpp>
#include <aubo_msgs/srv/set_io.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <nlohmann/json.hpp>
#include <peach_interfaces/msg/bag_fitting_array.hpp>
#include <peach_interfaces/msg/bag_grasp_candidate_array.hpp>
#include <peach_interfaces/msg/peach_target_observation_array.hpp>
#include <peach_interfaces/action/execute_target.hpp>
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

#include "peach_manipulation/cycle_state.hpp"
#include "peach_manipulation/cycle_context.hpp"
#include "peach_manipulation/execution_authority.hpp"
#include "peach_manipulation/grasp_task.hpp"
#include "peach_manipulation/motion.hpp"
#include "peach_manipulation/quality_gate.hpp"
#include "peach_manipulation/safety_gate.hpp"
#include "peach_manipulation/scan_budget.hpp"
#include "peach_manipulation/stage_timing.hpp"
#include "peach_manipulation/target_cache.hpp"
#include "peach_manipulation/tool_actuator.hpp"
#include "peach_manipulation/view_planner.hpp"
#include "peach_manipulation/scoped_timer.hpp"
// 参数声明/默认值/校验的单一事实源（generate_parameter_library 生成，
// 定义见 config/manipulation_parameters.yaml）。
#include "peach_manipulation/manipulation_parameters.hpp"

namespace moveit::planning_interface
{
class MoveGroupInterface;
}  // namespace moveit::planning_interface

namespace peach_manipulation
{
using Trigger = std_srvs::srv::Trigger;
using SetBool = std_srvs::srv::SetBool;
using CheckReachability = peach_interfaces::srv::CheckReachability;
using json = nlohmann::json;
using ExecuteTarget = peach_interfaces::action::ExecuteTarget;
using SurveyScene = peach_interfaces::action::SurveyScene;
using RunTargetGoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTarget>;
using SurveyGoalHandle = rclcpp_action::ServerGoalHandle<SurveyScene>;
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
  // 从监听器快照装载全部成员并重建可替换实现；onParameters 为 on-set 验证
  // 钩子（运行中拒改 + execution→grasp→tool 依赖链），无副作用。
  void loadParameters();
  rcl_interfaces::msg::SetParametersResult onParameters(
    const std::vector<rclcpp::Parameter> & parameters);
  void createInterfaces();
  void createSubscriptions();
  void createServices();
  void createActions();

  // 订阅回调（薄壳）：消息字段提取后委托 cache_ 做四源一致性调和。
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
  void onStart(
    const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response,
    bool action_driven);
  void onCancel(const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response);
  void onAcknowledgeRecovery(
    const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response);
  void onQuery(const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response);
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
  // 帧率自适应取值：等帧窗口在 EMA 未测得时可用 assumed_frame_interval_s
  // 按 2.5 FPS 估超时；新鲜度门在未测得前保持 yaml 回退，且不得收得比回退更紧。
  double waitIntervalS() const;
  double effectiveFrameWaitS() const;
  double effectiveTargetMaxAgeS() const;
  // 再确认窗口（2.7-RECONFIRM）：实测帧间隔 EMA 自适应伸缩，未测得时回退
  // reconfirm_wait_s_（配置值同时是自适应上限）。
  double effectiveReconfirmWaitS() const;
  // 精化等待（2.7-FINALIZE 的 T(refined)）：refit 实测耗时本包不可得（不跨包
  // 改接口），按观测帧间隔 EMA 近似（finalize 后约 3 帧内闩锁发布 refined），
  // refined_timeout_s_ 为回退值与自适应上限。
  double effectiveRefinedWaitS() const;
  // 观测话题到达间隔 EMA 更新（onTargets 每帧调用）。
  void trackFrameInterval();

  // 运动阶段授权（execution_authority.hpp 矩阵的唯一实现，cycle.cpp）：
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

  std::string base_frame_;
  std::string tip_frame_;  // 规划/IK 末端连杆（MoveIt 组 tip_link，当前为 tcp）
  std::string camera_frame_;
  std::string tool_frame_;
  std::string planning_group_;
  std::string pilz_pipeline_;
  std::string fallback_pipeline_;
  std::string mtc_free_space_pipeline_;
  std::string mtc_free_space_planner_;
  std::string photo_pose_named_target_;
  std::string harvest_stow_named_target_{"harvest_stow"};
  double planning_time_s_{1.5};
  int planning_attempts_{1};
  double velocity_scaling_{0.10};
  double acceleration_scaling_{0.10};
  double transit_velocity_scaling_{0.10};
  double transit_acceleration_scaling_{0.10};
  double mtc_cartesian_step_m_{0.005};
  double mtc_cartesian_min_fraction_{0.95};
  double mtc_cartesian_precision_m_{0.001};
  int mtc_max_solutions_{5};
  double mtc_approach_max_duration_s_{0.0};
  double mtc_approach_max_total_joint_travel_rad_{12.0};
  double mtc_approach_max_single_joint_travel_rad_{6.1};
  double mtc_approach_via_max_spacing_m_{0.08};
  double mtc_approach_via_min_spacing_m_{0.03};
  int mtc_approach_via_max_points_{1};
  double mtc_approach_max_detour_ratio_{2.2};
  double mtc_approach_max_chord_deviation_m_{0.25};
  double mtc_approach_max_recede_m_{0.08};
  double mtc_approach_cartesian_max_distance_m_{0.80};
  double mtc_approach_along_axis_m_{0.0};
  double mtc_approach_max_lateral_m_{0.05};
  double mtc_approach_max_align_deg_{20.0};
  double transit_max_duration_s_{0.0};
  double transit_max_total_joint_travel_rad_{6.0};
  double transit_max_single_joint_travel_rad_{2.5};
  int maximum_scan_moves_{5};
  // 观察段有效视点观测下限（2.13-E2）：未达此前不得收口（移动到位且收到
  // 新鲜目标观测计一次有效视点）。
  int min_effective_views_{1};
  // 观察段墙钟对照（秒）。停准则不按本值或 EMA 预测收口；只进日志。
  double scan_time_budget_s_{15.0};
  double scan_budget_cost_margin_{1.5};
  // 未测得观测间隔 EMA 时的回退帧间隔（秒）。0=不预填。
  double assumed_frame_interval_s_{0.4};
  // 本目标内移动+等帧成本（秒，≤0=未测得）：stageAcquireViews 开头清零，
  // 成功一次有效视点后 0.7/0.3 刷新，只进日志。
  double scan_move_cost_ema_s_{0.0};
  double frame_wait_s_{6.0};
  // 帧率自适应：观测话题到达间隔 EMA（≤0=未测得）与最近到达时刻。
  // onTargets（订阅线程）写、等帧取值（周期线程）读：relaxed 原子即可
  // （EMA 只作超时估计，读到偶发旧值无害）。
  std::atomic<double> frame_interval_ema_s_{0.0};
  std::atomic<double> last_targets_arrival_s_{0.0};
  double target_observation_max_age_config_s_{3.0};
  std::atomic_bool execution_enabled_{false};
  std::atomic_bool grasp_enabled_{false};
  double neck_margin_m_{0.015};
  double minimum_travel_m_{0.02};
  double maximum_travel_m_{0.20};
  // 环境几何保护区（阶段 F1，scan.protected_zones 解析结果）：base 系轴对齐
  // 盒列表。视点剔除由 view_planner_ 持有的配置副本执行；GraspTask 把同一
  // 列表写入 planning scene 参与碰撞检查。与 view_planner_ 同一重载纪律：
  // 仅空闲时 loadParameters 重写（运行中改参已被 onParameters 前置拒绝），
  // 周期工作线程读取，无需额外锁。
  std::vector<ProtectedZone> protected_zones_;
  // 抓取前再确认（2.7-RECONFIRM）三参数与回退开关，语义见
  // config/peach_manipulation.yaml grasp.* 注释（权威源）。
  double reconfirm_wait_s_{6.0};
  double reconfirm_tolerance_m_{0.03};
  int reconfirm_max_attempts_{3};
  bool allow_stale_anchor_{false};
  std::atomic_bool tool_enabled_{false};
  int tool_io_fun_{3};
  int tool_io_pin_{0};
  double tool_close_state_{1.0};
  double service_timeout_s_{3.0};
  double refined_timeout_s_{30.0};
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
  bool robot_status_valid_{false};

  std::mutex state_mutex_;
  json state_json_;
  // 唯一权威周期状态（枚举）；state_json_["state"] 只是它的发布层投影。
  CycleState current_state_{CycleState::IDLE};
  // 周期阶段耗时计时器（重构阶段 C）：仅在本互斥锁内访问（setState 喂入、
  // startCycleTiming/fillStageDurations 起止与读取）。
  StageTimer stage_timer_;
  // 当前周期上下文：action 受理（executeAction）或手动周期（onStart）创建，
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
  // 生命周期发布者：on_activate/on_deactivate 切换激活态；publishState 在
  // 未激活/已清理时只更新内存投影不发布（query_state 仍可读）。
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr status_pub_;
  // 回调耗时累计注册表（2.16-5）：关键回调入口的 ScopedTimer 析构时写入，
  // publishState 将其 JSON 投影随 ~/status 一起发布（不新增话题）。
  CallbackTimingRegistry callback_timing_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    marker_pub_;
  rclcpp::Publisher<peach_interfaces::msg::GraspHypothesis>::SharedPtr
    grasp_hyp_pub_;
  rclcpp::Service<Trigger>::SharedPtr start_service_;
  rclcpp::Service<Trigger>::SharedPtr preview_approach_service_;
  rclcpp::Service<Trigger>::SharedPtr preview_full_contact_service_;
  rclcpp::Service<Trigger>::SharedPtr cancel_service_;
  rclcpp::Service<Trigger>::SharedPtr recovery_service_;
  rclcpp::Service<Trigger>::SharedPtr query_service_;
  rclcpp::Service<Trigger>::SharedPtr photo_pose_service_;
  rclcpp::Service<CheckReachability>::SharedPtr reachability_service_;
  rclcpp::Service<SetBool>::SharedPtr arm_service_;
  // 长规划类服务独立互斥回调组（preview_approach_insert / preview_full_contact /
  // go_to_photo_pose）：组内串行，数秒级规划不挡默认组的订阅与快捷服务。
  rclcpp::CallbackGroup::SharedPtr planning_callback_group_;
  rclcpp_action::Server<ExecuteTarget>::SharedPtr cycle_action_server_;
  rclcpp_action::Server<SurveyScene>::SharedPtr survey_action_server_;
  std::thread survey_thread_;
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
  std::shared_ptr<peach_manipulation_node::ParamListener> param_listener_;
  rclcpp::Client<aubo_msgs::srv::SetIO>::SharedPtr tool_io_client_;
};

}  // namespace peach_manipulation

#endif  // PEACH_MANIPULATION__MANIPULATION_SKILLS_NODE_IMPL_HPP_
