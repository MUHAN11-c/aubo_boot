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
// 节点类声明（包内私有头，不安装）：approach_grasp_node 按职责拆为多个编译单元
// （节点本体 / motion_interface / bt_nodes / cycle_action），共享本声明。
#ifndef APPROACH_GRASP_NODE_IMPL_HPP_
#define APPROACH_GRASP_NODE_IMPL_HPP_

#include <Eigen/Geometry>
#include <behaviortree_cpp/bt_factory.h>
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
#include <peach_interfaces/msg/grasp_decision.hpp>
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

#include "peach_manipulation_skills/cycle_state.hpp"
#include "peach_manipulation_skills/grasp_task.hpp"
#include "peach_manipulation_skills/impl_factory.hpp"
#include "peach_manipulation_skills/motion_interface_base.hpp"
#include "peach_manipulation_skills/preplan_slot.hpp"
#include "peach_manipulation_skills/quality_gate.hpp"
#include "peach_manipulation_skills/safety_gate.hpp"
#include "peach_manipulation_skills/scan_budget.hpp"
#include "peach_manipulation_skills/stage_timing.hpp"
#include "peach_manipulation_skills/target_cache.hpp"
#include "peach_manipulation_skills/view_planner.hpp"
#include "scoped_timer.hpp"
// 参数声明/默认值/校验的单一事实源（generate_parameter_library 生成，
// 定义见 src/approach_grasp_node_parameters.yaml）。
#include "peach_manipulation_skills/approach_grasp_node_parameters.hpp"

namespace moveit::planning_interface
{
class MoveGroupInterface;
}  // namespace moveit::planning_interface

namespace peach_manipulation_skills
{
using Trigger = std_srvs::srv::Trigger;
using SetBool = std_srvs::srv::SetBool;
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
class ApproachGraspNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ApproachGraspNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ApproachGraspNode() override;

  // MoveIt/MTC 专用伴随节点：MoveGroupInterface 与 MTC PipelinePlanner 只接受
  // rclcpp::Node（不支持 LifecycleNode），故组合一个同名普通节点承载全部
  // MoveIt 接口（launch 参数经全局 ros-args 同名匹配自动落到伴随节点；
  // 其参数服务已关闭，避免与本节点 ~/set_parameters 撞名）。executor 需一并
  // add_node（MoveIt 的 CurrentStateMonitor 订阅在默认回调组）。
  rclcpp::Node::SharedPtr moveit_node() const {return moveit_node_;}

  // 生命周期回调（职责表见 approach_grasp_node.cpp 文件头注释）。
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

private:
  // 运动输出权限单点守卫（A8）：仅 Active 态放行；非 Active 时 why 给出原因。
  bool motionOutputAllowed(std::string & why) const;
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

  // ExecuteTarget action 服务端与周期控制服务（cycle_action.cpp）。
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

  // 运动相关（motion_interface.cpp）：可替换运动接口的装配、Trigger 调用、
  // 工具 IO、接触轨迹预览与 go_to_photo_pose 服务回调。规划/执行/TF 查询
  // 已下沉到 MotionInterfaceBase 实现（bt_nodes 经 motion_ 基类指针调用）。
  void rebuildMotionInterface();
  bool callTrigger(
    const rclcpp::Client<Trigger>::SharedPtr & client, const std::string & label,
    bool required = true);
  bool commandToolClose();
  void onPreviewApproachInsert(
    const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response);
  void onPreviewFullContact(
    const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response);
  void previewContact(bool include_retreat, Trigger::Response::SharedPtr response);
  void onGoToPhotoPose(
    const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response);

  // 数据快照与安全门薄壳：锁内组装纯值样本后委托 cache_/safety_gate_ 纯核。
  QualitySnapshot qualitySnapshot();
  std::optional<CachedTarget> targetSnapshot();
  // 周期生效目标快照（阶段 E 残局抬质量能力端）：OBSERVE_ONLY 周期按 goal
  // 钉入 ID 取锁定集锚点缓存，其余周期（FULL/PREVIEW/手动）恒等于感知
  // selected 缓存。周期执行体一律经本口取目标快照，不各自判模式。
  std::optional<CachedTarget> cycleTargetSnapshot();
  std::vector<Eigen::Vector3d> observedDirectionsSnapshot();
  std::optional<CachedRefined> refinedSnapshot();
  std::string graspDecisionTargetSnapshot();
  bool cycleTargetReady(const std::string & target_id, std::string & reason);
  bool safetyReady(std::string & reason);
  bool waitForNewView(std::size_t previous_views);
  // 等待一条晚于 after_s 的有效目标观测（视点到位后的新鲜帧）。
  // 数据源随周期生效目标走（OBSERVE_ONLY=goal 目标的锁定集条目，其余=
  // selected 缓存），窗口取帧率自适应值。
  bool waitForFreshTarget(double after_s);
  // waitForFreshTarget 的显式窗口版：再确认段用自己的自适应窗口
  // （effectiveReconfirmWaitS，不走 effectiveFrameWaitS），故窗口由调用方给出。
  bool waitForFreshCycleTarget(double after_s, double window_s);
  bool waitForRefined(const std::string & target_id);
  // 帧率自适应取值：ema 未测得时回退配置值；视点等待以配置为上限，
  // 新鲜度门以实测帧间隔放大（低帧率放宽防 stale 误判，高帧率收紧提速）。
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

  // MTC 预规划（2.13-E3 plan-while-waiting）三件套（bt_nodes.cpp）：
  //   launchPreplan 在再确认入口按当前几何后台预规划接近/插入任务；
  //   settlePreplanBeforeReplan 在内联重规划前清场（preempt 未落定规划并回收
  //   线程，杜绝 GraspTask active_task_ 并发）；cleanupPreplan 在周期终局与
  //   deactivate/shutdown 路径统一丢弃（槽+任务+线程回收，幂等）。
  void launchPreplan();
  void settlePreplanBeforeReplan();
  void cleanupPreplan();

  // 行为树节点注册与节点体（bt_nodes.cpp）。
  void registerBehaviorTreeNodes();
  BT::NodeStatus btFailure(const std::string & reason);
  BT::NodeStatus btPrepareCycle();
  BT::NodeStatus btPlanPreview();
  BT::NodeStatus btAcquireViews();
  BT::NodeStatus btFinalizeAndValidate();
  // 抓取前再确认（2.7-RECONFIRM，阶段 E1）：FinalizeAndValidate 之后、MTC 之前，
  // 等新鲜观测复核身份/锚点漂移/摆动平息；语义与判定委托 ReconfirmPolicy 纯核。
  BT::NodeStatus btReconfirmTarget();
  BT::NodeStatus btReportReady();
  BT::NodeStatus btReportObserveOnly();
  BT::NodeStatus btMtcApproachAndInsert();
  BT::NodeStatus btActuateTool();
  BT::NodeStatus btMtcRetreat();
  BT::NodeStatus btCompleteTarget();

  void runCycle();
  void publishViewMarkers(
    const Eigen::Vector3d & target, const std::vector<ViewCandidate> & candidates);
  void setState(CycleState state, const std::string & message);
  void publishState();
  // 周期阶段耗时埋点（重构阶段 C）：startCycleTiming 在周期真正启动点
  // （onStart/previewContact 放行后）开始计时；setState 在 cycle_state_ 变更点
  // 喂入投影；fillStageDurations 终局收口并填充 Result 的并行数组。
  void startCycleTiming();
  void fillStageDurations(
    const std::shared_ptr<ExecuteTarget::Result> & result);

  std::string base_frame_;
  std::string tip_frame_;  // 规划/IK 末端连杆（MoveIt 组 tip_link，当前为 tcp）
  std::string camera_frame_;
  std::string tool_frame_;
  std::string planning_group_;
  std::string pilz_pipeline_;
  std::string fallback_pipeline_;
  std::string mtc_free_space_planner_;
  std::string photo_pose_named_target_;
  std::string behavior_tree_xml_;
  double planning_time_s_{5.0};
  int planning_attempts_{5};
  // 速度双档：velocity_scaling_ 用于接触段（MTC 靠近/插入/撤离）；
  // transit_* 用于自由空间转移（环绕视点、拍照位姿往返）。2026-08-13 真机
  // 0.01 长转移蠕行 90s 肩部过流：低速档不得压到让高重力矩姿态持续过久。
  double velocity_scaling_{0.05};
  double acceleration_scaling_{0.05};
  double transit_velocity_scaling_{0.10};
  double transit_acceleration_scaling_{0.10};
  double mtc_cartesian_step_m_{0.005};
  double mtc_cartesian_precision_m_{0.001};
  int mtc_max_solutions_{5};
  int maximum_scan_moves_{5};
  // 观察段有效视点观测下限（2.13-E2）：未达此前不得收口（移动到位且收到
  // 新鲜目标观测计一次有效视点）。
  int min_effective_views_{2};
  // 观察段时间预算下限（秒，2.13-E2）：运行期预算 = max(本值,
  // 2.5×scan_move_cost_ema_s_)（协议 2.7-OBSERVE 的 T(scan_budget)）。
  double scan_time_budget_s_{5.0};
  // 单视点移动+等帧成本 EMA（秒，≤0=未测得）：btAcquireViews 每成功一次
  // 有效视点移动后按 0.7/0.3 刷新（与 trackFrameInterval 同形状）；仅 BT
  // 工作线程读写，跨周期保留（成本是机型/速度档属性，不随目标变）。
  double scan_move_cost_ema_s_{0.0};
  double frame_wait_s_{6.0};
  // 帧率自适应：观测话题到达间隔 EMA（≤0=未测得）与配置回退值。
  double frame_interval_ema_s_{0.0};
  double last_targets_arrival_s_{0.0};
  double target_observation_max_age_config_s_{3.0};
  std::atomic_bool execution_enabled_{false};
  bool reset_reconstruction_on_start_{false};
  std::atomic_bool grasp_enabled_{false};
  double neck_margin_m_{0.015};
  double minimum_travel_m_{0.02};
  double maximum_travel_m_{0.20};
  bool complete_target_after_retreat_{true};
  // 降级抓取入口的袋外预入口余量（米）：入口点=锚点−轴·(行程+本值)。
  double fallback_standoff_m_{0.05};
  // 环境几何保护区（阶段 F1，scan.protected_zones 解析结果）：base 系轴对齐
  // 盒列表。视点剔除由 view_planner_ 持有的配置副本执行；本成员供
  // btMtcApproachAndInsert 的接触段入口剔除使用。与 view_planner_ 同一重载
  // 纪律：仅空闲时 loadParameters 重写（运行中改参已被 onParameters 前置
  // 拒绝），BT 工作线程读取，无需额外锁。
  std::vector<ProtectedZone> protected_zones_;
  // 抓取前再确认（2.7-RECONFIRM）三参数与回退开关，语义见
  // config/approach_grasp.yaml grasp.* 注释（权威源）。
  double reconfirm_wait_s_{6.0};
  double reconfirm_tolerance_m_{0.03};
  int reconfirm_max_attempts_{3};
  bool allow_stale_anchor_{false};
  // MTC 预规划（2.13-E3）：再确认等待窗口期后台预规划接近/插入任务；
  // 复用阈值与开关语义见 config/approach_grasp.yaml grasp.* 注释。
  double replan_threshold_m_{0.02};
  bool mtc_preplan_enabled_{true};
  // 预规划状态槽（纯核，线程安全自给）与预规划线程；preplan_reuse_ 仅 BT
  // 工作线程读写（再确认 PASS 点置位、MTC 节点消费后复位）。
  PreplanSlot preplan_slot_;
  std::thread preplan_thread_;
  bool preplan_reuse_{false};
  std::atomic_bool tool_enabled_{false};
  int tool_io_fun_{3};
  int tool_io_pin_{0};
  double tool_close_state_{1.0};
  double service_timeout_s_{3.0};
  double refined_timeout_s_{10.0};

  // MoveIt/MTC 伴随节点（声明在所有 MoveIt 资源之前，保证析构时最后释放）。
  rclcpp::Node::SharedPtr moveit_node_;
  // 运动输出权限（A8）：on_activate 开放、on_deactivate/on_cleanup/on_shutdown/
  // on_error 关闭；一切运动类入口经 motionOutputAllowed 单点检查。
  std::atomic_bool motion_output_permitted_{false};
  // 行为树节点只需注册一次（重复注册同 ID 会抛异常）；cleanup 后再 configure 不重复注册。
  bool bt_nodes_registered_{false};

  // 可替换职责一律只持基类指针（重构协议 2.14），构造期/参数重载时经工厂
  // 按 *.impl 参数名创建；motion_ 还需 MoveIt 初始化后由 rebuildMotionInterface
  // 装配（MoveIt 未初始化前为空，调用端判空等价于原 !move_group_）。
  std::unique_ptr<ViewPlannerBase> view_planner_;
  std::unique_ptr<QualityGateBase> quality_gate_;
  std::unique_ptr<SafetyGateBase> safety_gate_;
  std::unique_ptr<MotionInterfaceBase> motion_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::unique_ptr<GraspTask> grasp_task_;
  BT::BehaviorTreeFactory bt_factory_;
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
  // 周期身份钉（goal 钉死，设计文档第 7 节）：仅 action 受理（executeAction）
  // 与 btPrepareCycle 写入；previewContact 等预览入口不得写本成员（preview
  // 隔离，见 motion_interface.cpp previewContact 注释）。OBSERVE_ONLY 周期
  // 本成员同时是周期生效目标快照（cycleTargetSnapshot）的锁定集查询键。
  std::string cycle_target_id_;
  std::string bt_failure_reason_;
  CycleState cycle_terminal_state_{CycleState::SUCCEEDED};
  std::string cycle_terminal_message_;
  std::optional<CachedTarget> cycle_target_;
  std::optional<CachedRefined> cycle_refined_;
  // 本周期是否为候选锚点降级抓取（精化未达标回退），用于状态消息标记。
  bool cycle_degraded_grasp_{false};
  std::vector<ViewCandidate> cycle_candidates_;
  Eigen::Isometry3d cycle_entry_tip_pose_{Eigen::Isometry3d::Identity()};
  double cycle_travel_m_{0.0};
  // 再确认漂移判定的参考锚点（base 系）：FinalizeAndValidate 出口几何对应的目标
  // 锚点（精化路径=0.5·(bottom+neck)，降级路径=感知候选 center）；再确认每次
  // 漂移超限重算后更新为最新观测锚点。
  Eigen::Vector3d cycle_reference_anchor_{Eigen::Vector3d::Zero()};
  std::atomic_bool running_{false};
  std::atomic_bool cancel_requested_{false};
  // 本周期为 OBSERVE_ONLY 模式：BT 在 FinalizeAndValidate 后经 IsObserveOnly
  // 分支短路，不进 MTC/工具/撤离段；由 executeAction 按 goal.mode 设置。
  std::atomic_bool cycle_observe_only_{false};
  std::atomic_bool execution_armed_{false};
  std::atomic_bool contact_recovery_required_{false};
  // abort 路径的终局分级（ExecuteTarget::Result 常量），由 BT 失败点按需覆盖。
  std::atomic<uint8_t> pending_outcome_{ExecuteTarget::Result::FAILED};
  // 本周期是否由 action（ExecuteTarget）驱动；决定 CompleteTarget 是否推进目标。
  bool cycle_action_driven_{false};
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
  rclcpp::Service<Trigger>::SharedPtr start_service_;
  rclcpp::Service<Trigger>::SharedPtr preview_approach_service_;
  rclcpp::Service<Trigger>::SharedPtr preview_full_contact_service_;
  rclcpp::Service<Trigger>::SharedPtr cancel_service_;
  rclcpp::Service<Trigger>::SharedPtr recovery_service_;
  rclcpp::Service<Trigger>::SharedPtr query_service_;
  rclcpp::Service<Trigger>::SharedPtr photo_pose_service_;
  rclcpp::Service<SetBool>::SharedPtr arm_service_;
  // 长规划类服务独立互斥回调组（preview_approach_insert / preview_full_contact /
  // go_to_photo_pose）：组内串行，数秒级规划不挡默认组的订阅与快捷服务。
  rclcpp::CallbackGroup::SharedPtr planning_callback_group_;
  rclcpp_action::Server<ExecuteTarget>::SharedPtr cycle_action_server_;
  rclcpp_action::Server<SurveyScene>::SharedPtr survey_action_server_;
  std::thread survey_thread_;
  std::string last_snapshot_id_;
  uint32_t last_observation_count_{0};
  bool last_target_set_locked_{false};
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  PostSetParametersCallbackHandle::SharedPtr post_parameter_callback_handle_;
  // 参数监听器（构造即声明全部参数并做启动校验；运行期 set 经其内置范围
  // 校验 + onParameters 钩子，post-set 后 loadParameters 重载快照）。
  std::shared_ptr<peach_manipulation_skills_node::ParamListener> param_listener_;
  rclcpp::Client<Trigger>::SharedPtr reset_client_;
  rclcpp::Client<Trigger>::SharedPtr finalize_client_;
  rclcpp::Client<Trigger>::SharedPtr save_client_;
  rclcpp::Client<aubo_msgs::srv::SetIO>::SharedPtr tool_io_client_;
};

}  // namespace peach_manipulation_skills

#endif  // APPROACH_GRASP_NODE_IMPL_HPP_
