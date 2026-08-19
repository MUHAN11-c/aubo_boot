// Copyright 2026 aubo_e5_ros2_ws authors
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
//    * Neither the name of the aubo_e5_ros2_ws authors nor the names of its
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
//
// 回调组划分（Robotics_Tutorial 2.16-4 推荐表，2.15 头注释标准）：
//   组A timer_sub_group_（互斥）：500ms refresh timer + targets/
//     reconstruction/robot_status 三路订阅——全部持 mutex_ 工作，组内串行
//     与锁纪律天然一致；
//   组B service_group_（互斥）：~/control 服务 + ~/run_harvest action 服务端
//     回调（均为快路径，无 RPC 等待）；
//   组C policy_callback_group_（独立互斥）：~/set_operation_policy 单独成组——
//     其第二阶段须在锁外等能力端参数 RPC（≤2s 有界等待）。rclcpp 服务响应
//     不可延后（无 deferred-response API），无法改纯异步链式，故按推荐表
//     "必要时配合分离回调组与多线程 executor"以独立互斥组隔离等待，
//     等待期间 refresh/订阅/其他服务照常调度；
//   组D client_callback_group_（可重入）：全部 client（4 个 Trigger、
//     AsyncParametersClient、RunTargetCycle action client）——与调用方所在
//     组分离，杜绝同组互斥自我等待死锁；
//   默认组：仅 lifecycle/参数服务（rclcpp 内建）。
// 组外线程：批次就位自校（F3）的 tf2_ros::TransformListener 自带独立自旋
// 线程（spin_thread=true），只写自有 TF 缓冲，与本节点回调组零耦合；
// 自校判定（canTransform 静态链查找）由 refresh 在组A内完成。
// 锁纪律：一切共享状态读写仍走 mutex_；异步续接回调（goal_response/
// feedback/result）重新取锁后先复查 revision/状态再提交，保持乐观锁语义。
// 回调内禁止 spin_until_future_complete / future 阻塞等待（唯一例外：
// policy_callback 的 ≤2s 有界等待，已被独立组隔离）；photo/reset/return/
// advance（感知推进）调用由 refresh 以 wait_for(0) 轮询推进，不阻塞。

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "aubo_msgs/msg/robot_status.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "peach_harvest_msgs/action/run_harvest.hpp"
#include "peach_harvest_msgs/action/run_target_cycle.hpp"
#include "peach_harvest_msgs/msg/harvest_event.hpp"
#include "peach_harvest_msgs/msg/harvest_state.hpp"
#include "peach_harvest_msgs/msg/harvest_summary.hpp"
#include "peach_harvest_msgs/msg/reconstruction_status.hpp"
#include "peach_harvest_msgs/msg/target_outcome.hpp"
#include "peach_harvest_msgs/srv/control_harvest.hpp"
#include "peach_harvest_msgs/srv/set_operation_policy.hpp"
#include "peach_harvest_orchestrator/state_machine.hpp"
// 参数声明/默认值/校验的单一事实源（generate_parameter_library 生成，
// 定义见 src/harvest_orchestrator_node_parameters.yaml）。
#include "peach_harvest_orchestrator/harvest_orchestrator_node_parameters.hpp"
// 进程内测试入口声明（本文件末尾给出定义）。
#include "orchestrator_test_access.hpp"
#include "peach_pose_msgs/msg/peach_target_observation_array.hpp"
#include "peach_pose_msgs/srv/reopen_target.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace peach_harvest_orchestrator
{
using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using ControlHarvest = peach_harvest_msgs::srv::ControlHarvest;
using HarvestEvent = peach_harvest_msgs::msg::HarvestEvent;
using HarvestState = peach_harvest_msgs::msg::HarvestState;
using HarvestSummary = peach_harvest_msgs::msg::HarvestSummary;
using TargetOutcomeMsg = peach_harvest_msgs::msg::TargetOutcome;
using RunHarvest = peach_harvest_msgs::action::RunHarvest;
using RunGoalHandle = rclcpp_action::ServerGoalHandle<RunHarvest>;
using RunTargetCycle = peach_harvest_msgs::action::RunTargetCycle;
using TargetGoalHandle = rclcpp_action::ClientGoalHandle<RunTargetCycle>;
using SetOperationPolicy = peach_harvest_msgs::srv::SetOperationPolicy;
using Trigger = std_srvs::srv::Trigger;
using TriggerFuture = rclcpp::Client<Trigger>::SharedFuture;
// 服务请求常量别名，仅用于下方枚举对齐断言。
using ControlRequest = ControlHarvest::Request;

// 纯核枚举（state_machine.hpp）底层值与 peach_harvest_msgs 常量按声明顺序隐式
// 对齐，节点侧靠 static_cast 互转（make_state / 反馈阶段 / 结果记账 / 控制命令
// 校验）；纯核零 ROS 不能 include 消息头，故断言放在本节点 TU。
// 新增枚举值必须同步消息常量并补断言。
// OperationMode ↔ HarvestState MODE_*
static_assert(
  static_cast<uint8_t>(OperationMode::AUTO) == HarvestState::MODE_AUTO,
  "OperationMode 与 HarvestState MODE_* 常量失配");
static_assert(
  static_cast<uint8_t>(OperationMode::PAUSED) == HarvestState::MODE_PAUSED,
  "OperationMode 与 HarvestState MODE_* 常量失配");
static_assert(
  static_cast<uint8_t>(OperationMode::MAINTENANCE) == HarvestState::MODE_MAINTENANCE,
  "OperationMode 与 HarvestState MODE_* 常量失配");
// BatchState ↔ HarvestState 同名批次状态常量
static_assert(
  static_cast<uint8_t>(BatchState::WAITING_READY) == HarvestState::WAITING_READY,
  "BatchState 与 HarvestState 批次状态常量失配");
static_assert(
  static_cast<uint8_t>(BatchState::DISCOVERY) == HarvestState::DISCOVERY,
  "BatchState 与 HarvestState 批次状态常量失配");
static_assert(
  static_cast<uint8_t>(BatchState::RUNNING) == HarvestState::RUNNING,
  "BatchState 与 HarvestState 批次状态常量失配");
static_assert(
  static_cast<uint8_t>(BatchState::PAUSE_PENDING) == HarvestState::PAUSE_PENDING,
  "BatchState 与 HarvestState 批次状态常量失配");
static_assert(
  static_cast<uint8_t>(BatchState::PAUSED) == HarvestState::PAUSED,
  "BatchState 与 HarvestState 批次状态常量失配");
static_assert(
  static_cast<uint8_t>(BatchState::MAINTENANCE) == HarvestState::MAINTENANCE,
  "BatchState 与 HarvestState 批次状态常量失配");
static_assert(
  static_cast<uint8_t>(BatchState::COMPLETED) == HarvestState::COMPLETED,
  "BatchState 与 HarvestState 批次状态常量失配");
static_assert(
  static_cast<uint8_t>(BatchState::RECOVERY_REQUIRED) == HarvestState::RECOVERY_REQUIRED,
  "BatchState 与 HarvestState 批次状态常量失配");
static_assert(
  static_cast<uint8_t>(BatchState::INTERRUPTED) == HarvestState::INTERRUPTED,
  "BatchState 与 HarvestState 批次状态常量失配");
// TargetPhase ↔ HarvestState 目标阶段常量（首尾带 TARGET_ 前缀）
static_assert(
  static_cast<uint8_t>(TargetPhase::IDLE) == HarvestState::TARGET_IDLE,
  "TargetPhase 与 HarvestState 目标阶段常量失配");
static_assert(
  static_cast<uint8_t>(TargetPhase::SELECTING) == HarvestState::SELECTING,
  "TargetPhase 与 HarvestState 目标阶段常量失配");
static_assert(
  static_cast<uint8_t>(TargetPhase::OBSERVING) == HarvestState::OBSERVING,
  "TargetPhase 与 HarvestState 目标阶段常量失配");
static_assert(
  static_cast<uint8_t>(TargetPhase::FINALIZING) == HarvestState::FINALIZING,
  "TargetPhase 与 HarvestState 目标阶段常量失配");
static_assert(
  static_cast<uint8_t>(TargetPhase::VALIDATING) == HarvestState::VALIDATING,
  "TargetPhase 与 HarvestState 目标阶段常量失配");
static_assert(
  static_cast<uint8_t>(TargetPhase::APPROACHING) == HarvestState::APPROACHING,
  "TargetPhase 与 HarvestState 目标阶段常量失配");
static_assert(
  static_cast<uint8_t>(TargetPhase::TOOL_ACTION) == HarvestState::TOOL_ACTION,
  "TargetPhase 与 HarvestState 目标阶段常量失配");
static_assert(
  static_cast<uint8_t>(TargetPhase::RETREATING) == HarvestState::RETREATING,
  "TargetPhase 与 HarvestState 目标阶段常量失配");
static_assert(
  static_cast<uint8_t>(TargetPhase::COMPLETING) == HarvestState::COMPLETING,
  "TargetPhase 与 HarvestState 目标阶段常量失配");
static_assert(
  static_cast<uint8_t>(TargetPhase::SUCCEEDED) == HarvestState::TARGET_SUCCEEDED,
  "TargetPhase 与 HarvestState 目标阶段常量失配");
static_assert(
  static_cast<uint8_t>(TargetPhase::SKIPPED) == HarvestState::TARGET_SKIPPED,
  "TargetPhase 与 HarvestState 目标阶段常量失配");
static_assert(
  static_cast<uint8_t>(TargetPhase::FAILED) == HarvestState::TARGET_FAILED,
  "TargetPhase 与 HarvestState 目标阶段常量失配");
// ControlCommand ↔ ControlHarvest 请求命令常量
static_assert(
  static_cast<uint8_t>(ControlCommand::PAUSE) == ControlRequest::PAUSE,
  "ControlCommand 与 ControlHarvest 命令常量失配");
static_assert(
  static_cast<uint8_t>(ControlCommand::RESUME) == ControlRequest::RESUME,
  "ControlCommand 与 ControlHarvest 命令常量失配");
static_assert(
  static_cast<uint8_t>(ControlCommand::ENTER_MAINTENANCE) ==
  ControlRequest::ENTER_MAINTENANCE,
  "ControlCommand 与 ControlHarvest 命令常量失配");
static_assert(
  static_cast<uint8_t>(ControlCommand::EXIT_MAINTENANCE) ==
  ControlRequest::EXIT_MAINTENANCE,
  "ControlCommand 与 ControlHarvest 命令常量失配");
static_assert(
  static_cast<uint8_t>(ControlCommand::CANCEL_NOW) == ControlRequest::CANCEL_NOW,
  "ControlCommand 与 ControlHarvest 命令常量失配");
static_assert(
  static_cast<uint8_t>(ControlCommand::SKIP_TARGET) == ControlRequest::SKIP_TARGET,
  "ControlCommand 与 ControlHarvest 命令常量失配");
static_assert(
  static_cast<uint8_t>(ControlCommand::ACKNOWLEDGE_RECOVERY) ==
  ControlRequest::ACKNOWLEDGE_RECOVERY,
  "ControlCommand 与 ControlHarvest 命令常量失配");
// TargetOutcome ↔ TargetOutcome.msg 同名常量
static_assert(
  static_cast<uint8_t>(TargetOutcome::SUCCEEDED) == TargetOutcomeMsg::SUCCEEDED,
  "TargetOutcome 与 TargetOutcome.msg 常量失配");
static_assert(
  static_cast<uint8_t>(TargetOutcome::SKIPPED_QUALITY) ==
  TargetOutcomeMsg::SKIPPED_QUALITY,
  "TargetOutcome 与 TargetOutcome.msg 常量失配");
static_assert(
  static_cast<uint8_t>(TargetOutcome::SKIPPED_UNREACHABLE) ==
  TargetOutcomeMsg::SKIPPED_UNREACHABLE,
  "TargetOutcome 与 TargetOutcome.msg 常量失配");
static_assert(
  static_cast<uint8_t>(TargetOutcome::FAILED) == TargetOutcomeMsg::FAILED,
  "TargetOutcome 与 TargetOutcome.msg 常量失配");
static_assert(
  static_cast<uint8_t>(TargetOutcome::CANCELED) == TargetOutcomeMsg::CANCELED,
  "TargetOutcome 与 TargetOutcome.msg 常量失配");

class HarvestOrchestratorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit HarvestOrchestratorNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : LifecycleNode("peach_harvest_orchestrator", options)
  {
    // ParamListener 构造即声明全部参数并做启动校验（yaml 覆盖值非法时抛
    // InvalidParameterValueException 启动失败）；内置范围校验随每次 set 生效。
    // 生效语义保持迁移前不变：readiness.* 由 refresh 每 tick 从监听器快照
    // 直读（热生效），其余参数在 on_configure 一次性缓存进成员。
    param_listener_ = std::make_shared<ParamListener>(
      get_node_parameters_interface(), get_logger());
  }

  ~HarvestOrchestratorNode() override
  {
    stop_requested_.store(true);
    if (execute_thread_.joinable()) {execute_thread_.join();}
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    // 缓存拍照前置/复扫策略参数（yaml 为权威源，运行期不预期动态改）。
    // 重试/熔断计数的下限已由参数定义的 validation 保证（原为此处钳制）。
    const auto params = param_listener_->get_params();
    photo_pose_enabled_ = params.photo_pose.enabled;
    photo_max_retries_ = static_cast<uint32_t>(params.photo_pose.max_retries);
    photo_retry_cooldown_s_ = params.photo_pose.retry_cooldown_s;
    photo_service_timeout_s_ = params.photo_pose.service_timeout_s;
    photo_return_on_complete_ = params.photo_pose.return_on_complete;
    rescan_until_empty_ = params.harvest.rescan_until_empty;
    max_rounds_ = static_cast<uint32_t>(params.harvest.max_rounds);
    harvest_stall_timeout_s_ = params.harvest.stall_timeout_s;
    // 残局抬质量（协议 2.8 残局 OBSERVE_ONLY，阶段 E3）：总开关与每目标
    // 每批次次数上限。
    observe_retry_enabled_ = params.harvest.observe_retry_enabled;
    observe_retry_max_ = static_cast<uint32_t>(params.harvest.observe_retry_max);
    // 换场景开关（协议 2.3）：批次首轮开窗前先清感知身份记忆。
    fresh_scene_ = params.harvest.fresh_scene;
    // 收齐窗口编排侧总超时（协议 2.3 监督）配置下限。
    collect_timeout_s_ = params.harvest.collect_timeout_s;
    dispatch_retry_delay_s_ = params.dispatch.retry_delay_s;
    max_dispatch_retries_ = static_cast<uint32_t>(params.dispatch.max_retries);
    max_consecutive_rejections_ =
      static_cast<uint32_t>(params.dispatch.max_consecutive_rejections);
    // 感知推进协议（2.6）：失败重试冷却/熔断上限/单次 RPC 超时。
    advance_retry_delay_s_ = params.advance.retry_delay_s;
    advance_max_retries_ = static_cast<uint32_t>(params.advance.max_retries);
    advance_timeout_s_ = params.advance.timeout_s;
    // 批次启动就位自校开关（阶段 F3；on_configure 缓存，与 harvest.* 其余
    // 参数同语义——yaml 为权威源，运行期不预期动态改）。
    preflight_check_enabled_ = params.harvest.preflight_check;
    // 就位自校的 TF 查询通道（F3）：TransformListener 自带独立自旋线程
    // （spin_thread=true），不占用本节点回调组/executor 配额；canTransform
    // 用 TimePointZero 查静态变换，纯缓冲查找、非阻塞。
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *tf_buffer_, this, true);

    state_pub_ = create_publisher<HarvestState>("~/state", rclcpp::QoS(1).transient_local());
    event_pub_ = create_publisher<HarvestEvent>("~/events", 50);
    // 回调组划分见文件头注释（组A–组D）。
    timer_sub_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    service_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    policy_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    client_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    control_service_ = create_service<ControlHarvest>(
      "~/control", std::bind(
        &HarvestOrchestratorNode::control_callback, this,
        std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), service_group_);
    policy_service_ = create_service<SetOperationPolicy>(
      "~/set_operation_policy", std::bind(
        &HarvestOrchestratorNode::policy_callback, this,
        std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), policy_callback_group_);
    action_server_ = rclcpp_action::create_server<RunHarvest>(
      this, "~/run_harvest",
      std::bind(&HarvestOrchestratorNode::goal_callback, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(&HarvestOrchestratorNode::cancel_callback, this, std::placeholders::_1),
      std::bind(&HarvestOrchestratorNode::accepted_callback, this, std::placeholders::_1),
      rcl_action_server_get_default_options(), service_group_);
    target_cycle_client_ = rclcpp_action::create_client<RunTargetCycle>(
      this, params.target_cycle_action_name, client_callback_group_);
    approach_parameters_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this, params.approach_node_name, rclcpp::ParametersQoS(),
      client_callback_group_);
    // 对端 lifecycle Active 判定（A8 最小实现）：GetState 异步轮询，结果折叠进
    // motion 路就绪与派发守卫（对端未 Active 时不派发、blockers 报 motion）；
    // 未实现自动激活序列——能力端激活由各自 launch 的 lifecycle 事件负责。
    grasp_state_client_ = create_client<lifecycle_msgs::srv::GetState>(
      params.approach_node_name + "/get_state", rclcpp::ServicesQoS(),
      client_callback_group_);
    // 感知固定优先级计划推进（Trigger）：协议 2.6 失败重试/熔断状态机，
    // future 只存不等待，由 refresh 轮询推进（与拍照前置同一模式）。
    complete_target_client_ = create_client<Trigger>(
      params.complete_target_service_name,
      rclcpp::ServicesQoS(), client_callback_group_);
    // 拍照前置两步（去拍照位姿 + 重置感知收齐）：future 只存不等待，
    // 由 refresh 轮询结果，回调内绝不持锁等 RPC。
    photo_pose_client_ = create_client<Trigger>(
      params.photo_pose_service_name,
      rclcpp::ServicesQoS(), client_callback_group_);
    reset_targets_client_ = create_client<Trigger>(
      params.reset_targets_service_name,
      rclcpp::ServicesQoS(), client_callback_group_);
    // fresh_scene（协议 2.3，阶段 D2）：换场景批次开窗前先清感知世界系
    // 身份记忆；同一 Trigger 异步轮询模式，失败处理与 reset 同级。
    clear_memory_client_ = create_client<Trigger>(
      params.clear_memory_service_name,
      rclcpp::ServicesQoS(), client_callback_group_);
    // E3 残局抬质量回路：OBSERVE_ONLY 成功终局后请感知重开该目标
    // （移出 completed_ids 恢复可选，下一帧按固定优先级重新选中 → 正常
    // 派发链 FULL 重试）。fire-and-forget 异步调用：失败/被拒只记审计
    // 事件，不阻塞终局路由、不进熔断/重试预算（抬质量本是 best-effort）。
    reopen_target_client_ = create_client<peach_pose_msgs::srv::ReopenTarget>(
      params.reopen_target_service_name,
      rclcpp::ServicesQoS(), client_callback_group_);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = timer_sub_group_;
    targets_sub_ = create_subscription<peach_pose_msgs::msg::PeachTargetObservationArray>(
      params.topics.target_observations, 10,
      std::bind(&HarvestOrchestratorNode::targets_callback, this, std::placeholders::_1),
      subscription_options);
    // 类型化重建诊断（2026-08 起替换裸 JSON）：就绪门只用到时刻，不读字段。
    // A10：订阅侧请求 DDS Deadline 2s（发布侧 offered 1.5s，requested≥offered
    // 兼容），违约计数+节流告警——DDS 层补强应用层 2s 新鲜度门，心跳断供在
    // DDS 事件层立即可见（不改就绪门判定本身）。
    rclcpp::SubscriptionOptions recon_subscription_options;
    recon_subscription_options.callback_group = timer_sub_group_;
    recon_subscription_options.event_callbacks.deadline_callback =
      [this](rclcpp::QOSDeadlineRequestedInfo & info) {
        recon_deadline_misses_.store(
          static_cast<uint64_t>(info.total_count), std::memory_order_relaxed);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "重建 diagnostics 心跳超过 requested deadline 2s（累计违约 %d 次）",
          info.total_count);
      };
    reconstruction_sub_ = create_subscription<peach_harvest_msgs::msg::ReconstructionStatus>(
      params.topics.reconstruction_status,
      rclcpp::QoS(1).transient_local().deadline(rclcpp::Duration::from_seconds(2.0)),
      std::bind(
        &HarvestOrchestratorNode::reconstruction_callback, this, std::placeholders::_1),
      recon_subscription_options);
    robot_sub_ = create_subscription<aubo_msgs::msg::RobotStatus>(
      params.topics.robot_status, 10,
      std::bind(&HarvestOrchestratorNode::robot_callback, this, std::placeholders::_1),
      subscription_options);
    timer_ = create_wall_timer(
      500ms, std::bind(&HarvestOrchestratorNode::refresh, this), timer_sub_group_);

    OperationPolicy policy{
      params.auto_start_enabled, params.execution_enabled,
      params.grasp_enabled, params.tool_enabled};
    const auto result = machine_.set_policy(policy, "startup-policy", 0);
    if (!result.accepted) {
      RCLCPP_ERROR(get_logger(), "启动策略无效: %s", result.message.c_str());
      return CallbackReturn::FAILURE;
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    state_pub_->on_activate();
    event_pub_->on_activate();
    refresh();
    publish_event("orchestrator_activated", "采摘编排器已激活");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    stop_requested_.store(true);
    TargetGoalHandle::SharedPtr target_goal;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      target_goal = active_target_goal_;
      // 对端 Active 判定随停用失效，重新激活后由轮询重建。
      grasp_node_active_ = false;
      grasp_state_poll_pending_ = false;
      // 停用期间不再推进拍照/复扫/感知推进：清空在途调用，重新激活后从头走
      // 拍照前置；已记账目标的推进随激活后的 refresh 重新登记会丢失——停用即
      // 批次中断（活动 goal 已取消），与 CANCEL_NOW 同等语义。
      photo_future_ = TriggerFuture();
      reset_future_ = TriggerFuture();
      clear_memory_future_ = TriggerFuture();
      return_future_ = TriggerFuture();
      return_call_pending_ = false;
      advance_step_ = AdvanceStep::IDLE;
      advance_future_ = TriggerFuture();
      advance_failures_ = 0;
      photo_step_ = PhotoStep::NOT_STARTED;
      // 就位自校（F3）随停用复位：停用即批次中断，重新激活后从头走拍照
      // 前置，自校一并重新判定（不复用旧通过态）。
      preflight_passed_ = false;
      preflight_blocked_ = false;
      preflight_failure_signature_.clear();
      preflight_last_failure_event_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
      // 停用即批次中断（同上）：丢失目标跟踪集一并清册，重启后按新帧重建；
      // 残局抬质量账（E3）同样清册——活动 goal 已取消，在途周期随取消终结。
      locked_target_ids_.clear();
      observe_retry_counts_.clear();
      observe_retry_retired_this_round_.clear();
      observe_retry_active_id_.clear();
    }
    // 取消传播到活动单目标周期，避免停用时能力端继续运动。
    if (target_goal) {target_cycle_client_->async_cancel_goal(target_goal);}
    state_pub_->on_deactivate();
    event_pub_->on_deactivate();
    // 等 RunHarvest 执行线程退出（其循环 200ms 一拍，随 stop 标志退出）。
    if (execute_thread_.joinable()) {execute_thread_.join();}
    return CallbackReturn::SUCCESS;
  }

private:
  // 拍照前置步骤（节点侧状态，不进状态机枚举）：批次进入 DISCOVERY 或复扫
  // 开启新一轮时，先把机械臂送到全局拍照位姿，再重置感知收齐锁定；
  // 两步均为异步调用，future 由 refresh 轮询，回调内不持锁等 RPC。
  enum class PhotoStep : uint8_t
  {
    NOT_STARTED,    // 本轮尚未发起（进入 DISCOVERY/RUNNING 且无活动目标时触发）
    MOVE_PENDING,   // 等待到点后发起 go_to_photo_pose（含冷却等待）
    MOVING,         // go_to_photo_pose 调用在途
    CLEAR_PENDING,  // 拍照位姿已到，待发起 clear_target_memory（仅 fresh_scene 首轮）
    CLEARING,       // clear_target_memory 调用在途
    RESET_PENDING,  // 拍照位姿已到（且清记忆完成/跳过），待发起 reset_global_targets
    RESETTING,      // reset_global_targets 调用在途
    DONE            // 本轮拍照前置完成（或被策略跳过），允许派发目标
  };

  HarvestState make_state() const
  {
    const auto & source = machine_.snapshot();
    HarvestState message;
    message.header.stamp = now();
    message.revision = source.revision;
    message.run_id = run_id_;
    message.cycle_id = current_cycle_id_;
    message.operation_mode = static_cast<uint8_t>(source.mode);
    message.batch_state = static_cast<uint8_t>(source.batch_state);
    message.target_phase = static_cast<uint8_t>(source.target_phase);
    message.action_active = source.run_active;
    message.target_id = source.target_id;
    message.auto_start_enabled = source.policy.auto_start_enabled;
    message.execution_enabled = source.policy.execution_enabled;
    message.grasp_enabled = source.policy.grasp_enabled;
    message.tool_enabled = source.policy.tool_enabled;
    message.recovery_required = source.recovery_required;
    // 进度 = 已处理目标数 / 本批次累计发现数（0 除保护，超界钳到 1）。
    const uint64_t processed = source.counters.attempted;
    message.progress = discovered_total_ > 0 ?
      static_cast<float>(std::min(processed, discovered_total_)) /
      static_cast<float>(discovered_total_) : 0.0f;
    // 轮次/拍照阶段是节点侧状态，拼进 message 供 dashboard 观测（状态机不感知）。
    message.message = source.message;
    if (source.batch_state == BatchState::DISCOVERY ||
      source.batch_state == BatchState::RUNNING)
    {
      if (explicit_mode_) {
        // 显式清单批次：无轮次/拍照前置概念，投影清单进度。
        message.message += "；显式清单 " +
          std::to_string(std::min(explicit_index_, explicit_queue_.size())) + "/" +
          std::to_string(explicit_queue_.size());
      } else {
        message.message += "；第" + std::to_string(round_) + "/" +
          std::to_string(max_rounds_) + "轮";
        if (preflight_blocked_) {
          // 就位自校（F3）未通过：拍照前置尚未发起，单独标注避免误读为
          // "拍照前置在途"（web 过程线据此区分挂起原因）。
          message.message += "，就位自校未通过";
        } else if (photo_step_ != PhotoStep::DONE) {
          message.message += "，拍照前置中";
        } else if (!targets_locked_) {
          // 收齐窗口进行中：透出感知发现进度摘要（R-D8 / 2.3 发现进度），
          // web 侧经 message 展示（HarvestState 无新字段，消息契约冻结）。
          message.message += "，收齐中 " + std::to_string(collecting_count_) +
            " 目标/" + std::to_string(pending_count_) + " 确认中";
        }
      }
      // 残局抬质量周期（E3）在途标注：活动周期是 OBSERVE_ONLY 时透出，
      // web/落盘可区分"采摘周期"与"抬质量观察周期"（消息契约冻结，无新字段）。
      if (source.target_active && !observe_retry_active_id_.empty()) {
        message.message += "，残局抬质量 observe_retry 中";
      }
    }
    message.blockers = source.blockers;
    // 就位自校（F3）失败原因投影：节点侧状态，状态机不感知——自校只在批次
    // 首轮拍照前置前判定（此时批次已在 DISCOVERY，goal 受理门读状态机
    // 快照 blockers 不受本投影影响，不会因自校失败误拒新批次 goal）。
    if (preflight_blocked_) {
      message.blockers.emplace_back("preflight");
    }
    // 可控命令投影：状态机允许表 → 消息命令常量（取值经 static_assert 钉死）。
    const auto commands = allowed_commands(source);
    message.permissions.reserve(commands.size());
    for (const auto command : commands) {
      message.permissions.push_back(static_cast<uint8_t>(command));
    }
    return message;
  }

  // 由状态机快照组装批次摘要；调用时必须已持有 mutex_。
  HarvestSummary make_summary_locked() const
  {
    const auto & snap = machine_.snapshot();
    HarvestSummary summary;
    summary.run_id = run_id_;
    summary.discovered = static_cast<uint32_t>(discovered_total_);
    summary.attempted = snap.counters.attempted;
    summary.succeeded = snap.counters.succeeded;
    summary.skipped_quality = snap.counters.skipped_quality;
    summary.skipped_unreachable = snap.counters.skipped_unreachable;
    summary.failed = snap.counters.failed;
    summary.canceled = snap.counters.canceled;
    if (batch_started_.nanoseconds() > 0) {
      summary.elapsed = static_cast<builtin_interfaces::msg::Duration>(now() - batch_started_);
    }
    summary.outcomes.reserve(snap.outcomes.size());
    for (const auto & record : snap.outcomes) {
      TargetOutcomeMsg entry;
      entry.target_id = record.target_id;
      entry.outcome = static_cast<uint8_t>(record.outcome);
      entry.reason = record.reason;
      // 单目标墙钟（派发→终局）与质量占位分（协议 2.13-E5 埋点；占位语义
      // 见 result_callback 填充处注释，待阶段 E 接入能力端真实质量分）。
      entry.elapsed = static_cast<builtin_interfaces::msg::Duration>(
        rclcpp::Duration::from_seconds(record.elapsed_s));
      entry.quality_score = record.quality_score;
      summary.outcomes.push_back(entry);
    }
    return summary;
  }

  // 批次吞吐（协议 2.13-E5）：targets/hour = attempted/elapsed_hours。
  // HarvestSummary 消息无吞吐字段且契约冻结（peach_harvest_msgs 本阶段不改），
  // 吞吐按约定拼进批次终局事件 message（web_runs 落盘可追踪）；elapsed<=0
  // （批次秒级结束或未计时）时不计算，返回空串。调用时必须已持有 mutex_。
  std::string batch_throughput_text_locked() const
  {
    if (batch_started_.nanoseconds() <= 0) {return "";}
    const double elapsed_s = (now() - batch_started_).seconds();
    if (elapsed_s <= 0.0) {return "";}
    const double per_hour =
      static_cast<double>(machine_.snapshot().counters.attempted) /
      (elapsed_s / 3600.0);
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "；吞吐 %.1f 目标/小时", per_hour);
    return buffer;
  }

  void targets_callback(
    const peach_pose_msgs::msg::PeachTargetObservationArray::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto arrival = now();
    // 发布周期 EMA（2.11 自适应新鲜度）：与下方到达时刻同一时钟源。
    targets_period_.observe(arrival.seconds());
    // 目标集合由未锁定变为锁定视为新的一轮发现：累计该轮目标数。
    // 复扫每轮都会经历 reset（解锁）→ 收齐（再锁定）的完整沿，多轮天然覆盖。
    if (message->target_set_locked && !targets_locked_) {
      discovered_total_ += message->target_count;
      publish_event(
        "round_locked",
        "第" + std::to_string(round_) + "轮锁定 " +
        std::to_string(message->target_count) + " 个目标",
        HarvestEvent::INFO);
      // 收齐窗口实测时长（2.3 监督的自适应余量）：从本轮 PhotoStep 完成
      // （编排侧开窗计时起点）到锁定沿的墙钟；下一轮超时上限取
      // max(配置下限, 1.5×本值)。collect_since_ 归零即"locked 到达重置计时"。
      if (collect_since_.nanoseconds() > 0) {
        last_collect_window_s_ = (arrival - collect_since_).seconds();
        collect_since_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
      }
    }
    last_target_count_ = message->target_count;
    // 发现进度摘要（R-D8 / 2.3）：锁定前透出"收齐中 N 目标/M 确认中"。
    collecting_count_ = message->collecting_count;
    pending_count_ = message->pending_count;
    // 派发锁随感知计划推进解锁：selected 变化（含变为空）说明上一目标的终局
    // 记账已被感知吸收，允许派发新目标；selected 不变则保持锁定防重派。
    if (message->selected_target_id != last_dispatched_target_) {
      last_dispatched_target_.clear();
    }
    // 重试预算与冷却只在 selected 真实跨帧变化时复位（与上一帧消息比，
    // 不能与 last_dispatched_target_ 比——拒绝路径会清空它，导致每帧消息
    // 都把冷却/计数误复位，冷却重试被架空成每帧重派）。
    if (message->selected_target_id != last_seen_selected_) {
      dispatch_retries_ = 0;
      retry_not_before_ = std::chrono::steady_clock::time_point{};
      last_seen_selected_ = message->selected_target_id;
    }
    selected_target_id_ = message->selected_target_id;
    targets_locked_ = message->target_set_locked;
    targets_received_ = arrival;
    // 最近一帧感知锁定集（含 selected）：显式清单批次派发前的 ID 校验依据；
    // 锁定帧内感知对锁定集每个 ID 各发一条观测，即本轮锁定集全集——丢失
    // 目标记账（2.4）的帧间比对基准。known_target_order_ 保留观测顺序
    // （=感知固定优先级序），供残局抬质量（E3）按优先级选候选。
    known_target_ids_.clear();
    known_target_ids_.reserve(message->observations.size() + 1);
    known_target_order_.clear();
    known_target_order_.reserve(message->observations.size() + 1);
    for (const auto & observation : message->observations) {
      known_target_ids_.insert(observation.target_id);
      known_target_order_.push_back(observation.target_id);
    }
    if (!message->selected_target_id.empty()) {
      if (known_target_ids_.insert(message->selected_target_id).second) {
        known_target_order_.push_back(message->selected_target_id);
      }
    }
    account_dropped_targets_locked(message->target_set_locked);
  }

  // 丢失目标记账（协议 2.4，阶段 D2）：跟踪本轮感知锁定集中尚无终局账的
  // 目标 ID（locked_target_ids_），锁定帧间比对。上一帧在册、本帧从锁定集
  // 消失的目标分三路：
  //   已有终局记账（含 HARVESTED 完成目标）→ 仅出册，不重复记账；
  //   正是当前活动周期目标 → 留在册等 result_callback 终局记账（本路径
  //     绝不双记；若终局走不记账的取消/拒绝冷却路径，随后帧目标已不活动
  //     且无账，再按丢失补记）；
  //   其余（未派发、无终局账）→ 感知 anchor_drop 语义：记
  //     SKIPPED_UNREACHABLE（reason="目标丢失超时(anchor_drop)"）并发
  //     AUDIT 事件 target_dropped。
  // 整集变化非丢失、不记账：locked=false 帧（reset_global_targets/复扫
  // 轮次切换的收齐窗口）与新批次（accepted_callback）直接清册——整集
  // 重置语义而非目标丢失；显式清单批次不消费感知锁定计划，整段跳过。
  // 与 decide_round 口径兼容（2.8）：丢失记账走 attempted（本轮已处理 +1），
  // 感知 target_count 随锁定集递减（-1），两者同向收敛，"本轮锁定集合
  // 全部处理完"的终止判定不受影响。调用时必须已持有 mutex_。
  void account_dropped_targets_locked(bool locked)
  {
    if (!locked || explicit_mode_) {
      locked_target_ids_.clear();
      return;
    }
    const auto & snap = machine_.snapshot();
    // 终局账判定：outcomes 线性扫描（锁定集规模小、感知帧率低，开销可忽略）。
    const auto accounted = [&snap](const std::string & id) {
        return std::any_of(
          snap.outcomes.begin(), snap.outcomes.end(),
          [&id](const TargetOutcomeRecord & record) {return record.target_id == id;});
      };
    for (auto it = locked_target_ids_.begin(); it != locked_target_ids_.end(); ) {
      if (known_target_ids_.count(*it) != 0) {++it;continue;}  // 仍在锁定集，留册
      if (accounted(*it)) {it = locked_target_ids_.erase(it);continue;}  // 已有终局账
      if (snap.target_active && snap.target_id == *it) {++it;continue;}  // 活动周期等终局
      // 批次无可记窗口（终态/待恢复）时 record_passive_outcome 返回 false：
      // 不记账不发事件，仅出册。
      if (machine_.record_passive_outcome(
          *it, TargetOutcome::SKIPPED_UNREACHABLE, "目标丢失超时(anchor_drop)"))
      {
        publish_event(
          "target_dropped", "目标丢失超时(anchor_drop)，已按不可达记账",
          HarvestEvent::AUDIT, "", *it);
      }
      it = locked_target_ids_.erase(it);
    }
    // 新入锁定集的目标纳入跟踪；已有终局账的完成目标（HARVESTED 留存锁定
    // 集内）不纳入——它们不会再"消失"，防御重复记账。
    for (const auto & id : known_target_ids_) {
      if (!accounted(id)) {locked_target_ids_.insert(id);}
    }
  }

  void reconstruction_callback(
    const peach_harvest_msgs::msg::ReconstructionStatus::SharedPtr)
  {
    // 只记录到达时刻：1Hz 心跳满足重建就绪门的新鲜度判定，不解析内容
    std::lock_guard<std::mutex> lock(mutex_);
    const auto arrival = now();
    recon_period_.observe(arrival.seconds());
    reconstruction_received_ = arrival;
  }

  void robot_callback(const aubo_msgs::msg::RobotStatus::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto arrival = now();
    robot_period_.observe(arrival.seconds());
    robot_status_ = *message;
    robot_received_ = arrival;
  }

  // G6 失能门样本折算（协议 2.5，I5）：robot_status 新鲜∧非急停∧非故障∧
  // 已上电（∧可运动）∧能力端 action 在线∧对端 lifecycle Active。复用纯核
  // ReadinessTracker 的 motion 路推算（与 refresh 的四路就绪同一套规则，
  // readiness.* 热读）；调用时必须已持有 mutex_。
  bool motion_ready_locked() const
  {
    const auto readiness = param_listener_->get_params().readiness;
    ReadinessSample sample;
    sample.now_s = now().seconds();
    sample.robot_received_s = robot_received_.seconds();
    sample.robot_period_s = robot_period_.period_ema();
    sample.robot_e_stopped = robot_status_.e_stopped != 0;
    sample.robot_in_error = robot_status_.in_error != 0;
    sample.robot_drives_powered = robot_status_.drives_powered != 0;
    sample.robot_motion_possible = robot_status_.motion_possible != 0;
    sample.action_server_ready =
      target_cycle_client_->action_server_is_ready() && grasp_node_active_;
    return ReadinessTracker(readiness.timeout_s, readiness.require_robot_status)
           .evaluate(sample).motion;
  }

  // 残局抬质量候选（协议 2.8 残局 OBSERVE_ONLY，阶段 E3；调用时必须已持有
  // mutex_）：总开关关闭时恒空串；否则按模式选优先级序（显式批次=清单自身
  // 顺序——清单外目标永不入选；自动批次=最近锁定帧的感知优先级序）委托纯核
  // pick_observe_retry_candidate 判定（资格判据见其实现注释）。
  std::string pick_observe_retry_candidate_locked() const
  {
    if (!observe_retry_enabled_) {return "";}
    return pick_observe_retry_candidate(
      machine_.snapshot().outcomes,
      explicit_mode_ ? explicit_queue_ : known_target_order_,
      known_target_ids_, observe_retry_counts_, observe_retry_retired_this_round_,
      observe_retry_max_);
  }

  void dispatch_target_locked()
  {
    // 派发拒绝后的冷却重试窗口：到期前不派发（steady 时钟，零值恒不阻塞）。
    if (std::chrono::steady_clock::now() < retry_not_before_) {return;}
    // 显式清单批次：selected 不从感知计划取，直接取清单头（保持给定顺序）；
    // 清单耗尽时不派发（收口由 refresh 的显式终止块完成）。
    std::string candidate = explicit_mode_ ?
      (explicit_index_ < explicit_queue_.size() ? explicit_queue_[explicit_index_] :
      std::string()) :
      selected_target_id_;
    // 残局抬质量（协议 2.8，阶段 E3）：正常链候选为空（自动批次=selected 空，
    // 显式批次=清单耗尽）且本轮收齐已锁定、拍照前置完成时，改派残局目标的
    // OBSERVE_ONLY 周期尝试抬质量。自动批次要求 targets_locked_（未锁定的
    // 收齐窗口期不是残局）；显式批次跳过拍照前置/不消费锁定计划，但仍要求
    // 候选在最近锁定帧观测集内（纯核资格 2），否则能力端无锚点必拒。
    bool observe_retry = false;
    if (candidate.empty() && photo_step_ == PhotoStep::DONE &&
      (explicit_mode_ || targets_locked_))
    {
      candidate = pick_observe_retry_candidate_locked();
      observe_retry = !candidate.empty();
    }
    // 拍照前置未完成（含等待感知重新锁定）时不派发；selected 非空本身隐含已锁定。
    // 六项前置（协议 2.5 G-DISP，含 G6 失能门）的判定在状态机库纯核
    // allow_dispatch，节点只做事实采集。
    if (!allow_dispatch(
        photo_step_ == PhotoStep::DONE, candidate, last_dispatched_target_,
        machine_.snapshot().target_active,
        target_cycle_client_->action_server_is_ready() && grasp_node_active_,
        motion_ready_locked()))
    {
      return;
    }
    if (!machine_.begin_target(candidate)) {return;}
    // 显式批次派发前校验：ID 须在最近一帧感知锁定集内；缺失直接按不可达记账
    // 并推进清单，不派发 goal（显式批次不消费感知锁定计划）。observe-retry
    // 候选不走进账/推进分支——其清单条目早已终局记账，且纯核资格 2 已保证
    // 候选在锁定帧观测集内。
    if (explicit_mode_ && !observe_retry && known_target_ids_.count(candidate) == 0) {
      machine_.record_target_outcome(
        candidate, TargetOutcome::SKIPPED_UNREACHABLE, "显式目标不在感知锁定集内");
      ++explicit_index_;
      publish_event(
        "target_skipped", "显式目标不在感知锁定集内，已跳过", HarvestEvent::WARNING,
        "", candidate);
      publish_state();
      return;
    }
    last_dispatched_target_ = candidate;
    // 残局抬质量派发即消耗次数并挂本轮去重/在途标记（E3）：次数派发时消耗、
    // 拒绝/取消不归还——能力端系统性拒绝（如缓存 selected 与残局目标不匹配）
    // 场景下批次仍有界收口；retired 集合保证每轮每目标至多抬一次，
    // 终局未改善时让位下一个残局目标而非原地反复。
    if (observe_retry) {
      ++observe_retry_counts_[candidate];
      observe_retry_retired_this_round_.insert(candidate);
      observe_retry_active_id_ = candidate;
      // 派发前先请感知重开该目标（E3 回路修正）：残局目标已完成于感知
      // 计划（completed_ids），不重开则 selected 恒空、重建不绑定不精化，
      // observe 周期只能走 waitForRefined 超时的降级锚点路径，抬质量落空。
      // 重开后感知下帧按优先级重选该目标 → 重建绑定精化 → observe 周期
      // 拿到真实精化数据；终局后正常派发链按新 selected 以 FULL 重试。
      // 显式批次不消费感知锁定计划，不调。
      if (!explicit_mode_) {
        request_reopen_target_locked(candidate);
      }
    } else {
      observe_retry_active_id_.clear();
    }
    if (run_id_.empty()) {
      // 防御兜底：auto 批次 run_id 正常在首个拍照前置启动时生成
      // （begin_photo_step_locked），走到这里说明被极端路径绕过。
      run_id_ = "auto-" + std::to_string(now().nanoseconds());
    }
    // cycle_id 唯一化（协议 2.5 goal 契约）：<run_id>-cycle-<批次内单调序号>，
    // 不再复用状态机 revision（revision 经 I7 防抖后可能与周期数脱钩，且
    // 跨批次会重复；唯一化后跳过的 cycle_id 精确绑定与日志/落盘对账更稳）。
    current_cycle_id_ =
      run_id_ + "-cycle-" + std::to_string(++cycle_sequence_);
    // 单目标墙钟起点（派发时刻），终局记账时折算 elapsed（2.13-E5）。
    target_dispatch_started_ = now();
    // 新周期开始：上一周期可能残留的跳过绑定作废（协议 2.9/2.6-2，
    // skip_cycle_id_ 只对本周期有效）。
    skip_cycle_id_.clear();
    RunTargetCycle::Goal goal;
    goal.request_id = current_cycle_id_;
    goal.run_id = run_id_;
    goal.cycle_id = current_cycle_id_;
    goal.target_id = candidate;
    goal.mode = observe_retry ?
      RunTargetCycle::Goal::OBSERVE_ONLY : RunTargetCycle::Goal::FULL;
    rclcpp_action::Client<RunTargetCycle>::SendGoalOptions options;
    options.goal_response_callback = [this](const TargetGoalHandle::SharedPtr & handle) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (handle) {
          // 保存活动 goal handle，供控制命令/停用时传播取消。
          active_target_goal_ = handle;
          consecutive_rejections_ = 0;
          dispatch_retries_ = 0;
          return;
        }
        const std::string target_id = machine_.snapshot().target_id;
        if (!observe_retry_active_id_.empty() && observe_retry_active_id_ == target_id) {
          // observe-retry goal 被拒（协议 2.8 残局，best-effort）：不进
          // R-1 熔断/R-2 重试预算/R-3 不可达记账、不推进感知计划——抬质量是
          // 锦上添花，拒绝多为能力端缓存仍以感知 selected 为准的竞态；次数
          // 已在派发时消耗，直接落安全检查点让位下一个残局目标/轮次判定。
          machine_.reach_safe_checkpoint();
          last_dispatched_target_.clear();
          observe_retry_active_id_.clear();
          publish_event(
            "observe_retry_rejected",
            "observe_retry 残局抬质量请求被拒，让位后续残局目标/轮次判定",
            HarvestEvent::WARNING, current_cycle_id_, target_id);
          current_cycle_id_.clear();
          publish_state();
          return;
        }
        if (!motion_ready_locked()) {
          // R3 修复（协议 2.5-R-2）：拒绝落入本端 motion 失能窗口（急停/故障/
          // 未上电/对端掉线）——属本端失能而非能力端故障，只冷却重试，不计
          // consecutive_rejections、不耗 dispatch_retries。失能期间 G6 失能门
          // 本就阻止再派发，冷却只是对齐恢复沿，不会形成忙等。
          retry_not_before_ = std::chrono::steady_clock::now() +
            std::chrono::milliseconds(
            static_cast<int64_t>(dispatch_retry_delay_s_ * 1000.0));
          machine_.reach_safe_checkpoint();
          last_dispatched_target_.clear();
          publish_event(
            "target_dispatch_retry",
            "目标请求被拒且本端 motion 失能，仅冷却等待恢复后重试（不计熔断）",
            HarvestEvent::INFO, current_cycle_id_, target_id);
          current_cycle_id_.clear();
          publish_state();
          return;
        }
        // 能力端拒绝多为瞬态竞态（上周期运行标志未清 / 能力端缓存尚未跟上
        // 新 selected）：先落安全检查点、冷却 retry_delay_s 后重派同一目标
        // （不记账、不推进感知计划）；重试耗尽才按不可达记账跳过；连续拒绝
        // （跨目标累计，接受即清零）达熔断上限才认定能力端故障进待恢复。
        ++consecutive_rejections_;
        if (consecutive_rejections_ >= max_consecutive_rejections_) {
          // 协议 2.5-R-1：连续拒绝熔断进 RECOVERY_REQUIRED（T10）。
          require_recovery_locked("单目标能力连续拒绝目标请求");
          publish_event(
            "target_rejected", "单目标能力连续拒绝目标请求，批次待人工确认",
            HarvestEvent::ERROR, current_cycle_id_, target_id);
        } else if (dispatch_retries_ < max_dispatch_retries_) {
          // 协议 2.5-R-2：清活动目标（不记账）+ 冷却 + 清派发锁允许重派。
          ++dispatch_retries_;
          retry_not_before_ = std::chrono::steady_clock::now() +
            std::chrono::milliseconds(
            static_cast<int64_t>(dispatch_retry_delay_s_ * 1000.0));
          machine_.reach_safe_checkpoint();
          // 冷却到点后重派同一 selected：清派发锁（selected 未变，
          // targets_callback 不会替我们复位）。
          last_dispatched_target_.clear();
          publish_event(
            "target_dispatch_retry",
            "目标请求被拒（多为缓存同步竞态），冷却后重试（第" +
            std::to_string(dispatch_retries_) + "次）",
            HarvestEvent::INFO, current_cycle_id_, target_id);
        } else {
          // 协议 2.5-R-3：重试耗尽按不可达记账，锁内登记推进（2.6 推进协议
          // 由 refresh 轮询发 RPC，回调链不再直接发 RPC）。
          machine_.record_target_outcome(
            target_id, TargetOutcome::SKIPPED_UNREACHABLE,
            "单目标能力持续拒绝目标请求");
          dispatch_retries_ = 0;
          if (advance_plan_locked()) {
            begin_advance_locked(current_cycle_id_, target_id);
          }
          publish_event(
            "target_rejected", "单目标能力持续拒绝目标请求，已跳过该目标",
            HarvestEvent::WARNING, current_cycle_id_, target_id);
        }
        current_cycle_id_.clear();
        publish_state();
      };
    options.feedback_callback = [this](
      TargetGoalHandle::SharedPtr,
      const std::shared_ptr<const RunTargetCycle::Feedback> feedback) {
        std::lock_guard<std::mutex> lock(mutex_);
        // 仅阶段变化时发布状态（反馈频率远高于状态订阅需求，做节流）。
        if (machine_.set_target_phase(static_cast<TargetPhase>(feedback->state.target_phase))) {
          publish_state();
        }
      };
    // cycle_id 按值捕获：RunTargetCycle.Result 不回传 cycle_id，操作员跳过的
    // 记账以捕获值与 skip_cycle_id_ 精确匹配为准（协议 2.9/2.6-2）。
    const std::string dispatched_cycle_id = current_cycle_id_;
    options.result_callback =
      [this, dispatched_cycle_id](const TargetGoalHandle::WrappedResult & result) {
        std::lock_guard<std::mutex> lock(mutex_);
        const std::string target_id = machine_.snapshot().target_id;
        // 本周期是否为残局抬质量（OBSERVE_ONLY）周期（协议 2.8，阶段 E3）：
        // 决定下方记账/推进路由——observe 终局只进审计账（不动 counters）、
        // 不推进感知计划（残局目标本就不在感知 selected 计划位上）。
        const bool observe_retry_result =
          !observe_retry_active_id_.empty() && observe_retry_active_id_ == target_id;
        const std::string reason = result.result && !result.result->reason.empty() ?
          result.result->reason : "单目标周期失败";
        active_target_goal_.reset();
        // 单目标墙钟（派发→终局，2.13-E5 埋点）；防御起点缺失时记 0。
        const double cycle_elapsed_s = target_dispatch_started_.nanoseconds() > 0 ?
          (now() - target_dispatch_started_).seconds() : 0.0;
        // 阶段耗时分解（协议 2.13-E5/2.12 时序）：能力端 Result 的
        // stage_names/stage_durations 并行数组。HarvestSummary/TargetOutcome
        // 无对应字段且消息契约冻结，按约定拼进终局事件 message（web_runs
        // 落盘后可按阶段下钻）；name 多于 duration 时该项标 "?" 防错位误读。
        std::string stage_text;
        if (result.result && !result.result->stage_names.empty()) {
          stage_text = "；阶段耗时";
          for (size_t i = 0; i < result.result->stage_names.size(); ++i) {
            char buffer[40];
            if (i < result.result->stage_durations.size()) {
              const auto & d = result.result->stage_durations[i];
              std::snprintf(
                buffer, sizeof(buffer), " %s=%.1fs",
                result.result->stage_names[i].c_str(),
                static_cast<double>(d.sec) + static_cast<double>(d.nanosec) * 1e-9);
            } else {
              std::snprintf(
                buffer, sizeof(buffer), " %s=?",
                result.result->stage_names[i].c_str());
            }
            stage_text += buffer;
          }
        }
        // 取消语义以 outcome==CANCELED 为准（能力端 2026-08 起在取消终局显式
        // 填充）；result.code 仅作旧版能力端（取消时复用 outcome=FAILED）的
        // 兼容兜底。
        const bool canceled_path =
          (result.result &&
          result.result->outcome == RunTargetCycle::Result::CANCELED) ||
          result.code == rclcpp_action::ResultCode::CANCELED;
        // 接触段故障等需人工确认的恢复：最高优先（协议 2.6 优先级链 1），
        // skip 绑定也让位；不记账、不推进。
        const bool operator_skip = canceled_path && !skip_cycle_id_.empty() &&
          skip_cycle_id_ == dispatched_cycle_id;
        if (result.result && result.result->recovery_required) {
          require_recovery_locked(reason);
          publish_event("target_failed", reason, HarvestEvent::ERROR, "", target_id);
        } else if (operator_skip) {
          // 操作员跳过（协议 2.6 链 2）：skip_cycle_id 与结果周期精确匹配才
          // 记账为 CANCELED 并推进计划。自动批次派发锁保留到感知新 selected
          // 到达（targets_callback 按 selected 变化解锁，2.5-L4 不变式），防止
          // 计划推进传播前的窗口期把同一目标立即重派（goal 必被能力端拒绝）；
          // 显式批次由清单索引推进并直接复位派发锁。
          if (observe_retry_result) {
            // observe-retry 被操作员跳过（E3）：只进审计账（reason 标
            // observe_retry），不动 counters、不推进感知计划；清派发锁允许
            // 恢复后按当前快照重派。
            machine_.record_observe_retry_outcome(
              target_id, TargetOutcome::CANCELED,
              "observe_retry：操作员跳过", cycle_elapsed_s);
            last_dispatched_target_.clear();
            publish_event(
              "observe_retry_finished",
              "observe_retry 残局抬质量被操作员跳过" + stage_text,
              HarvestEvent::AUDIT, "", target_id);
          } else {
            machine_.record_target_outcome(
              target_id, TargetOutcome::CANCELED, "操作员跳过", cycle_elapsed_s);
            if (advance_plan_locked()) {
              begin_advance_locked(dispatched_cycle_id, target_id);
            }
            // A13 事件语义拆分：操作员主动跳过（人因）独立成码，
            // target_skipped 只保留质量/不可达的系统判定跳过语义。
            publish_event(
              "target_operator_skipped", "操作员跳过当前目标" + stage_text,
              HarvestEvent::AUDIT, "", target_id);
          }
        } else if (canceled_path) {
          // 暂停/立即取消路径（协议 2.6 链 3）：安全检查点落地，不记账；
          // 清派发锁/重试预算，恢复后按当前 selected 重派。
          machine_.reach_safe_checkpoint();
          last_dispatched_target_.clear();
          dispatch_retries_ = 0;
          retry_not_before_ = std::chrono::steady_clock::time_point{};
          publish_event(
            "target_canceled", "单目标周期已取消", HarvestEvent::WARNING, "", target_id);
        } else {
          // 正常终态按 outcome 路由（协议 2.6 链 4），全部自动推进感知计划。
          const auto outcome = result.result ? result.result->outcome :
            RunTargetCycle::Result::FAILED;
          TargetOutcome recorded = TargetOutcome::FAILED;
          std::string event_code = "target_failed";
          uint8_t severity = HarvestEvent::ERROR;
          switch (outcome) {
            case RunTargetCycle::Result::SUCCEEDED:
              recorded = TargetOutcome::SUCCEEDED;
              event_code = "target_succeeded";
              severity = HarvestEvent::INFO;
              break;
            case RunTargetCycle::Result::SKIPPED_QUALITY:
              recorded = TargetOutcome::SKIPPED_QUALITY;
              event_code = "target_skipped";
              severity = HarvestEvent::AUDIT;
              break;
            case RunTargetCycle::Result::SKIPPED_UNREACHABLE:
              recorded = TargetOutcome::SKIPPED_UNREACHABLE;
              event_code = "target_skipped";
              severity = HarvestEvent::AUDIT;
              break;
            default:
              break;
          }
          // quality_score 占位语义（待阶段 E 接入能力端真实质量分/refined
          // rmse——现 Result 无 rmse/降级字段）：成功=1.0；reason 含降级标记
          // （候选锚点降级抓取链）=0.5；其余终局=0.0。
          float quality_score = 0.0f;
          if (recorded == TargetOutcome::SUCCEEDED) {
            quality_score = reason.find("降级") != std::string::npos ? 0.5f : 1.0f;
          }
          if (observe_retry_result) {
            // observe-retry 终局（协议 2.8，E3）：reason 标 observe_retry 进
            // 审计账（不动 counters，不重复计为采摘成功/失败）；不推进感知
            // 计划；清派发锁——抬质量成功使感知重新选中该目标时，正常派发链
            // 下一拍即可以 FULL 重试（2.5-L4-(a) 的"锁保持到 selected 变化"
            // 不变式对 observe 不适用：observe 无计划推进，锁保留反而永远
            // 挡住重新可选后的重派）。
            machine_.record_observe_retry_outcome(
              target_id, recorded, "observe_retry：" + reason, cycle_elapsed_s);
            last_dispatched_target_.clear();
            publish_event(
              "observe_retry_finished",
              "observe_retry 残局抬质量终局：" + reason + stage_text,
              severity, "", target_id);
          } else {
            machine_.record_target_outcome(
              target_id, recorded, reason, cycle_elapsed_s, quality_score);
            if (advance_plan_locked()) {
              begin_advance_locked(dispatched_cycle_id, target_id);
            }
            // 自动批次派发锁不在此复位：保留到感知新 selected 到达
            // （targets_callback 按 selected 变化解锁，2.5-L4-(a)），防止计划
            // 推进传播前的窗口期把刚终局的目标立即重派（能力端缓存已失效，goal
            // 必被拒绝并误入 RECOVERY_REQUIRED）；显式批次由 advance_plan_locked
            // 复位。
            publish_event(event_code, reason + stage_text, severity, "", target_id);
          }
        }
        skip_cycle_id_.clear();
        observe_retry_active_id_.clear();
        current_cycle_id_.clear();
        publish_state();
      };
    target_cycle_client_->async_send_goal(goal, options);
    if (observe_retry) {
      // E3：抬质量派发独立事件码（message 标 observe_retry），web 过程线
      // 与落盘可区分正常派发与残局抬质量。
      publish_event(
        "observe_retry_dispatched",
        "observe_retry 残局抬质量派发（本批次第 " +
        std::to_string(observe_retry_counts_[candidate]) + "/" +
        std::to_string(observe_retry_max_) + " 次）：" + candidate,
        HarvestEvent::INFO, goal.request_id, candidate);
    } else {
      publish_event(
        "target_dispatched", candidate, HarvestEvent::INFO,
        goal.request_id, candidate);
    }
  }

  // E3 残局抬质量回路：OBSERVE_ONLY 派发前请感知重开该目标，移出
  // completed_ids 恢复可选（下一帧按固定优先级重新选中 → 重建绑定精化，
  // observe 周期拿真实精化数据；终局后正常派发链按新 selected 以 FULL
  // 重试）。best-effort fire-and-forget：服务不可用/被拒/异常只发
  // 审计/警告事件，不进熔断与重试预算（抬质量本是增值路径，失败不阻断
  // 批次收口）。调用时必须已持有 mutex_（只读客户端与目标 ID，响应回调
  // 在 client_callback_group_ 线程自行取锁发事件）。
  void request_reopen_target_locked(const std::string & target_id)
  {
    if (!reopen_target_client_->service_is_ready()) {
      publish_event(
        "observe_retry_reopen_failed",
        "reopen_target 服务不可用，目标暂不恢复可选",
        HarvestEvent::WARNING, "", target_id);
      return;
    }
    auto request = std::make_shared<peach_pose_msgs::srv::ReopenTarget::Request>();
    request->target_id = target_id;
    reopen_target_client_->async_send_request(
      request,
      [this, target_id](
        rclcpp::Client<peach_pose_msgs::srv::ReopenTarget>::SharedFuture future) {
        std::lock_guard<std::mutex> lock(mutex_);
        try {
          const auto response = future.get();
          if (!response->success) {
            publish_event(
              "observe_retry_reopen_failed",
              "reopen_target 被拒：" + response->message,
              HarvestEvent::WARNING, "", target_id);
            return;
          }
          publish_event(
            "target_reopened", "残局目标已重开恢复可选",
            HarvestEvent::INFO, "", target_id);
        } catch (const std::exception & exc) {
          publish_event(
            "observe_retry_reopen_failed",
            std::string("reopen_target 调用异常：") + exc.what(),
            HarvestEvent::WARNING, "", target_id);
        }
      });
  }

  // 单目标终局后的计划推进（调用时必须已持有 mutex_）：
  // 显式清单批次直接推进清单索引并复位派发锁（不调 complete_selected_target，
  // 感知锁定计划不被显式批次消费），返回 false；
  // 自动批次返回 true，由调用方在锁外 RPC 推进感知固定优先级计划。
  bool advance_plan_locked()
  {
    if (!explicit_mode_) {return true;}
    if (explicit_index_ < explicit_queue_.size()) {++explicit_index_;}
    last_dispatched_target_.clear();
    return false;
  }

  // 感知推进协议（2.6）节点侧状态机：终局记账后把 selected 从感知固定优先级
  // 计划移除。与拍照前置同一模式——RPC 只由 refresh 以 wait_for(0) 轮询推进，
  // 任何回调内绝不持锁等 RPC。
  //
  // 失败处理（2.6）：服务不可用/超时/被拒 success=false 均按失败计 → 冷却
  // advance.retry_delay_s（默认 1s）后重试，每次失败发 ERROR 事件（事件带
  // 周期/目标上下文）；连续失败 advance.max_retries（默认 5）→
  // require_recovery("感知推进持续失败")（T10）。推进成功 → 感知下一帧
  // selected 变化经 targets_callback 清派发锁（2.5-L4-(a)），闭环。
  // 不变式（2.6）：任何路径下"记账"与"推进"要么都完成，要么进
  // RECOVERY_REQUIRED——不存在悬挂态。与派发锁不变式的配合：推进重试期间
  // last_dispatched_target_ 保持不清（终局记账路径均不清锁），G3 去重锁
  // 保证不会把刚终局的目标误重派。
  enum class AdvanceStep : uint8_t
  {
    IDLE,      // 无待推进
    COOLDOWN,  // 待发起或失败冷却中（到点由 refresh 发起/重发 RPC）
    CALLING    // complete_selected_target 调用在途
  };

  // 登记一次推进（调用时必须已持有 mutex_）：cycle_id/target_id 仅作后续
  // 失败事件的上下文。新一次登记会重置失败计数（新推进episode重新计）。
  void begin_advance_locked(
    const std::string & cycle_id, const std::string & target_id)
  {
    advance_step_ = AdvanceStep::COOLDOWN;
    advance_retry_not_before_ = now();
    advance_failures_ = 0;
    advance_cycle_id_ = cycle_id;
    advance_target_id_ = target_id;
  }

  // 推进失败处理：冷却重试或熔断进待恢复（调用时必须已持有 mutex_）。
  void advance_failed_locked(const std::string & reason)
  {
    ++advance_failures_;
    if (advance_failures_ >= advance_max_retries_) {
      // 熔断（2.6）：require_recovery_locked 会顺带清推进/派发锁状态
      // （2.5-L4 清除点 (c)），推进episode到此终结。
      require_recovery_locked("感知推进持续失败: " + reason);
      publish_event(
        "advance_failed", "感知推进持续失败: " + reason,
        HarvestEvent::ERROR, advance_cycle_id_, advance_target_id_);
      return;
    }
    advance_retry_not_before_ =
      now() + rclcpp::Duration::from_seconds(advance_retry_delay_s_);
    advance_step_ = AdvanceStep::COOLDOWN;
    publish_event(
      "advance_retry",
      reason + "（第 " + std::to_string(advance_failures_) + "/" +
      std::to_string(advance_max_retries_) + " 次推进失败，冷却后重试）",
      HarvestEvent::ERROR, advance_cycle_id_, advance_target_id_);
  }

  // 轮询/推进感知推进状态机（refresh 每拍调用，调用时必须已持有 mutex_）。
  void poll_advance_locked()
  {
    switch (advance_step_) {
      case AdvanceStep::COOLDOWN:
        if (now() < advance_retry_not_before_) {return;}
        if (!complete_target_client_->service_is_ready()) {
          advance_failed_locked("complete_selected_target 服务不可用");
          return;
        }
        advance_future_ = complete_target_client_->async_send_request(
          std::make_shared<Trigger::Request>()).future.share();
        advance_call_started_ = now();
        advance_step_ = AdvanceStep::CALLING;
        return;
      case AdvanceStep::CALLING:
        if (advance_future_.wait_for(0s) != std::future_status::ready) {
          if ((now() - advance_call_started_).seconds() > advance_timeout_s_) {
            advance_future_ = TriggerFuture();
            advance_failed_locked("complete_selected_target 调用超时");
          }
          return;
        }
        {
          const auto response = advance_future_.get();
          advance_future_ = TriggerFuture();
          if (!response->success) {
            advance_failed_locked(
              "complete_selected_target 被拒绝: " + response->message);
            return;
          }
          // 推进成功：清零失败计数，闭环等感知下一帧 selected 变化。
          advance_step_ = AdvanceStep::IDLE;
          advance_failures_ = 0;
        }
        return;
      case AdvanceStep::IDLE:
        return;
    }
  }

  // 派发锁清除集合（协议 2.5-L4）：last_dispatched_target_（去重锁）+
  // consecutive_rejections_（熔断计数）+ dispatch_retries_/retry_not_before_
  // （冷却重试预算）+ 推进重试状态（2.6 新增）。清除点 (a)/(b) 有各自的
  // 精细语义（见 targets_callback 与 goal_response_callback），本集合用于
  // (c) require_recovery / (d) reset_batch / (e) ACKNOWLEDGE_RECOVERY 三处。
  // 调用时必须已持有 mutex_。
  void clear_dispatch_locks_locked()
  {
    last_dispatched_target_.clear();
    consecutive_rejections_ = 0;
    dispatch_retries_ = 0;
    retry_not_before_ = std::chrono::steady_clock::time_point{};
    advance_step_ = AdvanceStep::IDLE;
    advance_future_ = TriggerFuture();
    advance_failures_ = 0;
  }

  // 节点侧 require_recovery 包装（调用时必须已持有 mutex_）：除状态机转移
  // （T10）外补齐 2.5-L4 清除点 (c)——进 RECOVERY_REQUIRED 时清派发锁与
  // 重试/推进状态，ACK+RESUME 后按全新快照派发，杜绝 R1 类死锁（恢复后
  // 能力端自身的 recovery 锁/缓存校验仍是重派同一故障目标的最后防线）。
  void require_recovery_locked(const std::string & reason)
  {
    machine_.require_recovery(reason);
    clear_dispatch_locks_locked();
  }

  // 是否需要机械臂移动到拍照位姿：execution 关或 photo_pose 关时不移动。
  // 调用时必须已持有 mutex_。
  bool photo_motion_required_locked() const
  {
    return machine_.snapshot().policy.execution_enabled && photo_pose_enabled_;
  }

  // 批次启动就位自校门（阶段 F3，harvest.preflight_check；调用时必须已持有
  // mutex_）：批次首轮进入拍照前置链之前，由 refresh 每拍调用本函数判定
  // "静态几何就位"。与四路就绪门互补不重复——就绪门管"话题心跳新鲜度"，
  // 本门管"TF 外参链 + robot_status 无故障"（检查项与 joint_states 取舍见
  // 纯核 PreflightFacts 注释）。
  // 语义：幂等门而非一次性熔断——失败只挂起拍照前置（本函数返回 false，
  // photo_step_ 保持 NOT_STARTED），发 ERROR 事件（列清失败项，签名变化或
  // 每 5s 节流重发）并把 "preflight" 投影进 state blockers；下拍 refresh
  // 自动重试，连续失败不熔断（环境可能稍后就位）；通过后置
  // preflight_passed_ 放行本批次（RESHAPE/RESCAN 轮次 round_>1 直接放行，
  // 不重复自校）。开关关闭/显式清单批次（photo_step_ 直接 DONE，不走本门）
  // 直通为旧行为。返回 true=允许进入/继续拍照前置。
  bool preflight_gate_passed_locked()
  {
    if (!preflight_check_enabled_ || round_ > 1 || preflight_passed_) {
      preflight_blocked_ = false;
      return true;
    }
    // 事实采集：TF 查静态链（TimePointZero 纯缓冲查找，非阻塞）；robot 两
    // 项复用 robot 就绪路缓存（robot_callback 维护的 robot_status_/
    // robot_received_），robot 检查跟随 readiness.require_robot_status 热读。
    PreflightFacts facts;
    facts.tf_extrinsics_ready = tf_buffer_ != nullptr &&
      tf_buffer_->canTransform(
      kPreflightBaseFrame, kPreflightCameraFrame, tf2::TimePointZero);
    facts.robot_check_required =
      param_listener_->get_params().readiness.require_robot_status;
    facts.robot_status_received = robot_received_.nanoseconds() > 0;
    facts.robot_fault = robot_status_.e_stopped != 0 || robot_status_.in_error != 0;
    const auto failures = evaluate_preflight(facts);
    if (failures.empty()) {
      // 曾在失败态的恢复沿与首次通过分别行文，便于 web/落盘区分"启动即
      // 就位"与"环境稍后就位"。
      const bool recovered = preflight_blocked_;
      preflight_blocked_ = false;
      preflight_passed_ = true;
      preflight_failure_signature_.clear();
      publish_event(
        "preflight_passed",
        recovered ?
        "批次就位自校恢复通过（TF 外参链与 robot_status 就绪），进入拍照前置" :
        "批次就位自校通过（TF 外参链与 robot_status 就绪），进入拍照前置",
        HarvestEvent::INFO);
      return true;
    }
    // 失败项标签 → 中文描述（事件列清失败项，定位直指外参链/机器人状态）。
    std::string detail;
    for (const auto & item : failures) {
      if (!detail.empty()) {detail += "；";}
      if (item == "tf_extrinsics") {
        detail += "TF 外参链 base_link→camera_link 不可查"
          "（extrinsics_publisher/robot_state_publisher 未就绪）";
      } else if (item == "robot_status_missing") {
        detail += "robot_status 尚未收到";
      } else {
        detail += "robot_status 存在故障/急停标志";
      }
    }
    // 事件节流：失败项签名变化立即发，否则每 5s 重发一次提醒（自校门由
    // refresh 每 500ms 重评，不节流会刷爆事件流）。
    const bool signature_changed = detail != preflight_failure_signature_;
    const bool remind_due = preflight_last_failure_event_.nanoseconds() == 0 ||
      (now() - preflight_last_failure_event_).seconds() >= 5.0;
    if (signature_changed || remind_due) {
      publish_event(
        "preflight_failed",
        "批次就位自校未通过（" + detail + "），拍照前置挂起，下拍自动重试",
        HarvestEvent::ERROR);
      preflight_last_failure_event_ = now();
    }
    preflight_failure_signature_ = detail;
    preflight_blocked_ = true;
    return false;
  }

  // 发起本轮拍照前置；调用时必须已持有 mutex_。
  void begin_photo_step_locked()
  {
    // auto_start 自动批次（无 RunHarvest goal）的批次起点锚在这里：首个
    // PhotoStep 启动时刻即批次开始（协议 2.2-T1 动作的节点侧落地）——
    // 生成唯一 run_id（auto-<纳秒时间戳>，多批次/重启不重名）并开启
    // summary.elapsed 计时；RunHarvest 批次的 run_id/batch_started_ 已在
    // accepted_callback 由 goal 设置，此处不会覆盖。
    if (run_id_.empty()) {run_id_ = "auto-" + std::to_string(now().nanoseconds());}
    if (batch_started_.nanoseconds() == 0) {batch_started_ = now();}
    photo_retries_ = 0;
    photo_retry_not_before_ = now();
    if (photo_motion_required_locked()) {
      photo_step_ = PhotoStep::MOVE_PENDING;
    } else if (round_ > 1) {
      // 复扫轮次即使无需移动机械臂，也必须重置感知以开启新一轮收齐。
      photo_step_ = PhotoStep::RESET_PENDING;
    } else {
      // 首轮且无需移动：不打扰感知，维持原行为（拍照前置语义直接满足）。
      photo_step_ = PhotoStep::DONE;
      round_start_attempted_ = machine_.snapshot().counters.attempted;
      publish_event(
        "round_started",
        "第" + std::to_string(round_) + "轮开始（拍照前置已按策略跳过）",
        HarvestEvent::INFO);
    }
  }

  // 拍照前置失败处理：冷却重试；连续 max_retries 次失败进入待恢复。
  // 调用时必须已持有 mutex_；retry_state 为冷却结束后回到的待发起阶段。
  void photo_step_failed_locked(const std::string & reason, PhotoStep retry_state)
  {
    ++photo_retries_;
    if (photo_retries_ >= photo_max_retries_) {
      require_recovery_locked("全局拍照位姿移动失败: " + reason);
      // 恢复确认并 RESUME 后从 NOT_STARTED 重新走完整拍照流程。
      photo_step_ = PhotoStep::NOT_STARTED;
      photo_retries_ = 0;
      publish_event(
        "recovery_required", "全局拍照位姿移动失败: " + reason, HarvestEvent::ERROR);
      return;
    }
    photo_retry_not_before_ =
      now() + rclcpp::Duration::from_seconds(photo_retry_cooldown_s_);
    photo_step_ = retry_state;
    publish_event(
      "photo_step_retry",
      reason + "（第 " + std::to_string(photo_retries_) + "/" +
      std::to_string(photo_max_retries_) + " 次失败，冷却后重试）",
      HarvestEvent::WARNING);
  }

  // 轮询/推进拍照前置状态机；future 只 poll 不等待（wait_for(0)），
  // 调用时必须已持有 mutex_，锁内绝不阻塞等 RPC。
  void poll_photo_step_locked()
  {
    switch (photo_step_) {
      case PhotoStep::MOVE_PENDING:
        photo_begin_move_locked();
        return;
      case PhotoStep::MOVING:
        photo_poll_move_locked();
        return;
      case PhotoStep::CLEAR_PENDING:
        photo_begin_clear_locked();
        return;
      case PhotoStep::CLEARING:
        photo_poll_clear_locked();
        return;
      case PhotoStep::RESET_PENDING:
        photo_begin_reset_locked();
        return;
      case PhotoStep::RESETTING:
        photo_poll_reset_locked();
        return;
      case PhotoStep::NOT_STARTED:
      case PhotoStep::DONE:
        return;
    }
  }

  // MOVE_PENDING：冷却结束后发起 go_to_photo_pose 异步调用。
  void photo_begin_move_locked()
  {
    if (now() < photo_retry_not_before_) {return;}
    if (!photo_pose_client_->service_is_ready()) {
      photo_step_failed_locked("go_to_photo_pose 服务不可用", PhotoStep::MOVE_PENDING);
      return;
    }
    photo_future_ = photo_pose_client_->async_send_request(
      std::make_shared<Trigger::Request>()).future.share();
    photo_call_started_ = now();
    photo_step_ = PhotoStep::MOVING;
  }

  // MOVING：轮询 go_to_photo_pose 结果；成功则转入重置感知阶段。
  void photo_poll_move_locked()
  {
    if (photo_future_.wait_for(0s) != std::future_status::ready) {
      if ((now() - photo_call_started_).seconds() > photo_service_timeout_s_) {
        photo_future_ = TriggerFuture();
        photo_step_failed_locked("go_to_photo_pose 调用超时", PhotoStep::MOVE_PENDING);
      }
      return;
    }
    const auto response = photo_future_.get();
    photo_future_ = TriggerFuture();
    if (!response->success) {
      photo_step_failed_locked(
        "go_to_photo_pose 被拒绝: " + response->message, PhotoStep::MOVE_PENDING);
      return;
    }
    publish_event("photo_pose_reached", response->message, HarvestEvent::INFO);
    // fresh_scene（协议 2.3，阶段 D2）：换场景批次在 reset_global_targets
    // 之前先清感知身份记忆，仅首轮生效——复扫轮次同场景，清记忆会破坏
    // 跨视角锚点恢复（target_memory.recovery_scale 兜底机制）。
    photo_step_ = (fresh_scene_ && round_ == 1) ?
      PhotoStep::CLEAR_PENDING : PhotoStep::RESET_PENDING;
  }

  // CLEAR_PENDING：冷却结束后发起 clear_target_memory 异步调用。
  void photo_begin_clear_locked()
  {
    if (now() < photo_retry_not_before_) {return;}
    if (!clear_memory_client_->service_is_ready()) {
      photo_step_failed_locked(
        "clear_target_memory 服务不可用", PhotoStep::CLEAR_PENDING);
      return;
    }
    clear_memory_future_ = clear_memory_client_->async_send_request(
      std::make_shared<Trigger::Request>()).future.share();
    photo_call_started_ = now();
    photo_step_ = PhotoStep::CLEARING;
  }

  // CLEARING：轮询清记忆结果；成功则转入重置感知阶段，失败与 reset 失败
  // 同级——冷却重试计入 photo_step 失败链（photo_retries_ 熔断进待恢复）。
  void photo_poll_clear_locked()
  {
    if (clear_memory_future_.wait_for(0s) != std::future_status::ready) {
      if ((now() - photo_call_started_).seconds() > photo_service_timeout_s_) {
        clear_memory_future_ = TriggerFuture();
        photo_step_failed_locked(
          "clear_target_memory 调用超时", PhotoStep::CLEAR_PENDING);
      }
      return;
    }
    const auto response = clear_memory_future_.get();
    clear_memory_future_ = TriggerFuture();
    if (!response->success) {
      photo_step_failed_locked(
        "clear_target_memory 被拒绝: " + response->message, PhotoStep::CLEAR_PENDING);
      return;
    }
    publish_event(
      "target_memory_cleared",
      "新场景批次：感知身份记忆已清空（" + response->message + "）",
      HarvestEvent::INFO);
    photo_step_ = PhotoStep::RESET_PENDING;
  }

  // RESET_PENDING：发起 reset_global_targets 异步调用。
  void photo_begin_reset_locked()
  {
    if (now() < photo_retry_not_before_) {return;}
    if (!reset_targets_client_->service_is_ready()) {
      photo_step_failed_locked(
        "reset_global_targets 服务不可用", PhotoStep::RESET_PENDING);
      return;
    }
    reset_future_ = reset_targets_client_->async_send_request(
      std::make_shared<Trigger::Request>()).future.share();
    photo_call_started_ = now();
    photo_step_ = PhotoStep::RESETTING;
  }

  // RESETTING：轮询重置结果；成功则本轮拍照前置完成，进入等待收齐锁定。
  void photo_poll_reset_locked()
  {
    if (reset_future_.wait_for(0s) != std::future_status::ready) {
      if ((now() - photo_call_started_).seconds() > photo_service_timeout_s_) {
        reset_future_ = TriggerFuture();
        photo_step_failed_locked(
          "reset_global_targets 调用超时", PhotoStep::RESET_PENDING);
      }
      return;
    }
    const auto response = reset_future_.get();
    reset_future_ = TriggerFuture();
    if (!response->success) {
      photo_step_failed_locked(
        "reset_global_targets 被拒绝: " + response->message, PhotoStep::RESET_PENDING);
      return;
    }
    // 首轮重置推翻批次开始前的旧锁定集合，发现数从本轮重新累计；
    // 复扫轮次的重置保留历史轮次累计（锁定沿继续累加）。
    if (round_ == 1) {
      discovered_total_ = 0;
    }
    round_start_attempted_ = machine_.snapshot().counters.attempted;
    photo_step_ = PhotoStep::DONE;
    photo_retries_ = 0;
    publish_event(
      "round_started",
      "第" + std::to_string(round_) + "轮开始：" + response->message,
      HarvestEvent::INFO);
  }

  // 对端 lifecycle Active 异步轮询（refresh 每拍调用，持 mutex_）：
  // 服务不在线 → 立即记非 Active；有在途轮询 → 等回调，超 2s 放弃并记非
  // Active（迟到的旧响应随后到达时只会多清一次 pending 标志，幂等无害）；
  // 否则发起新一轮 GetState，响应回调（组D，不进本锁时已完成 RPC）只写结果。
  void poll_grasp_lifecycle_locked()
  {
    if (!grasp_state_client_) {
      grasp_node_active_ = false;
      return;
    }
    if (!grasp_state_client_->service_is_ready()) {
      grasp_node_active_ = false;
      grasp_state_poll_pending_ = false;
      return;
    }
    if (grasp_state_poll_pending_) {
      if ((now() - grasp_state_poll_started_).seconds() > 2.0) {
        grasp_state_poll_pending_ = false;
        grasp_node_active_ = false;
      }
      return;
    }
    grasp_state_poll_pending_ = true;
    grasp_state_poll_started_ = now();
    grasp_state_client_->async_send_request(
      std::make_shared<lifecycle_msgs::srv::GetState::Request>(),
      [this](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future) {
        std::lock_guard<std::mutex> lock(mutex_);
        grasp_state_poll_pending_ = false;
        try {
          grasp_node_active_ = future.get()->current_state.id ==
          lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
        } catch (const std::exception & error) {
          grasp_node_active_ = false;
          RCLCPP_WARN(
            get_logger(), "对端 lifecycle GetState 调用失败: %s", error.what());
        }
      });
  }

  // 应用轮次判定结果（2.8 复扫与收口；2.3 收齐超时重判共用）：RESCAN 开
  // 新一轮（轮次+1、清停滞/开窗计时、拍照前置复位）；COMPLETE 完成批次并
  // best-effort 回位。WAIT 不动作。rescan_message 为 RESCAN 时 round_completed
  // 事件文本（两条调用路径语境不同）。调用时必须已持有 mutex_。
  void apply_round_verdict_locked(
    const RoundVerdict & verdict, const std::string & rescan_message)
  {
    if (verdict.decision == RoundDecision::RESCAN) {
      publish_event("round_completed", rescan_message, HarvestEvent::INFO);
      ++round_;
      selected_empty_since_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
      collect_since_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
      // 残局抬质量（E3）：新一轮恢复"每轮每目标至多一次"的去重册；
      // 批次级次数预算 observe_retry_counts_ 跨轮保留（每目标每批次上限）。
      observe_retry_retired_this_round_.clear();
      // 新一轮重新走拍照前置（回拍照位姿 + 重置感知收齐锁定）。
      photo_step_ = PhotoStep::NOT_STARTED;
    } else if (verdict.decision == RoundDecision::COMPLETE) {
      if (machine_.complete_batch(verdict.message)) {
        // batch_completed 单点发送：RunHarvest 在线时由 execute 线程随终局结果
        // 发送；仅无活动 RunHarvest（auto_start 自动批次）时才由 refresh 补发。
        // 吞吐（2.13-E5）无消息字段可放，拼进终局事件 message。
        if (!active_run_goal_) {
          publish_event(
            "batch_completed", verdict.message + batch_throughput_text_locked(),
            HarvestEvent::INFO);
        }
        // 批次完成后回一次拍照位姿（best-effort）：结果仅记事件，不影响批次结论。
        if (photo_return_on_complete_ && photo_motion_required_locked() &&
          photo_pose_client_->service_is_ready())
        {
          return_future_ = photo_pose_client_->async_send_request(
            std::make_shared<Trigger::Request>()).future.share();
          return_call_started_ = now();
          return_call_pending_ = true;
        }
      }
    }
  }

  void refresh()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // 对端 lifecycle Active 异步轮询（A8）：结果供下方 motion 就绪与派发守卫使用。
    poll_grasp_lifecycle_locked();
    // readiness.* 运行期热生效：每 tick 从监听器快照直读（其余参数保持
    // on_configure 缓存语义，不参与热更新）。
    const auto readiness = param_listener_->get_params().readiness;
    const double timeout = readiness.timeout_s;
    const bool require_robot = readiness.require_robot_status;
    // 节点把 ROS 时间/消息字段折算成纯值样本，四路推算在状态机库纯核完成；
    // 各路实测发布周期 EMA 一并传入（2.11 自适应新鲜度，0=无实测回退下限）。
    ReadinessSample readiness_sample;
    readiness_sample.now_s = now().seconds();
    readiness_sample.targets_received_s = targets_received_.seconds();
    readiness_sample.reconstruction_received_s = reconstruction_received_.seconds();
    readiness_sample.robot_received_s = robot_received_.seconds();
    readiness_sample.targets_period_s = targets_period_.period_ema();
    readiness_sample.reconstruction_period_s = recon_period_.period_ema();
    readiness_sample.robot_period_s = robot_period_.period_ema();
    readiness_sample.robot_e_stopped = robot_status_.e_stopped != 0;
    readiness_sample.robot_in_error = robot_status_.in_error != 0;
    readiness_sample.robot_drives_powered = robot_status_.drives_powered != 0;
    readiness_sample.robot_motion_possible = robot_status_.motion_possible != 0;
    readiness_sample.action_server_ready =
      target_cycle_client_->action_server_is_ready() && grasp_node_active_;
    readiness_sample.web_ready = readiness.web;
    machine_.update_readiness(
      ReadinessTracker(timeout, require_robot).evaluate(readiness_sample));
    const auto & snap = machine_.snapshot();
    const bool harvesting = snap.batch_state == BatchState::DISCOVERY ||
      snap.batch_state == BatchState::RUNNING;

    // 拍照前置：进入采摘态且无活动目标时发起并轮询推进（在途调用遇暂停/取消
    // 则冻结在当前阶段，恢复采摘态后继续轮询）。发起前先过批次就位自校门
    // （F3）：自校未通过时 photo_step_ 保持 NOT_STARTED，下拍重试。
    if (harvesting && !snap.target_active && photo_step_ == PhotoStep::NOT_STARTED &&
      preflight_gate_passed_locked())
    {
      begin_photo_step_locked();
    }
    if (harvesting && !snap.target_active) {
      poll_photo_step_locked();
    }

    // 感知推进协议（2.6）轮询：终局记账/拒绝耗尽/操作员跳过登记的推进在
    // 此发 RPC 与失败重试；不挂在 harvesting 门上——暂停/取消窗口内也允许
    // 把已记账目标的推进收尾（2.6 不变式：记账与推进不悬挂）。
    poll_advance_locked();

    // 收齐窗口编排侧总超时（协议 2.3 监督）：locked=false 且 PhotoStep=DONE
    // 起计时；超 max(collect_timeout_s 配置下限, 1.5×实测上次窗口时长) 发
    // round_stall WARNING 事件并按"本轮处理完"重判 decide_round（RESCAN 或
    // COMPLETE，不无限等感知锁定）。自适应余量：感知没有把窗口计划时长透出到
    // 本节点的实测通道，故用本节点实测的上次开窗→锁定墙钟作基准（I4）；
    // 首轮无实测（last_collect_window_s_=0）时即纯配置值。
    if (!explicit_mode_ && harvesting && !snap.target_active &&
      photo_step_ == PhotoStep::DONE && !targets_locked_)
    {
      if (collect_since_.nanoseconds() == 0) {collect_since_ = now();}
      const double collect_wait_s = (now() - collect_since_).seconds();
      const double collect_limit_s =
        std::max(collect_timeout_s_, 1.5 * last_collect_window_s_);
      if (collect_wait_s >= collect_limit_s) {
        publish_event(
          "round_stall",
          "第" + std::to_string(round_) + "轮收齐窗口超时（等待 " +
          std::to_string(static_cast<int>(collect_wait_s)) + "s ≥ 上限 " +
          std::to_string(static_cast<int>(collect_limit_s)) +
          "s），按本轮处理完重判轮次",
          HarvestEvent::WARNING);
        // "本轮处理完"重判：locked 视为 true 且 processed=target_count。
        // target_count 用最近一次锁定数（首轮恒 0 → COMPLETE"本轮未锁定到
        // 目标"；复扫轮沿用上一轮计数 → 允许 RESCAN 给感知再一次机会，
        // 轮次上限保证有界收口）。
        apply_round_verdict_locked(
          decide_round(
            true, last_target_count_, last_target_count_, round_, max_rounds_,
            rescan_until_empty_),
          "第" + std::to_string(round_) + "轮收齐超时，回拍照位姿开启复扫");
      }
    } else if (collect_since_.nanoseconds() != 0) {
      // 离开收齐等待（锁定/新轮次/暂停/活动目标）一律重置开窗计时。
      collect_since_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
    }

    // 复扫递减集循环（仅自动批次；显式清单批次一轮即止，不走复扫判定）：
    // 本轮锁定集合全部处理完（或锁定空集）后，决定再扫一轮或完成批次；
    // 等待收齐锁定期间（locked=false）不判定，保持 DISCOVERY。
    if (!explicit_mode_ && harvesting && !snap.target_active &&
      photo_step_ == PhotoStep::DONE &&
      targets_locked_ && selected_target_id_.empty())
    {
      // 残局抬质量（协议 2.8，E3）与停滞计时的协同：还有可抬质量的残局目标
      // 时本轮未结束——不累计停滞、不做轮次判定（"有目标在处理"），
      // OBSERVE_ONLY 派发由本拍末尾的 dispatch_target_locked 完成；
      // observe-retry 全部耗尽（次数到顶/本轮已抬/目标掉出锁定集/开关关闭）
      // 仍无可选目标后，才启动停滞计时与 RESCAN 判断。
      if (!pick_observe_retry_candidate_locked().empty()) {
        selected_empty_since_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
      } else {
        // 无可选目标停滞计时：selected 持续为空（剩余目标 WAITING_QUALITY 在
        // 当前视角迟迟不恢复）超过停滞上限时按本轮已收口处理，复扫回拍照位姿
        // 重新观测，避免批次在残局视角永久空等。
        if (selected_empty_since_.nanoseconds() == 0) {selected_empty_since_ = now();}
        const double stalled_s = (now() - selected_empty_since_).seconds();
        const uint32_t processed = snap.counters.attempted > round_start_attempted_ ?
          snap.counters.attempted - round_start_attempted_ : 0;
        auto verdict = decide_round(
          true, last_target_count_, processed, round_, max_rounds_,
          rescan_until_empty_);
        if (verdict.decision == RoundDecision::WAIT &&
          stalled_s >= harvest_stall_timeout_s_)
        {
          publish_event(
            "round_stall",
            "第" + std::to_string(round_) + "轮剩余目标持续不可选 " +
            std::to_string(static_cast<int>(stalled_s)) + "s，本轮提前收口",
            HarvestEvent::WARNING);
          verdict = decide_round(
            true, last_target_count_, last_target_count_, round_, max_rounds_,
            rescan_until_empty_);
        }
        apply_round_verdict_locked(
          verdict,
          "第" + std::to_string(round_) + "轮完成：本轮锁定 " +
          std::to_string(last_target_count_) + " 个目标，回拍照位姿开启复扫");
      }
    } else if (!selected_target_id_.empty()) {
      // 可选目标恢复（重新被观测确认）：停滞计时清零。
      selected_empty_since_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
    }

    // 显式清单批次终止：清单耗尽（含全部记账完成）即完成批次，一轮即止；
    // 不回拍照位姿（显式批次从未离开当前位姿），也不消费感知计划。
    // 残局抬质量门（E3）：清单内还有可抬质量的 SKIPPED_QUALITY 残局目标时
    // 先不收口，OBSERVE_ONLY 派发由下方 dispatch_target_locked 完成。
    if (explicit_mode_ && harvesting && !snap.target_active &&
      explicit_index_ >= explicit_queue_.size() &&
      pick_observe_retry_candidate_locked().empty())
    {
      const std::string done_message = "显式目标清单已处理完，批次完成";
      if (machine_.complete_batch(done_message) && !active_run_goal_) {
        // batch_completed 单点发送约定同自动批次（见复扫分支注释）；
        // 吞吐（2.13-E5）同样无消息字段可放，拼进终局事件 message。
        publish_event(
          "batch_completed", done_message + batch_throughput_text_locked(),
          HarvestEvent::INFO);
      }
    }

    // 批次完成后的回位调用轮询：与批次状态解耦，成功/失败/超时均只记事件。
    if (return_call_pending_) {
      if (return_future_.wait_for(0s) == std::future_status::ready) {
        const auto response = return_future_.get();
        return_future_ = TriggerFuture();
        return_call_pending_ = false;
        publish_event(
          response->success ? "photo_pose_returned" : "photo_pose_return_failed",
          response->message,
          response->success ? HarvestEvent::INFO : HarvestEvent::WARNING);
      } else if ((now() - return_call_started_).seconds() > photo_service_timeout_s_) {
        return_future_ = TriggerFuture();
        return_call_pending_ = false;
        publish_event(
          "photo_pose_return_failed", "回拍照位姿调用超时", HarvestEvent::WARNING);
      }
    }

    dispatch_target_locked();
    publish_state();
  }

  void publish_state()
  {
    if (state_pub_ && state_pub_->is_activated()) {
      state_pub_->publish(make_state());
    }
  }

  void publish_event(
    const std::string & code, const std::string & text,
    uint8_t severity = HarvestEvent::AUDIT,
    const std::string & request_id = std::string(),
    const std::string & target_id = std::string())
  {
    if (!event_pub_ || !event_pub_->is_activated()) {return;}
    HarvestEvent event;
    event.header.stamp = now();
    event.sequence = ++event_sequence_;
    event.severity = severity;
    event.code = code;
    event.message = text;
    event.request_id = request_id;
    event.run_id = run_id_;
    event.cycle_id = current_cycle_id_;
    event.target_id = target_id;
    event_pub_->publish(event);
  }

  void control_callback(
    const std::shared_ptr<ControlHarvest::Request> request,
    std::shared_ptr<ControlHarvest::Response> response)
  {
    TargetGoalHandle::SharedPtr goal_to_cancel;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (request->command > static_cast<uint8_t>(ControlCommand::ACKNOWLEDGE_RECOVERY)) {
        response->accepted = false;
        response->message = "未知控制命令";
        response->revision = machine_.snapshot().revision;
      } else {
        const auto command = static_cast<ControlCommand>(request->command);
        const auto result = machine_.control(
          command, request->request_id, request->expected_revision);
        response->accepted = result.accepted;
        response->message = result.message;
        response->revision = result.revision;
        if (result.accepted) {
          publish_event("control_accepted", result.message, HarvestEvent::AUDIT,
            request->request_id);
          // 取消传播：立即取消/跳过被状态机接受后，取消活动单目标 goal。
          if (command == ControlCommand::SKIP_TARGET) {
            // 跳过绑定活动周期（协议 2.9/2.6-2）：goal 取消落地后由
            // result_callback 按 cycle_id 精确匹配记账"操作员跳过"；
            // 绑定旧周期的取消结果不会误记。
            skip_cycle_id_ = current_cycle_id_;
            goal_to_cancel = active_target_goal_;
          } else if (command == ControlCommand::CANCEL_NOW) {
            goal_to_cancel = active_target_goal_;
          } else if (command == ControlCommand::ACKNOWLEDGE_RECOVERY) {
            // 2.5-L4 清除点 (e)：恢复确认只清标志保持暂停（2.9），同时清
            // 派发锁/重试预算/推进状态，RESUME 后按当前快照重新派发。
            clear_dispatch_locks_locked();
            skip_cycle_id_.clear();
          }
        }
      }
      response->state = make_state();
      publish_state();
    }
    // 取消请求放锁外发出，避免与 action 回调链形成锁耦合。
    if (goal_to_cancel) {target_cycle_client_->async_cancel_goal(goal_to_cancel);}
  }

  void policy_callback(
    const std::shared_ptr<SetOperationPolicy::Request> request,
    std::shared_ptr<SetOperationPolicy::Response> response)
  {
    const OperationPolicy requested{
      request->auto_start_enabled, request->execution_enabled,
      request->grasp_enabled, request->tool_enabled};
    uint64_t checked_revision{0};
    // 提交前旧值（2.10 回滚基准）：以编排器上次成功提交的策略为准（它是
    // 能力端三 enable 的唯一下发源，即"提交前旧值"的权威记录）。
    OperationPolicy previous;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // 第一阶段：锁内校验并记录 revision；依赖链/target_active 规则由状态机终审。
      if (request->expected_revision != machine_.snapshot().revision) {
        response->accepted = false;
        response->message = "状态版本已过期";
        response->revision = machine_.snapshot().revision;
        response->state = make_state();
        return;
      }
      if (!approach_parameters_->service_is_ready()) {
        response->accepted = false;
        response->message = "靠近抓取节点参数服务不可用";
        response->revision = machine_.snapshot().revision;
        response->state = make_state();
        return;
      }
      checked_revision = machine_.snapshot().revision;
      previous = machine_.snapshot().policy;
    }
    // 第二阶段：锁外下发参数到靠近抓取节点（≤2s 有界等待绝不持锁）。
    // rclcpp 服务响应不可延后（无 deferred-response API），本服务回调无法改
    // 纯异步链式；按回调组推荐表以独立互斥组（policy_callback_group_）隔离
    // 该等待，多线程 executor 下 refresh/订阅/control 回调照常调度。
    // 响应的到达由 client_callback_group_ 处理，与本组互不阻塞。
    auto future = approach_parameters_->set_parameters_atomically({
        rclcpp::Parameter("execution.enabled", requested.execution_enabled),
        rclcpp::Parameter("grasp.enabled", requested.grasp_enabled),
        rclcpp::Parameter("tool.enabled", requested.tool_enabled)});
    if (future.wait_for(2s) != std::future_status::ready || !future.get().successful) {
      std::lock_guard<std::mutex> lock(mutex_);
      response->accepted = false;
      response->message = "靠近抓取节点拒绝策略更新";
      response->revision = machine_.snapshot().revision;
      response->state = make_state();
      return;
    }
    // 第三阶段：复查 revision 在 RPC 期间未漂移，再提交状态机。
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (machine_.snapshot().revision == checked_revision) {
        const auto result = machine_.set_policy(
          requested, request->request_id, request->expected_revision);
        response->accepted = result.accepted;
        response->message = result.message;
        response->revision = result.revision;
        response->state = make_state();
        publish_state();
        if (result.accepted) {
          publish_event(
            "policy_updated", result.message, HarvestEvent::AUDIT, request->request_id);
        }
        return;
      }
    }
    // 2.10 阶段三回滚：revision 漂移拒提交时，锁外把能力端三 enable 回滚到
    // 提交前旧值（best-effort，沿用 set_parameters_atomically 通道，≤2s
    // 有界等待同样由独立 policy 回调组隔离）。
    auto rollback = approach_parameters_->set_parameters_atomically({
        rclcpp::Parameter("execution.enabled", previous.execution_enabled),
        rclcpp::Parameter("grasp.enabled", previous.grasp_enabled),
        rclcpp::Parameter("tool.enabled", previous.tool_enabled)});
    const bool rollback_ok =
      rollback.wait_for(2s) == std::future_status::ready && rollback.get().successful;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (rollback_ok) {
        publish_event(
          "policy_rolled_back",
          "策略下发期间状态已变化，能力端使能已回滚到提交前旧值",
          HarvestEvent::WARNING, request->request_id);
      } else {
        // 回滚失败（2.10）：能力端使能可能比编排器记录的更宽松，必须保守
        // 落地——ERROR 事件 + require_recovery，等人工核对现场使能状态。
        require_recovery_locked(
          "策略回滚失败：能力端使能状态可能与编排器不一致，请人工核对");
        publish_event(
          "policy_rollback_failed",
          "策略回滚失败：能力端使能状态可能与编排器不一致",
          HarvestEvent::ERROR, request->request_id);
      }
      response->accepted = false;
      response->message = "策略下发期间状态已变化，请刷新后重试";
      response->revision = machine_.snapshot().revision;
      response->state = make_state();
      publish_state();
    }
  }

  rclcpp_action::GoalResponse goal_callback(
    const rclcpp_action::GoalUUID &, const std::shared_ptr<const RunHarvest::Goal> goal)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto & state = machine_.snapshot();
    // 单活动批次 goal 守卫：已有 RunHarvest 在执行时拒绝新 goal。
    // 终局批次（COMPLETED/INTERRUPTED）豁免 mode!=AUTO 拒绝：CANCEL_NOW 会把
    // mode 落到 PAUSED，若不放行则 INTERRUPTED 后永远无法受理新 goal（T9 要求
    // canceled 终结后新 goal 可受理）；accepted_callback 会对终局批次做
    // reset_batch 清账重开。
    const bool terminal_batch = state.batch_state == BatchState::COMPLETED ||
      state.batch_state == BatchState::INTERRUPTED;
    if (state.recovery_required || !state.blockers.empty() || state.target_active ||
      (!terminal_batch && state.mode != OperationMode::AUTO) || active_run_goal_)
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
    // target_source 仅 AUTO_CATALOG/EXPLICIT_TARGETS 两档；显式清单为空无意义。
    if (goal->target_source > RunHarvest::Goal::EXPLICIT_TARGETS ||
      (goal->target_source == RunHarvest::Goal::EXPLICIT_TARGETS &&
      goal->target_ids.empty()))
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<RunGoalHandle>)
  {
    stop_requested_.store(true);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void accepted_callback(const std::shared_ptr<RunGoalHandle> goal_handle)
  {
    stop_requested_.store(false);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      active_run_goal_ = goal_handle;
      // 上一轮批次已完结/被中断：复位清账后重开一轮。
      const auto batch = machine_.snapshot().batch_state;
      if (batch == BatchState::COMPLETED || batch == BatchState::INTERRUPTED) {
        machine_.reset_batch();
      }
      const auto goal = goal_handle->get_goal();
      run_id_ = goal->request_id.empty() ?
        "run-" + std::to_string(now().nanoseconds()) : goal->request_id;
      batch_started_ = now();
      // 显式清单批次（EXPLICIT_TARGETS）：按给定顺序逐个派发，跳过拍照前置
      // （PhotoStep 直接 DONE，不调 go_to_photo_pose/reset_global_targets），
      // 一轮即止无复扫；发现数按清单长度播种以支撑 progress 投影。
      explicit_mode_ = goal->target_source == RunHarvest::Goal::EXPLICIT_TARGETS;
      explicit_queue_ = explicit_mode_ ? goal->target_ids : std::vector<std::string>{};
      explicit_index_ = 0;
      // 新一轮重新累计发现数：集合已锁定时先记入当前锁定轮；若拍照前置将重置
      // 感知（首轮 RESETTING 成功时清零），该种子值会被推翻后按新锁定沿重计。
      discovered_total_ = explicit_mode_ ?
        static_cast<uint64_t>(explicit_queue_.size()) :
        (targets_locked_ ? last_target_count_ : 0);
      // 轮次与拍照前置复位；丢弃可能仍在途的拍照/重置/回位调用（服务请求
      // 无法取消，丢弃 future 仅表示不再关心结果，拍照移动会走到位姿后停止）。
      round_ = 1;
      photo_step_ = explicit_mode_ ? PhotoStep::DONE : PhotoStep::NOT_STARTED;
      photo_retries_ = 0;
      // 就位自校（F3）随新批次复位：首轮拍照前置前重新判定一次（跨批次
      // 环境可能变化，不外带上批次的通过/失败态）。
      preflight_passed_ = false;
      preflight_blocked_ = false;
      preflight_failure_signature_.clear();
      preflight_last_failure_event_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
      round_start_attempted_ = machine_.snapshot().counters.attempted;
      photo_future_ = TriggerFuture();
      reset_future_ = TriggerFuture();
      clear_memory_future_ = TriggerFuture();
      return_future_ = TriggerFuture();
      return_call_pending_ = false;
      // 2.5-L4 清除点 (d)：reset_batch（上方 COMPLETED/INTERRUPTED 分支）配套
      // 的节点侧清锁；新批次一律从干净派发/推进状态起步。
      clear_dispatch_locks_locked();
      last_seen_selected_.clear();
      skip_cycle_id_.clear();
      // 残局抬质量账随新批次清零（E3）：次数预算是"每目标每批次"，旧批次的
      // 消耗/去重/在途标记不带入新批次。
      observe_retry_counts_.clear();
      observe_retry_retired_this_round_.clear();
      observe_retry_active_id_.clear();
      // 丢失目标记账跟踪集随新批次清册（2.4）：旧批次锁定集不带入，重新
      // 锁定后按新集建册——整批重置语义，旧集消失目标绝不误记丢失。
      locked_target_ids_.clear();
      selected_empty_since_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
      // cycle 序号随新批次归零（cycle_id = <run_id>-cycle-<批次内单调序号>，
      // run_id 已在上行按 goal 设置，跨批次天然不重名）；收齐开窗计时复位
      // （last_collect_window_s_ 自适应基准保留，跨批次复用上次实测窗口）。
      cycle_sequence_ = 0;
      collect_since_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
      publish_state();
    }
    // 防御性 join：单 goal 守卫下不应存在未退出的旧线程。
    if (execute_thread_.joinable()) {execute_thread_.join();}
    execute_thread_ = std::thread([this, goal_handle]() {execute(goal_handle);});
  }

  void execute(const std::shared_ptr<RunGoalHandle> goal_handle)
  {
    auto feedback = std::make_shared<RunHarvest::Feedback>();
    for (;; ) {
      if (!rclcpp::ok() || stop_requested_.load() || goal_handle->is_canceling()) {break;}
      std::shared_ptr<RunHarvest::Result> terminal;
      bool succeeded = false;
      bool canceled = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        feedback->state = make_state();
        const auto & snap = machine_.snapshot();
        if (snap.batch_state == BatchState::COMPLETED) {
          terminal = std::make_shared<RunHarvest::Result>();
          terminal->success = true;
          terminal->termination_reason = "批次完成";
          terminal->summary = make_summary_locked();
          active_run_goal_.reset();
          publish_event(
            "batch_completed", "批次完成" + batch_throughput_text_locked(),
            HarvestEvent::INFO);
          succeeded = true;
        } else if (snap.batch_state == BatchState::INTERRUPTED) {
          // T9（2.2/2.9）：CANCEL_NOW 被状态机接受后批次落 INTERRUPTED，
          // RunHarvest 以 canceled 终局退出——不再空转占用单 goal 守卫，
          // 新 goal 随之可受理（goal_callback 对终局批次豁免 mode 门）。
          // 活动单目标 goal 的取消已由 control_callback 当场传播。
          terminal = std::make_shared<RunHarvest::Result>();
          terminal->success = false;
          terminal->termination_reason = "批次已取消";
          terminal->summary = make_summary_locked();
          active_run_goal_.reset();
          publish_event(
            "batch_canceled", "批次已取消" + batch_throughput_text_locked(),
            HarvestEvent::WARNING);
          canceled = true;
        } else if (snap.recovery_required) {
          terminal = std::make_shared<RunHarvest::Result>();
          terminal->success = false;
          terminal->termination_reason = "批次需人工确认恢复: " + snap.message;
          terminal->summary = make_summary_locked();
          active_run_goal_.reset();
          publish_event(
            "batch_aborted",
            terminal->termination_reason + batch_throughput_text_locked(),
            HarvestEvent::ERROR);
        }
      }
      if (terminal) {
        if (succeeded) {
          goal_handle->succeed(terminal);
        } else if (canceled) {
          goal_handle->canceled(terminal);
        } else {
          goal_handle->abort(terminal);
        }
        return;
      }
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(200ms);
    }
    // 取消/停用路径：先把取消传播到活动单目标 goal，等其落安全检查点。
    TargetGoalHandle::SharedPtr target_goal;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      target_goal = active_target_goal_;
    }
    if (target_goal) {
      target_cycle_client_->async_cancel_goal(target_goal);
      const auto deadline = std::chrono::steady_clock::now() + 5s;
      bool settled = false;
      while (rclcpp::ok() && !settled && std::chrono::steady_clock::now() < deadline) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          settled = !active_target_goal_;
        }
        if (!settled) {std::this_thread::sleep_for(50ms);}
      }
    }
    auto result = std::make_shared<RunHarvest::Result>();
    result->success = false;
    result->termination_reason = "批次已取消";
    {
      std::lock_guard<std::mutex> lock(mutex_);
      result->summary = make_summary_locked();
      active_run_goal_.reset();
      publish_event(
        "batch_canceled", "批次已取消" + batch_throughput_text_locked(),
        HarvestEvent::WARNING);
    }
    goal_handle->canceled(result);
  }

  mutable std::mutex mutex_;
  HarvestStateMachine machine_;
  // 参数监听器（构造即声明+启动校验；readiness.* 热读、其余 on_configure 缓存）。
  std::shared_ptr<ParamListener> param_listener_;
  std::atomic_bool stop_requested_{false};
  uint64_t event_sequence_{0};
  rclcpp_lifecycle::LifecyclePublisher<HarvestState>::SharedPtr state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<HarvestEvent>::SharedPtr event_pub_;
  rclcpp::Service<ControlHarvest>::SharedPtr control_service_;
  // 回调组划分见文件头注释：timer_sub(组A) / service(组B) /
  // policy(组C，独立互斥，内含 ≤2s 锁外 RPC 等待) / client(组D，可重入)。
  rclcpp::CallbackGroup::SharedPtr timer_sub_group_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr policy_callback_group_;
  rclcpp::CallbackGroup::SharedPtr client_callback_group_;
  rclcpp::Service<SetOperationPolicy>::SharedPtr policy_service_;
  rclcpp_action::Server<RunHarvest>::SharedPtr action_server_;
  rclcpp_action::Client<RunTargetCycle>::SharedPtr target_cycle_client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> approach_parameters_;
  rclcpp::Client<Trigger>::SharedPtr complete_target_client_;
  rclcpp::Client<Trigger>::SharedPtr photo_pose_client_;
  rclcpp::Client<Trigger>::SharedPtr reset_targets_client_;
  // fresh_scene 清感知身份记忆客户端（协议 2.3，阶段 D2）。
  rclcpp::Client<Trigger>::SharedPtr clear_memory_client_;
  // E3 残局抬质量：感知重开已终局目标客户端（fire-and-forget，见创建处注释）。
  rclcpp::Client<peach_pose_msgs::srv::ReopenTarget>::SharedPtr reopen_target_client_;
  // 对端 lifecycle GetState 客户端与轮询状态（A8）：grasp_node_active_ 只在
  // 最近一次 GetState 响应为 PRIMARY_STATE_ACTIVE 时为 true；轮询超时不应
  // 答视为非 Active。全部只在 mutex_ 下读写。
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr grasp_state_client_;
  bool grasp_node_active_{false};
  bool grasp_state_poll_pending_{false};
  rclcpp::Time grasp_state_poll_started_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<peach_pose_msgs::msg::PeachTargetObservationArray>::SharedPtr targets_sub_;
  rclcpp::Subscription<peach_harvest_msgs::msg::ReconstructionStatus>::SharedPtr
    reconstruction_sub_;
  rclcpp::Subscription<aubo_msgs::msg::RobotStatus>::SharedPtr robot_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string selected_target_id_;
  std::string last_dispatched_target_;
  // 上一帧感知消息的 selected（识别 selected 真实跨帧变化，见 targets_callback）。
  std::string last_seen_selected_;
  bool targets_locked_{false};
  uint32_t last_target_count_{0};
  uint64_t discovered_total_{0};
  // 拍照前置/复扫轮次：节点侧状态（PhotoStep 见上），策略参数缓存自 on_configure。
  PhotoStep photo_step_{PhotoStep::NOT_STARTED};
  uint32_t round_{1};
  uint32_t round_start_attempted_{0};
  uint32_t photo_retries_{0};
  rclcpp::Time photo_retry_not_before_{0, 0, RCL_ROS_TIME};
  rclcpp::Time photo_call_started_{0, 0, RCL_ROS_TIME};
  TriggerFuture photo_future_;
  TriggerFuture reset_future_;
  // fresh_scene 清记忆在途调用（与拍照/重置同一 wait_for(0) 轮询模式）。
  TriggerFuture clear_memory_future_;
  // 批次完成后回拍照位姿的 best-effort 调用（与批次状态解耦，仅记事件）。
  TriggerFuture return_future_;
  bool return_call_pending_{false};
  rclcpp::Time return_call_started_{0, 0, RCL_ROS_TIME};
  bool photo_pose_enabled_{true};
  uint32_t photo_max_retries_{3};
  double photo_retry_cooldown_s_{5.0};
  double photo_service_timeout_s_{90.0};
  bool photo_return_on_complete_{true};
  // 批次启动就位自校（阶段 F3，harvest.preflight_check）：开关缓存自
  // on_configure；preflight_passed_=本批次首轮自校已通过（放行拍照前置，
  // RESCAN 轮次不重复判定）；preflight_blocked_=当前因自校失败挂起
  // （make_state 投影进 state blockers 与 message）；失败签名 + 上次失败
  // 事件时刻用于 ERROR 事件节流（签名变化即报，否则每 5s 提醒一次）。
  // 复位点：新批次 accepted_callback / on_deactivate。全部只在 mutex_ 下访问。
  static constexpr const char * kPreflightBaseFrame = "base_link";
  static constexpr const char * kPreflightCameraFrame = "camera_link";
  bool preflight_check_enabled_{true};
  bool preflight_passed_{false};
  bool preflight_blocked_{false};
  std::string preflight_failure_signature_;
  rclcpp::Time preflight_last_failure_event_{0, 0, RCL_ROS_TIME};
  // 自校 TF 查询通道（on_configure 装配；listener 自带自旋线程）。
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  // 换场景开关（协议 2.3，on_configure 缓存）：true 时批次首轮开窗前先
  // 调 clear_target_memory 清感知世界系身份记忆。
  bool fresh_scene_{false};
  bool rescan_until_empty_{true};
  uint32_t max_rounds_{3};
  double harvest_stall_timeout_s_{30.0};
  // 收齐窗口编排侧总超时（协议 2.3 监督）配置下限（秒）；运行期实际上限
  // 自适应为 max(本值, 1.5×last_collect_window_s_)（I4）。
  double collect_timeout_s_{40.0};
  // 收齐开窗计时起点（0=不在收齐等待）：PhotoStep 完成且未锁定时置位，
  // locked 到达（targets_callback）或离开收齐等待（refresh）时清零。
  rclcpp::Time collect_since_{0, 0, RCL_ROS_TIME};
  // 上次收齐窗口实测时长（秒，开窗→锁定沿墙钟）；0=尚无实测，超时上限
  // 直接用配置下限。跨批次保留复用。
  double last_collect_window_s_{0.0};
  // 发现进度摘要（R-D8）：最近一帧感知的收齐累积数/确认中数，
  // DISCOVERY 未锁定时拼进 message 透出。
  int32_t collecting_count_{0};
  int32_t pending_count_{0};
  // 无可选目标停滞计时起点（0=未停滞）；见轮次判定块的提前收口逻辑。
  rclcpp::Time selected_empty_since_{0, 0, RCL_ROS_TIME};
  std::string run_id_;
  std::string current_cycle_id_;
  // cycle_id 批次内单调序号（<run_id>-cycle-<序号> 的序号源，新批次归零）。
  uint32_t cycle_sequence_{0};
  // 当前活动周期的派发时刻（单目标墙钟 elapsed 的起点；0=无活动周期）。
  rclcpp::Time target_dispatch_started_{0, 0, RCL_ROS_TIME};
  // 能力端连续 goal 拒绝计数（接受即清零；达到上限才进待恢复）。
  uint32_t consecutive_rejections_{0};
  // 派发拒绝重试：当前目标已重试次数与冷却截止（steady 时钟，零值无冷却）。
  uint32_t dispatch_retries_{0};
  std::chrono::steady_clock::time_point retry_not_before_{};
  double dispatch_retry_delay_s_{2.0};
  uint32_t max_dispatch_retries_{4};
  // 连续拒绝熔断上限（on_configure 缓存，钳制 ≥1）。
  uint32_t max_consecutive_rejections_{6};
  rclcpp::Time batch_started_{0, 0, RCL_ROS_TIME};
  TargetGoalHandle::SharedPtr active_target_goal_;
  std::shared_ptr<RunGoalHandle> active_run_goal_;
  // 操作员跳过绑定（协议 2.9/2.6-2）：SKIP_TARGET 接受时记活动 cycle_id，
  // result_callback 仅当结果周期匹配时按"操作员跳过"记账；空串=无跳过请求。
  std::string skip_cycle_id_;
  // 感知推进协议（2.6）状态机参数缓存（on_configure）与运行状态：
  // 失败冷却/熔断上限/单次 RPC 超时；失败计数、冷却截止、在途 future 与
  // 事件上下文（发起推进时的 cycle_id/target_id）。
  double advance_retry_delay_s_{1.0};
  uint32_t advance_max_retries_{5};
  double advance_timeout_s_{2.0};
  AdvanceStep advance_step_{AdvanceStep::IDLE};
  uint32_t advance_failures_{0};
  rclcpp::Time advance_retry_not_before_{0, 0, RCL_ROS_TIME};
  rclcpp::Time advance_call_started_{0, 0, RCL_ROS_TIME};
  TriggerFuture advance_future_;
  std::string advance_cycle_id_;
  std::string advance_target_id_;
  // 显式清单批次（RunHarvest goal target_source=EXPLICIT_TARGETS）：按给定顺序
  // 逐个派发、终局后由索引推进、一轮即止；不消费感知锁定计划、不走拍照前置
  // 与复扫。known_target_ids_ 为最近一帧感知锁定集（targets_callback 维护），
  // 供显式批次派发前校验目标存在性。
  bool explicit_mode_{false};
  std::vector<std::string> explicit_queue_;
  size_t explicit_index_{0};
  std::unordered_set<std::string> known_target_ids_;
  // 最近锁定帧目标的观测顺序（=感知固定优先级序），与 known_target_ids_
  // 同步在 targets_callback 维护；残局抬质量（E3）按此序选候选。
  std::vector<std::string> known_target_order_;
  // 残局抬质量（协议 2.8 残局 OBSERVE_ONLY，阶段 E3）节点侧状态：
  // observe_retry_counts_——批次内每目标已派发的抬质量次数（派发时消耗，
  // 拒绝/取消不归还；新批次/停用时清零）；
  // observe_retry_retired_this_round_——本轮已抬过的目标（每轮每目标至多
  // 一次；RESCAN 开新一轮/新批次/停用时清册）；
  // observe_retry_active_id_——在途抬质量周期的目标 ID（空=当前无活动周期
  // 或活动周期非抬质量；派发时置位，终局/拒绝回调清除），result_callback
  // 据此路由记账（observe 终局只进审计账、不推进感知计划）。
  bool observe_retry_enabled_{true};
  uint32_t observe_retry_max_{2};
  std::unordered_map<std::string, uint32_t> observe_retry_counts_;
  std::unordered_set<std::string> observe_retry_retired_this_round_;
  std::string observe_retry_active_id_;
  // 丢失目标记账（2.4）跟踪集：本轮感知锁定集内尚无终局账的 target_id；
  // 锁定帧间比对（targets_callback），清册点：解锁帧/新批次/停用。
  std::unordered_set<std::string> locked_target_ids_;
  std::thread execute_thread_;
  aubo_msgs::msg::RobotStatus robot_status_;
  rclcpp::Time targets_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time reconstruction_received_{0, 0, RCL_ROS_TIME};
  // 三路就绪话题的实测发布周期 EMA（2.11 自适应新鲜度；纯核
  // PeriodEstimator，全部只在 mutex_ 下访问）。
  PeriodEstimator targets_period_;
  PeriodEstimator recon_period_;
  PeriodEstimator robot_period_;
  // A10：diagnostics 订阅 DDS Deadline(2s) 违约计数（rmw 事件回调写入，原子）
  std::atomic<uint64_t> recon_deadline_misses_{0};
  rclcpp::Time robot_received_{0, 0, RCL_ROS_TIME};
};
}  // namespace peach_harvest_orchestrator

// 进程内集成测试入口（test_dispatch_protocol）：测试进程直接构造真实节点，
// 生命周期转换经 configure()/activate() 直调（模式同 peach_approach_grasp 的
// test_lifecycle）。不随可执行目标使用。
std::shared_ptr<rclcpp_lifecycle::LifecycleNode>
peach_harvest_orchestrator::make_orchestrator_node_for_test(
  const rclcpp::NodeOptions & options)
{
  return std::make_shared<HarvestOrchestratorNode>(options);
}

#ifndef PEACH_HARVEST_ORCHESTRATOR_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // 4 线程：组A timer/订阅、组B control/action、组C policy（≤2s 锁外 RPC
  // 等待）各占一线程仍余一路给组D client 响应处理；回调组划分见
  // harvest_orchestrator_node.cpp 头注释。
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  const auto node = std::make_shared<peach_harvest_orchestrator::HarvestOrchestratorNode>();
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
#endif  // PEACH_HARVEST_ORCHESTRATOR_NO_MAIN
