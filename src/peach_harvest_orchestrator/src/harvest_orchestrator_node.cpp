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


#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "aubo_msgs/msg/robot_status.hpp"
#include "peach_harvest_msgs/action/run_harvest.hpp"
#include "peach_harvest_msgs/action/run_target_cycle.hpp"
#include "peach_harvest_msgs/msg/harvest_event.hpp"
#include "peach_harvest_msgs/msg/harvest_state.hpp"
#include "peach_harvest_msgs/msg/harvest_summary.hpp"
#include "peach_harvest_msgs/msg/target_outcome.hpp"
#include "peach_harvest_msgs/srv/control_harvest.hpp"
#include "peach_harvest_msgs/srv/set_operation_policy.hpp"
#include "peach_harvest_orchestrator/state_machine.hpp"
#include "peach_pose_msgs/msg/peach_target_observation_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

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

class HarvestOrchestratorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  HarvestOrchestratorNode()
  : LifecycleNode("peach_harvest_orchestrator")
  {
    declare_parameter("auto_start_enabled", true);
    declare_parameter("execution_enabled", false);
    declare_parameter("grasp_enabled", false);
    declare_parameter("tool_enabled", false);
    declare_parameter("readiness.web", true);
    declare_parameter("readiness.timeout_s", 2.0);
    declare_parameter("readiness.require_robot_status", true);
    // 抓取全局拍照位姿（权威关节顺序的关节角，rad），仅存档；
    // 实际移动走 go_to_photo_pose 服务 + SRDF 命名状态 global_photo_pose。
    declare_parameter("global_photo_joints", std::vector<double>{});
    // 全局拍照位姿前置：进入 DISCOVERY/复扫轮次先把机械臂送到拍照位姿，
    // 再重置感知收齐锁定；连续失败 max_retries 次后进入待恢复。
    declare_parameter("photo_pose.enabled", true);
    declare_parameter("photo_pose.max_retries", 3);
    declare_parameter("photo_pose.retry_cooldown_s", 5.0);
    declare_parameter("photo_pose.service_timeout_s", 90.0);
    // 批次完成后回一次拍照位姿（best-effort，失败仅记事件）。
    declare_parameter("photo_pose.return_on_complete", true);
    // 复扫递减集循环：本轮目标全部处理完后回拍照位姿重锁感知再摘一轮，
    // 直到新一轮锁定空集或达到 max_rounds（round 从 1 起计，含首轮）。
    declare_parameter("harvest.rescan_until_empty", true);
    declare_parameter("harvest.max_rounds", 3);
    // 跨包接口名称（默认值保持现有全网名，launch 不需要改）。
    declare_parameter(
      "target_cycle_action_name", "/peach_approach_grasp_node/run_target_cycle");
    declare_parameter("approach_node_name", "/peach_approach_grasp_node");
    declare_parameter(
      "complete_target_service_name", "/peach_pose_node/complete_selected_target");
    declare_parameter(
      "reset_targets_service_name", "/peach_pose_node/reset_global_targets");
    declare_parameter(
      "photo_pose_service_name", "/peach_approach_grasp_node/go_to_photo_pose");
  }

  ~HarvestOrchestratorNode() override
  {
    stop_requested_.store(true);
    if (execute_thread_.joinable()) {execute_thread_.join();}
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    // 缓存拍照前置/复扫策略参数（yaml 为权威源，运行期不预期动态改）。
    photo_pose_enabled_ = get_parameter("photo_pose.enabled").as_bool();
    photo_max_retries_ =
      static_cast<uint32_t>(std::max<int64_t>(get_parameter("photo_pose.max_retries").as_int(), 1));
    photo_retry_cooldown_s_ = get_parameter("photo_pose.retry_cooldown_s").as_double();
    photo_service_timeout_s_ = get_parameter("photo_pose.service_timeout_s").as_double();
    photo_return_on_complete_ = get_parameter("photo_pose.return_on_complete").as_bool();
    rescan_until_empty_ = get_parameter("harvest.rescan_until_empty").as_bool();
    max_rounds_ =
      static_cast<uint32_t>(std::max<int64_t>(get_parameter("harvest.max_rounds").as_int(), 1));

    state_pub_ = create_publisher<HarvestState>("~/state", rclcpp::QoS(1).transient_local());
    event_pub_ = create_publisher<HarvestEvent>("~/events", 50);
    control_service_ = create_service<ControlHarvest>(
      "~/control", std::bind(
        &HarvestOrchestratorNode::control_callback, this,
        std::placeholders::_1, std::placeholders::_2));
    policy_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
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
      std::bind(&HarvestOrchestratorNode::accepted_callback, this, std::placeholders::_1));
    target_cycle_client_ = rclcpp_action::create_client<RunTargetCycle>(
      this, get_parameter("target_cycle_action_name").as_string());
    approach_parameters_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this, get_parameter("approach_node_name").as_string(), rclcpp::ParametersQoS(),
      policy_callback_group_);
    // 感知固定优先级计划推进（Trigger）：放可重入组，RPC 等待不阻塞默认组回调。
    complete_target_client_ = create_client<Trigger>(
      get_parameter("complete_target_service_name").as_string(),
      rclcpp::ServicesQoS(), policy_callback_group_);
    // 拍照前置两步（去拍照位姿 + 重置感知收齐）：future 只存不等待，
    // 由 refresh 轮询结果，回调内绝不持锁等 RPC。
    photo_pose_client_ = create_client<Trigger>(
      get_parameter("photo_pose_service_name").as_string(),
      rclcpp::ServicesQoS(), policy_callback_group_);
    reset_targets_client_ = create_client<Trigger>(
      get_parameter("reset_targets_service_name").as_string(),
      rclcpp::ServicesQoS(), policy_callback_group_);
    targets_sub_ = create_subscription<peach_pose_msgs::msg::PeachTargetObservationArray>(
      "/peach/perception/target_observations", 10,
      std::bind(&HarvestOrchestratorNode::targets_callback, this, std::placeholders::_1));
    reconstruction_sub_ = create_subscription<std_msgs::msg::String>(
      "/peach/reconstruction/diagnostics", rclcpp::QoS(1).transient_local(),
      std::bind(
        &HarvestOrchestratorNode::reconstruction_callback, this, std::placeholders::_1));
    robot_sub_ = create_subscription<aubo_msgs::msg::RobotStatus>(
      "/aubo_io_controller/robot_status", 10,
      std::bind(&HarvestOrchestratorNode::robot_callback, this, std::placeholders::_1));
    timer_ = create_wall_timer(500ms, std::bind(&HarvestOrchestratorNode::refresh, this));

    OperationPolicy policy{
      get_parameter("auto_start_enabled").as_bool(),
      get_parameter("execution_enabled").as_bool(),
      get_parameter("grasp_enabled").as_bool(),
      get_parameter("tool_enabled").as_bool()};
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
      // 停用期间不再推进拍照/复扫：清空在途调用，重新激活后从头走拍照前置。
      photo_future_ = TriggerFuture();
      reset_future_ = TriggerFuture();
      return_future_ = TriggerFuture();
      return_call_pending_ = false;
      photo_step_ = PhotoStep::NOT_STARTED;
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
    RESET_PENDING,  // 拍照位姿已到，待发起 reset_global_targets
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
      message.message += "；第" + std::to_string(round_) + "/" +
        std::to_string(max_rounds_) + "轮";
      if (photo_step_ != PhotoStep::DONE) {
        message.message += "，拍照前置中";
      } else if (!targets_locked_) {
        // 重置后感知需经收齐窗口（最长约 max_collect_s 秒）才重新锁定，属正常等待。
        message.message += "，等待感知收齐锁定";
      }
    }
    message.blockers = source.blockers;
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
      // quality_score 与单目标 elapsed 本阶段不采集，保持默认 0。
      summary.outcomes.push_back(entry);
    }
    return summary;
  }

  void targets_callback(
    const peach_pose_msgs::msg::PeachTargetObservationArray::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // 目标集合由未锁定变为锁定视为新的一轮发现：累计该轮目标数。
    // 复扫每轮都会经历 reset（解锁）→ 收齐（再锁定）的完整沿，多轮天然覆盖。
    if (message->target_set_locked && !targets_locked_) {
      discovered_total_ += message->target_count;
      publish_event(
        "round_locked",
        "第" + std::to_string(round_) + "轮锁定 " +
        std::to_string(message->target_count) + " 个目标",
        HarvestEvent::INFO);
    }
    last_target_count_ = message->target_count;
    // 派发锁随感知计划推进解锁：selected 变化（含变为空）说明上一目标的终局
    // 记账已被感知吸收，允许派发新目标；selected 不变则保持锁定防重派。
    if (message->selected_target_id != last_dispatched_target_) {
      last_dispatched_target_.clear();
    }
    selected_target_id_ = message->selected_target_id;
    targets_locked_ = message->target_set_locked;
    targets_received_ = now();
  }

  void reconstruction_callback(const std_msgs::msg::String::SharedPtr)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    reconstruction_received_ = now();
  }

  void robot_callback(const aubo_msgs::msg::RobotStatus::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    robot_status_ = *message;
    robot_received_ = now();
  }

  void dispatch_target_locked()
  {
    // 拍照前置未完成（含等待感知重新锁定）时不派发；selected 非空本身隐含已锁定。
    // 五项前置的判定在状态机库纯核 allow_dispatch，节点只做事实采集。
    if (!allow_dispatch(
        photo_step_ == PhotoStep::DONE, selected_target_id_, last_dispatched_target_,
        machine_.snapshot().target_active,
        target_cycle_client_->action_server_is_ready()))
    {
      return;
    }
    if (!machine_.begin_target(selected_target_id_)) {return;}
    last_dispatched_target_ = selected_target_id_;
    if (run_id_.empty()) {run_id_ = "auto";}
    current_cycle_id_ = "cycle-" + std::to_string(machine_.snapshot().revision);
    RunTargetCycle::Goal goal;
    goal.request_id = current_cycle_id_;
    goal.run_id = run_id_;
    goal.cycle_id = current_cycle_id_;
    goal.target_id = selected_target_id_;
    goal.mode = RunTargetCycle::Goal::FULL;
    rclcpp_action::Client<RunTargetCycle>::SendGoalOptions options;
    options.goal_response_callback = [this](const TargetGoalHandle::SharedPtr & handle) {
        // 锁内按接受/拒绝路由改状态，锁外再推进感知计划（RPC 绝不持锁等待）。
        bool advance_perception = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (handle) {
            // 保存活动 goal handle，供控制命令/停用时传播取消。
            active_target_goal_ = handle;
            consecutive_rejections_ = 0;
            return;
          }
          // 能力端拒绝（目标当前不可观测/缓存未命中等单目标级原因）：按不可达
          // 记账并自动推进感知计划跳过该目标，批次不停；连续拒绝达到上限才认定
          // 能力端故障，进入待恢复。拒绝原因无语义字段可区分，熔断计数是唯一
          // 可靠边界。last_dispatched_target_ 由 targets_callback 在感知
          // selected 变化时复位，本路径无需处理。
          ++consecutive_rejections_;
          const std::string target_id = machine_.snapshot().target_id;
          if (consecutive_rejections_ >= 3) {
            machine_.require_recovery("单目标能力连续拒绝目标请求");
            publish_event(
              "target_rejected", "单目标能力连续拒绝目标请求，批次待人工确认",
              HarvestEvent::ERROR, current_cycle_id_, target_id);
          } else {
            machine_.record_target_outcome(
              target_id, TargetOutcome::SKIPPED_UNREACHABLE,
              "单目标能力拒绝目标请求");
            advance_perception = true;
            publish_event(
              "target_rejected", "单目标能力拒绝目标请求，已跳过该目标",
              HarvestEvent::WARNING, current_cycle_id_, target_id);
          }
          current_cycle_id_.clear();
          publish_state();
        }
        if (advance_perception) {call_complete_selected_target();}
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
    options.result_callback = [this](const TargetGoalHandle::WrappedResult & result) {
        // 锁内按终态路由改状态，锁外再推进感知计划（RPC 绝不持锁等待）。
        bool advance_perception = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          const std::string target_id = machine_.snapshot().target_id;
          const std::string reason = result.result && !result.result->reason.empty() ?
            result.result->reason : "单目标周期失败";
          active_target_goal_.reset();
          // 接触段故障等需人工确认的恢复：最高优先，skip 标志也让位。
          if (result.result && result.result->recovery_required) {
            machine_.require_recovery(reason);
            // 恢复路径保留 last_dispatched_target_，防止确认后无限重派同一故障目标。
            publish_event("target_failed", reason, HarvestEvent::ERROR, "", target_id);
          } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
            if (skip_requested_) {
              // 操作员跳过：记账为 CANCELED 并推进感知计划。派发锁保留到感知新
              // selected 到达（targets_callback 按 selected 变化解锁），防止计划
              // 推进传播前的窗口期把同一目标立即重派（goal 必被能力端拒绝）。
              machine_.record_target_outcome(target_id, TargetOutcome::CANCELED, "操作员跳过");
              advance_perception = true;
              publish_event(
                "target_skipped", "操作员跳过当前目标", HarvestEvent::AUDIT, "", target_id);
            } else {
              // 暂停/立即取消路径：落安全检查点，复位派发锁允许恢复后重派。
              machine_.reach_safe_checkpoint();
              last_dispatched_target_.clear();
              publish_event(
                "target_canceled", "单目标周期已取消", HarvestEvent::WARNING, "", target_id);
            }
          } else {
            // 正常终态按 outcome 路由，全部自动推进感知计划（不等人）。
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
            machine_.record_target_outcome(target_id, recorded, reason);
            advance_perception = true;
            // 派发锁不在此复位：保留到感知新 selected 到达（targets_callback 按
            // selected 变化解锁），防止计划推进传播前的窗口期把刚终局的目标
            // 立即重派（能力端缓存已失效，goal 必被拒绝并误入 RECOVERY_REQUIRED）。
            publish_event(event_code, reason, severity, "", target_id);
          }
          skip_requested_ = false;
          current_cycle_id_.clear();
          publish_state();
        }
        if (advance_perception) {call_complete_selected_target();}
      };
    target_cycle_client_->async_send_goal(goal, options);
    publish_event(
      "target_dispatched", selected_target_id_, HarvestEvent::INFO,
      goal.request_id, selected_target_id_);
  }

  // 推进感知固定优先级计划：异步 + 短等待；调用时不得持有 mutex_。
  void call_complete_selected_target()
  {
    if (!complete_target_client_->service_is_ready()) {
      RCLCPP_WARN(
        get_logger(), "感知节点 complete_selected_target 服务不可用，计划推进被跳过");
      return;
    }
    auto future = complete_target_client_->async_send_request(
      std::make_shared<Trigger::Request>());
    if (future.wait_for(1s) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "complete_selected_target 调用超时");
      return;
    }
    if (!future.get()->success) {
      RCLCPP_WARN(
        get_logger(), "complete_selected_target 被拒绝: %s", future.get()->message.c_str());
    }
  }

  // 是否需要机械臂移动到拍照位姿：execution 关或 photo_pose 关时不移动。
  // 调用时必须已持有 mutex_。
  bool photo_motion_required_locked() const
  {
    return machine_.snapshot().policy.execution_enabled && photo_pose_enabled_;
  }

  // 发起本轮拍照前置；调用时必须已持有 mutex_。
  void begin_photo_step_locked()
  {
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
      machine_.require_recovery("全局拍照位姿移动失败: " + reason);
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

  void refresh()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double timeout = get_parameter("readiness.timeout_s").as_double();
    const bool require_robot = get_parameter("readiness.require_robot_status").as_bool();
    // 节点把 ROS 时间/消息字段折算成纯值样本，四路推算在状态机库纯核完成。
    ReadinessSample readiness_sample;
    readiness_sample.now_s = now().seconds();
    readiness_sample.targets_received_s = targets_received_.seconds();
    readiness_sample.reconstruction_received_s = reconstruction_received_.seconds();
    readiness_sample.robot_received_s = robot_received_.seconds();
    readiness_sample.robot_e_stopped = robot_status_.e_stopped != 0;
    readiness_sample.robot_in_error = robot_status_.in_error != 0;
    readiness_sample.robot_drives_powered = robot_status_.drives_powered != 0;
    readiness_sample.robot_motion_possible = robot_status_.motion_possible != 0;
    readiness_sample.action_server_ready =
      target_cycle_client_->action_server_is_ready();
    readiness_sample.web_ready = get_parameter("readiness.web").as_bool();
    machine_.update_readiness(
      ReadinessTracker(timeout, require_robot).evaluate(readiness_sample));
    const auto & snap = machine_.snapshot();
    const bool harvesting = snap.batch_state == BatchState::DISCOVERY ||
      snap.batch_state == BatchState::RUNNING;

    // 拍照前置：进入采摘态且无活动目标时发起并轮询推进（在途调用遇暂停/取消
    // 则冻结在当前阶段，恢复采摘态后继续轮询）。
    if (harvesting && !snap.target_active && photo_step_ == PhotoStep::NOT_STARTED) {
      begin_photo_step_locked();
    }
    if (harvesting && !snap.target_active) {
      poll_photo_step_locked();
    }

    // 复扫递减集循环：本轮锁定集合全部处理完（或锁定空集）后，决定再扫一轮
    // 或完成批次；等待收齐锁定期间（locked=false）不判定，保持 DISCOVERY。
    if (harvesting && !snap.target_active && photo_step_ == PhotoStep::DONE &&
      targets_locked_ && selected_target_id_.empty())
    {
      const uint32_t processed = snap.counters.attempted > round_start_attempted_ ?
        snap.counters.attempted - round_start_attempted_ : 0;
      const auto verdict = decide_round(
        true, last_target_count_, processed, round_, max_rounds_, rescan_until_empty_);
      if (verdict.decision == RoundDecision::RESCAN) {
        publish_event(
          "round_completed",
          "第" + std::to_string(round_) + "轮完成：本轮锁定 " +
          std::to_string(last_target_count_) + " 个目标，回拍照位姿开启复扫",
          HarvestEvent::INFO);
        ++round_;
        // 新一轮重新走拍照前置（回拍照位姿 + 重置感知收齐锁定）。
        photo_step_ = PhotoStep::NOT_STARTED;
      } else if (verdict.decision == RoundDecision::COMPLETE) {
        if (machine_.complete_batch(verdict.message)) {
          publish_event("batch_completed", verdict.message, HarvestEvent::INFO);
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
            // 挂起跳过标志：goal 取消落地后由 result_callback 记账并推进感知。
            skip_requested_ = true;
            goal_to_cancel = active_target_goal_;
          } else if (command == ControlCommand::CANCEL_NOW) {
            goal_to_cancel = active_target_goal_;
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
    }
    // 第二阶段：锁外下发参数到靠近抓取节点（2s RPC 等待绝不持锁）。
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
    std::lock_guard<std::mutex> lock(mutex_);
    // 第三阶段：复查 revision 在 RPC 期间未漂移，再提交状态机。
    if (machine_.snapshot().revision != checked_revision) {
      response->accepted = false;
      response->message = "策略下发期间状态已变化，请刷新后重试";
      response->revision = machine_.snapshot().revision;
      response->state = make_state();
      return;
    }
    const auto result = machine_.set_policy(
      requested, request->request_id, request->expected_revision);
    response->accepted = result.accepted;
    response->message = result.message;
    response->revision = result.revision;
    response->state = make_state();
    publish_state();
    if (result.accepted) {
      publish_event("policy_updated", result.message, HarvestEvent::AUDIT, request->request_id);
    }
  }

  rclcpp_action::GoalResponse goal_callback(
    const rclcpp_action::GoalUUID &, const std::shared_ptr<const RunHarvest::Goal>)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto & state = machine_.snapshot();
    // 单活动批次 goal 守卫：已有 RunHarvest 在执行时拒绝新 goal。
    if (state.recovery_required || !state.blockers.empty() || state.target_active ||
      state.mode != OperationMode::AUTO || active_run_goal_)
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
      // 新一轮重新累计发现数：集合已锁定时先记入当前锁定轮；若拍照前置将重置
      // 感知（首轮 RESETTING 成功时清零），该种子值会被推翻后按新锁定沿重计。
      discovered_total_ = targets_locked_ ? last_target_count_ : 0;
      // 轮次与拍照前置复位；丢弃可能仍在途的拍照/重置/回位调用（服务请求
      // 无法取消，丢弃 future 仅表示不再关心结果，拍照移动会走到位姿后停止）。
      round_ = 1;
      photo_step_ = PhotoStep::NOT_STARTED;
      photo_retries_ = 0;
      round_start_attempted_ = machine_.snapshot().counters.attempted;
      photo_future_ = TriggerFuture();
      reset_future_ = TriggerFuture();
      return_future_ = TriggerFuture();
      return_call_pending_ = false;
      last_dispatched_target_.clear();
      consecutive_rejections_ = 0;
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
          publish_event("batch_completed", "批次完成", HarvestEvent::INFO);
          succeeded = true;
        } else if (snap.recovery_required) {
          terminal = std::make_shared<RunHarvest::Result>();
          terminal->success = false;
          terminal->termination_reason = "批次需人工确认恢复: " + snap.message;
          terminal->summary = make_summary_locked();
          active_run_goal_.reset();
          publish_event("batch_aborted", terminal->termination_reason, HarvestEvent::ERROR);
        }
      }
      if (terminal) {
        if (succeeded) {goal_handle->succeed(terminal);} else {goal_handle->abort(terminal);}
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
      publish_event("batch_canceled", "批次已取消", HarvestEvent::WARNING);
    }
    goal_handle->canceled(result);
  }

  mutable std::mutex mutex_;
  HarvestStateMachine machine_;
  std::atomic_bool stop_requested_{false};
  uint64_t event_sequence_{0};
  rclcpp_lifecycle::LifecyclePublisher<HarvestState>::SharedPtr state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<HarvestEvent>::SharedPtr event_pub_;
  rclcpp::Service<ControlHarvest>::SharedPtr control_service_;
  rclcpp::CallbackGroup::SharedPtr policy_callback_group_;
  rclcpp::Service<SetOperationPolicy>::SharedPtr policy_service_;
  rclcpp_action::Server<RunHarvest>::SharedPtr action_server_;
  rclcpp_action::Client<RunTargetCycle>::SharedPtr target_cycle_client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> approach_parameters_;
  rclcpp::Client<Trigger>::SharedPtr complete_target_client_;
  rclcpp::Client<Trigger>::SharedPtr photo_pose_client_;
  rclcpp::Client<Trigger>::SharedPtr reset_targets_client_;
  rclcpp::Subscription<peach_pose_msgs::msg::PeachTargetObservationArray>::SharedPtr targets_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr reconstruction_sub_;
  rclcpp::Subscription<aubo_msgs::msg::RobotStatus>::SharedPtr robot_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string selected_target_id_;
  std::string last_dispatched_target_;
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
  // 批次完成后回拍照位姿的 best-effort 调用（与批次状态解耦，仅记事件）。
  TriggerFuture return_future_;
  bool return_call_pending_{false};
  rclcpp::Time return_call_started_{0, 0, RCL_ROS_TIME};
  bool photo_pose_enabled_{true};
  uint32_t photo_max_retries_{3};
  double photo_retry_cooldown_s_{5.0};
  double photo_service_timeout_s_{90.0};
  bool photo_return_on_complete_{true};
  bool rescan_until_empty_{true};
  uint32_t max_rounds_{3};
  std::string run_id_;
  std::string current_cycle_id_;
  // 能力端连续 goal 拒绝计数（接受即清零；达到上限才进待恢复）。
  uint32_t consecutive_rejections_{0};
  rclcpp::Time batch_started_{0, 0, RCL_ROS_TIME};
  TargetGoalHandle::SharedPtr active_target_goal_;
  std::shared_ptr<RunGoalHandle> active_run_goal_;
  bool skip_requested_{false};
  std::thread execute_thread_;
  aubo_msgs::msg::RobotStatus robot_status_;
  rclcpp::Time targets_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time reconstruction_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time robot_received_{0, 0, RCL_ROS_TIME};
};
}  // namespace peach_harvest_orchestrator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  const auto node = std::make_shared<peach_harvest_orchestrator::HarvestOrchestratorNode>();
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
