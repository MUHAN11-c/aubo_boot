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


#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "aubo_msgs/msg/robot_status.hpp"
#include "peach_harvest_msgs/action/run_harvest.hpp"
#include "peach_harvest_msgs/action/run_target_cycle.hpp"
#include "peach_harvest_msgs/msg/harvest_event.hpp"
#include "peach_harvest_msgs/msg/harvest_state.hpp"
#include "peach_harvest_msgs/srv/control_harvest.hpp"
#include "peach_harvest_msgs/srv/set_operation_policy.hpp"
#include "peach_harvest_orchestrator/state_machine.hpp"
#include "peach_pose_msgs/msg/peach_target_observation_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

namespace peach_harvest_orchestrator
{
using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using ControlHarvest = peach_harvest_msgs::srv::ControlHarvest;
using HarvestEvent = peach_harvest_msgs::msg::HarvestEvent;
using HarvestState = peach_harvest_msgs::msg::HarvestState;
using RunHarvest = peach_harvest_msgs::action::RunHarvest;
using RunGoalHandle = rclcpp_action::ServerGoalHandle<RunHarvest>;
using RunTargetCycle = peach_harvest_msgs::action::RunTargetCycle;
using TargetGoalHandle = rclcpp_action::ClientGoalHandle<RunTargetCycle>;
using SetOperationPolicy = peach_harvest_msgs::srv::SetOperationPolicy;

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
    declare_parameter("readiness.perception", false);
    declare_parameter("readiness.reconstruction", false);
    declare_parameter("readiness.motion", false);
    declare_parameter("readiness.web", true);
    declare_parameter("readiness.timeout_s", 2.0);
    declare_parameter("readiness.require_robot_status", true);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
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
      this, "/peach_approach_grasp_node/run_target_cycle");
    approach_parameters_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this, "/peach_approach_grasp_node", rclcpp::ParametersQoS(),
      policy_callback_group_);
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
    state_pub_->on_deactivate();
    event_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

private:
  HarvestState make_state() const
  {
    const auto & source = machine_.snapshot();
    HarvestState message;
    message.header.stamp = now();
    message.revision = source.revision;
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
    message.message = source.message;
    message.blockers = source.blockers;
    return message;
  }

  void targets_callback(
    const peach_pose_msgs::msg::PeachTargetObservationArray::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
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

  bool fresh(const rclcpp::Time & stamp, double timeout_s) const
  {
    return stamp.nanoseconds() > 0 && (now() - stamp).seconds() <= timeout_s;
  }

  void dispatch_target_locked()
  {
    if (selected_target_id_.empty() || selected_target_id_ == last_dispatched_target_ ||
      machine_.snapshot().target_active || !target_cycle_client_->action_server_is_ready())
    {
      return;
    }
    if (!machine_.begin_target(selected_target_id_)) {return;}
    last_dispatched_target_ = selected_target_id_;
    RunTargetCycle::Goal goal;
    goal.request_id = "auto-" + std::to_string(machine_.snapshot().revision);
    goal.run_id = "auto";
    goal.cycle_id = goal.request_id;
    goal.target_id = selected_target_id_;
    goal.mode = RunTargetCycle::Goal::FULL;
    rclcpp_action::Client<RunTargetCycle>::SendGoalOptions options;
    options.goal_response_callback = [this](const TargetGoalHandle::SharedPtr & handle) {
        if (handle) {return;}
        std::lock_guard<std::mutex> lock(mutex_);
        machine_.require_recovery("单目标能力拒绝目标请求");
        publish_state();
      };
    options.result_callback = [this](const TargetGoalHandle::WrappedResult & result) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
          result.result->outcome == RunTargetCycle::Result::SUCCEEDED)
        {
          machine_.reach_safe_checkpoint();
          publish_event("target_succeeded", result.result->reason);
        } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
          machine_.reach_safe_checkpoint();
          publish_event("target_canceled", "单目标周期已取消");
        } else {
          const std::string reason = result.result ? result.result->reason : "单目标周期失败";
          machine_.require_recovery(reason);
          publish_event("target_failed", reason);
        }
        publish_state();
      };
    target_cycle_client_->async_send_goal(goal, options);
    publish_event("target_dispatched", selected_target_id_);
  }

  void refresh()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double timeout = get_parameter("readiness.timeout_s").as_double();
    const bool perception_ready = targets_locked_ && fresh(targets_received_, timeout);
    const bool reconstruction_ready = fresh(reconstruction_received_, timeout);
    const bool require_robot = get_parameter("readiness.require_robot_status").as_bool();
    const bool robot_ready = !require_robot || (
      fresh(robot_received_, timeout) && robot_status_.e_stopped == 0 &&
      robot_status_.in_error == 0 && robot_status_.drives_powered != 0 &&
      robot_status_.motion_possible != 0);
    const bool motion_ready = target_cycle_client_->action_server_is_ready() && robot_ready;
    machine_.update_readiness({
        perception_ready,
        reconstruction_ready,
        motion_ready,
        get_parameter("readiness.web").as_bool()});
    dispatch_target_locked();
    publish_state();
  }

  void publish_state()
  {
    if (state_pub_ && state_pub_->is_activated()) {
      state_pub_->publish(make_state());
    }
  }

  void publish_event(const std::string & code, const std::string & text)
  {
    if (!event_pub_ || !event_pub_->is_activated()) {return;}
    HarvestEvent event;
    event.header.stamp = now();
    event.sequence = ++event_sequence_;
    event.severity = HarvestEvent::AUDIT;
    event.code = code;
    event.message = text;
    event_pub_->publish(event);
  }

  void control_callback(
    const std::shared_ptr<ControlHarvest::Request> request,
    std::shared_ptr<ControlHarvest::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (request->command > static_cast<uint8_t>(ControlCommand::ACKNOWLEDGE_RECOVERY)) {
      response->accepted = false;
      response->message = "未知控制命令";
      response->revision = machine_.snapshot().revision;
    } else {
      const auto result = machine_.control(
        static_cast<ControlCommand>(request->command), request->request_id,
        request->expected_revision);
      response->accepted = result.accepted;
      response->message = result.message;
      response->revision = result.revision;
      if (result.accepted) {publish_event("control_accepted", result.message);}
    }
    response->state = make_state();
    publish_state();
  }

  void policy_callback(
    const std::shared_ptr<SetOperationPolicy::Request> request,
    std::shared_ptr<SetOperationPolicy::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const OperationPolicy requested{
      request->auto_start_enabled, request->execution_enabled,
      request->grasp_enabled, request->tool_enabled};
    if (request->expected_revision != machine_.snapshot().revision) {
      response->accepted = false;
      response->message = "状态版本已过期";
      response->revision = machine_.snapshot().revision;
      response->state = make_state();
      return;
    }
    if (machine_.snapshot().target_active ||
      (requested.grasp_enabled && !requested.execution_enabled) ||
      (requested.tool_enabled && !requested.grasp_enabled))
    {
      response->accepted = false;
      response->message = machine_.snapshot().target_active ?
        "目标周期运行时不能修改操作策略" :
        "使能依赖必须满足 execution→grasp→tool";
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
    auto future = approach_parameters_->set_parameters_atomically({
        rclcpp::Parameter("execution.enabled", requested.execution_enabled),
        rclcpp::Parameter("grasp.enabled", requested.grasp_enabled),
        rclcpp::Parameter("tool.enabled", requested.tool_enabled)});
    if (future.wait_for(2s) != std::future_status::ready || !future.get().successful) {
      response->accepted = false;
      response->message = "靠近抓取节点拒绝策略更新";
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
    if (result.accepted) {publish_event("policy_updated", result.message);}
  }

  rclcpp_action::GoalResponse goal_callback(
    const rclcpp_action::GoalUUID &, const std::shared_ptr<const RunHarvest::Goal>)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto & state = machine_.snapshot();
    if (state.recovery_required || !state.blockers.empty() || state.target_active ||
      state.mode != OperationMode::AUTO)
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
    std::thread([this, goal_handle]() {execute(goal_handle);}).detach();
  }

  void execute(const std::shared_ptr<RunGoalHandle> goal_handle)
  {
    auto feedback = std::make_shared<RunHarvest::Feedback>();
    while (rclcpp::ok() && !stop_requested_.load() && !goal_handle->is_canceling()) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        feedback->state = make_state();
      }
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(200ms);
    }
    auto result = std::make_shared<RunHarvest::Result>();
    result->success = false;
    result->termination_reason = "批次已取消";
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
  rclcpp::Subscription<peach_pose_msgs::msg::PeachTargetObservationArray>::SharedPtr targets_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr reconstruction_sub_;
  rclcpp::Subscription<aubo_msgs::msg::RobotStatus>::SharedPtr robot_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string selected_target_id_;
  std::string last_dispatched_target_;
  bool targets_locked_{false};
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
