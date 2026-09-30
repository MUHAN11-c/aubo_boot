// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
#include "peach2_task/task_node.hpp"

#include <algorithm>
#include <cstdlib>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "peach2_task/bt/logic_nodes.hpp"
#include "peach2_task/ros/conversions.hpp"
#include "peach2_task/ros/ros_nodes.hpp"

namespace peach2_task
{

using std::placeholders::_1;
using std::placeholders::_2;
using SteadyClock = std::chrono::steady_clock;
namespace msg = peach2_interfaces::msg;
namespace srv = peach2_interfaces::srv;
namespace action = peach2_interfaces::action;

namespace
{

constexpr double kStatePublishMinPeriodS = 0.1;

rclcpp::QoS latched_qos()
{
  return rclcpp::QoS(1).reliable().transient_local();
}

}  // namespace

TaskNode::TaskNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("peach2_task", options)
{
  param_listener_ = std::make_shared<ParamListener>(get_node_parameters_interface(), get_logger());
}

TaskNode::~TaskNode()
{
  release();
}

// ------------------------------------------------------------------ lifecycle

TaskNode::CallbackReturn TaskNode::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    const Params params = param_listener_->get_params();
    core::SelectionConfig sel;
    sel.min_mask_quality = static_cast<float>(params.selection.min_mask_quality);
    sel.min_depth_coverage = static_cast<float>(params.selection.min_depth_coverage);
    sel.depth_min_m = params.selection.depth_min_m;
    sel.depth_max_m = params.selection.depth_max_m;
    sel.height_band_m = params.selection.height_band_m;
    const std::string sel_error = core::validate(sel);
    if (!sel_error.empty()) {
      RCLCPP_ERROR(get_logger(), "configure failed: selection: %s", sel_error.c_str());
      return CallbackReturn::FAILURE;
    }
    runs_dir_ = resolve_runs_dir(params);
    tree_file_ = resolve_tree_file(params);
    if (!std::filesystem::is_regular_file(tree_file_)) {
      RCLCPP_ERROR(get_logger(), "configure failed: tree file %s missing", tree_file_.c_str());
      return CallbackReturn::FAILURE;
    }

    ctx_ = std::make_shared<ros::RosContext>();
    ctx_->logger = get_logger();
    ctx_->ros_clock = get_clock();
    ctx_->node_start = now();
    ctx_->move_to = rclcpp_action::create_client<action::MoveTo>(
      this, "/peach/manipulation/move_to");
    ctx_->observe = rclcpp_action::create_client<action::ObserveTarget>(
      this, "/peach/target_model/observe");
    ctx_->harvest = rclcpp_action::create_client<action::HarvestTarget>(
      this, "/peach/manipulation/harvest_target");
    ctx_->snapshot = create_client<srv::BuildSceneSnapshot>("/peach/scene/build_snapshot");
    ctx_->begin_scene = create_client<srv::BeginScene>("/peach/perception/begin_scene");
    ctx_->reachability =
      create_client<srv::CheckReachability>("/peach/manipulation/check_reachability");
    ctx_->decision = create_client<srv::GetDecision>("/peach/target_model/get_decision");
    refresh_context(params);

    session_ = std::make_shared<core::BatchSession>();
    factory_ = std::make_unique<BT::BehaviorTreeFactory>();
    bt::register_logic_nodes(*factory_, session_);
    ros::register_ros_nodes(*factory_, ctx_, session_);
    factory_->registerBehaviorTreeFromFile(tree_file_.string());

    obs_sub_ = create_subscription<msg::TargetObservationArray>(
      "/peach/perception/observations", rclcpp::QoS(10).reliable(),
      [this](msg::TargetObservationArray::ConstSharedPtr m) {
        ctx_->observations = std::move(m);
        ctx_->observations_received_at = SteadyClock::now();
      });
    robot_status_sub_ = create_subscription<aubo_msgs::msg::RobotStatus>(
      "/aubo_io_controller/robot_status", rclcpp::QoS(1).best_effort(),
      [this](aubo_msgs::msg::RobotStatus::ConstSharedPtr m) {
        ctx_->robot.received = true;
        ctx_->robot.received_at = SteadyClock::now();
        ctx_->robot.drives_powered = m->drives_powered != 0;
        ctx_->robot.e_stopped = m->e_stopped != 0;
        ctx_->robot.in_error = m->in_error != 0;
      });
    tool_state_sub_ = create_subscription<msg::ToolState>(
      "/peach/end_effector/tool_state", latched_qos(),
      [this](msg::ToolState::ConstSharedPtr m) {ctx_->tool_state = std::move(m);});
    peer_recovery_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/peach/manipulation/recovery_required", latched_qos(),
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        if (m->data != session_->peer_recovery_required()) {
          RCLCPP_WARN(
            get_logger(), "manipulation recovery_required=%s", m->data ? "true" : "false");
        }
        session_->on_peer_recovery(m->data);
        publish_state(true);
      });

    enables_pub_ = create_publisher<msg::Enables>("/peach/enables", latched_qos());
    state_pub_ = create_publisher<msg::BatchState>("/peach/task/state", latched_qos());

    set_enables_srv_ = create_service<srv::SetEnables>(
      "/peach/task/set_enables",
      [this](const std::shared_ptr<srv::SetEnables::Request> req,
      std::shared_ptr<srv::SetEnables::Response> res) {on_set_enables(req, res);});
    ack_forward_client_ =
      create_client<std_srvs::srv::Trigger>("/peach/manipulation/acknowledge_recovery");
    ack_srv_ = create_service<std_srvs::srv::Trigger>(
      "/peach/task/acknowledge_recovery",
      [this](const std::shared_ptr<rmw_request_id_t> header,
      const std::shared_ptr<std_srvs::srv::Trigger::Request> req) {on_acknowledge(header, req);});

    run_batch_server_ = rclcpp_action::create_server<RunBatch>(
      this, "/peach/task/run_batch",
      std::bind(&TaskNode::handle_goal, this, _1, _2),
      std::bind(&TaskNode::handle_cancel, this, _1),
      std::bind(&TaskNode::handle_accepted, this, _1));

    heartbeat_timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&TaskNode::heartbeat, this));

    diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this, 1.0);
    diagnostics_->setHardwareID("peach2_task");
    diagnostics_->add("batch", this, &TaskNode::diagnose_batch);
    diagnostics_->add("servers", this, &TaskNode::diagnose_servers);
    diagnostics_->add("safety", this, &TaskNode::diagnose_safety);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "configure failed: %s", e.what());
    release();
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(
    get_logger(), "configured: tree=%s runs_dir=%s", tree_file_.c_str(), runs_dir_.c_str());
  return CallbackReturn::SUCCESS;
}

TaskNode::CallbackReturn TaskNode::on_activate(const rclcpp_lifecycle::State &)
{
  enables_pub_->on_activate();
  state_pub_->on_activate();
  active_ = true;
  set_enables(core::Enables{});
  publish_state(true);
  if (!bond_) {
    bond_ = std::make_unique<bond::Bond>("/bond", get_name(), shared_from_this());
    bond_->setHeartbeatTimeout(param_listener_->get_params().bond_heartbeat_timeout_s);
    bond_->setHeartbeatPeriod(0.1);
    bond_->start();
  }
  RCLCPP_INFO(get_logger(), "active: enables all false; RunBatch is operator-initiated only");
  return CallbackReturn::SUCCESS;
}

TaskNode::CallbackReturn TaskNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  abort_running_batch("deactivated");
  set_enables(core::Enables{});
  active_ = false;
  enables_pub_->on_deactivate();
  state_pub_->on_deactivate();
  if (bond_) {
    bond_->breakBond();
    bond_.reset();
  }
  return CallbackReturn::SUCCESS;
}

TaskNode::CallbackReturn TaskNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  release();
  return CallbackReturn::SUCCESS;
}

TaskNode::CallbackReturn TaskNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  abort_running_batch("shutdown");
  release();
  return CallbackReturn::SUCCESS;
}

TaskNode::CallbackReturn TaskNode::on_error(const rclcpp_lifecycle::State &)
{
  abort_running_batch("lifecycle_error");
  release();
  return CallbackReturn::SUCCESS;
}

void TaskNode::release()
{
  try {
    if (tree_) {
      tree_->haltTree();
    }
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(get_logger(), "halt during release failed: %s", e.what());
  }
  tree_.reset();
  goal_.reset();
  if (bond_) {
    try {
      bond_->breakBond();
    } catch (const std::exception &) {
    }
    bond_.reset();
  }
  tick_timer_.reset();
  heartbeat_timer_.reset();
  diagnostics_.reset();
  run_batch_server_.reset();
  pending_acks_.clear();
  ack_srv_.reset();
  ack_forward_client_.reset();
  set_enables_srv_.reset();
  obs_sub_.reset();
  robot_status_sub_.reset();
  tool_state_sub_.reset();
  peer_recovery_sub_.reset();
  enables_pub_.reset();
  state_pub_.reset();
  factory_.reset();
  session_.reset();
  ctx_.reset();
  active_ = false;
}

void TaskNode::refresh_context(const Params & p)
{
  ctx_->safety.require_robot_status = p.safety.require_robot_status;
  ctx_->safety.robot_status_max_age_s = p.safety.robot_status_max_age_s;
  ctx_->timeouts.server_wait_s = p.timeouts.server_wait_s;
  ctx_->timeouts.move_to_s = p.timeouts.move_to_s;
  ctx_->timeouts.snapshot_s = p.timeouts.snapshot_s;
  ctx_->timeouts.begin_scene_s = p.timeouts.begin_scene_s;
  ctx_->timeouts.lock_wait_s = p.timeouts.lock_wait_s;
  ctx_->timeouts.reachability_s = p.timeouts.reachability_s;
  ctx_->timeouts.observe_s = p.timeouts.observe_s;
  ctx_->timeouts.decision_s = p.timeouts.decision_s;
  ctx_->timeouts.harvest_s = p.timeouts.harvest_s;
}

std::filesystem::path TaskNode::resolve_runs_dir(const Params & p) const
{
  if (!p.runs_dir.empty()) {
    return p.runs_dir;
  }
  if (const char * env = std::getenv("PEACH_RUNS_DIR"); env != nullptr && env[0] != '\0') {
    return env;
  }
  return std::filesystem::current_path() / "runs";
}

std::filesystem::path TaskNode::resolve_tree_file(const Params & p) const
{
  if (!p.tree_file.empty()) {
    return p.tree_file;
  }
  return std::filesystem::path(ament_index_cpp::get_package_share_directory("peach2_task")) /
         "trees" / "harvest_batch.xml";
}

// ------------------------------------------------------------------ run_batch

std::string TaskNode::check_goal(const RunBatch::Goal & goal, core::BatchLimits * limits) const
{
  if (!active_) {
    return "node not active";
  }
  if (goal_) {
    return "another batch is running";
  }
  if (session_->peer_recovery_required()) {
    return "manipulation recovery_required is latched; ACK via /peach/task/acknowledge_recovery";
  }
  core::Intent intent = core::Intent::PREGRASP_ONLY;
  if (!core::intent_from_uint(goal.intent, &intent)) {
    return "invalid intent " + std::to_string(goal.intent);
  }
  if (!goal.request_id.empty()) {
    if (!core::is_safe_request_id(goal.request_id)) {
      return "request_id '" + goal.request_id + "' is not a safe directory name";
    }
    if (core::Ledger::exists(runs_dir_, goal.request_id)) {
      return "request_id '" + goal.request_id + "' already used (runs dir exists)";
    }
  }
  for (const auto & id : goal.target_ids) {
    if (id.empty()) {
      return "empty id in target_ids";
    }
  }
  const Params p = param_listener_->get_params();
  const std::string tool = goal.tool_id.empty() ? p.default_tool_id : goal.tool_id;
  if (tool.empty() && intent != core::Intent::SURVEY_ONLY) {
    return "tool_id required (no default_tool_id configured)";
  }
  if (!goal.tool_id.empty() && !p.default_tool_id.empty() && goal.tool_id != p.default_tool_id) {
    return "tool_id '" + goal.tool_id + "' differs from mounted tool '" + p.default_tool_id + "'";
  }
  limits->max_targets = goal.max_targets;
  limits->target_harvest_ratio = goal.target_harvest_ratio;
  limits->per_target_timeout_s =
    goal.per_target_timeout_s > 0.0 ? goal.per_target_timeout_s : p.per_target_timeout_s;
  limits->empty_survey_limit = static_cast<uint32_t>(p.empty_survey_limit);
  return core::validate(*limits);
}

rclcpp_action::GoalResponse TaskNode::handle_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const RunBatch::Goal> goal)
{
  core::BatchLimits limits;
  const std::string error = check_goal(*goal, &limits);
  if (!error.empty()) {
    RCLCPP_WARN(get_logger(), "RunBatch rejected: %s", error.c_str());
    if (session_ && !session_->active()) {
      session_->set_message("RunBatch rejected: " + error);
      publish_state(true);
    }
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TaskNode::handle_cancel(std::shared_ptr<GoalHandle>)
{
  RCLCPP_INFO(get_logger(), "RunBatch cancel requested");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TaskNode::handle_accepted(std::shared_ptr<GoalHandle> goal_handle)
{
  start_batch(std::move(goal_handle));
}

void TaskNode::start_batch(std::shared_ptr<GoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto fail_now = [&](const std::string & reason) {
      RCLCPP_ERROR(get_logger(), "RunBatch aborted before start: %s", reason.c_str());
      auto result = std::make_shared<RunBatch::Result>();
      result->request_id = goal->request_id;
      result->termination_reason = reason;
      goal_handle->abort(result);
    };
  if (goal_ || !active_) {
    fail_now(goal_ ? "another batch is running" : "node not active");
    return;
  }
  const Params p = param_listener_->get_params();
  core::BatchLimits limits;
  const std::string error = check_goal(*goal, &limits);
  if (!error.empty()) {
    fail_now(error);
    return;
  }
  const auto id = core::resolve_request_id(goal->request_id, std::chrono::system_clock::now());
  if (!id.ok) {
    fail_now(id.error);
    return;
  }
  std::string ledger_error;
  auto ledger = core::Ledger::create(runs_dir_, id.id, &ledger_error);
  if (!ledger) {
    fail_now("ledger: " + ledger_error);
    return;
  }

  core::BatchRequest request;
  request.request_id = id.id;
  core::intent_from_uint(goal->intent, &request.intent);
  request.tool_id = goal->tool_id.empty() ? p.default_tool_id : goal->tool_id;
  request.target_ids = goal->target_ids;
  request.limits = limits;

  core::SessionConfig config;
  config.selection.min_mask_quality = static_cast<float>(p.selection.min_mask_quality);
  config.selection.min_depth_coverage = static_cast<float>(p.selection.min_depth_coverage);
  config.selection.depth_min_m = p.selection.depth_min_m;
  config.selection.depth_max_m = p.selection.depth_max_m;
  config.selection.height_band_m = p.selection.height_band_m;
  config.selection.require_reachability = p.selection.require_reachability;
  config.ack_each_pregrasp = p.ack_each_pregrasp;
  config.wait_retry_s = p.wait_retry_s;
  refresh_context(p);

  session_->begin(request, config, std::move(ledger));
  auto bb = BT::Blackboard::create();
  bb->set<int>("intent", static_cast<int>(goal->intent));
  bb->set<std::string>("tool_id", request.tool_id);
  bb->set<std::string>("photo_pose", p.photo_pose);
  bb->set<unsigned>("max_views", static_cast<unsigned>(p.observe_max_views));
  bb->set<unsigned>("retry_attempts", static_cast<unsigned>(p.retry_attempts));
  try {
    tree_ = std::make_unique<BT::Tree>(factory_->createTree("HarvestBatch", bb));
  } catch (const std::exception & e) {
    goal_ = goal_handle;
    session_->abort(std::string("tree_error:") + e.what());
    end_batch(core::BatchEnd::FAILED);
    return;
  }
  goal_ = goal_handle;
  published_revision_ = 0;
  RCLCPP_INFO(
    get_logger(), "RunBatch %s started: intent=%s tool=%s targets=%zu max=%u ratio=%.2f",
    request.request_id.c_str(), core::intent_name(request.intent), request.tool_id.c_str(),
    request.target_ids.size(), limits.max_targets, limits.target_harvest_ratio);
  const double hz = p.tick_hz;
  tick_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / hz)),
    std::bind(&TaskNode::tick, this));
  publish_state(true);
}

void TaskNode::tick()
{
  if (!tree_ || !goal_) {
    return;
  }
  if (goal_->is_canceling()) {
    tree_->haltTree();
    end_batch(core::BatchEnd::CANCELED);
    return;
  }
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  try {
    status = tree_->tickOnce();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "tree error: %s", e.what());
    try {
      tree_->haltTree();
    } catch (const std::exception &) {
    }
    session_->abort(std::string("tree_error:") + e.what());
    end_batch(core::BatchEnd::FAILED);
    return;
  }
  publish_state(false);
  if (status == BT::NodeStatus::SUCCESS) {
    end_batch(core::BatchEnd::SUCCEEDED);
  } else if (status == BT::NodeStatus::FAILURE) {
    end_batch(core::BatchEnd::FAILED);
  }
}

void TaskNode::end_batch(core::BatchEnd end)
{
  const std::string reason = session_->finish(end);
  auto result = std::make_shared<RunBatch::Result>();
  result->request_id = session_->request().request_id;
  result->termination_reason = reason;
  const auto & counts = session_->counts();
  result->discovered = counts.discovered;
  result->attempted = counts.attempted;
  result->succeeded = counts.succeeded;
  result->skipped = counts.skipped;
  result->failed = counts.failed;
  for (const auto & r : session_->results()) {
    result->results.push_back(ros::to_msg(r));
  }
  if (session_->ledger_write_failures() > 0) {
    RCLCPP_ERROR(
      get_logger(), "ledger write failed %u time(s): %s", session_->ledger_write_failures(),
      session_->last_ledger_error().c_str());
  }
  if (goal_) {
    try {
      switch (end) {
        case core::BatchEnd::SUCCEEDED: goal_->succeed(result); break;
        case core::BatchEnd::CANCELED: goal_->canceled(result); break;
        case core::BatchEnd::FAILED: goal_->abort(result); break;
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "could not report RunBatch result: %s", e.what());
    }
  }
  RCLCPP_INFO(
    get_logger(),
    "RunBatch %s finished: %s (attempted=%u succeeded=%u skipped=%u failed=%u plan_only=%u)",
    result->request_id.c_str(), reason.c_str(), counts.attempted, counts.succeeded,
    counts.skipped, counts.failed, counts.plan_only);
  goal_.reset();
  tick_timer_.reset();
  tree_.reset();
  publish_state(true);
}

void TaskNode::abort_running_batch(const std::string & reason)
{
  if (!tree_ || !goal_ || !session_) {
    return;
  }
  try {
    tree_->haltTree();
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "halt failed: %s", e.what());
  }
  session_->abort(reason);
  end_batch(core::BatchEnd::FAILED);
}

// ------------------------------------------------------------------ enables / ack

void TaskNode::on_set_enables(
  const std::shared_ptr<srv::SetEnables::Request> request,
  std::shared_ptr<srv::SetEnables::Response> response)
{
  if (!active_) {
    response->accepted = false;
    response->message = "node not active";
    return;
  }
  const core::Enables wanted{request->execution, request->grasp, request->tool};
  const std::string error = core::validate_chain(wanted);
  if (!error.empty()) {
    response->accepted = false;
    response->message = error;
    return;
  }
  set_enables(wanted);
  response->accepted = true;
  response->message = "execution=" + std::to_string(wanted.execution) +
    " grasp=" + std::to_string(wanted.grasp) + " tool=" + std::to_string(wanted.tool);
  RCLCPP_INFO(get_logger(), "enables set: %s", response->message.c_str());
}

void TaskNode::set_enables(const core::Enables & enables)
{
  enables_ = enables;
  if (ctx_) {
    ctx_->enables = enables;
  }
  publish_enables();
}

void TaskNode::publish_enables()
{
  if (!enables_pub_ || !enables_pub_->is_activated()) {
    return;
  }
  msg::Enables m;
  m.header.stamp = now();
  m.seq = ++enables_seq_;
  m.execution = enables_.execution;
  m.grasp = enables_.grasp;
  m.tool = enables_.tool;
  enables_pub_->publish(m);
}

void TaskNode::on_acknowledge(
  const std::shared_ptr<rmw_request_id_t> header,
  const std::shared_ptr<std_srvs::srv::Trigger::Request>)
{
  if (!ack_forward_client_->service_is_ready()) {
    std_srvs::srv::Trigger::Response response;
    response.success = false;
    response.message = "/peach/manipulation/acknowledge_recovery not available";
    ack_srv_->send_response(*header, response);
    return;
  }
  auto forwarded = ack_forward_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>(),
    [this, header](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      const auto response = future.get();
      pending_acks_.erase(
        std::remove_if(
          pending_acks_.begin(), pending_acks_.end(),
          [&header](const PendingAck & p) {return p.header == header;}),
        pending_acks_.end());
      std_srvs::srv::Trigger::Response out = *response;
      if (response->success && session_ && session_->active() && session_->grant_ack()) {
        std::string task_state = "batch released. ";
        if (session_->peer_recovery_required()) {
          task_state = "ACK recorded; waiting for manipulation recovery_required=false. ";
        }
        out.message = task_state + "manipulation: " + response->message;
        RCLCPP_INFO(get_logger(), "operator ACK accepted");
        publish_state(true);
      }
      if (ack_srv_) {
        ack_srv_->send_response(*header, out);
      }
    });
  const double timeout_s = param_listener_->get_params().timeouts.ack_forward_s;
  pending_acks_.push_back(
    PendingAck{header, forwarded.request_id,
      SteadyClock::now() + std::chrono::duration_cast<SteadyClock::duration>(
        std::chrono::duration<double>(timeout_s))});
}

void TaskNode::expire_pending_acks()
{
  const auto now_s = SteadyClock::now();
  for (auto it = pending_acks_.begin(); it != pending_acks_.end(); ) {
    if (now_s < it->deadline) {
      ++it;
      continue;
    }
    ack_forward_client_->remove_pending_request(it->forward_id);
    std_srvs::srv::Trigger::Response response;
    response.success = false;
    response.message = "/peach/manipulation/acknowledge_recovery timed out";
    ack_srv_->send_response(*it->header, response);
    it = pending_acks_.erase(it);
  }
}

// ------------------------------------------------------------------ state / diagnostics

void TaskNode::heartbeat()
{
  if (!active_) {
    return;
  }
  publish_enables();
  expire_pending_acks();
  if (session_ && !session_->active()) {
    session_->set_safety_blockers(idle_safety().blockers);
  }
  publish_state(true);
}

core::SafetyVerdict TaskNode::idle_safety() const
{
  core::SafetyInputs in;
  in.robot = ctx_->robot_sample(SteadyClock::now());
  in.enables = enables_;
  in.intent = session_ && session_->active() ?
    session_->request().intent : core::Intent::PREGRASP_ONLY;
  in.tool_fault = ctx_->tool_fault();
  return core::evaluate_safety(ctx_->safety, in);
}

void TaskNode::publish_state(bool force)
{
  if (!state_pub_ || !state_pub_->is_activated() || !session_) {
    return;
  }
  const auto now_s = SteadyClock::now();
  if (!force) {
    if (session_->revision() == published_revision_) {
      return;
    }
    if (std::chrono::duration<double>(now_s - last_state_publish_).count() <
      kStatePublishMinPeriodS)
    {
      return;
    }
  }
  published_revision_ = session_->revision();
  last_state_publish_ = now_s;
  const auto state = ros::to_msg(session_->snapshot(), now());
  state_pub_->publish(state);
  if (goal_ && session_->active()) {
    auto feedback = std::make_shared<RunBatch::Feedback>();
    feedback->state = state;
    goal_->publish_feedback(feedback);
  }
}

void TaskNode::diagnose_batch(diagnostic_updater::DiagnosticStatusWrapper & st)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  if (!session_) {
    st.summary(DiagnosticStatus::STALE, "not configured");
    return;
  }
  const auto snap = session_->snapshot();
  st.add("request_id", snap.request_id);
  st.add("phase", core::phase_name(snap.phase));
  st.add("current_target", snap.current_target_id);
  st.add("attempted", snap.counts.attempted);
  st.add("succeeded", snap.counts.succeeded);
  st.add("skipped", snap.counts.skipped);
  st.add("failed", snap.counts.failed);
  st.add("plan_only", snap.counts.plan_only);
  st.add("scene_epoch", session_->scene_epoch());
  st.add("manipulation_recovery_required", session_->peer_recovery_required());
  st.add("ledger_write_failures", session_->ledger_write_failures());
  if (session_->ledger_write_failures() > 0) {
    st.summary(DiagnosticStatus::ERROR, "ledger write failed: " + session_->last_ledger_error());
  } else if (snap.recovery_required || session_->peer_recovery_required()) {
    st.summary(DiagnosticStatus::WARN, "waiting for operator ACK");
  } else if (session_->active()) {
    st.summary(DiagnosticStatus::OK, std::string("running: ") + core::phase_name(snap.phase));
  } else {
    st.summary(DiagnosticStatus::OK, "idle");
  }
}

void TaskNode::diagnose_servers(diagnostic_updater::DiagnosticStatusWrapper & st)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  if (!ctx_) {
    st.summary(DiagnosticStatus::STALE, "not configured");
    return;
  }
  std::vector<std::string> missing;
  auto check = [&](const char * name, bool ready) {
      st.add(name, ready ? "ready" : "missing");
      if (!ready) {
        missing.emplace_back(name);
      }
    };
  check("move_to", ctx_->move_to->action_server_is_ready());
  check("observe", ctx_->observe->action_server_is_ready());
  check("harvest_target", ctx_->harvest->action_server_is_ready());
  check("build_snapshot", ctx_->snapshot->service_is_ready());
  check("begin_scene", ctx_->begin_scene->service_is_ready());
  check("check_reachability", ctx_->reachability->service_is_ready());
  check("get_decision", ctx_->decision->service_is_ready());
  check("acknowledge_recovery", ack_forward_client_->service_is_ready());
  if (missing.empty()) {
    st.summary(DiagnosticStatus::OK, "all servers ready");
  } else {
    std::string text = "missing:";
    for (const auto & m : missing) {
      text += " " + m;
    }
    st.summary(DiagnosticStatus::WARN, text);
  }
}

void TaskNode::diagnose_safety(diagnostic_updater::DiagnosticStatusWrapper & st)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  if (!ctx_) {
    st.summary(DiagnosticStatus::STALE, "not configured");
    return;
  }
  st.add("execution", enables_.execution);
  st.add("grasp", enables_.grasp);
  st.add("tool", enables_.tool);
  st.add("enables_seq", enables_seq_);
  st.add("robot_status_received", ctx_->robot.received);
  const auto verdict = idle_safety();
  if (verdict.ok) {
    st.summary(DiagnosticStatus::OK, "gate open");
  } else {
    st.summary(DiagnosticStatus::WARN, "gate closed: " + verdict.reason());
  }
}

}  // namespace peach2_task
