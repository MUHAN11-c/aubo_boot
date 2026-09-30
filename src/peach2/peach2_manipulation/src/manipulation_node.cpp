#include "manipulation_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit_motion_backend.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/planning_scene_components.hpp"
#include "peach2_end_effector/aubo_io_backend.hpp"
#include "peach2_end_effector/failure_codes.hpp"
#include "peach2_end_effector/io_backend.hpp"
#include "peach2_end_effector/tool_profile.hpp"
#include "peach2_manipulation/conversions.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

namespace peach2_manipulation
{

namespace failure = peach2_end_effector::failure;
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace
{

rclcpp::QoS latched_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

std::chrono::nanoseconds to_period(double seconds)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(seconds));
}

}  // namespace

// ---------------------------------------------------------------- RosDecisionClient

RosDecisionClient::RosDecisionClient(
  rclcpp::Client<peach2_interfaces::srv::GetDecision>::SharedPtr client,
  rclcpp::Clock::SharedPtr clock, double timeout_s)
: client_(std::move(client)), clock_(std::move(clock)), timeout_s_(timeout_s) {}

std::optional<DecisionView> RosDecisionClient::get(
  const std::string & target_id, const std::string & tool_id, uint64_t min_revision)
{
  if (!client_->service_is_ready()) {
    return std::nullopt;
  }
  auto request = std::make_shared<peach2_interfaces::srv::GetDecision::Request>();
  request->target_id = target_id;
  request->tool_id = tool_id;
  request->min_model_revision = min_revision;
  auto future = client_->async_send_request(request);
  if (future.wait_for(std::chrono::duration<double>(timeout_s_)) != std::future_status::ready) {
    client_->remove_pending_request(future);
    return std::nullopt;
  }
  const auto response = future.get();
  if (!response->found) {
    return std::nullopt;
  }
  return decision_from_msg(response->decision);
}

double RosDecisionClient::now_s()
{
  return clock_->now().seconds();
}

// ---------------------------------------------------------------- ModelCache

void ModelCache::update(const peach2_interfaces::msg::TargetModelArray & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & model : msg.models) {
    auto g = geometry_from_msg(model);
    if (g) {
      models_[model.target_id] = *g;
    } else {
      models_.erase(model.target_id);
    }
  }
}

std::optional<peach2_end_effector::TargetGeometry> ModelCache::get(const std::string & target_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = models_.find(target_id);
  if (it == models_.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::vector<peach2_end_effector::TargetGeometry> ModelCache::all()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<peach2_end_effector::TargetGeometry> out;
  out.reserve(models_.size());
  for (const auto & [id, g] : models_) {
    out.push_back(g);
  }
  return out;
}

std::size_t ModelCache::size()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return models_.size();
}

// ---------------------------------------------------------------- ManipulationNode

ManipulationNode::ManipulationNode(const rclcpp::NodeOptions & options)
: LifecycleNode("peach2_manipulation", options)
{
  // Same overrides as this node so robot_description / pipelines reach MoveGroupInterface;
  // parameter services off so the two nodes never race for one service name.
  rclcpp::NodeOptions moveit_options;
  moveit_options.use_global_arguments(options.use_global_arguments());
  moveit_options.parameter_overrides(options.parameter_overrides());
  moveit_options.automatically_declare_parameters_from_overrides(true);
  moveit_options.start_parameter_services(false);
  moveit_options.start_parameter_event_publisher(false);
  moveit_node_ = std::make_shared<rclcpp::Node>("peach2_manipulation_moveit", moveit_options);

  param_listener_ = std::make_shared<ParamListener>(get_node_parameters_interface(), get_logger());
  params_ = param_listener_->get_params();
}

ManipulationNode::~ManipulationNode()
{
  stop_worker("destroyed");
  release();
}

double ManipulationNode::steady_now() const
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

ManipulationNode::CallbackReturn ManipulationNode::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    params_ = param_listener_->get_params();
    const auto & p = params_;
    if (p.io.cmd_pin == p.io.feedback_pin) {
      throw std::invalid_argument("io.feedback_pin must differ from io.cmd_pin");
    }
    if (p.plugin_tool_ids.size() != p.plugin_classes.size()) {
      throw std::invalid_argument("plugin_tool_ids and plugin_classes differ in length");
    }

    GateConfig gate_config;
    gate_config.require_robot_status = p.require_robot_status;
    gate_config.robot_status_max_age_s = p.robot_status_max_age_s;
    gate_config.enables_timeout_s = p.enables_timeout_s;
    {
      std::lock_guard<std::mutex> lock(gate_mutex_);
      gate_ = CommandGate(gate_config);
      execution_edge_.reset();
      last_io_close_ = false;
    }

    state_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    action_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions state_options;
    state_options.callback_group = state_group_;
    enables_sub_ = create_subscription<peach2_interfaces::msg::Enables>(
      "/peach/enables", latched_qos(),
      std::bind(&ManipulationNode::on_enables, this, _1), state_options);
    robot_status_sub_ = create_subscription<aubo_msgs::msg::RobotStatus>(
      "/aubo_io_controller/robot_status", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
      std::bind(&ManipulationNode::on_robot_status, this, _1), state_options);
    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&ManipulationNode::on_joint_states, this, _1), state_options);
    models_sub_ = create_subscription<peach2_interfaces::msg::TargetModelArray>(
      "/peach/target_model/models", latched_qos(),
      std::bind(&ManipulationNode::on_models, this, _1), state_options);

    tool_state_pub_ = create_publisher<peach2_interfaces::msg::ToolState>(
      "/peach/end_effector/tool_state", latched_qos());
    recovery_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/peach/manipulation/recovery_required", latched_qos());
    {
      std::lock_guard<std::mutex> lock(recovery_pub_mutex_);
      recovery_published_.reset();
    }

    decision_client_ = create_client<peach2_interfaces::srv::GetDecision>(
      "/peach/target_model/get_decision", rclcpp::ServicesQoS(), client_group_);
    apply_scene_client_ = create_client<moveit_msgs::srv::ApplyPlanningScene>(
      "/apply_planning_scene", rclcpp::ServicesQoS(), client_group_);
    get_scene_client_ = create_client<moveit_msgs::srv::GetPlanningScene>(
      "/get_planning_scene", rclcpp::ServicesQoS(), client_group_);
    {
      std::lock_guard<std::mutex> lock(scene_mutex_);
      bag_objects_.forget_all();
      scene_adopted_ = false;
    }
    decisions_ = std::make_unique<RosDecisionClient>(
      decision_client_, get_clock(), p.timeouts.decision_s);

    build_end_effector(p);

    MoveItBackendConfig mc;
    mc.group = p.moveit.group;
    mc.tip_link = p.moveit.tip_link;
    mc.base_frame = p.moveit.base_frame;
    mc.free_pipeline = p.moveit.free_pipeline;
    mc.free_planner_id = p.moveit.free_planner_id;
    mc.linear_pipeline = p.moveit.linear_pipeline;
    mc.linear_planner_id = p.moveit.linear_planner_id;
    mc.planning_time_s = p.moveit.planning_time_s;
    mc.planning_attempts = static_cast<int>(p.moveit.planning_attempts);
    mc.cartesian_max_trans_vel_mps = p.moveit.cartesian_max_trans_vel_mps;
    mc.wait_for_servers_s = p.moveit.wait_for_servers_s;
    mc.stop_grace_s = p.timeouts.stop_grace_s;
    mc.validate_stride = static_cast<int>(p.moveit.validate_stride);
    mc.at_goal_tolerance_rad = p.at_goal_tolerance_rad;
    motion_ = std::make_shared<MoveItMotionBackend>(moveit_node_, mc);

    reachability_srv_ = create_service<peach2_interfaces::srv::CheckReachability>(
      "/peach/manipulation/check_reachability",
      std::bind(&ManipulationNode::on_check_reachability, this, _1, _2),
      rclcpp::ServicesQoS(), service_group_);
    ack_srv_ = create_service<std_srvs::srv::Trigger>(
      "/peach/manipulation/acknowledge_recovery",
      std::bind(&ManipulationNode::on_acknowledge_recovery, this, _1, _2),
      rclcpp::ServicesQoS(), service_group_);

    harvest_server_ = rclcpp_action::create_server<HarvestTarget>(
      this, "/peach/manipulation/harvest_target",
      std::bind(&ManipulationNode::handle_harvest_goal, this, _1, _2),
      [this](const std::shared_ptr<HarvestGoalHandle> h) {return handle_cancel(h);},
      std::bind(&ManipulationNode::handle_harvest_accepted, this, _1),
      rcl_action_server_get_default_options(), action_group_);
    move_to_server_ = rclcpp_action::create_server<MoveTo>(
      this, "/peach/manipulation/move_to",
      std::bind(&ManipulationNode::handle_move_to_goal, this, _1, _2),
      [this](const std::shared_ptr<MoveToGoalHandle> h) {return handle_cancel(h);},
      std::bind(&ManipulationNode::handle_move_to_accepted, this, _1),
      rcl_action_server_get_default_options(), action_group_);

    gate_timer_ = create_wall_timer(
      to_period(p.gate_monitor_period_s), std::bind(&ManipulationNode::on_gate_timer, this),
      timer_group_);
    tool_timer_ = create_wall_timer(
      to_period(p.tool_state_period_s), std::bind(&ManipulationNode::on_tool_timer, this),
      timer_group_);

    diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this);
    diagnostics_->setHardwareID("peach2_manipulation");
    diagnostics_->add("gate", this, &ManipulationNode::diagnose_gate);
    diagnostics_->add("tool", this, &ManipulationNode::diagnose_tool);
    diagnostics_->add("cycle", this, &ManipulationNode::diagnose_cycle);
    diagnostics_->add("inputs", this, &ManipulationNode::diagnose_inputs);
    diagnostics_->add("scene", this, &ManipulationNode::diagnose_scene);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "configure failed: %s", e.what());
    release();
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(
    get_logger(), "configured: tool=%s io=%s require_robot_status=%s (plan-only until enabled)",
    params_.tool_id.c_str(), params_.io_backend.c_str(),
    params_.require_robot_status ? "true" : "false");
  return CallbackReturn::SUCCESS;
}

void ManipulationNode::build_end_effector(const Params & p)
{
  const auto it = std::find(p.plugin_tool_ids.begin(), p.plugin_tool_ids.end(), p.tool_id);
  if (it == p.plugin_tool_ids.end()) {
    throw std::invalid_argument("tool_id '" + p.tool_id + "' has no plugin mapping");
  }
  const std::string plugin_class =
    p.plugin_classes[static_cast<std::size_t>(it - p.plugin_tool_ids.begin())];
  const std::string config_dir = p.tool_description_dir.empty() ?
    ament_index_cpp::get_package_share_directory("aubo_description") + "/config" :
    p.tool_description_dir;
  auto profile = peach2_end_effector::load_tool_profile(p.tool_id, config_dir);
  {
    std::lock_guard<std::mutex> lock(gate_mutex_);
    close_level_ = profile.io.close_state > 0.5;
  }

  if (p.io_backend == "aubo") {
    peach2_end_effector::AuboIoBackend::Config io_config;
    io_config.fun = profile.io.fun;
    io_config.cmd_pin = static_cast<int>(p.io.cmd_pin);
    io_config.feedback_pin = static_cast<int>(p.io.feedback_pin);
    io_config.service_timeout = std::chrono::milliseconds(
      static_cast<int64_t>(p.io.service_timeout_s * 1000.0));
    raw_io_ = std::make_shared<peach2_end_effector::AuboIoBackend>(
      peach2_end_effector::AuboIoBackend::NodeInterfaces(*this), io_config, client_group_);
  } else {
    peach2_end_effector::MockIoBackend::Config mock;
    mock.cmd_pin = static_cast<int>(p.io.cmd_pin);
    mock.feedback_pin = static_cast<int>(p.io.feedback_pin);
    mock.close_level = profile.io.close_state > 0.5;
    mock.feedback_active_high = p.io.feedback_active_high;
    mock.simulate_current = p.io.current_signature.enabled;
    raw_io_ = std::make_shared<peach2_end_effector::MockIoBackend>(
      mock, [this]() {return steady_now();});
  }
  auto gated = std::make_shared<peach2_end_effector::GatedIoBackend>(
    raw_io_, [this](int pin, bool level) {return permit_io(pin, level);});

  loader_ = std::make_unique<pluginlib::ClassLoader<peach2_end_effector::EndEffector>>(
    "peach2_end_effector", "peach2_end_effector::EndEffector");
  ee_ = loader_->createSharedInstance(plugin_class);

  peach2_end_effector::EndEffectorContext ctx;
  ctx.profile = profile;
  ctx.io = gated;
  ctx.pins.cmd_pin = static_cast<int>(p.io.cmd_pin);
  ctx.pins.feedback_pin = static_cast<int>(p.io.feedback_pin);
  ctx.pins.feedback_active_high = p.io.feedback_active_high;
  ctx.timing.min_actuation_s = p.io.min_actuation_s;
  ctx.timing.poll_period_s = p.io.poll_period_s;
  ctx.timing.max_close_energized_s = p.io.max_close_energized_s;
  ctx.current.enabled = p.io.current_signature.enabled;
  ctx.current.peak_min_a = p.io.current_signature.peak_min_a;
  ctx.current.drop_ratio = p.io.current_signature.drop_ratio;
  ctx.current.window_s = p.io.current_signature.window_s;
  ctx.now_s = [this]() {return steady_now();};
  ctx.sleep_s = [](double s) {
      std::this_thread::sleep_for(std::chrono::duration<double>(std::max(0.0, s)));
    };
  auto logger = get_logger();
  ctx.log = [logger](const std::string & msg) {RCLCPP_INFO(logger, "%s", msg.c_str());};
  ee_->initialize(ctx);
  RCLCPP_INFO(
    get_logger(), "end effector %s loaded (%s), blade %.3f m behind TCP, state UNKNOWN",
    p.tool_id.c_str(), plugin_class.c_str(), profile.geometry.l_blade);
}

ManipulationNode::CallbackReturn ManipulationNode::on_activate(const rclcpp_lifecycle::State &)
{
  tool_state_pub_->on_activate();
  recovery_pub_->on_activate();
  publish_recovery_state(true);
  {
    std::lock_guard<std::mutex> lock(gate_mutex_);
    gate_.set_cancel(false);
    gate_.set_active(true);
    execution_edge_.reset();
  }
  if (!bond_) {
    bond_ = std::make_unique<bond::Bond>("/bond", get_name(), shared_from_this());
    bond_->setHeartbeatTimeout(params_.bond_heartbeat_timeout_s);
    bond_->setHeartbeatPeriod(0.1);
    bond_->start();
  }
  RCLCPP_INFO(get_logger(), "active: no motion or tool IO until a goal passes the command gate");
  return CallbackReturn::SUCCESS;
}

ManipulationNode::CallbackReturn ManipulationNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  stop_worker("deactivated");
  clear_bag_obstacles();
  if (tool_state_pub_) {
    tool_state_pub_->on_deactivate();
  }
  if (recovery_pub_) {
    recovery_pub_->on_deactivate();
  }
  {
    std::lock_guard<std::mutex> lock(recovery_pub_mutex_);
    recovery_published_.reset();
  }
  if (bond_) {
    bond_->breakBond();
    bond_.reset();
  }
  return CallbackReturn::SUCCESS;
}

ManipulationNode::CallbackReturn ManipulationNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  clear_bag_obstacles();
  release();
  return CallbackReturn::SUCCESS;
}

ManipulationNode::CallbackReturn ManipulationNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  stop_worker("shutdown");
  clear_bag_obstacles();
  release();
  return CallbackReturn::SUCCESS;
}

ManipulationNode::CallbackReturn ManipulationNode::on_error(const rclcpp_lifecycle::State &)
{
  stop_worker("lifecycle_error");
  clear_bag_obstacles();
  release();
  return CallbackReturn::SUCCESS;
}

void ManipulationNode::stop_worker(const std::string & why)
{
  {
    std::lock_guard<std::mutex> lock(gate_mutex_);
    gate_.set_active(false);
  }
  cancel_requested_ = true;
  if (motion_ && busy_) {
    RCLCPP_WARN(get_logger(), "stopping in-flight motion: %s", why.c_str());
    motion_->stop();
  }
  std::lock_guard<std::mutex> lock(worker_mutex_);
  if (worker_.joinable()) {
    worker_.join();
  }
}

void ManipulationNode::release()
{
  if (bond_) {
    try {
      bond_->breakBond();
    } catch (const std::exception &) {
    }
    bond_.reset();
  }
  gate_timer_.reset();
  tool_timer_.reset();
  diagnostics_.reset();
  harvest_server_.reset();
  move_to_server_.reset();
  reachability_srv_.reset();
  ack_srv_.reset();
  motion_.reset();
  ee_.reset();
  loader_.reset();
  raw_io_.reset();
  decisions_.reset();
  decision_client_.reset();
  apply_scene_client_.reset();
  get_scene_client_.reset();
  tool_state_pub_.reset();
  recovery_pub_.reset();
  enables_sub_.reset();
  robot_status_sub_.reset();
  joint_states_sub_.reset();
  models_sub_.reset();
}

// ---------------------------------------------------------------- gate

GateVerdict ManipulationNode::gate_check(GateStage stage, bool new_trajectory)
{
  const double now = steady_now();
  GateVerdict verdict;
  {
    std::lock_guard<std::mutex> lock(gate_mutex_);
    verdict = gate_.check(stage, now, new_trajectory);
  }
  const bool motion_stage = stage == GateStage::TRANSIT || stage == GateStage::APPROACH ||
    stage == GateStage::CONTACT || stage == GateStage::RETREAT;
  if (verdict.open && new_trajectory && motion_stage) {
    const double received = joint_states_received_s_.load();
    if (received < 0.0 || now - received > params_.joint_states_max_age_s) {
      verdict.open = false;
      verdict.failure_code = failure::ROBOT_NOT_READY;
      verdict.reason = received < 0.0 ? "joint_states_missing" : "joint_states_stale";
    }
  }
  return verdict;
}

EffectiveEnables ManipulationNode::current_enables()
{
  std::lock_guard<std::mutex> lock(gate_mutex_);
  return gate_.enables(steady_now());
}

bool ManipulationNode::permit_io(int pin, bool level)
{
  if (pin != params_.io.cmd_pin) {
    RCLCPP_ERROR(get_logger(), "tool write to pin %d refused (cmd_pin %ld)", pin,
      params_.io.cmd_pin);
    return false;
  }
  std::lock_guard<std::mutex> lock(gate_mutex_);
  const double now = steady_now();
  const bool closing = level == close_level_;
  bool allowed = false;
  std::string reason;
  if (closing) {
    const auto v = gate_.check(GateStage::TOOL, now, false);
    allowed = v.open;
    reason = v.reason;
  } else {
    // Opening is the safe direction: allowed without perception permission, but only when the
    // tool chain is enabled or it undoes our own close (abort / release after a cut).
    const auto v = gate_.check(GateStage::TOOL_SAFE, now, false);
    allowed = v.open && (gate_.enables(now).tool || last_io_close_);
    reason = v.open ? "tool_disabled" : v.reason;
  }
  if (!allowed) {
    RCLCPP_WARN(
      get_logger(), "tool %s refused by command gate: %s", closing ? "close" : "open",
      reason.c_str());
    return false;
  }
  last_io_close_ = closing;
  return true;
}

void ManipulationNode::latch_recovery(const std::string & reason)
{
  recovery_required_ = true;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (recovery_reason_.empty()) {
      recovery_reason_ = reason;
    }
  }
  RCLCPP_ERROR(get_logger(), "recovery required: %s (operator ACK needed)", reason.c_str());
  publish_recovery_state(false);
}

void ManipulationNode::publish_recovery_state(bool force)
{
  std::lock_guard<std::mutex> lock(recovery_pub_mutex_);
  if (!recovery_pub_ || !recovery_pub_->is_activated()) {
    return;
  }
  const bool value = recovery_required_.load();
  if (!force && recovery_published_ && *recovery_published_ == value) {
    return;
  }
  std_msgs::msg::Bool msg;
  msg.data = value;
  recovery_pub_->publish(msg);
  recovery_published_ = value;
}

// ---------------------------------------------------------------- neighbour-bag scene

bool ManipulationNode::fetch_scene_object_ids(std::vector<std::string> & ids, std::string * why)
{
  const auto timeout = std::chrono::duration<double>(params_.timeouts.scene_s);
  if (!get_scene_client_ || !get_scene_client_->wait_for_service(timeout)) {
    *why = "get_planning_scene_unavailable";
    return false;
  }
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components = moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES;
  auto future = get_scene_client_->async_send_request(request);
  if (future.wait_for(timeout) != std::future_status::ready) {
    get_scene_client_->remove_pending_request(future);
    *why = "get_planning_scene_timeout";
    return false;
  }
  for (const auto & object : future.get()->scene.world.collision_objects) {
    ids.push_back(object.id);
  }
  return true;
}

bool ManipulationNode::apply_bag_diff(const BagSceneDiff & diff, std::string * why)
{
  if (diff.empty()) {
    return true;
  }
  const auto timeout = std::chrono::duration<double>(params_.timeouts.scene_s);
  if (!apply_scene_client_ || !apply_scene_client_->wait_for_service(timeout)) {
    *why = "apply_planning_scene_unavailable";
    return false;
  }
  auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  auto & scene = request->scene;
  scene.is_diff = true;
  scene.robot_state.is_diff = true;
  const std::string & frame = params_.moveit.base_frame;
  for (const auto & c : diff.upsert) {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = frame;
    object.id = c.id;
    const Eigen::Vector3d center = c.center();
    const Eigen::Quaterniond q = c.orientation();
    object.pose.position.x = center.x();
    object.pose.position.y = center.y();
    object.pose.position.z = center.z();
    object.pose.orientation.w = q.w();
    object.pose.orientation.x = q.x();
    object.pose.orientation.y = q.y();
    object.pose.orientation.z = q.z();
    shape_msgs::msg::SolidPrimitive cylinder;
    cylinder.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    cylinder.dimensions.resize(2);
    cylinder.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = c.length_m();
    cylinder.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = c.radius_m;
    geometry_msgs::msg::Pose identity;
    identity.orientation.w = 1.0;
    object.primitives.push_back(cylinder);
    object.primitive_poses.push_back(identity);
    // ADD replaces an object with the same id.
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    scene.world.collision_objects.push_back(object);
  }
  for (const auto & id : diff.remove) {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = frame;
    object.id = id;
    object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    scene.world.collision_objects.push_back(object);
  }
  auto future = apply_scene_client_->async_send_request(request);
  if (future.wait_for(timeout) != std::future_status::ready) {
    apply_scene_client_->remove_pending_request(future);
    *why = "apply_planning_scene_timeout";
    return false;
  }
  if (!future.get()->success) {
    *why = "apply_planning_scene_rejected";
    return false;
  }
  bag_objects_.commit(diff);
  return true;
}

bool ManipulationNode::sync_bag_obstacles(const std::string & exclude_target_id, std::string * why)
{
  const double margin = param_listener_->get_params().bag_margin_m;
  const auto desired = desired_bag_capsules(models_.all(), margin, exclude_target_id);
  std::string local;
  std::string & reason = why ? *why : local;
  std::lock_guard<std::mutex> lock(scene_mutex_);
  if (!scene_adopted_) {
    // Objects a previous process left behind are replaced or removed on the first sync.
    std::vector<std::string> existing;
    if (!fetch_scene_object_ids(existing, &reason)) {
      ++scene_failures_;
      scene_last_ok_ = false;
      return false;
    }
    bag_objects_.adopt(existing);
    scene_adopted_ = true;
  }
  if (!apply_bag_diff(bag_objects_.diff_to(desired), &reason)) {
    ++scene_failures_;
    scene_last_ok_ = false;
    return false;
  }
  scene_last_ok_ = true;
  return true;
}

void ManipulationNode::restore_bag_obstacles()
{
  try {
    std::string why;
    if (!sync_bag_obstacles("", &why)) {
      RCLCPP_WARN(get_logger(), "bag obstacles not restored: %s", why.c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "bag obstacles not restored: %s", e.what());
  }
}

void ManipulationNode::clear_bag_obstacles()
{
  try {
    std::lock_guard<std::mutex> lock(scene_mutex_);
    std::string why;
    const auto diff = bag_objects_.clear_all();
    if (!apply_bag_diff(diff, &why)) {
      ++scene_failures_;
      RCLCPP_WARN(
        get_logger(), "%zu bag obstacles left in the planning scene: %s", diff.remove.size(),
        why.c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "bag obstacles not cleared: %s", e.what());
  }
}

// ---------------------------------------------------------------- subscriptions / timers

void ManipulationNode::on_enables(const peach2_interfaces::msg::Enables & msg)
{
  std::lock_guard<std::mutex> lock(gate_mutex_);
  gate_.on_enables(enables_from_msg(msg, steady_now()));
}

void ManipulationNode::on_robot_status(const aubo_msgs::msg::RobotStatus & msg)
{
  bool stop_edge = false;
  std::string why;
  {
    std::lock_guard<std::mutex> lock(gate_mutex_);
    const bool was_stopped = last_robot_status_ &&
      (last_robot_status_->e_stopped == 1 || last_robot_status_->in_error == 1);
    const bool stopped = msg.e_stopped == 1 || msg.in_error == 1;
    stop_edge = stopped && !was_stopped;
    why = msg.e_stopped == 1 ? "robot_e_stopped" :
      "robot_in_error:" + std::to_string(msg.error_code);
    last_robot_status_ = msg;
    gate_.on_robot_status(robot_status_from_msg(msg, steady_now()));
  }
  if (stop_edge) {
    // After a hardware stop the blade position and any in-flight goal are untrusted: drop the
    // goal, mark the tool UNKNOWN and wait for the operator (never resume the old trajectory).
    cancel_requested_ = true;
    if (motion_) {
      motion_->stop();
    }
    if (ee_) {
      ee_->mark_unknown(why);
    }
    latch_recovery(why);
  }
}

void ManipulationNode::on_joint_states(const sensor_msgs::msg::JointState &)
{
  joint_states_received_s_ = steady_now();
}

void ManipulationNode::on_models(const peach2_interfaces::msg::TargetModelArray & msg)
{
  models_.update(msg);
  models_received_s_ = steady_now();
}

void ManipulationNode::on_gate_timer()
{
  bool edge = false;
  std::string reason;
  {
    std::lock_guard<std::mutex> lock(gate_mutex_);
    const auto v = gate_.check(GateStage::TRANSIT, steady_now(), false);
    edge = execution_edge_.update(v.open);
    reason = v.reason;
  }
  if (edge && busy_ && motion_) {
    ++gate_stops_;
    RCLCPP_WARN(get_logger(), "command gate closed (%s): stopping motion", reason.c_str());
    motion_->stop();
  }
}

void ManipulationNode::on_tool_timer()
{
  if (!ee_) {
    return;
  }
  ee_->poll();
  const auto status = ee_->status();
  if (status.state == peach2_end_effector::ToolState::FAULT && !recovery_required_) {
    latch_recovery("tool_fault:" + status.fault_reason);
  }
  if (tool_state_pub_ && tool_state_pub_->is_activated()) {
    auto msg = tool_state_to_msg(status);
    msg.header.stamp = now();
    tool_state_pub_->publish(msg);
  }
}

// ---------------------------------------------------------------- cycle wiring

CycleConfig ManipulationNode::cycle_config() const
{
  const auto p = param_listener_->get_params();
  CycleConfig c;
  c.transit_velocity_scaling = p.speed.transit_velocity_scaling;
  c.transit_acceleration_scaling = p.speed.transit_acceleration_scaling;
  c.approach_speed_mps = p.speed.approach_speed_mps;
  c.retreat_speed_mps = p.speed.retreat_speed_mps;
  c.staging_distance_m = p.staging.distance_m;
  c.roll_samples = static_cast<int>(p.staging.roll_samples);
  c.residual.lateral_m = p.pregrasp.lateral_tol_m;
  c.residual.axial_m = p.pregrasp.axial_tol_m;
  c.residual.angle_rad = p.pregrasp.angle_tol_deg * M_PI / 180.0;
  c.max_corrections = static_cast<int>(p.pregrasp.max_corrections);
  c.model_shift_tol_m = p.pregrasp.model_shift_tol_m;
  c.insert_lateral_tol_m = p.pregrasp.insert_lateral_tol_m;
  c.start_tolerance_rad = p.start_tolerance_rad;
  c.release_named_target = p.release_named_target;
  c.pregrasp_only_retreat = p.pregrasp_only_retreat;
  c.cut_retry_max = static_cast<int>(p.cut_retry_max);
  c.confirm_margin_s = p.timeouts.confirm_margin_s;
  c.execute_timeout_scale = p.timeouts.execute_timeout_scale;
  c.execute_timeout_margin_s = p.timeouts.execute_timeout_margin_s;
  return c;
}

MoveToConfig ManipulationNode::move_to_config() const
{
  const auto p = param_listener_->get_params();
  MoveToConfig c;
  c.default_velocity_scaling = p.speed.transit_velocity_scaling;
  c.acceleration_scaling = p.speed.transit_acceleration_scaling;
  c.max_velocity_scaling = p.speed.move_to_max_velocity_scaling;
  c.start_tolerance_rad = p.start_tolerance_rad;
  c.execute_timeout_scale = p.timeouts.execute_timeout_scale;
  c.execute_timeout_margin_s = p.timeouts.execute_timeout_margin_s;
  return c;
}

CycleDeps ManipulationNode::cycle_deps(std::function<bool()> cancel_requested)
{
  CycleDeps deps;
  deps.motion = motion_.get();
  deps.ee = ee_.get();
  deps.decisions = decisions_.get();
  deps.targets = &models_;
  deps.gate = [this](GateStage stage, bool new_trajectory) {
      return gate_check(stage, new_trajectory);
    };
  deps.enables = [this]() {return current_enables();};
  deps.cancel_requested = std::move(cancel_requested);
  deps.now_s = [this]() {return steady_now();};
  deps.sleep_s = [](double s) {
      std::this_thread::sleep_for(std::chrono::duration<double>(std::max(0.0, s)));
    };
  auto logger = get_logger();
  deps.log = [logger](const std::string & msg) {RCLCPP_INFO(logger, "%s", msg.c_str());};
  deps.scene = [this](ScenePhase phase, const std::string & target_id, std::string * why) {
      return sync_bag_obstacles(phase == ScenePhase::CONTACT ? target_id : std::string(), why);
    };
  return deps;
}

bool ManipulationNode::try_acquire()
{
  bool expected = false;
  return busy_.compare_exchange_strong(expected, true);
}

void ManipulationNode::release_busy()
{
  busy_ = false;
}

// ---------------------------------------------------------------- HarvestTarget

rclcpp_action::GoalResponse ManipulationNode::handle_harvest_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const HarvestTarget::Goal> goal)
{
  {
    std::lock_guard<std::mutex> lock(gate_mutex_);
    if (!gate_.active()) {
      RCLCPP_WARN(get_logger(), "harvest_target rejected: node not active");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  if (!motion_ || !ee_ || !try_acquire()) {
    RCLCPP_WARN(
      get_logger(), "harvest_target %s rejected: busy", goal->target_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void ManipulationNode::handle_harvest_accepted(const std::shared_ptr<HarvestGoalHandle> handle)
{
  std::lock_guard<std::mutex> lock(worker_mutex_);
  if (worker_.joinable()) {
    worker_.join();
  }
  worker_ = std::thread([this, handle]() {run_harvest(handle);});
}

void ManipulationNode::run_harvest(const std::shared_ptr<HarvestGoalHandle> handle)
{
  const auto goal = handle->get_goal();
  auto result = std::make_shared<HarvestTarget::Result>();
  cancel_requested_ = false;
  {
    std::lock_guard<std::mutex> lock(gate_mutex_);
    gate_.set_cancel(false);
  }
  CycleResult cr;
  cr.target_id = goal->target_id;
  cr.tool_id = goal->tool_id.empty() ? params_.tool_id : goal->tool_id;
  bool cycle_ran = false;
  try {
    if (recovery_required_) {
      cr.outcome = Outcome::SKIPPED;
      cr.failure_code = failure::RECOVERY_REQUIRED;
      std::lock_guard<std::mutex> lock(status_mutex_);
      cr.reason = "recovery_required:" + recovery_reason_;
      cr.recovery_required = true;
    } else {
      CycleRequest request;
      request.request_id = goal->request_id;
      request.target_id = goal->target_id;
      request.tool_id = cr.tool_id;
      request.mode = goal->mode == HarvestTarget::Goal::MODE_FULL ?
        CycleMode::FULL : CycleMode::PREGRASP_ONLY;
      auto deps = cycle_deps(
        [this, handle]() {return cancel_requested_.load() || handle->is_canceling();});
      deps.on_stage = [handle](CycleStage stage, int index, double elapsed) {
          auto feedback = std::make_shared<HarvestTarget::Feedback>();
          feedback->stage = to_string(stage);
          feedback->stage_index = static_cast<uint8_t>(std::max(0, index));
          feedback->elapsed_s = elapsed;
          handle->publish_feedback(feedback);
        };
      HarvestCycle cycle(cycle_config(), deps);
      RCLCPP_INFO(
        get_logger(), "harvest_target %s/%s mode=%s", request.request_id.c_str(),
        request.target_id.c_str(), request.mode == CycleMode::FULL ? "FULL" : "PREGRASP_ONLY");
      cycle_ran = true;
      cr = cycle.run(request);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "harvest cycle threw: %s", e.what());
    if (motion_) {
      motion_->stop();
    }
    cr.outcome = Outcome::FAILED;
    cr.failure_code = failure::EXEC_FAILED;
    cr.reason = std::string("exception:") + e.what();
    cr.recovery_required = true;
  }
  if (cycle_ran) {
    restore_bag_obstacles();
  }
  if (cr.recovery_required && cr.failure_code != failure::RECOVERY_REQUIRED) {
    latch_recovery(cr.reason);
  }
  result->result = result_to_msg(cr);
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    last_result_ = cr.target_id + ":" + std::to_string(static_cast<int>(cr.outcome)) + ":" +
      std::to_string(cr.failure_code) + ":" + cr.reason;
  }
  RCLCPP_INFO(
    get_logger(), "harvest_target %s done: outcome=%d code=%u reason=%s (%.1f s)",
    cr.target_id.c_str(), static_cast<int>(cr.outcome), cr.failure_code, cr.reason.c_str(),
    cr.cycle_time_s);
  try {
    if (handle->is_canceling()) {
      handle->canceled(result);
    } else if (cr.outcome == Outcome::SUCCEEDED || cr.outcome == Outcome::SKIPPED) {
      handle->succeed(result);
    } else {
      handle->abort(result);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "harvest_target result not delivered: %s", e.what());
  }
  release_busy();
}

// ---------------------------------------------------------------- MoveTo

rclcpp_action::GoalResponse ManipulationNode::handle_move_to_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveTo::Goal>)
{
  {
    std::lock_guard<std::mutex> lock(gate_mutex_);
    if (!gate_.active()) {
      RCLCPP_WARN(get_logger(), "move_to rejected: node not active");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  if (!motion_ || !try_acquire()) {
    RCLCPP_WARN(get_logger(), "move_to rejected: busy");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void ManipulationNode::handle_move_to_accepted(const std::shared_ptr<MoveToGoalHandle> handle)
{
  std::lock_guard<std::mutex> lock(worker_mutex_);
  if (worker_.joinable()) {
    worker_.join();
  }
  worker_ = std::thread([this, handle]() {run_move_to_goal(handle);});
}

void ManipulationNode::run_move_to_goal(const std::shared_ptr<MoveToGoalHandle> handle)
{
  const auto goal = handle->get_goal();
  auto result = std::make_shared<MoveTo::Result>();
  cancel_requested_ = false;
  {
    std::lock_guard<std::mutex> lock(gate_mutex_);
    gate_.set_cancel(false);
  }
  MoveToResult mr;
  try {
    if (recovery_required_) {
      mr.failure_code = failure::RECOVERY_REQUIRED;
      std::lock_guard<std::mutex> lock(status_mutex_);
      mr.message = "recovery_required:" + recovery_reason_;
    } else if (goal->named_target.empty() && !goal->tcp_pose.header.frame_id.empty() &&
      goal->tcp_pose.header.frame_id != params_.moveit.base_frame)
    {
      mr.failure_code = failure::PLAN_FAILED;
      mr.message = "tcp_pose_frame_must_be:" + params_.moveit.base_frame;
    } else {
      MoveToRequest request;
      request.named_target = goal->named_target;
      if (goal->named_target.empty()) {
        request.tcp_pose = pose_from_msg(goal->tcp_pose.pose);
      }
      request.velocity_scaling = goal->velocity_scaling;
      MoveToDeps deps;
      deps.motion = motion_.get();
      deps.gate = [this](GateStage stage, bool new_trajectory) {
          return gate_check(stage, new_trajectory);
        };
      deps.enables = [this]() {return current_enables();};
      deps.cancel_requested = [this, handle]() {
          return cancel_requested_.load() || handle->is_canceling();
        };
      deps.scene = [this](std::string * why) {return sync_bag_obstacles("", why);};
      auto feedback = std::make_shared<MoveTo::Feedback>();
      feedback->progress = 0.0;
      handle->publish_feedback(feedback);
      mr = run_move_to(request, move_to_config(), deps);
      feedback->progress = 1.0;
      handle->publish_feedback(feedback);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "move_to threw: %s", e.what());
    if (motion_) {
      motion_->stop();
    }
    mr.success = false;
    mr.failure_code = failure::EXEC_FAILED;
    mr.message = std::string("exception:") + e.what();
  }
  result->success = mr.success;
  result->failure_code = mr.failure_code;
  result->message = mr.message;
  RCLCPP_INFO(
    get_logger(), "move_to done: success=%d code=%u %s", mr.success ? 1 : 0, mr.failure_code,
    mr.message.c_str());
  try {
    if (handle->is_canceling()) {
      handle->canceled(result);
    } else if (mr.success) {
      handle->succeed(result);
    } else {
      handle->abort(result);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "move_to result not delivered: %s", e.what());
  }
  release_busy();
}

// ---------------------------------------------------------------- services

void ManipulationNode::on_check_reachability(
  const std::shared_ptr<peach2_interfaces::srv::CheckReachability::Request> request,
  std::shared_ptr<peach2_interfaces::srv::CheckReachability::Response> response)
{
  const auto fill_all = [&](uint32_t code, const std::string & reason) {
      for (const auto & id : request->target_ids) {
        response->target_ids.push_back(id);
        response->reachable.push_back(false);
        response->failure_codes.push_back(code);
        response->reasons.push_back(reason);
      }
    };
  bool active = false;
  {
    std::lock_guard<std::mutex> lock(gate_mutex_);
    active = gate_.active();
  }
  if (!active || !motion_ || !ee_) {
    fill_all(failure::ROBOT_NOT_READY, "node_not_active");
    return;
  }
  if (!try_acquire()) {
    fill_all(failure::PLAN_FAILED, "busy");
    return;
  }
  try {
    const std::string tool_id = request->tool_id.empty() ? params_.tool_id : request->tool_id;
    auto deps = cycle_deps([this]() {return cancel_requested_.load();});
    HarvestCycle cycle(cycle_config(), deps);
    for (const auto & id : request->target_ids) {
      CycleRequest cr;
      cr.request_id = "reachability";
      cr.target_id = id;
      cr.tool_id = tool_id;
      cr.mode = request->mode == peach2_interfaces::srv::CheckReachability::Request::MODE_FULL ?
        CycleMode::FULL : CycleMode::PREGRASP_ONLY;
      cr.plan_only = true;
      const CycleResult r = cycle.run(cr);
      const bool ok = r.plan_only && r.failure_code == failure::NONE;
      response->target_ids.push_back(id);
      response->reachable.push_back(ok);
      response->failure_codes.push_back(ok ? failure::NONE : r.failure_code);
      response->reasons.push_back(r.reason);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "check_reachability failed: %s", e.what());
    response->target_ids.clear();
    response->reachable.clear();
    response->failure_codes.clear();
    response->reasons.clear();
    fill_all(failure::PLAN_FAILED, std::string("exception:") + e.what());
  }
  restore_bag_obstacles();
  release_busy();
}

void ManipulationNode::on_acknowledge_recovery(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  // Idempotent: with nothing latched and no tool fault the call succeeds without side effects
  // (an OPEN_CONFIRMED tool is not reset to UNKNOWN by a repeated ACK).
  const bool tool_fault = ee_ && ee_->status().state == peach2_end_effector::ToolState::FAULT;
  if (!recovery_required_ && !tool_fault) {
    response->success = true;
    response->message = "nothing_to_acknowledge";
    publish_recovery_state(false);
    return;
  }
  if (busy_) {
    response->success = false;
    response->message = "busy";
    return;
  }
  GateVerdict ready;
  {
    std::lock_guard<std::mutex> lock(gate_mutex_);
    ready = gate_.robot_ready(steady_now(), false);
  }
  if (!ready.open) {
    response->success = false;
    response->message = "robot_not_ready:" + ready.reason;
    return;
  }
  std::string reason;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    reason = recovery_reason_;
    recovery_reason_.clear();
  }
  if (ee_) {
    ee_->reset_by_ack();
  }
  recovery_required_ = false;
  publish_recovery_state(false);
  response->success = true;
  response->message = "acknowledged " + reason + "; tool must be re-confirmed open by prepare()";
  RCLCPP_WARN(get_logger(), "recovery acknowledged by operator: %s", reason.c_str());
}

// ---------------------------------------------------------------- diagnostics

void ManipulationNode::diagnose_gate(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const double now = steady_now();
  std::lock_guard<std::mutex> lock(gate_mutex_);
  const auto en = gate_.enables(now);
  const auto robot = gate_.robot_ready(now, false);
  stat.add("active", gate_.active());
  stat.add("heartbeat_ok", en.heartbeat_ok);
  stat.add("execution", en.execution);
  stat.add("grasp", en.grasp);
  stat.add("tool", en.tool);
  stat.add("robot_ready", robot.open);
  stat.add("robot_reason", robot.reason);
  stat.add("gate_stops", gate_stops_.load());
  if (!gate_.active()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "inactive");
  } else if (!robot.open) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "robot_not_ready:" + robot.reason);
  } else if (!en.execution) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "plan_only (execution disabled)");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "open");
  }
}

void ManipulationNode::diagnose_tool(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (!ee_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "not_loaded");
    return;
  }
  const auto s = ee_->status();
  stat.add("tool_id", s.tool_id);
  stat.add("state", peach2_end_effector::to_string(s.state));
  stat.add("command_closed", s.command_closed);
  stat.add("feedback", s.feedback_closed ? (*s.feedback_closed ? "closed" : "open") : "unknown");
  stat.add("suspected_loopback", s.suspected_loopback);
  if (s.state == peach2_end_effector::ToolState::FAULT) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "FAULT:" + s.fault_reason);
  } else if (!s.feedback_closed) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "feedback_unavailable");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, peach2_end_effector::to_string(
        s.state));
  }
}

void ManipulationNode::diagnose_cycle(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  stat.add("busy", busy_.load());
  stat.add("recovery_required", recovery_required_.load());
  stat.add("recovery_reason", recovery_reason_);
  stat.add("last_result", last_result_);
  if (recovery_required_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "recovery_required");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, busy_ ? "busy" : "idle");
  }
}

void ManipulationNode::diagnose_inputs(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const double now = steady_now();
  const auto age = [now](double received) {return received < 0.0 ? -1.0 : now - received;};
  const double js_age = age(joint_states_received_s_.load());
  stat.add("joint_states_age_s", js_age);
  stat.add("models_age_s", age(models_received_s_.load()));
  stat.add("models", models_.size());
  stat.add("decision_service_ready", decision_client_ && decision_client_->service_is_ready());
  if (js_age < 0.0 || js_age > params_.joint_states_max_age_s) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "joint_states_stale");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "ok");
  }
}

void ManipulationNode::diagnose_scene(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::size_t objects = 0;
  {
    // try_lock: a scene call in flight must not stall the diagnostics timer.
    std::unique_lock<std::mutex> lock(scene_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "updating");
      return;
    }
    objects = bag_objects_.size();
  }
  stat.add("bag_objects", objects);
  stat.add("scene_failures", scene_failures_.load());
  stat.add(
    "apply_planning_scene_ready", apply_scene_client_ && apply_scene_client_->service_is_ready());
  if (!scene_last_ok_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "last_scene_update_failed");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "ok");
  }
}

}  // namespace peach2_manipulation
