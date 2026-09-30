#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "aubo_msgs/msg/robot_status.hpp"
#include "bondcpp/bond.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "peach2_end_effector/end_effector.hpp"
#include "peach2_interfaces/action/harvest_target.hpp"
#include "peach2_interfaces/action/move_to.hpp"
#include "peach2_interfaces/msg/enables.hpp"
#include "peach2_interfaces/msg/target_model_array.hpp"
#include "peach2_interfaces/msg/tool_state.hpp"
#include "peach2_interfaces/srv/check_reachability.hpp"
#include "peach2_interfaces/srv/get_decision.hpp"
#include "peach2_manipulation/bag_obstacles.hpp"
#include "peach2_manipulation/command_gate.hpp"
#include "peach2_manipulation/decision_client.hpp"
#include "peach2_manipulation/harvest_cycle.hpp"
#include "peach2_manipulation/motion_backend.hpp"
#include "peach2_manipulation/move_to.hpp"
#include "peach2_manipulation/peach2_manipulation_parameters.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace peach2_manipulation
{

/// Synchronous GetDecision over a client whose callback group is spun by another executor
/// thread. Called only from the action worker / reachability callback, never from the client's
/// own group.
class RosDecisionClient : public DecisionClient
{
public:
  RosDecisionClient(
    rclcpp::Client<peach2_interfaces::srv::GetDecision>::SharedPtr client,
    rclcpp::Clock::SharedPtr clock, double timeout_s);
  std::optional<DecisionView> get(
    const std::string & target_id, const std::string & tool_id, uint64_t min_revision) override;
  double now_s() override;

private:
  rclcpp::Client<peach2_interfaces::srv::GetDecision>::SharedPtr client_;
  rclcpp::Clock::SharedPtr clock_;
  double timeout_s_;
};

/// Latest TargetModel per id from the latched models topic.
class ModelCache : public TargetSource
{
public:
  void update(const peach2_interfaces::msg::TargetModelArray & msg);
  std::optional<peach2_end_effector::TargetGeometry> get(const std::string & target_id) override;
  std::vector<peach2_end_effector::TargetGeometry> all();
  std::size_t size();

private:
  std::mutex mutex_;
  std::unordered_map<std::string, peach2_end_effector::TargetGeometry> models_;
};

/// peach2_manipulation lifecycle node: the single command gate for every trajectory and tool
/// SetIO. Nothing moves or switches at startup; with execution disabled every request is
/// plan-only.
class ManipulationNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using HarvestTarget = peach2_interfaces::action::HarvestTarget;
  using MoveTo = peach2_interfaces::action::MoveTo;
  using HarvestGoalHandle = rclcpp_action::ServerGoalHandle<HarvestTarget>;
  using MoveToGoalHandle = rclcpp_action::ServerGoalHandle<MoveTo>;

  explicit ManipulationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ManipulationNode() override;

  /// Companion node for MoveGroupInterface; must be added to the same executor.
  rclcpp::Node::SharedPtr moveit_node() const {return moveit_node_;}

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

private:
  double steady_now() const;
  void build_end_effector(const Params & params);
  void release();
  void stop_worker(const std::string & why);

  GateVerdict gate_check(GateStage stage, bool new_trajectory);
  EffectiveEnables current_enables();
  bool permit_io(int pin, bool level);
  CycleConfig cycle_config() const;
  MoveToConfig move_to_config() const;
  CycleDeps cycle_deps(std::function<bool()> cancel_requested);
  void latch_recovery(const std::string & reason);
  /// Publishes /peach/manipulation/recovery_required when the value changed (or `force`).
  void publish_recovery_state(bool force);

  /// Neighbour-bag collision objects (主审跨包决定): make the scene hold one `peach_bag_*` per
  /// cached model except `exclude_target_id` (empty = every bag), in one ApplyPlanningScene diff.
  bool sync_bag_obstacles(const std::string & exclude_target_id, std::string * why);
  /// Back to every bag as an obstacle after a cycle / reachability check; logs on failure.
  void restore_bag_obstacles();
  /// Remove every `peach_bag_*` object this node wrote (deactivate / cleanup / shutdown).
  void clear_bag_obstacles();
  bool apply_bag_diff(const BagSceneDiff & diff, std::string * why);
  bool fetch_scene_object_ids(std::vector<std::string> & ids, std::string * why);

  void on_enables(const peach2_interfaces::msg::Enables & msg);
  void on_robot_status(const aubo_msgs::msg::RobotStatus & msg);
  void on_joint_states(const sensor_msgs::msg::JointState & msg);
  void on_models(const peach2_interfaces::msg::TargetModelArray & msg);
  void on_gate_timer();
  void on_tool_timer();

  bool try_acquire();
  void release_busy();

  rclcpp_action::GoalResponse handle_harvest_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const HarvestTarget::Goal> goal);
  void handle_harvest_accepted(const std::shared_ptr<HarvestGoalHandle> handle);
  void run_harvest(const std::shared_ptr<HarvestGoalHandle> handle);

  rclcpp_action::GoalResponse handle_move_to_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveTo::Goal> goal);
  void handle_move_to_accepted(const std::shared_ptr<MoveToGoalHandle> handle);
  void run_move_to_goal(const std::shared_ptr<MoveToGoalHandle> handle);

  template<typename GoalHandleT>
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleT>)
  {
    cancel_requested_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void on_check_reachability(
    const std::shared_ptr<peach2_interfaces::srv::CheckReachability::Request> request,
    std::shared_ptr<peach2_interfaces::srv::CheckReachability::Response> response);
  void on_acknowledge_recovery(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void diagnose_gate(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void diagnose_tool(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void diagnose_cycle(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void diagnose_inputs(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void diagnose_scene(diagnostic_updater::DiagnosticStatusWrapper & stat);

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
  rclcpp::Node::SharedPtr moveit_node_;

  mutable std::mutex gate_mutex_;
  CommandGate gate_;
  ClosedEdge execution_edge_;
  bool last_io_close_{false};
  bool close_level_{true};
  std::optional<aubo_msgs::msg::RobotStatus> last_robot_status_;
  std::atomic<double> joint_states_received_s_{-1.0};
  std::atomic<double> models_received_s_{-1.0};
  std::atomic<uint64_t> gate_stops_{0};

  rclcpp::CallbackGroup::SharedPtr state_group_;
  rclcpp::CallbackGroup::SharedPtr client_group_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr action_group_;
  rclcpp::CallbackGroup::SharedPtr timer_group_;

  rclcpp::Subscription<peach2_interfaces::msg::Enables>::SharedPtr enables_sub_;
  rclcpp::Subscription<aubo_msgs::msg::RobotStatus>::SharedPtr robot_status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<peach2_interfaces::msg::TargetModelArray>::SharedPtr models_sub_;
  rclcpp_lifecycle::LifecyclePublisher<peach2_interfaces::msg::ToolState>::SharedPtr
    tool_state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr recovery_pub_;
  rclcpp::Client<peach2_interfaces::srv::GetDecision>::SharedPtr decision_client_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_client_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_scene_client_;
  rclcpp::Service<peach2_interfaces::srv::CheckReachability>::SharedPtr reachability_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ack_srv_;
  rclcpp_action::Server<HarvestTarget>::SharedPtr harvest_server_;
  rclcpp_action::Server<MoveTo>::SharedPtr move_to_server_;
  rclcpp::TimerBase::SharedPtr gate_timer_;
  rclcpp::TimerBase::SharedPtr tool_timer_;
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
  std::unique_ptr<bond::Bond> bond_;

  // Loader must outlive the instance it created (declared before ee_).
  std::unique_ptr<pluginlib::ClassLoader<peach2_end_effector::EndEffector>> loader_;
  std::shared_ptr<peach2_end_effector::IoBackend> raw_io_;
  std::shared_ptr<peach2_end_effector::EndEffector> ee_;
  std::shared_ptr<MotionBackend> motion_;
  std::unique_ptr<RosDecisionClient> decisions_;
  ModelCache models_;

  std::atomic<bool> busy_{false};
  std::atomic<bool> cancel_requested_{false};
  std::atomic<bool> recovery_required_{false};
  std::mutex worker_mutex_;
  std::thread worker_;
  std::mutex status_mutex_;
  std::string recovery_reason_;
  std::string last_result_;

  std::mutex recovery_pub_mutex_;
  std::optional<bool> recovery_published_;

  std::mutex scene_mutex_;
  BagObstacleSet bag_objects_;
  bool scene_adopted_{false};
  std::atomic<uint64_t> scene_failures_{0};
  std::atomic<bool> scene_last_ok_{true};
};

}  // namespace peach2_manipulation
