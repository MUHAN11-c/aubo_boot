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
#include <nlohmann/json.hpp>
#include <peach_pose_msgs/msg/bag_fitting_array.hpp>
#include <peach_pose_msgs/msg/bag_grasp_candidate_array.hpp>
#include <peach_pose_msgs/msg/peach_target_observation_array.hpp>
#include <peach_harvest_msgs/action/run_target_cycle.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "peach_approach_grasp/cycle_state.hpp"
#include "peach_approach_grasp/grasp_task.hpp"
#include "peach_approach_grasp/quality_gate.hpp"
#include "peach_approach_grasp/safety_gate.hpp"
#include "peach_approach_grasp/target_cache.hpp"
#include "peach_approach_grasp/view_planner.hpp"

namespace moveit::planning_interface
{
class MoveGroupInterface;
}  // namespace moveit::planning_interface

namespace peach_approach_grasp
{
using Trigger = std_srvs::srv::Trigger;
using SetBool = std_srvs::srv::SetBool;
using json = nlohmann::json;
using RunTargetCycle = peach_harvest_msgs::action::RunTargetCycle;
using RunTargetGoalHandle = rclcpp_action::ServerGoalHandle<RunTargetCycle>;

// 主动视觉靠近与抓取编排节点：只通过 MoveIt 和现有 ROS 接口工作，不直接访问 SDK。
class ApproachGraspNode : public rclcpp::Node
{
public:
  ApproachGraspNode();
  ~ApproachGraspNode() override;

  void initializeMoveIt();

private:
  double insertionTravel(const CachedRefined & refined) const;
  void declareParameters();
  void loadParameters();
  rcl_interfaces::msg::SetParametersResult onParameters(
    const std::vector<rclcpp::Parameter> & parameters);
  void createInterfaces();

  // 订阅回调（薄壳）：消息字段提取后委托 cache_ 做四源一致性调和。
  void onTargets(
    const peach_pose_msgs::msg::PeachTargetObservationArray::SharedPtr message);
  void onDiagnostics(const std_msgs::msg::String::SharedPtr message);
  void onDecision(const std_msgs::msg::String::SharedPtr message);
  void onRefinedPose(
    const peach_pose_msgs::msg::BagGraspCandidateArray::SharedPtr message);
  void onRefinedDiagnostics(
    const peach_pose_msgs::msg::BagFittingArray::SharedPtr message);
  void onRobotStatus(const aubo_msgs::msg::RobotStatus::SharedPtr message);

  // RunTargetCycle action 服务端与周期控制服务（cycle_action.cpp）。
  rclcpp_action::GoalResponse onActionGoal(
    const rclcpp_action::GoalUUID &,
    const std::shared_ptr<const RunTargetCycle::Goal> goal);
  rclcpp_action::CancelResponse onActionCancel(
    const std::shared_ptr<RunTargetGoalHandle>);
  void onActionAccepted(const std::shared_ptr<RunTargetGoalHandle> goal_handle);
  void executeAction(const std::shared_ptr<RunTargetGoalHandle> goal_handle);
  void onStart(
    const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response,
    bool action_driven);
  void onCancel(const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response);
  void onAcknowledgeRecovery(
    const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response);
  void onQuery(const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response);
  void onArm(
    const SetBool::Request::SharedPtr request, SetBool::Response::SharedPtr response);

  // 运动接口（motion_interface.cpp）：TF 查询、plan/execute、服务触发、工具 IO、
  // 接触轨迹预览与 go_to_photo_pose 的规划执行体。
  std::optional<Eigen::Isometry3d> lookupTransform(
    const std::string & target, const std::string & source);
  bool planOrMoveTip(
    const Eigen::Isometry3d & tip_pose, const std::string & planner_id, bool execute,
    const std::string & label, bool allow_fallback);
  bool planOrMoveCamera(
    const Eigen::Isometry3d & camera_pose, const std::string & planner_id, bool execute,
    const std::string & label, bool allow_fallback = true);
  bool planOrMoveTool(
    const Eigen::Isometry3d & tool_pose, const std::string & planner_id,
    const std::string & label);
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
  std::vector<Eigen::Vector3d> observedDirectionsSnapshot();
  std::optional<CachedRefined> refinedSnapshot();
  std::string graspDecisionTargetSnapshot();
  bool cycleTargetReady(const std::string & target_id, std::string & reason);
  bool safetyReady(std::string & reason);
  bool waitForNewView(std::size_t previous_views);
  // 等待一条晚于 after_s 的有效目标观测（视点到位后的新鲜帧）。
  bool waitForFreshTarget(double after_s);
  bool waitForRefined(const std::string & target_id);

  // 行为树节点注册与节点体（bt_nodes.cpp）。
  void registerBehaviorTreeNodes();
  BT::NodeStatus btFailure(const std::string & reason);
  BT::NodeStatus btPrepareCycle();
  BT::NodeStatus btPlanPreview();
  BT::NodeStatus btAcquireViews();
  BT::NodeStatus btFinalizeAndValidate();
  BT::NodeStatus btReportReady();
  BT::NodeStatus btMtcApproachAndInsert();
  BT::NodeStatus btActuateTool();
  BT::NodeStatus btMtcRetreat();
  BT::NodeStatus btCompleteTarget();

  void runCycle();
  void publishViewMarkers(
    const Eigen::Vector3d & target, const std::vector<ViewCandidate> & candidates);
  void setState(CycleState state, const std::string & message);
  void publishState();

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
  double transit_velocity_scaling_{0.05};
  double transit_acceleration_scaling_{0.05};
  double mtc_cartesian_step_m_{0.005};
  double mtc_cartesian_precision_m_{0.001};
  int mtc_max_solutions_{5};
  int maximum_scan_moves_{8};
  double frame_wait_s_{6.0};
  std::atomic_bool execution_enabled_{false};
  bool reset_reconstruction_on_start_{false};
  std::atomic_bool grasp_enabled_{false};
  double neck_margin_m_{0.015};
  double minimum_travel_m_{0.02};
  double maximum_travel_m_{0.20};
  bool complete_target_after_retreat_{true};
  std::atomic_bool tool_enabled_{false};
  int tool_io_fun_{3};
  int tool_io_pin_{0};
  double tool_close_state_{1.0};
  double service_timeout_s_{3.0};
  double refined_timeout_s_{10.0};

  std::unique_ptr<ViewPlanner> view_planner_;
  std::unique_ptr<QualityGate> quality_gate_;
  std::unique_ptr<SafetyGate> safety_gate_;
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
  std::string cycle_target_id_;
  std::string bt_failure_reason_;
  CycleState cycle_terminal_state_{CycleState::SUCCEEDED};
  std::string cycle_terminal_message_;
  std::optional<CachedTarget> cycle_target_;
  std::optional<CachedRefined> cycle_refined_;
  std::vector<ViewCandidate> cycle_candidates_;
  Eigen::Isometry3d cycle_entry_tip_pose_{Eigen::Isometry3d::Identity()};
  double cycle_travel_m_{0.0};
  std::atomic_bool running_{false};
  std::atomic_bool cancel_requested_{false};
  std::atomic_bool execution_armed_{false};
  std::atomic_bool contact_recovery_required_{false};
  // abort 路径的终局分级（RunTargetCycle::Result 常量），由 BT 失败点按需覆盖。
  std::atomic<uint8_t> pending_outcome_{RunTargetCycle::Result::FAILED};
  // 本周期是否由 action（RunTargetCycle）驱动；决定 CompleteTarget 是否推进目标。
  bool cycle_action_driven_{false};
  std::thread worker_;
  // action 执行线程保持可 join，析构时先取消再回收，避免 shutdown 后访问悬空 this。
  std::thread action_thread_;

  rclcpp::Subscription<peach_pose_msgs::msg::PeachTargetObservationArray>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr diagnostics_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr decision_sub_;
  rclcpp::Subscription<peach_pose_msgs::msg::BagGraspCandidateArray>::SharedPtr
    refined_pose_sub_;
  rclcpp::Subscription<peach_pose_msgs::msg::BagFittingArray>::SharedPtr refined_diag_sub_;
  rclcpp::Subscription<aubo_msgs::msg::RobotStatus>::SharedPtr robot_status_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Service<Trigger>::SharedPtr start_service_;
  rclcpp::Service<Trigger>::SharedPtr preview_approach_service_;
  rclcpp::Service<Trigger>::SharedPtr preview_full_contact_service_;
  rclcpp::Service<Trigger>::SharedPtr cancel_service_;
  rclcpp::Service<Trigger>::SharedPtr recovery_service_;
  rclcpp::Service<Trigger>::SharedPtr query_service_;
  rclcpp::Service<Trigger>::SharedPtr photo_pose_service_;
  rclcpp::Service<SetBool>::SharedPtr arm_service_;
  rclcpp_action::Server<RunTargetCycle>::SharedPtr cycle_action_server_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rclcpp::Client<Trigger>::SharedPtr reset_client_;
  rclcpp::Client<Trigger>::SharedPtr finalize_client_;
  rclcpp::Client<Trigger>::SharedPtr save_client_;
  rclcpp::Client<Trigger>::SharedPtr complete_client_;
  rclcpp::Client<aubo_msgs::srv::SetIO>::SharedPtr tool_io_client_;
};

}  // namespace peach_approach_grasp

#endif  // APPROACH_GRASP_NODE_IMPL_HPP_
