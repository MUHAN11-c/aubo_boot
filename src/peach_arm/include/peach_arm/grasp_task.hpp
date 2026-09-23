// 功能：MTC 接触。约束四层（2026-09-14 重设计）：①果实胶囊审查（工具
// 有限圆柱 vs 感知果实胶囊，仅接近段）+②从下方半空间（反爬锚定果底）
// 在 grasp_geometry；③octomap 场景碰撞（臂/相机受查、工具链豁免）在
// moveit 配置与 scene ACM；④近果低速档（本文件 solver 档位）与接触检测
// （节点侧 contact_monitor，默认关）。接近主路径 = 果平面折线 LIN
// （面内斜插 keep-roll 对轴 + 沿轴垂直进入）；
// 规划失败才 PTP staging 兜底。套入/撤退沿轴；返程倒放同一接近轨迹。
// G/under 单弦档已删。
// 刀具 IO 不在此。
#ifndef PEACH_MANIPULATION__GRASP_TASK_HPP_
#define PEACH_MANIPULATION__GRASP_TASK_HPP_

#include <Eigen/Geometry>

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <tf2_eigen/tf2_eigen.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "peach_arm/acm_policy.hpp"
#include "peach_arm/grasp_geometry.hpp"
#include "peach_arm/math_utils.hpp"
#include "peach_arm/protected_zones.hpp"
#include "peach_arm/staging_selector.hpp"

namespace moveit::task_constructor
{
class Task;
namespace solvers
{
class CartesianPath;
class PipelinePlanner;
}  // namespace solvers
namespace stages
{
class MoveRelative;
class MoveTo;
}  // namespace stages
class SerialContainer;
}  // namespace moveit::task_constructor

namespace peach_arm
{

class RetireBucket;  // execution_guard.hpp（src/ 私有头）

// tip 姿态不偏离 target_pose 超过 tol_deg 的三轴等宽容差约束集。
// 只挂已齐 LIN（分档要求起点对轴，拦笛卡尔插值中途侧翻）。
// 未齐第一段 LIN-align 不挂：Jazzy ValidateSolution 验每个路点含起点，
// 起点相对目标 >20° 会 INVALID_MOTION_PLAN。果平面折线斜插段（未齐）
// 不挂门；其后已齐沿轴 LIN 挂门。PTP staging 兜底不挂姿态门（关节目标即 IK 解）。
inline moveit_msgs::msg::Constraints makeOrientationGate(
  const std::string & link_name, const std::string & frame_id,
  const Eigen::Isometry3d & target_pose, double tol_deg,
  const std::string & name)
{
  moveit_msgs::msg::OrientationConstraint orientation;
  orientation.link_name = link_name;
  orientation.header.frame_id = frame_id;
  orientation.orientation = tf2::toMsg(Eigen::Quaterniond(target_pose.linear()));
  const double tol = tol_deg * static_cast<double>(EIGEN_PI) / 180.0;
  orientation.absolute_x_axis_tolerance = tol;
  orientation.absolute_y_axis_tolerance = tol;
  orientation.absolute_z_axis_tolerance = tol;
  orientation.weight = 1.0;
  moveit_msgs::msg::Constraints constraints;
  constraints.name = name;
  constraints.orientation_constraints.push_back(orientation);
  return constraints;
}

// 接近分档结论（只出结论，不规划）。同一 (config, entry, axis) 每条公共入口算一次，
// 下游装配/预览复用，避免周期内重复 classifyApproach。
// STAGING=主路径（果平面折线 LIN，失败才 PTP staging）；LIN/LIN_ALIGN_THEN_LIN=
// 已在袋底侧的直连短修正；G/under 单弦档已删（见文件头）。
struct ApproachSplit
{
  bool need_lin{true};     ///< True=需要沿轴 LIN。
  bool need_align{true};   ///< True=需要先对轴。
  enum class Kind
  {
    SKIP,                 ///< 已在入口，无需接近。
    LIN,                  ///< 已齐，直连 LIN。
    LIN_ALIGN_THEN_LIN,   ///< 先短 LIN 对轴再插入。
    STAGING,              ///< 主路径：果平面折线 LIN（失败才 PTP staging）。
    BLOCKED               ///< 无法接近。
  } kind{
    Kind::STAGING};
  double lin_to_entry_m{0.0};   ///< 沿轴到入口剩余 [m]。
  double lateral_m{0.0};        ///< 侧向偏差 [m]。
  double axial_m{0.0};          ///< 轴向偏差 [m]。
  double align_deg{180.0};      ///< 工具 Z 与轴夹角 [deg]。
  double sweep_deg{0.0};        ///< 绕行扫角 [deg]。
  double radius_m{0.0};         ///< 绕行半径 [m]。
  /// classifyApproach 取到的当前 TCP；后续装配复用，避免多次 TF。
  std::optional<Eigen::Isometry3d> current_tip;
  double tool_roll_rad{0.0};    ///< 对轴后绕工具 Z 的刀口滚转；0=keep-roll。
  std::string blocked_reason{"无当前 TCP"};  ///< Kind::BLOCKED 原因。
};

struct GraspTaskConfig
{
  std::string planning_group;  ///< MoveIt 规划组。
  std::string tip_frame;       ///< IK 末端连杆（当前 tcp）。
  std::string base_frame;      ///< 位姿参考系（base_link）。
  std::string free_space_pipeline{"pilz_industrial_motion_planner"};  ///< 自由空间规划管线。
  std::string free_space_planner{"LIN"};  ///< 自由空间规划器 ID。
  double planning_time_s{1.5};            ///< 规划时限 [s]。
  double velocity_scaling{0.10};          ///< 接触段速度缩放。
  double acceleration_scaling{0.10};      ///< 接触段加速度缩放。
  double cartesian_step_m{0.005};         ///< 直线插入步长 [m]。
  double cartesian_min_fraction{0.95};    ///< 直线完成比例；1.0 会因末步离散失败。
  double cartesian_precision_m{0.001};    ///< 笛卡尔精度 [m]。
  std::size_t max_solutions{5U};          ///< 最多尝试 IK/规划解数。
  double approach_max_duration_s{0.0};    ///< 接近时限 [s]；0=不限。
  double approach_max_total_joint_travel_rad{12.0};   ///< 接近总关节行程上限 [rad]。
  double approach_max_single_joint_travel_rad{6.1};   ///< 单关节行程上限 [rad]。
  double approach_max_detour_ratio{1.8};              ///< 笛卡尔绕行比；≤0 跳过。
  double approach_max_chord_deviation_m{0.25};        ///< 弦偏差上限 [m]。
  double approach_max_recede_m{0.08};                 ///< 回退上限 [m]。
  double staging_max_detour_ratio{1.8};
  double staging_max_chord_deviation_m{0.25};
  double staging_max_recede_m{0.08};
  double approach_max_tcp_rotation_deg{110.0};        ///< TCP 转角上限 [deg]。
  double approach_tcp_rotation_slack_deg{20.0};       ///< 转角松弛 [deg]。
  double approach_keepout_radius_m{0.12};             ///< 果实胶囊回退半径 [m]。
  double approach_keepout_axial_m{0.12};              ///< 轴向审查长度 [m]；≤0 关闭。
  double fruit_inflation_m{0.01};                     ///< 感知半径外膨胀 [m]。
  double approach_cartesian_max_distance_m{0.80};     ///< 接触笛卡尔弦长上限 [m]。
  double approach_along_axis_m{0.0};                  ///< 预抓取相对入口沿 −axis 后撤 [m]。
  double approach_staging_standoff_m{0.10};           ///< staging 相对预抓取再退 [m]。
  double approach_max_lateral_m{0.05};                ///< 小于此值视为已对轴 [m]。
  double approach_max_align_deg{20.0};                ///< 小于此值视为已齐 [deg]。
  double approach_near_velocity_scaling{0.05};        ///< 近果低速档（仅套入/撤退）。
  // 工具档案（W5-6，GPL yaml tool.*；默认值=原三处硬编码）：
  std::vector<std::string> tool_links{
    "tool_axis", "cutting_plane", "tcp", "sleeve_mouth",
    "tool_body_link", "quick_changer_link"};  ///< 工具链连杆（整图 octomap 豁免）。
  std::vector<std::string> contact_tool_links{
    "sleeve_mouth", "tcp", "tool_axis",
    "cutting_plane"};  ///< 接触阶段 × 目标对象豁免的连杆。
  double tool_body_length_m{0.200};  ///< 工具筒体长 [m]（①层审查，tcp.xacro 对齐）。
  double tool_body_radius_m{0.060};  ///< 工具筒体半径 [m]。
  std::function<std::optional<Eigen::Isometry3d>()> lookup_current_tip;  ///< 查当前 TCP。
  // staging 关节目标（主路径 PTP 落点）：候选编排（keep-roll 及 ±30°/±60° ×
  // 当前+N-1 随机种子、腕轴加权距离+滚转惩罚排序、top_n 截断）在
  // StagingCandidateSelector 纯核（staging_selector.hpp，W5-2）；转移逐候选
  // 试规划，救弧穿袋囊与自碰构型。候选类型即纯核 StagingCandidate。
  using StagingCandidate = peach_arm::StagingCandidate;
  std::function<std::vector<StagingCandidate>(
      const Eigen::Isometry3d & staging_pose)> select_goal_joints;
  std::vector<ProtectedZone> protected_zones;  // base 系 AABB → planning scene
  std::function<bool(std::string &)> approach_execution_gate;  // 下发接近轨迹前
  std::function<bool(std::string &)> retreat_execution_gate;   // 撤离不依赖视觉
  // MTC 执行有界等待：MTC Task 不暴露 stop，超时/取消经 execution_stop
  // （节点注入 move_group_->stop()，MGI stop 打节点级停止服务）兜底。
  std::function<void()> execution_stop;
  double execute_timeout_s{90.0};  ///< MTC 解执行有界等待 [s]。
};

struct GraspTaskResult
{
  bool success{false};
  bool execution_started{false};  // 已向控制器下发
  std::string reason;
};

// 接触运动只走 MTC；工具 IO 留在阶段执行器（stages.cpp），失败才能进撤离。
// active_task_：正在 plan/execute 的任务，串行独占（task_mutex_ 保护）。
class GraspTask
{
public:
  GraspTask(rclcpp::Node::SharedPtr node, GraspTaskConfig config);
  ~GraspTask();

  void setContactAcm(const std::string & target_id, ContactAcmStage stage);

  // 只规划（PREVIEW / preview Trigger）：接近分档 + 到预抓取 + 沿轴插入
  // 整链一次装配预览，不下发。执行的接触走 moveToPregrasp / sleeveLinear。
  // fruit：果实胶囊（感知直径+膨胀；阶段执行器按精化几何构造）。
  GraspTaskResult approachAndInsert(
    const Eigen::Isometry3d & entry_tip_pose,
    const Eigen::Vector3d & insertion_axis,
    double insertion_distance_m,
    const FruitCapsule & fruit);

  // 入口→插入→原轴撤离，只规划不下发。
  GraspTaskResult previewFullContact(
    const Eigen::Isometry3d & entry_tip_pose,
    const Eigen::Vector3d & insertion_axis,
    double insertion_distance_m,
    const FruitCapsule & fruit);

  GraspTaskResult moveToPregrasp(
    const Eigen::Isometry3d & entry_tip_pose,
    const Eigen::Vector3d & insertion_axis,
    const FruitCapsule & fruit,
    bool execute);

  GraspTaskResult sleeveLinear(
    const Eigen::Vector3d & insertion_axis,
    double insertion_distance_m,
    bool execute);

  GraspTaskResult retreat(
    const Eigen::Vector3d & insertion_axis,
    double retreat_distance_m,
    bool execute);

  void cancel();  // preempt 当前 active 任务

  // 最近一次过护栏且已下发成功的接近轨迹（多段已拼接）。空 = 无可返程记录。
  trajectory_msgs::msg::JointTrajectory lastApproachTrajectory() const;

private:
  GraspTaskResult planAndMaybeExecute(
    std::unique_ptr<moveit::task_constructor::Task> task,
    bool execute,
    const std::function<bool(std::string &)> & execution_gate,
    bool guard_approach = false,
    std::size_t guard_skip_tail = 0,
    bool staging_guard = false,
    bool cartesian_per_part = false);
  GraspTaskResult planTaskOnly(
    moveit::task_constructor::Task * active, bool guard_approach,
    std::size_t guard_skip_tail = 0, bool staging_guard = false,
    std::size_t max_solutions = 0,  // 0 = config_.max_solutions；执行路径传 1
    bool cartesian_per_part = false);
  GraspTaskResult executeSolution(
    moveit::task_constructor::Task * active,
    const std::function<bool(std::string &)> & execution_gate);

  std::unique_ptr<moveit::task_constructor::Task> makeTaskShell(
    const std::string & task_name) const;
  std::unique_ptr<moveit::task_constructor::Task> makeApproachInsertTask(
    const std::string & task_name,
    const Eigen::Vector3d & insertion_axis,
    double along_axis_m,
    double insertion_distance_m);
  std::unique_ptr<moveit::task_constructor::Task> makeApproachOnlyTask(
    const std::string & task_name,
    const Eigen::Isometry3d & entry_tip_pose,
    const Eigen::Vector3d & insertion_axis,
    const ApproachSplit & split);
  // 果平面折线 LIN（主路径）；正式转移与预览共用。
  // in_plane_* <0 = min(config, 斜插封顶)；沿轴跳另走 along 封顶。
  std::unique_ptr<moveit::task_constructor::SerialContainer>
  makePlanarApproachSequence(
    const PlanarApproachPath & path, const std::string & label,
    double in_plane_vel = -1.0, double in_plane_acc = -1.0) const;
  std::unique_ptr<moveit::task_constructor::Task> makePlanarApproachTask(
    const std::string & task_name, const PlanarApproachPath & path,
    double in_plane_vel = -1.0, double in_plane_acc = -1.0);
  // PTP staging 兜底（果平面 LIN 失败时）；正式转移与预览共用。
  std::unique_ptr<moveit::task_constructor::SerialContainer> makeStagingSequence(
    const Eigen::Isometry3d & pregrasp_tip_pose,
    const Eigen::Isometry3d & staging_tip_pose,
    const std::map<std::string, double> & staging_joints,
    const std::string & label) const;
  std::unique_ptr<moveit::task_constructor::Task> makeStagingTransitTask(
    const std::string & task_name,
    const Eigen::Isometry3d & pregrasp_tip_pose,
    const Eigen::Isometry3d & staging_tip_pose,
    const std::map<std::string, double> & staging_joints);
  // staging 候选（预抓取下方轴上，alignFrameZ keep-roll；滚转/种子/自碰
  // 过滤由 config.select_goal_joints 完成，按距离升序）。空 = standoff
  // 关闭 / 无当前 TCP / 无可行候选。
  std::vector<GraspTaskConfig::StagingCandidate> stagingCandidate(
    const Eigen::Isometry3d & entry_tip_pose,
    const Eigen::Vector3d & insertion_axis,
    const ApproachSplit & split) const;
  bool tryRolledApproach(
    const std::string & task_name,
    const Eigen::Isometry3d & entry_tip_pose,
    const Eigen::Vector3d & insertion_axis,
    ApproachSplit split,
    ApproachSplit::Kind kind,
    bool execute,
    GraspTaskResult & last);
  bool tryStagingTransit(
    const std::string & task_name,
    const Eigen::Isometry3d & entry_tip_pose,
    const Eigen::Vector3d & insertion_axis,
    const ApproachSplit & split,
    bool execute,
    GraspTaskResult & last);
  GraspTaskResult planToPregrasp(
    const std::string & task_name,
    const Eigen::Isometry3d & entry_tip_pose,
    const Eigen::Vector3d & insertion_axis,
    const ApproachSplit & split,
    const FruitCapsule & fruit,
    bool execute);
  std::unique_ptr<moveit::task_constructor::Task> makeInsertOnlyTask(
    const std::string & task_name,
    const Eigen::Vector3d & insertion_axis,
    double insertion_distance_m);
  std::unique_ptr<moveit::task_constructor::SerialContainer> makeApproachInsertSequence(
    const Eigen::Vector3d & insertion_axis,
    double along_axis_m,
    double insertion_distance_m) const;
  void appendLinToPose(
    moveit::task_constructor::SerialContainer & sequence,
    const Eigen::Isometry3d & target_tip_pose,
    const std::string & label,
    bool gate_orientation,
    double velocity_scaling = -1.0,       // <0 = config_.velocity_scaling
    double acceleration_scaling = -1.0) const;  // <0 = min(1, vel*2)
  void appendApproachToPregrasp(
    moveit::task_constructor::SerialContainer & sequence,
    const Eigen::Isometry3d & entry_tip_pose,
    const Eigen::Vector3d & insertion_axis,
    const ApproachSplit & split) const;
  void appendAlongAxisMove(
    moveit::task_constructor::SerialContainer & sequence,
    const Eigen::Vector3d & insertion_axis,
    double along_axis_m,
    const std::string & label) const;
  void syncKeepoutCollisionObjects() const;

public:
  // ③层工具豁免（周期级）策略（W5-5 合并原整图/轮内两份 90% 重复实现）：
  //   WholeMap  —— 整表回写 工具链 × <octomap> = allowed（on_activate 后台
  //                线程应用一次，Survey/观察/接近全程生效）；
  //   PerTarget —— 整图豁免（若策略开启）+ 接触阶段 指定目标对象 × 接触
  //                连杆（pending_acm_* 由 setContactAcm 预置）。
  // 09-17 真机实锤：眼在手上时 updater self-filter 漏收工具点云，不豁免则
  // 工具×自家幽灵体素自碰死锁。臂/相机连杆保持受查（防撞主力）。
  enum class OctomapExemptionPolicy
  {
    WholeMap,
    PerTarget
  };

  // 整图豁免 static 入口（无对象依赖）：tool_links 来自调用方参数档案。
  static void applyWholeOctomapToolExemption(
    const rclcpp::Logger & logger,
    moveit::planning_interface::PlanningSceneInterface & scene,
    const std::vector<std::string> & tool_links);
  // 接触轮内版本：策略默认 PerTarget（整图豁免 + 接触阶段目标对象豁免）。
  void applyToolOctomapExemption(
    moveit::planning_interface::PlanningSceneInterface & scene,
    OctomapExemptionPolicy policy = OctomapExemptionPolicy::PerTarget) const;

private:
  // ACM 豁免合并核心（W5-5）：整图豁免 + 可选接触阶段目标对象豁免，
  // 供 static 整图入口与轮内 PerTarget 入口共用。
  static void applyOctomapExemptionImpl(
    const rclcpp::Logger & logger,
    moveit::planning_interface::PlanningSceneInterface & scene,
    const std::vector<std::string> & tool_links,
    const std::vector<std::string> & contact_tool_links,
    const std::string & per_target_id,
    ContactAcmStage per_target_stage);

  std::shared_ptr<moveit::task_constructor::solvers::PipelinePlanner>
  makePilzSolver(
    const std::string & planner_id,
    double velocity_scaling = -1.0,       // <0 = config_.velocity_scaling
    double acceleration_scaling = -1.0) const;  // <0 = min(1, vel*2)
  std::shared_ptr<moveit::task_constructor::solvers::PipelinePlanner>
  makePtpSolver() const;
  std::shared_ptr<moveit::task_constructor::solvers::CartesianPath>
  makeCartesianSolver(double velocity_scaling = -1.0) const;
  std::unique_ptr<moveit::task_constructor::stages::MoveTo> makeMoveToEntry(
    const std::shared_ptr<moveit::task_constructor::solvers::PipelinePlanner> & solver,
    const Eigen::Isometry3d & entry_tip_pose,
    const std::string & label) const;
  std::unique_ptr<moveit::task_constructor::stages::MoveRelative> makeLinearMove(
    const std::string & label,
    const std::shared_ptr<moveit::task_constructor::solvers::CartesianPath> & solver,
    const Eigen::Vector3d & direction, double distance_m) const;

  rclcpp::Node::SharedPtr node_;
  GraspTaskConfig config_;
  std::mutex task_mutex_;
  std::unique_ptr<moveit::task_constructor::Task> active_task_;
  std::atomic_bool task_abandoned_{false};  // 弃等线程已接管任务（reset 前须 release）
  std::unique_ptr<peach_arm::RetireBucket> retiring_;  // 弃等线程桶（execution_guard.hpp）
  FruitCapsule pending_fruit_;
  bool inspect_fruit_{false};
  std::string pending_acm_target_id_;
  ContactAcmStage pending_acm_stage_{ContactAcmStage::Transit};
  mutable std::vector<std::string> published_keepout_ids_;  // 上次写入 scene 的 id
  std::vector<trajectory_msgs::msg::JointTrajectory> planned_approach_parts_;
  std::vector<trajectory_msgs::msg::JointTrajectory> last_approach_parts_;
};

}  // namespace peach_arm

#endif  // PEACH_MANIPULATION__GRASP_TASK_HPP_
