// 功能：MTC 接触。约束四层（2026-09-14 重设计）：①果实胶囊审查（工具
// 有限圆柱 vs 感知果实胶囊，仅接近段）+②从下方半空间（反爬锚定果底）
// 在 grasp_geometry；③octomap 场景碰撞（臂/相机受查、工具链豁免）在
// moveit 配置与 scene ACM；④近果低速档（本文件 solver 档位）与接触检测
// （节点侧 contact_monitor，默认关）。接近主路径 = staging 转移；套入/
// 撤退沿轴；返程倒放同一接近轨迹。G/under 单弦档已删。刀具 IO 不在此。
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

#include "peach_manipulation/acm_policy.hpp"
#include "peach_manipulation/grasp_geometry.hpp"
#include "peach_manipulation/math_utils.hpp"
#include "peach_manipulation/protected_zones.hpp"

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

namespace peach_manipulation
{

// tip 姿态不偏离 target_pose 超过 tol_deg 的三轴等宽容差约束集。
// 只挂已齐 LIN（分档要求起点对轴，拦笛卡尔插值中途侧翻）。
// 未齐第一段 LIN-align 不挂：Jazzy ValidateSolution 验每个路点含起点，
// 起点相对目标 >20° 会 INVALID_MOTION_PLAN。staging 转移 PTP 不挂姿态门
// （关节目标本体就是目标姿态的 IK 解）。
inline moveit_msgs::msg::Constraints makeOrientationGate(
  const std::string & link_name, const std::string & frame_id,
  const Eigen::Isometry3d & target_pose, double tol_deg,
  const std::string & name)
{
  moveit_msgs::msg::OrientationConstraint orientation;
  orientation.link_name = link_name;
  orientation.header.frame_id = frame_id;
  orientation.orientation = tf2::toMsg(Eigen::Quaterniond(target_pose.linear()));
  const double tol = tol_deg * kPi / 180.0;
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
// STAGING=主路径（预抓取下方 PTP + 轴向 LIN）；LIN/LIN_ALIGN_THEN_LIN=
// 已在袋底侧的直连短修正；G/under 单弦档已删（见文件头）。
struct ApproachSplit
{
  bool need_lin{true};
  bool need_align{true};
  enum class Kind
  {
    SKIP, LIN, LIN_ALIGN_THEN_LIN, STAGING, BLOCKED
  } kind{
    Kind::STAGING};
  double lin_to_entry_m{0.0};
  double lateral_m{0.0};
  double axial_m{0.0};
  double align_deg{180.0};
  double sweep_deg{0.0};
  double radius_m{0.0};
  // classifyApproach 取到的当前 TCP 快照：后续装配/日志复用，避免
  // 一次接近分档内多次 TF 查询（各带 1s 超时）且保证几何一致。
  std::optional<Eigen::Isometry3d> current_tip;
  // 对轴后绕工具 Z 的刀口滚转。0=keep-roll；规划扫描写入，不参与分档。
  double tool_roll_rad{0.0};
  std::string blocked_reason{"无当前 TCP"};
};

struct GraspTaskConfig
{
  std::string planning_group;  // MoveIt 规划组
  std::string tip_frame;       // IK 末端连杆（当前 tcp）
  std::string base_frame;      // 位姿参考系（base_link）
  std::string free_space_pipeline{"pilz_industrial_motion_planner"};
  std::string free_space_planner{"LIN"};
  double planning_time_s{1.5};
  double velocity_scaling{0.10};       // 接触段（靠近/插入/撤离）
  double acceleration_scaling{0.10};
  double cartesian_step_m{0.005};      // 直线插入步长 [m]
  double cartesian_min_fraction{0.95};  // 直线完成比例；1.0 会因末步离散失败
  double cartesian_precision_m{0.001};
  std::size_t max_solutions{5U};
  double approach_max_duration_s{0.0};
  double approach_max_total_joint_travel_rad{12.0};
  double approach_max_single_joint_travel_rad{6.1};
  // 笛卡尔绕行/姿态审查；任一项 <=0 则跳过该项。
  double approach_max_detour_ratio{1.8};
  double approach_max_chord_deviation_m{0.25};
  double approach_max_recede_m{0.08};
  double staging_max_detour_ratio{1.8};
  double staging_max_chord_deviation_m{0.25};
  double staging_max_recede_m{0.08};
  double approach_max_tcp_rotation_deg{110.0};
  double approach_tcp_rotation_slack_deg{20.0};
  // 果实胶囊回退半径（感知直径无效时的保守值）与开关（axial<=0 关闭
  // 果实审查）。正常半径 = 感知直径/2 + fruit_inflation_m，逐目标随
  // FruitCapsule 参数传入（半无限 BagKeepout 已删，2026-09-14）。
  double approach_keepout_radius_m{0.12};
  double approach_keepout_axial_m{0.12};
  double fruit_inflation_m{0.01};
  // 接触笛卡尔弦长/弧长上限；超过则 skipped_unreachable，不改 PTP。
  double approach_cartesian_max_distance_m{0.80};
  // 预抓取点在入口沿 −axis 后撤量；0=与入口重合（拟合袋底）。轴向 LIN 只走这一段。
  double approach_along_axis_m{0.0};
  // staging=预抓取沿 −axis 再退本值（主路径 PTP 落点，即「预抓取点下方」）。
  double approach_staging_standoff_m{0.10};
  // 侧向小于此值视为已对轴，LIN 是最短直线。
  double approach_max_lateral_m{0.05};
  // 工具 Z 与轴夹角小于此值视为已齐；已齐 LIN 才挂同值姿态路径约束。
  double approach_max_align_deg{20.0};
  // 近果低速档（④层）：staging→预抓取轴向 LIN 与套入/撤退段的独立速度
  // 缩放，低于 velocity_scaling 以限制接触动能；staging PTP 不降档。
  double approach_near_velocity_scaling{0.05};
  std::function<std::optional<Eigen::Isometry3d>()> lookup_current_tip;
  // staging 关节目标（主路径 PTP 落点；keep-roll 及 ±30°/±60° × 当前+4随机种子取最近且无自碰的
  // 最多 5 个候选，按关节距离（腕轴加权）+滚转惩罚升序）。转移逐候选试规划，救弧穿袋囊与
  // 自碰构型。
  struct StagingCandidate
  {
    std::map<std::string, double> joints;
    Eigen::Isometry3d pose;
  };
  std::function<std::vector<StagingCandidate>(
      const Eigen::Isometry3d & staging_pose)> select_goal_joints;
  std::vector<ProtectedZone> protected_zones;  // base 系 AABB → planning scene
  std::function<bool(std::string &)> approach_execution_gate;  // 下发接近轨迹前
  std::function<bool(std::string &)> retreat_execution_gate;   // 撤离不依赖视觉
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
    bool staging_guard = false);
  GraspTaskResult planTaskOnly(
    moveit::task_constructor::Task * active, bool guard_approach,
    std::size_t guard_skip_tail = 0, bool staging_guard = false,
    std::size_t max_solutions = 0);  // 0 = config_.max_solutions；执行路径传 1
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
  // staging 序列（PTP 到预抓取下方 + 轴向 LIN）；正式转移与预览共用。
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
    double velocity_scaling = -1.0) const;  // <0 = config_.velocity_scaling
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
  // ③层工具豁免：先取现行 ACM 再 setEntry 工具链 × <octomap>，回写全表
  // （MoveIt ACM diff 是整表替换，不能只发子方阵）。臂/相机保持受查。
  void applyToolOctomapExemption(
    moveit::planning_interface::PlanningSceneInterface & scene) const;

  std::shared_ptr<moveit::task_constructor::solvers::PipelinePlanner>
  makePilzSolver(
    const std::string & planner_id,
    double velocity_scaling = -1.0) const;  // <0 = config_.velocity_scaling
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
  FruitCapsule pending_fruit_;
  bool inspect_fruit_{false};
  std::string pending_acm_target_id_;
  ContactAcmStage pending_acm_stage_{ContactAcmStage::Transit};
  mutable std::vector<std::string> published_keepout_ids_;  // 上次写入 scene 的 id
  std::vector<trajectory_msgs::msg::JointTrajectory> planned_approach_parts_;
  std::vector<trajectory_msgs::msg::JointTrajectory> last_approach_parts_;
};

}  // namespace peach_manipulation

#endif  // PEACH_MANIPULATION__GRASP_TASK_HPP_
