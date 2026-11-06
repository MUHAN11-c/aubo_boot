// 功能：MTC 接触。约束四层：①果实胶囊审查（工具有限圆柱 vs 感知果实
// 胶囊，仅接近段）+②从下方半空间（反爬锚定果底）在 grasp_geometry；
// ③场景障碍快照 ACM（保护相机；updater 关闭，<octomap> 保留名无害）
// 在 acm_policy；④近果低速档（本文件 solver）与接触检测（节点侧
// contact_monitor，默认关）。接近：几何允许直连时先完整 LIN（含对轴）；
// 直接 PTP/LIN 到轨迹净空点，再单条轴向套入；视觉预抓取不另停。
// 返程先沿轴到净空点，再倒放同一实际转移轨迹。
// 刀具 IO 不在此。
#ifndef PEACH_MANIPULATION__GRASP_TASK_HPP_
#define PEACH_MANIPULATION__GRASP_TASK_HPP_

#include <Eigen/Geometry>

#include <atomic>
#include <cstdint>
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
#include "peach_arm/staging_candidates.hpp"
#include "peach_arm/protected_zones.hpp"

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
// 未齐姿态走直接 PTP：Jazzy ValidateSolution 验每个路点含起点，
// 起点相对目标 >20° 的 LIN 会 INVALID_MOTION_PLAN；PTP 关节目标即 IK 解。
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
// STAGING=直接 PTP 到净空点 A；LIN=已对轴、果实几何允许的单条直连。
// 视觉预抓取 N 不再是另一个运动段的停点。
struct ApproachSplit
{
  bool need_lin{true};     ///< True=需要转移到净空点。
  bool need_align{true};   ///< True=直接 PTP 需完成对轴。
  enum class Kind
  {
    SKIP,                 ///< 已在净空点，无需接近。
    LIN,                  ///< 已齐，直连 LIN。
    STAGING,              ///< 直接 PTP 到轨迹净空停点；不经中间走廊。
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
  double acceleration_scaling{0.10};      ///< 兼容旧配置；MTC 实际 a=min(1,2v)。
  double cartesian_step_m{0.005};         ///< 直线插入步长 [m]。
  double cartesian_min_fraction{0.95};    ///< 直线完成比例；1.0 会因末步离散失败。
  double cartesian_precision_m{0.001};    ///< 笛卡尔精度 [m]。
  std::size_t max_solutions{5U};          ///< 最多尝试 IK/规划解数。
  double approach_max_duration_s{0.0};    ///< 接近时限 [s]；0=不限。
  double approach_max_total_joint_travel_rad{12.0};   ///< 接近总关节行程上限 [rad]。
  double approach_max_single_joint_travel_rad{6.1};   ///< 单关节行程上限 [rad]。
  double approach_max_detour_ratio{1.5};              ///< 笛卡尔绕行比；≤0 跳过。
  double approach_max_chord_deviation_m{0.12};        ///< 弦偏差上限 [m]。
  double approach_max_recede_m{0.08};                 ///< 回退上限 [m]。
  double staging_max_detour_ratio{1.5};
  double staging_max_chord_deviation_m{0.12};
  double staging_max_recede_m{0.08};
  double approach_max_tcp_rotation_deg{110.0};        ///< TCP 转角上限 [deg]。
  double approach_tcp_rotation_slack_deg{20.0};       ///< 转角松弛 [deg]。
  double approach_keepout_radius_m{0.12};             ///< 果实胶囊回退半径 [m]。
  double approach_keepout_axial_m{0.12};              ///< 轴向审查长度 [m]；≤0 关闭。
  double fruit_inflation_m{0.01};                     ///< 感知半径外膨胀 [m]。
  double approach_cartesian_max_distance_m{0.80};     ///< 接触笛卡尔弦长上限 [m]。
  double approach_along_axis_m{0.0};                  ///< 预抓取相对入口沿 −axis 后撤 [m]。
  double approach_clearance_m{0.05};                 ///< 轨迹点在视觉预抓取后方的净空 [m]。
  double approach_max_align_deg{20.0};                ///< 小于此值视为已齐 [deg]。
  double approach_near_velocity_scaling{0.10};        ///< 近果低速档（仅套入/撤退）。
  // 工具档案（W5-6，GPL yaml tool.*；默认值=原三处硬编码）：
  std::vector<std::string> tool_links{
    "tool_axis", "cutting_plane", "tcp", "sleeve_mouth",
    "tool_body_link", "quick_changer_link"};  ///< 工具链连杆（整图 octomap 豁免）。
  std::vector<std::string> contact_tool_links{
    "sleeve_mouth", "tcp", "tool_axis",
    "cutting_plane"};  ///< 接触阶段 × 目标对象豁免的连杆。
  // 场景障碍豁免（2026-09-29 避障只为保护相机；条目组装见
  // acm_policy::obstacleExemptionEntries，纯核可测）：
  std::vector<std::string> obstacle_object_ids{"peach_scene_obstacles"};
  ///< 场景障碍对象 id（快照节点写入）；豁免连杆 × 这些对象 + <octomap>。
  std::vector<std::string> obstacle_guard_links{"camera_body_link"};
  ///< guard 开时唯一受查连杆（避障目标=保护相机）。
  bool obstacle_guard_enabled{false};  ///< 枝叶方块默认不挡相机；true 才受查。
  double tool_body_length_m{0.200};  ///< 工具筒体长 [m]（①层审查，tcp.xacro 对齐）。
  double tool_body_radius_m{0.060};  ///< 工具筒体半径 [m]。
  std::function<std::optional<Eigen::Isometry3d>()> lookup_current_tip;  ///< 查当前 TCP。
  /// staging 滚转梯子 IK 返回候选列表（关节解、滚转rad、加权关节代价）；
  /// 执行按 0 / ±15° / ±30° / ±45° 惰性请求同绝对滚转组；组内每档
  /// 当前种子+随机重启≤3，FULL 从净空停点沿轴到套入终点全部验证。
  /// 原候选类型名保留；落点为唯一轨迹净空点，不含世界垂直或中段点。
  std::function<std::vector<StagingCandidate>(
      const Eigen::Isometry3d & staging_pose,
      const std::vector<Eigen::Isometry3d> & corridor,
      const std::vector<double> & roll_group,
      const std::map<std::string, double> & seed_override)> staging_ik;
  /// 某刀口滚转从预抓取沿轴插完 travel 是否达标（与 CheckReachability 同形）。
  /// 空或不调用 = 接近不筛套入直线（仅预抓取）。
  std::function<bool(
      double roll_rad, const Eigen::Isometry3d & entry, double travel_m)> sleeve_roll_feasible;
  std::vector<ProtectedZone> protected_zones;  // base 系 AABB → planning scene
  std::function<bool(std::string &)> approach_execution_gate;  // 下发接近轨迹前
  std::function<bool(std::string &)> retreat_execution_gate;   // 撤离不依赖视觉
  // MTC 执行有界等待：MTC Task 不暴露 stop，超时/取消经 execution_stop
  // （节点注入 move_group_->stop()，MGI stop 打节点级停止服务）兜底。
  std::function<void()> execution_stop;
  double execute_timeout_s{5.0};  ///< MTC 解执行有界等待 [s]。
  /// 接触链 /rosout 子 logger（peach_arm.contact）；空则回退 node logger。
  std::optional<rclcpp::Logger> contact_logger;
};

struct GraspTaskResult
{
  bool success{false};
  bool execution_started{false};  // 已向控制器下发
  bool retry_forbidden{false};  // 取消/授权拒绝不能通过更换候选重试
  std::string reason;
};

// 接触运动只走 MTC；stages.cpp 编排工具 IO，正常套入/工具阶段通过才进入撤离。
// active_task_：正在 plan/execute 的任务，串行独占（task_mutex_ 保护）。
class GraspTask
{
public:
  GraspTask(rclcpp::Node::SharedPtr node, GraspTaskConfig config);
  ~GraspTask();

  void setContactAcm(const std::string & target_id, ContactAcmStage stage);

  // 观测性（P0）：接触段日志的 target 上下文；周期入口刷新，空=非周期调用
  void setTargetContext(const std::string & target_id);

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
    const FruitCapsule & fruit, bool prepare_execution = false, bool include_retreat = true,
    const std::string & start_named = {});

  GraspTaskResult moveToPregrasp(
    const Eigen::Isometry3d & entry_tip_pose,
    const Eigen::Vector3d & insertion_axis,
    const FruitCapsule & fruit,
    bool execute,
    double insert_travel_m = 0.0);

  GraspTaskResult sleeveLinear(
    const Eigen::Isometry3d & entry_tip_pose,
    const Eigen::Vector3d & insertion_axis,
    double insertion_distance_m,
    bool execute,
    const std::function<bool(std::string &)> & execution_gate);

  GraspTaskResult retreat(
    const Eigen::Isometry3d & entry_tip_pose,
    const Eigen::Vector3d & insertion_axis,
    double insertion_distance_m,
    bool execute);

  void cancel();  // preempt 当前 active 任务

  // 最近一次过护栏且已执行成功的接近段；保留段界以逐段原路返程。
  std::vector<trajectory_msgs::msg::JointTrajectory> lastApproachTrajectories() const;

private:
  enum class TimingCheck { NONE, WHOLE_GOAL, CONTACT_LEGS };
  GraspTaskResult planAndMaybeExecute(
    std::unique_ptr<moveit::task_constructor::Task> task,
    bool execute,
    const std::function<bool(std::string &)> & execution_gate,
    bool guard_approach = false,
    std::size_t guard_skip_tail = 0,
    bool staging_guard = false,
    bool cartesian_per_part = false, bool prepare_contact_execution = false,
    const std::vector<Eigen::Vector3d> & endpoint_targets = {});
  GraspTaskResult planTaskOnly(
    moveit::task_constructor::Task * active, bool guard_approach,
    std::size_t guard_skip_tail, bool staging_guard, TimingCheck timing,
    bool cartesian_per_part, const std::vector<Eigen::Vector3d> & endpoint_targets);
  GraspTaskResult executeSolution(
    moveit::task_constructor::Task * active,
    const std::function<bool(std::string &)> & execution_gate);

  std::unique_ptr<moveit::task_constructor::Task> makeTaskShell(
    const std::string & task_name) const;
  std::unique_ptr<moveit::task_constructor::Task> makeApproachOnlyTask(
    const std::string & task_name,
    const Eigen::Isometry3d & entry_tip_pose,
    const Eigen::Vector3d & insertion_axis,
    const ApproachSplit & split);
  std::unique_ptr<moveit::task_constructor::Task> makePtpTransferTask(
    const std::string & task_name, const std::map<std::string, double> & joints);
  GraspTaskResult preflightContact(
    std::unique_ptr<moveit::task_constructor::Task> task,
    const TrajectoryApproachGeometry & geometry, bool has_transfer, bool ptp_transfer,
    bool include_retreat = true);
  bool tryRolledApproach(
    const std::string & task_name,
    const Eigen::Isometry3d & entry_tip_pose,
    const Eigen::Vector3d & insertion_axis,
    ApproachSplit split,
    ApproachSplit::Kind kind,
    bool execute,
    GraspTaskResult & last,
    double insert_travel_m, bool contact_preview = false, bool include_retreat = true);
  bool tryStagingTransit(
    const std::string & task_name,
    const Eigen::Isometry3d & entry_tip_pose,
    const Eigen::Vector3d & insertion_axis,
    const ApproachSplit & split,
    bool execute,
    GraspTaskResult & last,
    double insert_travel_m, bool contact_preview = false, bool include_retreat = true);
  GraspTaskResult planToPregrasp(
    const std::string & task_name,
    const Eigen::Isometry3d & entry_tip_pose,
    const Eigen::Vector3d & insertion_axis,
    const ApproachSplit & split,
    const FruitCapsule & fruit,
    bool execute,
    double insert_travel_m = 0.0, bool contact_preview = false, bool include_retreat = true);
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
  void syncKeepoutCollisionObjects() const;

public:
  // ③层工具豁免（周期级）策略（W5-5 合并原整图/轮内两份 90% 重复实现）：
  //   WholeMap  —— 整表回写 工具链 × <octomap> = allowed（on_activate 后台
  //                线程应用一次，Survey/观察/接近全程生效）；
  //   PerTarget —— 整图豁免（若策略开启）+ 接触阶段 指定目标对象 × 接触
  //                连杆（pending_acm_* 由 setContactAcm 预置）。
  // <octomap> 保留兼容名，当前不用 updater。场景障碍只保护配置的 guard
  // 连杆；其余豁免连杆由 tool_links 指定。guard 关闭时相机也显式豁免。
  enum class OctomapExemptionPolicy
  {
    WholeMap,
    PerTarget
  };

  // 整图豁免 static 入口（无对象依赖）：tool_links 来自调用方参数档案；
  // obstacle_spec 含相机 guard 开关（2026-09-29 避障只为保护相机）。
  static void applyWholeOctomapToolExemption(
    const rclcpp::Logger & logger,
    moveit::planning_interface::PlanningSceneInterface & scene,
    const std::vector<std::string> & tool_links,
    const ObstacleExemptionSpec & obstacle_spec,
    const std::function<bool()> & still_current = {});
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
    const ObstacleExemptionSpec & obstacle_spec,
    const std::vector<std::string> & contact_tool_links,
    const std::string & per_target_id,
    ContactAcmStage per_target_stage,
    const std::function<bool()> & still_current = {});

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
  std::unique_ptr<moveit::task_constructor::stages::MoveTo> makeCartesianMoveToPoint(
    const std::string & label, const Eigen::Vector3d & target) const;

  rclcpp::Node::SharedPtr node_;
  GraspTaskConfig config_;
  struct RobotModelResources;
  std::shared_ptr<RobotModelResources> model_resources_;
  std::mutex task_mutex_;
  std::atomic<std::uint64_t> cancel_generation_{0};
  std::unique_ptr<moveit::task_constructor::Task> active_task_;
  std::atomic_bool task_abandoned_{false};  // 弃等线程已接管任务（reset 前须 release）
  std::unique_ptr<peach_arm::RetireBucket> retiring_;  // 弃等线程桶（execution_guard.hpp）
  FruitCapsule pending_fruit_;
  bool inspect_fruit_{false};
  std::map<std::string, double> planning_start_override_;  // 仅命名起点预览的局部关节种子
  std::string pending_acm_target_id_;
  ContactAcmStage pending_acm_stage_{ContactAcmStage::Transit};
  std::string target_context_;  ///< 当前周期 target_id（仅日志上下文，不参与判定）
  std::string tctx() const;  ///< 日志前缀 "[target_id] "；无周期上下文为空串
  std::optional<rclcpp::Logger> contact_logger_;
  rclcpp::Logger contactLogger() const
  {
    return contact_logger_ ? *contact_logger_ : node_->get_logger();
  }
  mutable std::vector<std::string> published_keepout_ids_;  // 上次写入 scene 的 id
  std::vector<trajectory_msgs::msg::JointTrajectory> planned_approach_parts_;
  std::vector<trajectory_msgs::msg::JointTrajectory> last_approach_parts_;
};

}  // namespace peach_arm

#endif  // PEACH_MANIPULATION__GRASP_TASK_HPP_
