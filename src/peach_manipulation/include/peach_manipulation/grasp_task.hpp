// 功能：MTC 接触。到预抓取只走 Pilz LIN / CIRC（TCP 约束轨迹）；沿轴套入
// 与撤退。PTP 不用于接触。刀具 IO 不在此（阶段执行器 / ToolActuator）。
#ifndef PEACH_MANIPULATION__GRASP_TASK_HPP_
#define PEACH_MANIPULATION__GRASP_TASK_HPP_

#include <Eigen/Geometry>

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "peach_manipulation/protected_zones.hpp"
#include <rclcpp/rclcpp.hpp>

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
  // 笛卡尔绕行审查；任一项 <=0 则跳过该项。拦「先远离再绕回」。
  double approach_max_detour_ratio{2.2};
  double approach_max_chord_deviation_m{0.25};
  double approach_max_recede_m{0.08};
  // 过渡点参数保留给 generate_parameter_library；不插 via（会把路径拉长）。
  double approach_via_max_spacing_m{0.08};
  double approach_via_min_spacing_m{0.03};
  int approach_via_max_points{1};
  // 接触笛卡尔弦长/弧长上限；超过则 skipped_unreachable，不改 PTP。
  double approach_cartesian_max_distance_m{0.80};
  // 预抓取点在入口沿 −axis 后撤量；0=与入口重合（拟合袋底）。已对轴时 LIN 只走这一段。
  double approach_along_axis_m{0.0};
  // 侧向小于此值视为已对轴，LIN 是最短直线。
  double approach_max_lateral_m{0.05};
  // 工具 Z 与轴夹角小于此值视为已齐；已齐 LIN 才挂同值姿态路径约束。
  double approach_max_align_deg{20.0};
  std::function<std::optional<Eigen::Isometry3d>()> lookup_current_tip;
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

  // 到预抓取：先由阶段执行器 PTP 回拍照位。直线不穿预抓取球则 LIN（未齐则
  // 先 LIN 原地对齐工具 Z）；直线会穿球则 CIRC 再沿轴 LIN。失败不改 PTP。
  // 再沿轴插入。execute=false 只规划。
  GraspTaskResult approachAndInsert(
    const Eigen::Isometry3d & entry_tip_pose,
    const Eigen::Vector3d & insertion_axis,
    double insertion_distance_m,
    bool execute);

  // 入口→插入→原轴撤离，只规划不下发。
  GraspTaskResult previewFullContact(
    const Eigen::Isometry3d & entry_tip_pose,
    const Eigen::Vector3d & insertion_axis,
    double insertion_distance_m);

  GraspTaskResult moveToPregrasp(
    const Eigen::Isometry3d & entry_tip_pose,
    const Eigen::Vector3d & insertion_axis,
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

private:
  GraspTaskResult planAndMaybeExecute(
    std::unique_ptr<moveit::task_constructor::Task> task,
    bool execute,
    const std::function<bool(std::string &)> & execution_gate,
    bool guard_approach = false,
    std::size_t guard_skip_tail = 0);
  GraspTaskResult planTaskOnly(
    moveit::task_constructor::Task * active, bool guard_approach,
    std::size_t guard_skip_tail = 0);
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
    const Eigen::Vector3d & insertion_axis);
  GraspTaskResult planToPregrasp(
    const std::string & task_name,
    const Eigen::Isometry3d & entry_tip_pose,
    const Eigen::Vector3d & insertion_axis,
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
    bool gate_orientation) const;
  void appendCircToPose(
    moveit::task_constructor::SerialContainer & sequence,
    const Eigen::Isometry3d & target_tip_pose,
    const Eigen::Vector3d & center,
    const std::string & label) const;
  void appendApproachToPregrasp(
    moveit::task_constructor::SerialContainer & sequence,
    const Eigen::Isometry3d & entry_tip_pose,
    const Eigen::Vector3d & insertion_axis) const;
  void appendAlongAxisMove(
    moveit::task_constructor::SerialContainer & sequence,
    const Eigen::Vector3d & insertion_axis,
    double along_axis_m,
    const std::string & label) const;
  void syncKeepoutCollisionObjects() const;

  std::shared_ptr<moveit::task_constructor::solvers::PipelinePlanner>
  makePilzSolver(const std::string & planner_id) const;
  std::shared_ptr<moveit::task_constructor::solvers::PipelinePlanner>
  makeLinSolver() const;
  std::shared_ptr<moveit::task_constructor::solvers::PipelinePlanner>
  makeCircSolver() const;
  std::shared_ptr<moveit::task_constructor::solvers::CartesianPath>
  makeCartesianSolver() const;
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
  mutable std::vector<std::string> published_keepout_ids_;  // 上次写入 scene 的 id
};

}  // namespace peach_manipulation

#endif  // PEACH_MANIPULATION__GRASP_TASK_HPP_
