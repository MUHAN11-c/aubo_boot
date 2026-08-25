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
#ifndef PEACH_MANIPULATION_SKILLS__GRASP_TASK_HPP_
#define PEACH_MANIPULATION_SKILLS__GRASP_TASK_HPP_

#include <Eigen/Geometry>

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "peach_manipulation_skills/protected_zones.hpp"
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

namespace peach_manipulation_skills
{

struct GraspTaskConfig
{
  std::string planning_group;  // MoveIt 规划组
  std::string tip_frame;       // IK 末端连杆（当前 tcp）
  std::string base_frame;      // 位姿参考系（base_link）
  std::string free_space_pipeline{"pilz_industrial_motion_planner"};
  std::string free_space_planner{"PTP"};
  double planning_time_s{1.5};
  double velocity_scaling{0.10};       // 接触段（靠近/插入/撤离）
  double acceleration_scaling{0.10};
  double cartesian_step_m{0.005};      // 直线插入步长 [m]
  double cartesian_min_fraction{0.95};  // 直线完成比例；1.0 会因末步离散失败
  double cartesian_precision_m{0.001};
  std::size_t max_solutions{5U};
  double approach_max_duration_s{20.0};
  double approach_max_total_joint_travel_rad{10.0};
  double approach_max_single_joint_travel_rad{3.2};
  // 过渡点参数保留给 generate_parameter_library；接近已禁止多段笛卡尔爬行。
  double approach_via_max_spacing_m{0.08};
  double approach_via_min_spacing_m{0.03};
  int approach_via_max_points{1};
  // 沿检测轴 LIN 上限；超过则先 PTP 到轴上预抓取点，再短程沿轴进入。
  double approach_cartesian_max_distance_m{0.15};
  // 预抓取点在入口沿 −axis 后撤量；接触 LIN 只走这一段（垂直检测、尽量短）。
  double approach_along_axis_m{0.10};
  // 末端到抓取轴线侧向偏差超过本值则先 PTP 对轴，不斜着笛卡尔顶进去。
  double approach_max_lateral_m{0.05};
  // 末端 Z 与检测轴夹角超过本值则先 PTP 把工具转到轴上。
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

// 接触运动只走 MTC；工具 IO 留在行为树，失败才能进撤离。
// active_task_：正在 plan/execute；preplanned_task_：等待段预规划解。
// 内联重规划前须先等预规划线程退出（settlePreplanBeforeReplan）。
class GraspTask
{
public:
  GraspTask(rclcpp::Node::SharedPtr node, GraspTaskConfig config);
  ~GraspTask();

  // 碰撞感知短路径到入口：距入口近才笛卡尔，否则单段 PTP，禁止 OMPL 与
  // 多段笛卡尔爬行。再沿轴直线插入。execute=false 只规划。
  GraspTaskResult approachAndInsert(
    const Eigen::Isometry3d & entry_tip_pose,
    const Eigen::Vector3d & insertion_axis,
    double insertion_distance_m,
    bool execute);

  // 同 approachAndInsert(false)；成功则把任务留在预规划槽。
  GraspTaskResult preplanApproachAndInsert(
    const Eigen::Isometry3d & entry_tip_pose,
    const Eigen::Vector3d & insertion_axis,
    double insertion_distance_m);

  // 取出预规划槽并下发（只能一次）。无解时 success=false。
  GraspTaskResult executePreplannedApproach();

  void discardPreplanned();  // 幂等清空预规划槽

  // 入口→插入→原轴撤离，只规划不下发。
  GraspTaskResult previewFullContact(
    const Eigen::Isometry3d & entry_tip_pose,
    const Eigen::Vector3d & insertion_axis,
    double insertion_distance_m);

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
  GraspTaskResult preplanOneTask(
    std::unique_ptr<moveit::task_constructor::Task> task,
    std::size_t guard_skip_tail = 1U);
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
    const Eigen::Isometry3d & target_tip_pose);
  std::unique_ptr<moveit::task_constructor::Task> makeInsertOnlyTask(
    const std::string & task_name,
    const Eigen::Vector3d & insertion_axis,
    double insertion_distance_m);
  std::unique_ptr<moveit::task_constructor::SerialContainer> makeApproachInsertSequence(
    const Eigen::Vector3d & insertion_axis,
    double along_axis_m,
    double insertion_distance_m) const;
  void appendPtpToPose(
    moveit::task_constructor::SerialContainer & sequence,
    const Eigen::Isometry3d & target_tip_pose) const;
  void appendAlongAxisMove(
    moveit::task_constructor::SerialContainer & sequence,
    const Eigen::Vector3d & insertion_axis,
    double along_axis_m,
    const std::string & label) const;
  void syncKeepoutCollisionObjects() const;

  std::shared_ptr<moveit::task_constructor::solvers::PipelinePlanner>
  makeFreeSpaceSolver() const;
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
  std::unique_ptr<moveit::task_constructor::Task> preplanned_task_;
  mutable std::vector<std::string> published_keepout_ids_;  // 上次写入 scene 的 id
};

}  // namespace peach_manipulation_skills

#endif  // PEACH_MANIPULATION_SKILLS__GRASP_TASK_HPP_
