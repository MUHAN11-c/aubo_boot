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
#include <string>

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
}  // namespace moveit::task_constructor

namespace peach_manipulation_skills
{

struct GraspTaskConfig
{
  std::string planning_group;
  std::string tip_frame;  // IK/规划末端连杆（MoveIt 组 tip_link，当前为 tcp）
  std::string base_frame;
  std::string free_space_pipeline{"ompl"};
  std::string free_space_planner{"RRTConnectkConfigDefault"};
  double planning_time_s{5.0};
  double velocity_scaling{0.05};
  double acceleration_scaling{0.05};
  double cartesian_step_m{0.005};
  double cartesian_precision_m{0.001};
  std::size_t max_solutions{5U};
  // MTC 完成规划后、真正下发前再次检查机器人状态和目标新鲜度。
  std::function<bool(std::string &)> approach_execution_gate;
  std::function<bool(std::string &)> retreat_execution_gate;
};

struct GraspTaskResult
{
  bool success{false};
  bool execution_started{false};
  std::string reason;
};

// MTC 只负责接触动作的运动学序列。工具 IO 和感知反馈留在行为树层，避免把外部副作用
// 隐藏进 MoveIt 轨迹，才能在工具失败时明确进入撤离分支。
//
// 线程模型（2.13-E3 预规划后）：BT 工作线程之外新增一个预规划线程
// （preplanApproachAndInsert 在其上运行）。task_mutex_ 保护 active_task_ /
// preplanned_task_ 两个槽；任一时刻至多一个任务在 plan/execute（调用端纪律：
// 内联重规划前必须先等预规划线程退出，见 bt_nodes.cpp settlePreplanBeforeReplan）。
// MTC Task 的 plan/execute 分离复用是官方支持的用法（solution 持有完整轨迹，
// execute 只负责下发）；唯一前提是规划时刻的 CurrentState 采样到执行前机械臂
// 未运动——再确认等待段恰好静止，语义成立。
class GraspTask
{
public:
  GraspTask(rclcpp::Node::SharedPtr node, GraspTaskConfig config);
  ~GraspTask();

  GraspTaskResult approachAndInsert(
    const Eigen::Isometry3d & entry_tip_pose,
    const Eigen::Vector3d & insertion_axis,
    double insertion_distance_m,
    bool execute);

  // 预规划（2.13-E3 plan-while-waiting）：与 approachAndInsert(execute=false)
  // 同结构，但规划成功后任务连同解移入 preplanned_task_ 槽保留，供
  // executePreplannedApproach 复用；失败则丢弃。预期在独立预规划线程调用，
  // 运行期间可被 cancel() preempt。
  GraspTaskResult preplanApproachAndInsert(
    const Eigen::Isometry3d & entry_tip_pose,
    const Eigen::Vector3d & insertion_axis,
    double insertion_distance_m);

  // 执行预规划解：取出 preplanned_task_（取出即清槽，不可二次执行），复核
  // approach 执行门后下发。前置：调用方已按 PreplanSlot 判定复用成立且预规划
  // 线程已退出。无可用预规划解时返回 success=false（execution_started=false）。
  GraspTaskResult executePreplannedApproach();

  // 丢弃预规划槽中的任务（不复用/取消/周期结束路径）；幂等。
  void discardPreplanned();

  // 在同一 MTC 解中预览“到入口→插入→原轴撤离”，硬编码只规划，永不执行。
  GraspTaskResult previewFullContact(
    const Eigen::Isometry3d & entry_tip_pose,
    const Eigen::Vector3d & insertion_axis,
    double insertion_distance_m);

  GraspTaskResult retreat(
    const Eigen::Vector3d & insertion_axis,
    double retreat_distance_m,
    bool execute);

  void cancel();

private:
  GraspTaskResult planAndMaybeExecute(
    std::unique_ptr<moveit::task_constructor::Task> task,
    bool execute,
    const std::function<bool(std::string &)> & execution_gate);

  // plan/execute 拆分原语（预规划复用与 planAndMaybeExecute 共用）：
  // planTaskOnly 只规划+发布 introspection，成功时解留在任务对象内；
  // executeSolution 复核执行门后下发首解。两者都不动 active/preplanned 槽。
  GraspTaskResult planTaskOnly(moveit::task_constructor::Task * active);
  GraspTaskResult executeSolution(
    moveit::task_constructor::Task * active,
    const std::function<bool(std::string &)> & execution_gate);

  // 接近/插入任务搭建（approachAndInsert 与 preplanApproachAndInsert 共用）。
  std::unique_ptr<moveit::task_constructor::Task> makeApproachInsertTask(
    const std::string & task_name,
    const Eigen::Isometry3d & entry_tip_pose,
    const Eigen::Vector3d & insertion_axis,
    double insertion_distance_m);

  // 三个任务共用的 solver/stage 搭建工厂：集中配置，行为与原内联实现一致。
  std::shared_ptr<moveit::task_constructor::solvers::PipelinePlanner>
  makeFreeSpaceSolver() const;
  std::shared_ptr<moveit::task_constructor::solvers::CartesianPath>
  makeCartesianSolver() const;
  std::unique_ptr<moveit::task_constructor::stages::MoveTo> makeMoveToEntry(
    const std::shared_ptr<moveit::task_constructor::solvers::PipelinePlanner> & solver,
    const Eigen::Isometry3d & entry_tip_pose) const;
  // 轴向直线段：direction 内部归一化，距离固定为 [distance_m, distance_m]。
  std::unique_ptr<moveit::task_constructor::stages::MoveRelative> makeLinearMove(
    const std::string & label,
    const std::shared_ptr<moveit::task_constructor::solvers::CartesianPath> & solver,
    const Eigen::Vector3d & direction, double distance_m) const;

  rclcpp::Node::SharedPtr node_;
  GraspTaskConfig config_;
  std::mutex task_mutex_;
  std::unique_ptr<moveit::task_constructor::Task> active_task_;
  // 预规划任务槽（2.13-E3）：preplanApproachAndInsert 成功后保留任务与解，
  // executePreplannedApproach 取出执行，discardPreplanned 丢弃。
  std::unique_ptr<moveit::task_constructor::Task> preplanned_task_;
};

}  // namespace peach_manipulation_skills

#endif  // PEACH_MANIPULATION_SKILLS__GRASP_TASK_HPP_
