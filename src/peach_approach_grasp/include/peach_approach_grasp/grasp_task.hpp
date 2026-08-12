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
#ifndef PEACH_APPROACH_GRASP__GRASP_TASK_HPP_
#define PEACH_APPROACH_GRASP__GRASP_TASK_HPP_

#include <Eigen/Geometry>

#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace moveit::task_constructor
{
class Task;
}  // namespace moveit::task_constructor

namespace peach_approach_grasp
{

struct GraspTaskConfig
{
  std::string planning_group;
  std::string wrist_frame;
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
class GraspTask
{
public:
  GraspTask(rclcpp::Node::SharedPtr node, GraspTaskConfig config);
  ~GraspTask();

  GraspTaskResult approachAndInsert(
    const Eigen::Isometry3d & entry_wrist_pose,
    const Eigen::Vector3d & insertion_axis,
    double insertion_distance_m,
    bool execute);

  // 在同一 MTC 解中预览“到入口→插入→原轴撤离”，硬编码只规划，永不执行。
  GraspTaskResult previewFullContact(
    const Eigen::Isometry3d & entry_wrist_pose,
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

  rclcpp::Node::SharedPtr node_;
  GraspTaskConfig config_;
  std::mutex task_mutex_;
  std::unique_ptr<moveit::task_constructor::Task> active_task_;
};

}  // namespace peach_approach_grasp

#endif  // PEACH_APPROACH_GRASP__GRASP_TASK_HPP_
