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
// 运动接口默认实现（注册名 moveit_motion，包内私有头不安装）：
// MoveGroupInterface + TF 的规划/执行体。行为与原 ApproachGraspNode 成员函数
// 内联实现逐行等价；节点经 src/motion_factory.hpp 按 motion.impl 创建。
#ifndef MOTION_INTERFACE_IMPL_HPP_
#define MOTION_INTERFACE_IMPL_HPP_

#include <Eigen/Geometry>
#include <tf2_ros/buffer.h>

#include <functional>
#include <optional>
#include <string>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include "peach_manipulation_skills/motion_interface_base.hpp"

namespace peach_manipulation_skills
{

// moveit_motion 的运行配置（默认值以 config/approach_grasp.yaml 为权威源）。
struct MoveItMotionConfig
{
  std::string base_frame;
  std::string tip_frame;  // 规划/IK 末端连杆（MoveIt 组 tip_link，当前为 tcp）
  std::string camera_frame;
  std::string tool_frame;
  std::string pilz_pipeline;
  std::string fallback_pipeline;
  // 自由空间转移速度档（观察视点、拍照位姿往返）。
  double transit_velocity_scaling{0.10};
  double transit_acceleration_scaling{0.10};
};

// MotionInterfaceBase 的 MoveIt 默认实现。
// 用途：tip/camera/tool 位姿规划执行、TF 查询、拍照位姿往返。
// 生命周期：节点持有（unique_ptr<MotionInterfaceBase>）；本对象不拥有
//   MoveGroup/TF Buffer，仅引用节点持有的实例，析构先于它们发生由节点保证。
// 线程安全：与节点既有同步模型一致（周期运行独占调用），内部不新增线程。
// 可替换性：经 createMotionInterface 按名创建；安全门经回调注入，I5 不得旁路。
class MoveItMotionInterface : public MotionInterfaceBase
{
public:
  // 前置：move_group/tf_buffer 非空且生命周期长于本对象；clock 为节点时钟
  //   （规划目标 header 时间戳）；safety_gate 为硬件安全门回调（节点
  //   safetyReady），所有 execute 路径下发前必须复核；safety_block_hook 在
  //   执行被安全门拦下时由节点投影状态（可为空）。
  MoveItMotionInterface(
    moveit::planning_interface::MoveGroupInterface * move_group,
    tf2_ros::Buffer * tf_buffer,
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock,
    MoveItMotionConfig config,
    std::function<bool(std::string &)> safety_gate,
    std::function<void(const std::string &)> safety_block_hook);

  std::optional<Eigen::Isometry3d> lookupTransform(
    const std::string & target, const std::string & source) override;
  bool planOrMoveTip(
    const Eigen::Isometry3d & tip_pose, const std::string & planner_id,
    bool execute, const std::string & label, bool allow_fallback) override;
  bool planOrMoveCamera(
    const Eigen::Isometry3d & camera_pose, const std::string & planner_id,
    bool execute, const std::string & label, bool allow_fallback) override;
  bool planOrMoveTool(
    const Eigen::Isometry3d & tool_pose, const std::string & planner_id,
    const std::string & label) override;
  bool goToPhotoPose(
    const std::string & named_target, bool execute, std::string & message) override;

private:
  moveit::planning_interface::MoveGroupInterface * move_group_;
  tf2_ros::Buffer * tf_buffer_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  MoveItMotionConfig config_;
  std::function<bool(std::string &)> safety_gate_;
  std::function<void(const std::string &)> safety_block_hook_;
};

}  // namespace peach_manipulation_skills

#endif  // MOTION_INTERFACE_IMPL_HPP_
