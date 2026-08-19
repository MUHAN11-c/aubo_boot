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
// 职责：运动执行职责（TF 查询 + MoveIt 规划/执行）的可替换接口（重构协议
// 2.14：抽象基类、签名最小化、纯数据输入输出、零 ROS 类型——Eigen 与
// std 类型视为纯数据）。
// 契约：调用面以 bt_nodes / 预览与拍照位姿服务的实际使用为准；真实运动
// 下发前实现必须复核注入的硬件安全门（I5：不得旁路）。调用端只持
// std::unique_ptr<MotionInterfaceBase>，经 src/motion_factory.hpp 按名字
// 创建（工厂涉及 MoveIt/TF 类型，故为包内私有头）。
#ifndef PEACH_MANIPULATION_SKILLS__MOTION_INTERFACE_BASE_HPP_
#define PEACH_MANIPULATION_SKILLS__MOTION_INTERFACE_BASE_HPP_

#include <Eigen/Geometry>

#include <optional>
#include <string>

namespace peach_manipulation_skills
{

// 运动接口抽象基类。
// 用途：观察视点/工具位姿的规划与执行、坐标变换查询、拍照位姿往返。
// 生命周期：由节点在 MoveIt 初始化后及参数重载时经工厂创建，unique_ptr
//   独占持有；实现不拥有 MoveGroup/TF Buffer，仅引用节点持有的实例。
// 线程安全：与节点既有同步模型一致——BT 工作线程与 executor 回调互斥
//   使用（周期运行独占），实现内部不新增线程。
// 可替换性：注册名见 src/motion_factory.hpp（默认实现 moveit_motion）。
class MotionInterfaceBase
{
public:
  virtual ~MotionInterfaceBase() = default;

  // 查询 target<-source 的最新变换。
  // 后置：失败返回 std::nullopt（实现负责记录原因日志）。
  virtual std::optional<Eigen::Isometry3d> lookupTransform(
    const std::string & target, const std::string & source) = 0;

  // 以 tip 连杆目标位姿规划（execute=true 时并执行）。
  // 前置：planner_id 为实现认识的规划器标识（如 PTP/LIN）；allow_fallback
  //   允许直线规划失败时回退自由空间管线。
  // 后置：返回规划（及执行）是否成功；execute=true 时实现必须在下发前复核
  //   硬件安全门（I5）。
  // 失败语义：返回 false，不抛异常。
  virtual bool planOrMoveTip(
    const Eigen::Isometry3d & tip_pose, const std::string & planner_id,
    bool execute, const std::string & label, bool allow_fallback) = 0;

  // 以相机目标位姿规划/执行（内部经 TF 换算到 tip）。
  virtual bool planOrMoveCamera(
    const Eigen::Isometry3d & camera_pose, const std::string & planner_id,
    bool execute, const std::string & label, bool allow_fallback) = 0;

  // 以工具连杆目标位姿规划并执行（恒 execute，无管线回退）。
  virtual bool planOrMoveTool(
    const Eigen::Isometry3d & tool_pose, const std::string & planner_id,
    const std::string & label) = 0;

  // 移动到 SRDF 命名状态（拍照位姿）：先点对点管线规划，失败回退自由空间；
  // execute=false 时仅规划。真实下发前必须复核硬件安全门（I5）。
  // 后置：message 始终写入面向操作员的结果描述（成功/失败原因）。
  virtual bool goToPhotoPose(
    const std::string & named_target, bool execute, std::string & message) = 0;
};

}  // namespace peach_manipulation_skills

#endif  // PEACH_MANIPULATION_SKILLS__MOTION_INTERFACE_BASE_HPP_
