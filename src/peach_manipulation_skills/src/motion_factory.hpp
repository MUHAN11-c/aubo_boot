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
// 运动接口工厂（包内私有头不安装）：按 motion.impl 名字符串创建实现。
// 显式注册清单，一处 if 链，不用自注册宏（重构协议 2.14）；未知名抛
// std::invalid_argument 且信息列出可用名。工厂涉及 MoveIt/TF 类型，故与
// 纯核三件套的 impl_factory.hpp 分离。
#ifndef MOTION_FACTORY_HPP_
#define MOTION_FACTORY_HPP_

#include <tf2_ros/buffer.h>

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include "motion_interface_impl.hpp"
#include "peach_manipulation_skills/motion_interface_base.hpp"

namespace peach_manipulation_skills
{

// 运动接口注册表：moveit_motion=MoveGroupInterface 规划/执行（当前唯一实现）。
// 前置/后置与线程模型见 MoveItMotionInterface 类注释。
inline std::unique_ptr<MotionInterfaceBase> createMotionInterface(
  const std::string & name,
  moveit::planning_interface::MoveGroupInterface * move_group,
  tf2_ros::Buffer * tf_buffer,
  const rclcpp::Logger & logger,
  const rclcpp::Clock::SharedPtr & clock,
  const MoveItMotionConfig & config,
  std::function<bool(std::string &)> safety_gate,
  std::function<void(const std::string &)> safety_block_hook)
{
  if (name == "moveit_motion") {
    return std::make_unique<MoveItMotionInterface>(
      move_group, tf_buffer, logger, clock, config, std::move(safety_gate),
      std::move(safety_block_hook));
  }
  throw std::invalid_argument(
          "未知 motion.impl: '" + name + "'（可用: moveit_motion）");
}

}  // namespace peach_manipulation_skills

#endif  // MOTION_FACTORY_HPP_
