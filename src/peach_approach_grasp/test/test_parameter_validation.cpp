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
//
// 参数校验钩子用例：execution→grasp→tool 依赖链（on-set 验证钩子，按
// "现行值叠加本批改动"合并判定）与生成定义的范围校验（非法值拒绝）。
// 进程内构造真实节点（不初始化 MoveIt，无需硬件/仿真）。
#include <gtest/gtest.h>

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "approach_grasp_node_impl.hpp"

namespace
{

TEST(ApproachGraspParameterValidation, EnableDependencyChain)
{
  rclcpp::init(0, nullptr);
  const auto node = std::make_shared<peach_approach_grasp::ApproachGraspNode>();

  // 默认 execution/grasp/tool 全关：单独开 grasp 或 tool 违链被拒。
  auto result = node->set_parameters_atomically(
    {rclcpp::Parameter("grasp.enabled", true)});
  EXPECT_FALSE(result.successful);
  EXPECT_FALSE(node->get_parameter("grasp.enabled").as_bool());
  result = node->set_parameters_atomically(
    {rclcpp::Parameter("tool.enabled", true)});
  EXPECT_FALSE(result.successful);

  // 同批满足依赖链则接受，并实际写入。
  result = node->set_parameters_atomically({
      rclcpp::Parameter("execution.enabled", true),
      rclcpp::Parameter("grasp.enabled", true)});
  EXPECT_TRUE(result.successful);
  EXPECT_TRUE(node->get_parameter("execution.enabled").as_bool());
  EXPECT_TRUE(node->get_parameter("grasp.enabled").as_bool());

  // grasp 已开时单关 execution 违链（合并判定覆盖"现行值+本批"）。
  result = node->set_parameters_atomically(
    {rclcpp::Parameter("execution.enabled", false)});
  EXPECT_FALSE(result.successful);
  EXPECT_TRUE(node->get_parameter("execution.enabled").as_bool());
  rclcpp::shutdown();
}

TEST(ApproachGraspParameterValidation, RangeValidationRejectsInvalid)
{
  rclcpp::init(0, nullptr);
  const auto node = std::make_shared<peach_approach_grasp::ApproachGraspNode>();

  // 速度缩放限 (0, 1]：越界拒绝且不写入。
  auto result = node->set_parameters_atomically(
    {rclcpp::Parameter("moveit.velocity_scaling", 2.0)});
  EXPECT_FALSE(result.successful);
  result = node->set_parameters_atomically(
    {rclcpp::Parameter("moveit.velocity_scaling", 0.0)});
  EXPECT_FALSE(result.successful);
  EXPECT_DOUBLE_EQ(node->get_parameter("moveit.velocity_scaling").as_double(), 0.05);

  result = node->set_parameters_atomically(
    {rclcpp::Parameter("moveit.velocity_scaling", 0.2)});
  EXPECT_TRUE(result.successful);
  EXPECT_DOUBLE_EQ(node->get_parameter("moveit.velocity_scaling").as_double(), 0.2);

  // 计数/时长下限：maximum_moves ≥1、service_s >0。
  result = node->set_parameters_atomically(
    {rclcpp::Parameter("scan.maximum_moves", 0)});
  EXPECT_FALSE(result.successful);
  result = node->set_parameters_atomically(
    {rclcpp::Parameter("timeouts.service_s", -1.0)});
  EXPECT_FALSE(result.successful);
  rclcpp::shutdown();
}

}  // namespace
