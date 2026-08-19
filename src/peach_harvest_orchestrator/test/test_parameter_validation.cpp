// Copyright 2026 aubo_e5_ros2_ws authors
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
//    * Neither the name of the aubo_e5_ros2_ws authors nor the names of its
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
// 参数定义校验用例（generate_parameter_library 生成的范围校验）：
// 启动期非法覆盖值抛 InvalidParameterValueException；运行期非法 set 被拒、
// 合法 set 接受且监听器快照即时可见（readiness.* 热读依赖此路径）。
#include <gtest/gtest.h>

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "peach_harvest_orchestrator/harvest_orchestrator_node_parameters.hpp"

namespace
{

TEST(OrchestratorParameterValidation, InvalidStartupValueThrows)
{
  rclcpp::init(0, nullptr);
  // max_rounds 下限为 1（原 on_configure 钳制，现由参数定义校验直接拒绝）
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("harvest.max_rounds", 0)});
  const auto node = std::make_shared<rclcpp::Node>(
    "orchestrator_param_validation_startup", options);
  EXPECT_THROW(
    {
      peach_harvest_orchestrator::ParamListener listener(
        node->get_node_parameters_interface());
    },
    rclcpp::exceptions::InvalidParameterValueException);
  rclcpp::shutdown();
}

TEST(OrchestratorParameterValidation, RuntimeInvalidValueRejected)
{
  rclcpp::init(0, nullptr);
  const auto node = std::make_shared<rclcpp::Node>(
    "orchestrator_param_validation_runtime");
  peach_harvest_orchestrator::ParamListener listener(
    node->get_node_parameters_interface());

  // 范围外一律拒绝：超时须 >0、熔断上限须 ≥1（原为静默钳制）。
  auto result = node->set_parameters_atomically(
    {rclcpp::Parameter("readiness.timeout_s", -1.0)});
  EXPECT_FALSE(result.successful);
  result = node->set_parameters_atomically(
    {rclcpp::Parameter("readiness.timeout_s", 0.0)});
  EXPECT_FALSE(result.successful);
  result = node->set_parameters_atomically(
    {rclcpp::Parameter("dispatch.max_consecutive_rejections", 0)});
  EXPECT_FALSE(result.successful);
  result = node->set_parameters_atomically(
    {rclcpp::Parameter("dispatch.max_retries", -1)});
  EXPECT_FALSE(result.successful);
  // 拒绝不污染监听器快照。
  EXPECT_DOUBLE_EQ(listener.get_params().readiness.timeout_s, 2.0);

  // 合法值接受，监听器快照即时可见（refresh 热读 readiness.* 的通道）。
  result = node->set_parameters_atomically(
    {rclcpp::Parameter("readiness.timeout_s", 2.5)});
  EXPECT_TRUE(result.successful);
  EXPECT_DOUBLE_EQ(listener.get_params().readiness.timeout_s, 2.5);
  // configure 期缓存参数运行期也可合法改写（生效仍待下次 configure，语义不变）。
  result = node->set_parameters_atomically(
    {rclcpp::Parameter("photo_pose.enabled", false)});
  EXPECT_TRUE(result.successful);
  EXPECT_FALSE(listener.get_params().photo_pose.enabled);
  rclcpp::shutdown();
}

}  // namespace
