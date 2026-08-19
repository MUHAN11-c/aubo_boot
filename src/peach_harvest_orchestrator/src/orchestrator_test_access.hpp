// Copyright 2026 aubo_e5_jazzy_ws authors
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
// 进程内集成测试入口（仅 test 目标使用，不安装）：测试进程直接构造真实
// 编排器节点（模式同 peach_approach_grasp 的 test_lifecycle），生命周期
// 转换经 LifecycleNode::configure()/activate() 直调，其余全部走标准 ROS
// 接口（服务/action/话题）驱动与观测。

#ifndef ORCHESTRATOR_TEST_ACCESS_HPP_
#define ORCHESTRATOR_TEST_ACCESS_HPP_

#include <memory>

#include "rclcpp/node_options.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace peach_harvest_orchestrator
{
// 在调用进程内构造 HarvestOrchestratorNode（定义见
// harvest_orchestrator_node.cpp；options 可携带参数覆盖/参数文件）。
std::shared_ptr<rclcpp_lifecycle::LifecycleNode> make_orchestrator_node_for_test(
  const rclcpp::NodeOptions & options);
}  // namespace peach_harvest_orchestrator
#endif  // ORCHESTRATOR_TEST_ACCESS_HPP_
