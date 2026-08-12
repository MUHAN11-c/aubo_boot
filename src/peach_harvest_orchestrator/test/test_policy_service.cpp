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
#include <gtest/gtest.h>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>


#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "peach_harvest_msgs/msg/harvest_state.hpp"
#include "peach_harvest_msgs/srv/set_operation_policy.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
using namespace std::chrono_literals;
using ChangeState = lifecycle_msgs::srv::ChangeState;
using HarvestState = peach_harvest_msgs::msg::HarvestState;
using SetOperationPolicy = peach_harvest_msgs::srv::SetOperationPolicy;

class ChildProcess
{
public:
  ChildProcess()
  {
    pid_ = fork();
    if (pid_ == 0) {
      execl(ORCHESTRATOR_EXECUTABLE, ORCHESTRATOR_EXECUTABLE, nullptr);
      std::_Exit(127);
    }
  }

  ~ChildProcess()
  {
    if (pid_ <= 0) {return;}
    kill(pid_, SIGINT);
    waitpid(pid_, nullptr, 0);
  }

  bool started() const {return pid_ > 0;}

private:
  pid_t pid_{-1};
};

bool change_lifecycle_state(
  const rclcpp::Client<ChangeState>::SharedPtr & client, uint8_t transition)
{
  auto request = std::make_shared<ChangeState::Request>();
  request->transition.id = transition;
  auto future = client->async_send_request(request);
  return future.wait_for(3s) == std::future_status::ready && future.get()->success;
}

TEST(HarvestOrchestratorPolicyService, AppliesExecutionPolicyToApproachNode)
{
  ChildProcess orchestrator;
  ASSERT_TRUE(orchestrator.started());

  rclcpp::init(0, nullptr);
  const auto approach = std::make_shared<rclcpp::Node>("peach_approach_grasp_node");
  approach->declare_parameter("execution.enabled", false);
  approach->declare_parameter("grasp.enabled", false);
  approach->declare_parameter("tool.enabled", false);
  const auto client_node = std::make_shared<rclcpp::Node>("policy_service_test_client");

  std::atomic_uint64_t revision{0};
  const auto state_sub = client_node->create_subscription<HarvestState>(
    "/peach_harvest_orchestrator/state", rclcpp::QoS(1).transient_local(),
    [&revision](const HarvestState::SharedPtr message) {revision.store(message->revision);});
  const auto lifecycle_client = client_node->create_client<ChangeState>(
    "/peach_harvest_orchestrator/change_state");
  const auto policy_client = client_node->create_client<SetOperationPolicy>(
    "/peach_harvest_orchestrator/set_operation_policy");

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(approach);
  executor.add_node(client_node);
  std::thread spin_thread([&executor]() {executor.spin();});

  ASSERT_TRUE(lifecycle_client->wait_for_service(5s));
  ASSERT_TRUE(change_lifecycle_state(
      lifecycle_client, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
  ASSERT_TRUE(change_lifecycle_state(
      lifecycle_client, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));
  ASSERT_TRUE(policy_client->wait_for_service(2s));

  const auto state_deadline = std::chrono::steady_clock::now() + 3s;
  while (revision.load() == 0 && std::chrono::steady_clock::now() < state_deadline) {
    std::this_thread::sleep_for(20ms);
  }
  ASSERT_GT(revision.load(), 0u);
  std::this_thread::sleep_for(600ms);

  auto request = std::make_shared<SetOperationPolicy::Request>();
  request->request_id = "policy-integration-test";
  request->expected_revision = revision.load();
  request->auto_start_enabled = true;
  request->execution_enabled = true;
  request->grasp_enabled = false;
  request->tool_enabled = false;
  auto response_future = policy_client->async_send_request(request);

  ASSERT_EQ(response_future.wait_for(3s), std::future_status::ready);
  const auto response = response_future.get();
  EXPECT_TRUE(response->accepted) << response->message;
  EXPECT_TRUE(approach->get_parameter("execution.enabled").as_bool());
  EXPECT_FALSE(approach->get_parameter("grasp.enabled").as_bool());
  EXPECT_FALSE(approach->get_parameter("tool.enabled").as_bool());

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  (void)state_sub;
}
}  // namespace
