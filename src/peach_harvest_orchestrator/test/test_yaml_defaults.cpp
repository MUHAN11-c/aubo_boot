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
// yaml 权威哲学强制：config/orchestrator.yaml 与生成的参数定义默认值逐项对齐
// （与 peach_pose_ros2 的 YamlDeclareSyncTest 同义；参数声明/默认值已迁移到
// generate_parameter_library 的 src/harvest_orchestrator_node_parameters.yaml）。
// 唯一有意例外是 global_photo_joints（定义默认空数组，yaml 存档真机实测位姿）。
#include <gtest/gtest.h>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rcl_yaml_param_parser/parser.h"
#include "rcl_yaml_param_parser/types.h"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/allocator.h"

namespace
{
using namespace std::chrono_literals;

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

rclcpp::ParameterValue to_parameter_value(const rcl_variant_t & value)
{
  if (value.bool_value != nullptr) {return rclcpp::ParameterValue(*value.bool_value);}
  if (value.integer_value != nullptr) {return rclcpp::ParameterValue(*value.integer_value);}
  if (value.double_value != nullptr) {return rclcpp::ParameterValue(*value.double_value);}
  if (value.string_value != nullptr) {
    return rclcpp::ParameterValue(std::string(value.string_value));
  }
  if (value.double_array_value != nullptr) {
    return rclcpp::ParameterValue(std::vector<double>(
        value.double_array_value->values,
        value.double_array_value->values + value.double_array_value->size));
  }
  if (value.integer_array_value != nullptr) {
    return rclcpp::ParameterValue(std::vector<int64_t>(
        value.integer_array_value->values,
        value.integer_array_value->values + value.integer_array_value->size));
  }
  if (value.bool_array_value != nullptr) {
    return rclcpp::ParameterValue(std::vector<bool>(
        value.bool_array_value->values,
        value.bool_array_value->values + value.bool_array_value->size));
  }
  if (value.string_array_value != nullptr) {
    std::vector<std::string> items;
    for (size_t i = 0; i < value.string_array_value->size; ++i) {
      items.emplace_back(value.string_array_value->data[i]);
    }
    return rclcpp::ParameterValue(items);
  }
  return {};
}

std::map<std::string, rclcpp::ParameterValue> load_yaml_params(const char * path)
{
  std::map<std::string, rclcpp::ParameterValue> result;
  rcl_params_t * params = rcl_yaml_node_struct_init(rcutils_get_default_allocator());
  EXPECT_NE(params, nullptr);
  if (params == nullptr) {return result;}
  if (!rcl_parse_yaml_file(path, params)) {
    ADD_FAILURE() << "解析失败: " << path;
    rcl_yaml_node_struct_fini(params);
    return result;
  }
  for (size_t n = 0; n < params->num_nodes; ++n) {
    const std::string node_name = params->node_names[n];
    if (node_name != "peach_harvest_orchestrator" && node_name != "/**") {continue;}
    const rcl_node_params_t & node_params = params->params[n];
    for (size_t i = 0; i < node_params.num_params; ++i) {
      result[node_params.parameter_names[i]] =
        to_parameter_value(node_params.parameter_values[i]);
    }
  }
  rcl_yaml_node_struct_fini(params);
  return result;
}
}  // namespace

TEST(HarvestOrchestratorYamlDefaults, YamlMatchesDeclareDefaults)
{
  // 子进程不带 params-file 启动：读到的即生成参数定义的默认值
  ChildProcess orchestrator;
  ASSERT_TRUE(orchestrator.started());

  rclcpp::init(0, nullptr);
  const auto node = std::make_shared<rclcpp::Node>("yaml_defaults_test_client");
  const auto client = std::make_shared<rclcpp::AsyncParametersClient>(
    node, "/peach_harvest_orchestrator");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  const auto yaml_params = load_yaml_params(ORCHESTRATOR_YAML_PATH);
  ASSERT_FALSE(yaml_params.empty());
  // yaml 中 global_photo_joints 有意与生成定义默认（空数组）不同：yaml 为
  // 真机实测位姿存档，定义只给空占位
  const std::vector<std::string> exceptions = {"global_photo_joints"};

  std::vector<std::string> names;
  for (const auto & [name, value] : yaml_params) {
    names.push_back(name);
  }
  ASSERT_TRUE(client->wait_for_service(5s));
  auto future = client->get_parameters(names);
  ASSERT_EQ(future.wait_for(3s), std::future_status::ready);
  const auto declared = future.get();
  ASSERT_EQ(declared.size(), names.size());
  for (size_t i = 0; i < names.size(); ++i) {
    const auto & name = names[i];
    ASSERT_NE(declared[i].get_type(), rclcpp::ParameterType::PARAMETER_NOT_SET)
      << "yaml 键未在参数定义中声明: " << name;
    if (std::find(exceptions.begin(), exceptions.end(), name) != exceptions.end()) {
      continue;
    }
    EXPECT_EQ(declared[i].get_parameter_value(), yaml_params.at(name))
      << "yaml 与 declare 默认值失配: " << name;
  }

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
}
