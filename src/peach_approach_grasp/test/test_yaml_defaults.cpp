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
// yaml 权威哲学强制：config/approach_grasp.yaml 与生成的参数定义默认值逐项
// 对齐（与 peach_harvest_orchestrator 的 test_yaml_defaults 同义；参数
// 声明/默认值已迁移到 generate_parameter_library 的
// src/approach_grasp_node_parameters.yaml）。进程内构造 ParamListener 即可
// 声明全部参数（默认即生成定义值），无需启动节点二进制。
#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rcl_yaml_param_parser/parser.h"
#include "rcl_yaml_param_parser/types.h"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/allocator.h"

#include "peach_approach_grasp/approach_grasp_node_parameters.hpp"

namespace
{

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
    if (node_name != "peach_approach_grasp_node" && node_name != "/**") {continue;}
    const rcl_node_params_t & node_params = params->params[n];
    for (size_t i = 0; i < node_params.num_params; ++i) {
      result[node_params.parameter_names[i]] =
        to_parameter_value(node_params.parameter_values[i]);
    }
  }
  rcl_yaml_node_struct_fini(params);
  return result;
}

TEST(ApproachGraspYamlDefaults, YamlMatchesGeneratedDefaults)
{
  rclcpp::init(0, nullptr);
  // 不带任何覆盖构造：ParamListener 声明出的值即生成参数定义的默认值。
  const auto node = std::make_shared<rclcpp::Node>("peach_approach_grasp_node");
  peach_approach_grasp_node::ParamListener listener(
    node->get_node_parameters_interface());

  const auto yaml_params = load_yaml_params(APPROACH_GRASP_YAML_PATH);
  ASSERT_FALSE(yaml_params.empty());
  for (const auto & [name, value] : yaml_params) {
    const auto declared = node->get_parameter(name);
    ASSERT_NE(declared.get_type(), rclcpp::ParameterType::PARAMETER_NOT_SET)
      << "yaml 键未在参数定义中声明: " << name;
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      // rcl_yaml_param_parser 对空序列 `key: []` 不保留元素类型（变体各指针
      // 全空，实测见 2026-08-19 探针）：yaml 空数组与声明端的空数组默认值
      // 视为一致（scan.protected_zones 默认即空列表）；声明端不是空数组才算
      // 失配。
      const auto declared_value = declared.get_parameter_value();
      const bool empty_array =
        (declared_value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY &&
        declared_value.get<std::vector<double>>().empty()) ||
        (declared_value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY &&
        declared_value.get<std::vector<int64_t>>().empty()) ||
        (declared_value.get_type() == rclcpp::ParameterType::PARAMETER_BOOL_ARRAY &&
        declared_value.get<std::vector<bool>>().empty()) ||
        (declared_value.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY &&
        declared_value.get<std::vector<std::string>>().empty());
      EXPECT_TRUE(empty_array)
        << "yaml 空数组键的声明默认值非空数组: " << name;
      continue;
    }
    EXPECT_EQ(declared.get_parameter_value(), value)
      << "yaml 与生成定义默认值失配: " << name;
  }
  rclcpp::shutdown();
}

}  // namespace
