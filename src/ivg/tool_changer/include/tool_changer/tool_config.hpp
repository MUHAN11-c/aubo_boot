// tool_changer 工具档案纯核：ToolConfig 解析（tools.yaml）与快换方向解析。
// 从 gripper_swap_worker 抽出为零 ROS 图依赖模块，gtest 直测。
#ifndef TOOL_CHANGER__TOOL_CONFIG_HPP_
#define TOOL_CHANGER__TOOL_CONFIG_HPP_

#include <array>
#include <map>
#include <string>
#include <vector>

namespace tool_changer
{

/// 轨迹策略
enum class TrajectoryStrategy
{
  kVertical,
  kSlide
};

struct VerticalStrategyParams
{
  double depth = 0.210;
  double lift = 0.210;
  double settle_sec = 0.5;
};

struct SlideStrategyParams
{
  double depth = 0.210;
  double seat = 0.012;
  double slide_y = 0.100;
  double lift = 0.210;
  double settle_sec = 0.5;
  double release_sec = 0.3;
  double lock_sec = 0.5;
};

/// tools.yaml 单工具运行时配置
struct ToolConfig
{
  std::string id;
  std::string name;
  std::string type;
  std::string parameters;  // JSON 字符串

  TrajectoryStrategy strategy = TrajectoryStrategy::kVertical;
  std::array<double, 6> dock_approach_joints{};

  // 笛卡尔直线接近（dock_above 的 XYZ，姿态保持当前值）— 关节角缺失时的替代
  bool has_dock_approach_xyz = false;
  std::array<double, 3> dock_approach_xyz{};

  VerticalStrategyParams vertical;
  SlideStrategyParams slide;
};

/// 解析快换方向字符串：
///   "gripper0_to_gripper2" → source=gripper0, target=gripper2
///   "gripper2"             → source="",       target=gripper2
/// 返回 false 当 direction 为空。
bool parseSwapDirection(
  const std::string & direction, std::string & source, std::string & target);

/// 从 tools.yaml 加载全部工具配置。失败时置 err 并返回 false。
bool loadToolConfigs(
  const std::string & yaml_path,
  std::map<std::string, ToolConfig> & out,
  std::string & err);

}  // namespace tool_changer

#endif  // TOOL_CHANGER__TOOL_CONFIG_HPP_
