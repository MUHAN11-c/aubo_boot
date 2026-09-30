// tools.yaml 解析纯核 gtest：方向解析 + yaml 加载（用包内真实 tools.yaml）。
#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>

#include "tool_changer/tool_config.hpp"

TEST(ToolConfig, ParseSwapDirection)
{
  std::string src, dst;
  EXPECT_TRUE(tool_changer::parseSwapDirection("gripper0_to_gripper2", src, dst));
  EXPECT_EQ(src, "gripper0");
  EXPECT_EQ(dst, "gripper2");

  EXPECT_TRUE(tool_changer::parseSwapDirection("gripper2", src, dst));
  EXPECT_TRUE(src.empty());
  EXPECT_EQ(dst, "gripper2");

  EXPECT_FALSE(tool_changer::parseSwapDirection("", src, dst));
}

TEST(ToolConfig, LoadRealToolsYaml)
{
  const std::string path =
    ament_index_cpp::get_package_share_directory("tool_changer") + "/config/tools.yaml";
  std::map<std::string, tool_changer::ToolConfig> configs;
  std::string err;
  ASSERT_TRUE(tool_changer::loadToolConfigs(path, configs, err)) << err;

  // 三个有 dock 定位的夹爪必在；仿真杯具（无 dock）被跳过
  EXPECT_EQ(configs.count("gripper0"), 1u);
  EXPECT_EQ(configs.count("gripper2"), 1u);
  EXPECT_EQ(configs.count("gripper1coffeecup"), 0u);

  const auto & g0 = configs.at("gripper0");
  EXPECT_EQ(g0.strategy, tool_changer::TrajectoryStrategy::kVertical);
  EXPECT_FALSE(g0.has_dock_approach_xyz);
  EXPECT_NEAR(g0.dock_approach_joints[0], 1.137820, 1e-9);

  const auto & g2 = configs.at("gripper2");
  EXPECT_EQ(g2.strategy, tool_changer::TrajectoryStrategy::kSlide);
  EXPECT_NEAR(g2.slide.slide_y, 0.100, 1e-9);

  const auto & g1 = configs.at("gripper1");
  EXPECT_TRUE(g1.has_dock_approach_xyz);
  EXPECT_NEAR(g1.dock_approach_xyz[2], 0.4755, 1e-9);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
