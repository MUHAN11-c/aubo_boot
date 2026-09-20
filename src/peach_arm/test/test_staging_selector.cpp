// W5-2：StagingCandidateSelector 纯核——假 IK 回调注入验证排序/权重/
// top_n/种子路径/滚转位姿（公式与并发编排自节点 lambda 逐字抽出）。
#include "peach_arm/staging_selector.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace
{
using peach_arm::StagingCandidate;
using peach_arm::StagingIkSolve;
using peach_arm::StagingSelectorConfig;

constexpr double kPi = 3.14159265358979323846;

Eigen::Isometry3d poseWithZ(double angle_from_z)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() =
    Eigen::AngleAxisd(angle_from_z, Eigen::Vector3d::UnitX()).toRotationMatrix();
  return pose;
}
}  // namespace

TEST(StagingSelector, WristWeightBeatsEqualRawDistance)
{
  // 非腕关节偏 0.10（dist²=0.010）应排在腕关节偏 0.06（dist²=2.5×0.0036=0.009）
  // 之后——腕轴加权 2.5 使“原始距离更小”的腕解反而更远。
  StagingSelectorConfig config;
  config.seeds = 2;
  config.wrist_weight = 2.5;
  config.roll_penalty = 4.0;
  config.top_n = 5;
  const std::vector<std::string> names{"shoulder_joint", "wrist1_joint"};
  const std::vector<double> current{0.0, 0.0};
  int attempt = 0;
  const StagingIkSolve solve = [&](int, const Eigen::Isometry3d &, int)
    -> std::optional<std::vector<double>>
    {
      // attempt 0：非腕小偏；attempt 1：腕更小偏（原始欧氏距离更大）。
      if (attempt == 0) {
        ++attempt;
        return std::vector<double>{0.10, 0.0};
      }
      return std::vector<double>{0.0, 0.06};
    };
  const auto out = peach_arm::selectStagingCandidates(
    config, poseWithZ(0.0), current, names, {0.0}, solve);
  ASSERT_EQ(out.size(), 2U);
  EXPECT_NEAR(out[0].joints.at("wrist1_joint"), 0.06, 1e-12);
  EXPECT_NEAR(out[0].joints.at("shoulder_joint"), 0.0, 1e-12);
  EXPECT_NEAR(out[1].joints.at("shoulder_joint"), 0.10, 1e-12);
}

TEST(StagingSelector, RollPenaltyPrefersKeepRoll)
{
  // 两 roll 给出完全相同的关节解：keep-roll（0）应排在 ±30° 之前
  //（penalty=4.0×roll²），且带 roll 候选的姿态 = keep×AngleAxis(roll, Z)。
  StagingSelectorConfig config;
  config.seeds = 1;
  config.wrist_weight = 2.5;
  config.roll_penalty = 4.0;
  config.top_n = 5;
  const std::vector<std::string> names{"j"};
  const std::vector<double> current{0.0};
  const double roll = kPi / 6.0;
  const StagingIkSolve solve = [](int, const Eigen::Isometry3d &, int)
    -> std::optional<std::vector<double>> {return std::vector<double>{0.05};};
  const auto out = peach_arm::selectStagingCandidates(
    config, poseWithZ(0.0), current, names, {0.0, roll}, solve);
  ASSERT_EQ(out.size(), 2U);
  // 排序按 dist²（含滚转惩罚）升序：keep-roll 在前。
  EXPECT_TRUE(out[0].pose.linear().isApprox(poseWithZ(0.0).linear(), 1e-12));
  EXPECT_TRUE(out[1].pose.linear().isApprox(
      poseWithZ(0.0).linear() *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ()), 1e-12));
}

TEST(StagingSelector, TopNTruncatesSortedAscending)
{
  // seeds=6 产出 6 个非腕距离递增的解；top_n=2 只留最近两个、升序。
  StagingSelectorConfig config;
  config.seeds = 6;
  config.wrist_weight = 2.5;
  config.roll_penalty = 4.0;
  config.top_n = 2;
  const std::vector<std::string> names{"j"};
  const std::vector<double> current{0.0};
  const StagingIkSolve solve = [](int, const Eigen::Isometry3d &, int attempt)
    -> std::optional<std::vector<double>> {
      return std::vector<double>{
      0.01 * static_cast<double>(attempt + 1)};
    };
  const auto out = peach_arm::selectStagingCandidates(
    config, poseWithZ(0.0), current, names, {0.0}, solve);
  ASSERT_EQ(out.size(), 2U);
  EXPECT_NEAR(out[0].joints.at("j"), 0.01, 1e-12);
  EXPECT_NEAR(out[1].joints.at("j"), 0.02, 1e-12);
}

TEST(StagingSelector, SeedsDriveAttemptPathPerRoll)
{
  // 每 roll 各扫 attempt=0..seeds-1（0=当前种子，>0=随机种子），
  // roll_index 与 attempt 一并传入回调（节点侧据此取 env 池/随机采样）。
  StagingSelectorConfig config;
  config.seeds = 3;
  config.top_n = 5;
  const std::vector<std::string> names{"j"};
  const std::vector<double> current{0.0};
  std::set<int> roll_indices;
  std::set<int> attempts;
  std::mutex record_mutex;
  const StagingIkSolve solve = [&](int roll_index, const Eigen::Isometry3d &,
    int attempt) -> std::optional<std::vector<double>>
    {
      std::lock_guard<std::mutex> lock(record_mutex);
      roll_indices.insert(roll_index);
      attempts.insert(attempt);
      return std::nullopt;
    };
  const auto out = peach_arm::selectStagingCandidates(
    config, poseWithZ(0.0), current, names, {0.0, 0.2, 0.4}, solve);
  EXPECT_TRUE(out.empty());
  EXPECT_EQ(roll_indices, (std::set<int>{0, 1, 2}));
  EXPECT_EQ(attempts, (std::set<int>{0, 1, 2}));
}

TEST(StagingSelector, DegenerateInputsReturnEmpty)
{
  StagingSelectorConfig config;
  const std::vector<std::string> names{"j"};
  const std::vector<double> current{0.0};
  const StagingIkSolve solve = [](int, const Eigen::Isometry3d &, int)
    -> std::optional<std::vector<double>> {return std::vector<double>{0.0};};
  EXPECT_TRUE(peach_arm::selectStagingCandidates(
      config, poseWithZ(0.0), current, names, {0.0}, {}).empty());
  // 维度不一致（names 与 current 数量不符）：
  EXPECT_TRUE(peach_arm::selectStagingCandidates(
      config, poseWithZ(0.0), {0.0, 0.0}, names, {0.0}, solve).empty());
  // names 为空：
  EXPECT_TRUE(peach_arm::selectStagingCandidates(
      config, poseWithZ(0.0), current, {}, {0.0}, solve).empty());
}

TEST(StagingSelector, SolutionDimensionMismatchDropped)
{
  // 回调返回的解维度与 joint_names 不符：丢弃，不产出候选。
  StagingSelectorConfig config;
  config.seeds = 1;
  const std::vector<std::string> names{"a", "b"};
  const std::vector<double> current{0.0, 0.0};
  const StagingIkSolve solve = [](int, const Eigen::Isometry3d &, int)
    -> std::optional<std::vector<double>> {return std::vector<double>{0.1};};
  const auto out = peach_arm::selectStagingCandidates(
    config, poseWithZ(0.0), current, names, {0.0}, solve);
  EXPECT_TRUE(out.empty());
}
