// W5-4：PregraspResidualChecker 纯核——构造已知位姿断言角度/横向/轴向计算
// 与三阈值判定（公式自 stageVerifyPregrasp 逐字抽出，门限来自注入）。
#include "peach_arm/pregrasp_residual.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;

peach_arm::PregraspThresholds defaultThresholds()
{
  return peach_arm::PregraspThresholds{1.5, 2.0, 0.003};
}

// 位姿构造：Z 轴绕 X 倾 angle_rad、平移 translation。
Eigen::Isometry3d frameAt(double tilt_rad, const Eigen::Vector3d & translation)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() =
    Eigen::AngleAxisd(tilt_rad, Eigen::Vector3d::UnitX()).toRotationMatrix();
  pose.translation() = translation;
  return pose;
}
}  // namespace

TEST(PregraspResidual, AlignedSamplePasses)
{
  // 袋轴 +Z、底 (0,0,0)、颈 (0,0,0.2)；工具 Z 对轴、mouth 正对底、cut 正对颈。
  const peach_arm::PregraspPoseSample sample{
    frameAt(0.0, Eigen::Vector3d(0, 0, 0.1)),
    frameAt(0.0, Eigen::Vector3d(0, 0, 0)),
    frameAt(0.0, Eigen::Vector3d(0, 0, 0.2))};
  const auto report = peach_arm::evaluatePregraspResidual(
    defaultThresholds(), sample, sample, Eigen::Vector3d(0, 0, 0),
    Eigen::Vector3d(0, 0, 0.2), Eigen::Vector3d(0, 0, 1));
  EXPECT_TRUE(report.consistent);
  EXPECT_NEAR(report.frames_deg, 0.0, 1e-9);
  EXPECT_NEAR(report.angle_deg, 0.0, 1e-9);
  EXPECT_NEAR(report.lateral_m, 0.0, 1e-12);
  EXPECT_NEAR(report.axial_m, 0.0, 1e-12);
  EXPECT_TRUE(report.passed);
}

TEST(PregraspResidual, AxisAngleComputedFromTiltedToolZ)
{
  // 第二次采样工具 Z 倾 3°（帧间一致成立：first==second 夹角 0）→ 对轴 3°>2° 拒。
  const peach_arm::PregraspPoseSample first{
    frameAt(3.0 * kDeg2Rad, Eigen::Vector3d(0, 0, 0.1)),
    frameAt(0.0, Eigen::Vector3d(0, 0, 0)),
    frameAt(0.0, Eigen::Vector3d(0, 0, 0.2))};
  const auto report = peach_arm::evaluatePregraspResidual(
    defaultThresholds(), first, first, Eigen::Vector3d(0, 0, 0),
    Eigen::Vector3d(0, 0, 0.2), Eigen::Vector3d(0, 0, 1));
  EXPECT_TRUE(report.consistent);
  EXPECT_NEAR(report.angle_deg, 3.0, 1e-6);
  EXPECT_FALSE(report.passed);
}

TEST(PregraspResidual, FrameInconsistencyIsStrictThreshold)
{
  // 帧间 5°>1.5°：不一致（即使对轴完美也拒）。边界：帧间恰 1.5° 仍不一致
  //（严格小于）。
  const peach_arm::PregraspPoseSample first{
    frameAt(5.0 * kDeg2Rad, Eigen::Vector3d(0, 0, 0.1)),
    frameAt(0.0, Eigen::Vector3d(0, 0, 0)),
    frameAt(0.0, Eigen::Vector3d(0, 0, 0.2))};
  const peach_arm::PregraspPoseSample second{
    frameAt(0.0, Eigen::Vector3d(0, 0, 0.1)),
    frameAt(0.0, Eigen::Vector3d(0, 0, 0)),
    frameAt(0.0, Eigen::Vector3d(0, 0, 0.2))};
  const auto report = peach_arm::evaluatePregraspResidual(
    defaultThresholds(), first, second, Eigen::Vector3d(0, 0, 0),
    Eigen::Vector3d(0, 0, 0.2), Eigen::Vector3d(0, 0, 1));
  EXPECT_NEAR(report.frames_deg, 5.0, 1e-6);
  EXPECT_FALSE(report.consistent);
  EXPECT_FALSE(report.passed);

  // 门限是严格小于：1.0° 过、2.0° 不过（1.4999…这类 acos 浮点贴边值
  // 不当判据，取远离 knife-edge 的样本钉语义）。
  const peach_arm::PregraspPoseSample mild_first{
    frameAt(1.0 * kDeg2Rad, Eigen::Vector3d(0, 0, 0.1)),
    frameAt(0.0, Eigen::Vector3d(0, 0, 0)),
    frameAt(0.0, Eigen::Vector3d(0, 0, 0.2))};
  const auto mild = peach_arm::evaluatePregraspResidual(
    defaultThresholds(), mild_first, second, Eigen::Vector3d(0, 0, 0),
    Eigen::Vector3d(0, 0, 0.2), Eigen::Vector3d(0, 0, 1));
  EXPECT_NEAR(mild.frames_deg, 1.0, 1e-6);
  EXPECT_TRUE(mild.consistent);
  const peach_arm::PregraspPoseSample loud_first{
    frameAt(2.0 * kDeg2Rad, Eigen::Vector3d(0, 0, 0.1)),
    frameAt(0.0, Eigen::Vector3d(0, 0, 0)),
    frameAt(0.0, Eigen::Vector3d(0, 0, 0.2))};
  const auto loud = peach_arm::evaluatePregraspResidual(
    defaultThresholds(), loud_first, second, Eigen::Vector3d(0, 0, 0),
    Eigen::Vector3d(0, 0, 0.2), Eigen::Vector3d(0, 0, 1));
  EXPECT_NEAR(loud.frames_deg, 2.0, 1e-6);
  EXPECT_FALSE(loud.consistent);
}

TEST(PregraspResidual, LateralTakesMaxOfMouthAndCut)
{
  // mouth 横向 0.001、cut 横向 0.004（轴向余量另置）→ lateral=0.004>0.003 拒。
  const peach_arm::PregraspPoseSample aligned{
    frameAt(0.0, Eigen::Vector3d(0, 0, 0.1)),
    frameAt(0.0, Eigen::Vector3d(0, 0, 0)),
    frameAt(0.0, Eigen::Vector3d(0, 0, 0.2))};
  const peach_arm::PregraspPoseSample shifted{
    frameAt(0.0, Eigen::Vector3d(0, 0, 0.1)),
    frameAt(0.0, Eigen::Vector3d(0.001, 0, 0)),
    frameAt(0.0, Eigen::Vector3d(0.004, 0, 0.2))};
  const auto report = peach_arm::evaluatePregraspResidual(
    defaultThresholds(), aligned, shifted, Eigen::Vector3d(0, 0, 0),
    Eigen::Vector3d(0, 0, 0.2), Eigen::Vector3d(0, 0, 1));
  EXPECT_NEAR(report.lateral_m, 0.004, 1e-12);
  EXPECT_NEAR(report.axial_m, 0.0, 1e-12);
  EXPECT_FALSE(report.passed);
}

TEST(PregraspResidual, LateralBoundaryIsInclusive)
{
  // 横向恰 0.003：<= 门放行；对轴恰 2.0° 同样放行。
  const peach_arm::PregraspPoseSample aligned{
    frameAt(2.0 * kDeg2Rad, Eigen::Vector3d(0, 0, 0.1)),
    frameAt(0.0, Eigen::Vector3d(0, 0, 0)),
    frameAt(0.0, Eigen::Vector3d(0, 0, 0.2))};
  const peach_arm::PregraspPoseSample boundary{
    frameAt(2.0 * kDeg2Rad, Eigen::Vector3d(0, 0, 0.1)),
    frameAt(0.0, Eigen::Vector3d(0.003, 0, 0)),
    frameAt(0.0, Eigen::Vector3d(0, 0, 0.2))};
  const auto report = peach_arm::evaluatePregraspResidual(
    defaultThresholds(), aligned, boundary, Eigen::Vector3d(0, 0, 0),
    Eigen::Vector3d(0, 0, 0.2), Eigen::Vector3d(0, 0, 1));
  EXPECT_NEAR(report.lateral_m, 0.003, 1e-12);
  EXPECT_NEAR(report.angle_deg, 2.0, 1e-6);
  EXPECT_TRUE(report.consistent);
  EXPECT_TRUE(report.passed);
}

TEST(PregraspResidual, AxialOffsetIsRecordedOnly)
{
  // cut 沿袋轴偏 0.01：axial=0.01、横向不受影响，三阈值判定照常放行。
  const peach_arm::PregraspPoseSample aligned{
    frameAt(0.0, Eigen::Vector3d(0, 0, 0.1)),
    frameAt(0.0, Eigen::Vector3d(0, 0, 0)),
    frameAt(0.0, Eigen::Vector3d(0, 0, 0.2))};
  const peach_arm::PregraspPoseSample offset{
    frameAt(0.0, Eigen::Vector3d(0, 0, 0.1)),
    frameAt(0.0, Eigen::Vector3d(0, 0, 0)),
    frameAt(0.0, Eigen::Vector3d(0, 0, 0.21))};
  const auto report = peach_arm::evaluatePregraspResidual(
    defaultThresholds(), aligned, offset, Eigen::Vector3d(0, 0, 0),
    Eigen::Vector3d(0, 0, 0.2), Eigen::Vector3d(0, 0, 1));
  EXPECT_NEAR(report.axial_m, 0.01, 1e-12);
  EXPECT_NEAR(report.lateral_m, 0.0, 1e-12);
  EXPECT_TRUE(report.passed);
}

TEST(PregraspResidual, DegenerateToolZMapsToMaxMismatch)
{
  // 袋轴零向量/工具 Z 退化：按 180°（最大不对轴）走拒绝侧，不得误放行。
  const peach_arm::PregraspPoseSample sample{
    frameAt(0.0, Eigen::Vector3d(0, 0, 0.1)),
    frameAt(0.0, Eigen::Vector3d(0, 0, 0)),
    frameAt(0.0, Eigen::Vector3d(0, 0, 0.2))};
  const auto report = peach_arm::evaluatePregraspResidual(
    defaultThresholds(), sample, sample, Eigen::Vector3d(0, 0, 0),
    Eigen::Vector3d(0, 0, 0.2), Eigen::Vector3d::Zero());
  EXPECT_NEAR(report.angle_deg, 180.0, 1e-9);
  EXPECT_FALSE(report.passed);
}

TEST(PregraspResidual, ThresholdsEchoedInReport)
{
  const peach_arm::PregraspThresholds custom{2.5, 5.0, 0.010};
  const peach_arm::PregraspPoseSample sample{
    frameAt(0.0, Eigen::Vector3d(0, 0, 0.1)),
    frameAt(0.0, Eigen::Vector3d(0, 0, 0)),
    frameAt(0.0, Eigen::Vector3d(0, 0, 0.2))};
  const auto report = peach_arm::evaluatePregraspResidual(
    custom, sample, sample, Eigen::Vector3d(0, 0, 0),
    Eigen::Vector3d(0, 0, 0.2), Eigen::Vector3d(0, 0, 1));
  EXPECT_DOUBLE_EQ(report.thresholds.frame_consistent_deg, 2.5);
  EXPECT_DOUBLE_EQ(report.thresholds.axis_deg, 5.0);
  EXPECT_DOUBLE_EQ(report.thresholds.lateral_m, 0.010);
}
