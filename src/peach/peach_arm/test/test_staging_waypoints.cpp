// v4 接近三路点构造（stagingWaypoints）最小单测：垂直/水平袋、参数退化。
#include <gtest/gtest.h>

#include <cmath>

#include "peach_arm/grasp_geometry.hpp"

namespace
{

Eigen::Vector3d axisOf(const peach_arm::StagingWaypoints & w)
{
  return w.staging.linear().col(2);
}

}  // namespace

TEST(StagingWaypoints, VerticalAxisStraightLine)
{
  // 袋轴 = +Z：三路点应在同一铅垂线上，间距 = along/final_axial/canopy。
  const Eigen::Isometry3d entry = Eigen::Isometry3d::Identity();
  const Eigen::Vector3d axis(0.0, 0.0, 1.0);
  const auto w = peach_arm::stagingWaypoints(
    entry, axis, Eigen::Matrix3d::Identity(), 0.0, 0.03, 0.05, 0.05);
  EXPECT_NEAR(w.pregrasp.translation().z(), entry.translation().z() - 0.03, 1e-9);
  EXPECT_NEAR(w.mid.translation().z(), entry.translation().z() - 0.08, 1e-9);
  EXPECT_NEAR(w.staging.translation().z(), entry.translation().z() - 0.13, 1e-9);
  EXPECT_LT((w.pregrasp.translation() - w.staging.translation())
    .head<2>().norm(), 1e-9);
}

TEST(StagingWaypoints, TiltedAxisDecouplesCanopyFromAxis)
{
  // 斜袋：入冠段仍世界垂直（staging 在 mid 正下方），末段才顺袋轴。
  const Eigen::Isometry3d entry = Eigen::Isometry3d::Identity();
  const Eigen::Vector3d axis =
    Eigen::Vector3d(0.6, 0.0, 0.8).normalized();
  const auto w = peach_arm::stagingWaypoints(
    entry, axis, Eigen::Matrix3d::Identity(), 0.0, 0.0, 0.05, 0.05);
  // mid = entry − 0.05·axis（along=0）
  EXPECT_NEAR(
    (w.mid.translation() - entry.translation()).dot(axis), -0.05, 1e-9);
  // staging = mid − 0.05·ẑ：水平坐标与 mid 相同
  EXPECT_NEAR(w.staging.translation().x(), w.mid.translation().x(), 1e-9);
  EXPECT_NEAR(w.staging.translation().y(), w.mid.translation().y(), 1e-9);
  EXPECT_NEAR(w.staging.translation().z(), w.mid.translation().z() - 0.05, 1e-9);
}

TEST(StagingWaypoints, RollRotatesOrientationAboutToolZ)
{
  const Eigen::Isometry3d entry = Eigen::Isometry3d::Identity();
  const Eigen::Vector3d axis(0.0, 0.0, 1.0);
  const double roll = M_PI / 3.0;
  const auto w = peach_arm::stagingWaypoints(
    entry, axis, Eigen::Matrix3d::Identity(), roll, 0.0, 0.05, 0.05);
  // 三路点同姿态；滚转不改变 Z 列（工具轴仍对袋轴）
  EXPECT_TRUE(w.staging.linear().isApprox(w.mid.linear()));
  EXPECT_TRUE(w.pregrasp.linear().isApprox(w.staging.linear()));
  EXPECT_NEAR(axisOf(w).dot(axis), 1.0, 1e-9);
  // 滚转 60°：X 列相对 roll-0 版转 60°
  const auto w0 = peach_arm::stagingWaypoints(
    entry, axis, Eigen::Matrix3d::Identity(), 0.0, 0.0, 0.05, 0.05);
  const double cos60 =
    w.staging.linear().col(0).dot(w0.staging.linear().col(0));
  EXPECT_NEAR(cos60, std::cos(roll), 1e-9);
}

TEST(StagingWaypoints, DegenerateZeroParametersCollapse)
{
  // final_axial=canopy=0：staging 与 pregrasp 重合（PTP 直达，LIN 段被
  // makeStagingSequence 的 0.005 判据跳过）。
  const Eigen::Isometry3d entry = Eigen::Isometry3d::Identity();
  const auto w = peach_arm::stagingWaypoints(
    entry, Eigen::Vector3d::UnitZ(), Eigen::Matrix3d::Identity(), 0.0,
    0.0, 0.0, 0.0);
  EXPECT_LT(
    (w.staging.translation() - w.pregrasp.translation()).norm(), 1e-9);
  EXPECT_LT((w.mid.translation() - w.pregrasp.translation()).norm(), 1e-9);
}
