#include <gtest/gtest.h>

#include <cmath>

#include "peach2_manipulation/staging_geometry.hpp"

namespace pm = peach2_manipulation;

namespace
{

Eigen::Isometry3d pregrasp_at(const Eigen::Vector3d & p, const Eigen::Vector3d & axis)
{
  return pm::pose_with_roll(p, axis.normalized(), 0.3);
}

}  // namespace

TEST(StagingGeometry, StagingBelowBottomOnPregraspLine)
{
  const Eigen::Vector3d axis(0.0, 0.0, 1.0);
  const Eigen::Vector3d bottom(0.5, 0.0, 0.80);
  const auto pre = pregrasp_at(Eigen::Vector3d(0.5, 0.0, 0.77), axis);
  const auto st = pm::staging_pose(pre, bottom, 0.20);
  EXPECT_TRUE(st.linear().isApprox(pre.linear(), 1e-12));
  // Pregrasp is already below the bottom: staging = pregrasp - 0.20 along the axis.
  EXPECT_NEAR(st.translation().z(), 0.57, 1e-12);
  EXPECT_NEAR(st.translation().x(), 0.5, 1e-12);
  EXPECT_LE(st.translation().z(), bottom.z() - pm::kStagingMinM);
}

TEST(StagingGeometry, PregraspAboveBottomStillClearsBottom)
{
  const Eigen::Vector3d axis(0.0, 0.0, 1.0);
  const Eigen::Vector3d bottom(0.5, 0.0, 0.80);
  const auto pre = pregrasp_at(Eigen::Vector3d(0.5, 0.0, 0.82), axis);
  const auto st = pm::staging_pose(pre, bottom, 0.20);
  EXPECT_NEAR(st.translation().z(), 0.60, 1e-12);
}

TEST(StagingGeometry, DistanceClamped)
{
  const Eigen::Vector3d axis = Eigen::Vector3d(0.3, 0.0, 1.0).normalized();
  const Eigen::Vector3d bottom(0.5, 0.0, 0.80);
  const auto pre = pregrasp_at(bottom - 0.02 * axis, axis);
  const auto near = pm::staging_pose(pre, bottom, 0.05);
  EXPECT_NEAR((pre.translation() - near.translation()).dot(axis), 0.15, 1e-9);
  const auto far = pm::staging_pose(pre, bottom, 0.60);
  EXPECT_NEAR((pre.translation() - far.translation()).dot(axis), 0.25, 1e-9);
  // Lateral offset from the pregrasp line stays zero.
  const Eigen::Vector3d d = far.translation() - pre.translation();
  EXPECT_NEAR((d - d.dot(axis) * axis).norm(), 0.0, 1e-12);
}

TEST(StagingGeometry, TcpForBladePutsBladeOnTarget)
{
  const Eigen::Vector3d axis = Eigen::Vector3d(0.1, -0.2, 1.0).normalized();
  const Eigen::Vector3d neck(0.4, 0.1, 0.9);
  Eigen::Isometry3d blade = Eigen::Isometry3d::Identity();
  blade.translation() = Eigen::Vector3d(0.0, 0.0, -0.079);
  const Eigen::Matrix3d rot = pm::pose_with_roll(Eigen::Vector3d::Zero(), axis, 1.0).linear();
  const auto tcp = pm::tcp_for_blade(neck, rot, blade);
  EXPECT_TRUE((tcp * blade).translation().isApprox(neck, 1e-12));
  EXPECT_TRUE(tcp.translation().isApprox(neck + 0.079 * axis, 1e-12));
}

TEST(StagingGeometry, Residual)
{
  const Eigen::Vector3d axis(0.0, 0.0, 1.0);
  const auto target = pregrasp_at(Eigen::Vector3d(0.5, 0.0, 0.8), axis);
  pm::ResidualTolerance tol;
  auto actual = target;
  EXPECT_TRUE(pm::pregrasp_residual(actual, target, tol).ok);
  actual.translation().x() += 0.004;
  auto r = pm::pregrasp_residual(actual, target, tol);
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.reason, "lateral_residual");
  EXPECT_NEAR(r.lateral_m, 0.004, 1e-12);
  actual = target;
  actual.translation().z() += 0.004;
  EXPECT_TRUE(pm::pregrasp_residual(actual, target, tol).ok);
  actual.translation().z() += 0.002;
  EXPECT_EQ(pm::pregrasp_residual(actual, target, tol).reason, "axial_residual");
  actual = target;
  actual.linear() = pm::pose_with_roll(
    Eigen::Vector3d::Zero(), Eigen::Vector3d(std::sin(0.05), 0.0, std::cos(0.05)), 0.3).linear();
  r = pm::pregrasp_residual(actual, target, tol);
  EXPECT_EQ(r.reason, "angle_residual");
  EXPECT_NEAR(r.angle_rad, 0.05, 1e-9);
  // Roll alone is not a residual.
  actual = pm::pose_with_roll(target.translation(), axis, 2.0);
  EXPECT_TRUE(pm::pregrasp_residual(actual, target, tol).ok);
}

TEST(StagingGeometry, InsertGeometry)
{
  const Eigen::Vector3d axis(0.0, 0.0, 1.0);
  const auto pre = pregrasp_at(Eigen::Vector3d(0.5, 0.0, 0.77), axis);
  const auto g = pm::insert_geometry(pre, Eigen::Vector3d(0.501, 0.0, 0.959));
  EXPECT_NEAR(g.travel_m, 0.189, 1e-12);
  EXPECT_NEAR(g.lateral_m, 0.001, 1e-12);
}
