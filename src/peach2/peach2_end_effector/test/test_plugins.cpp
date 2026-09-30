#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <string>

#include "peach2_end_effector/failure_codes.hpp"
#include "test_helpers.hpp"

namespace ee = peach2_end_effector;
using peach2_test::Rig;

namespace
{

ee::TargetGeometry bag(double d95, double length)
{
  ee::TargetGeometry t;
  t.target_id = "t1";
  t.axis = Eigen::Vector3d::UnitZ();
  t.bottom = Eigen::Vector3d(0.5, 0.0, 0.8);
  t.neck = t.bottom + length * t.axis;
  t.d95_m = d95;
  t.length_m = length;
  return t;
}

ee::BudgetView allow_all()
{
  ee::BudgetView b;
  b.sleeve_allowed = true;
  b.cut_allowed = true;
  b.radial_margin_m = 0.01;
  b.axial_margin_m = 0.01;
  return b;
}

}  // namespace

TEST(Plugins, BladeInTcpIsMinusLBladeAlongZ)
{
  for (const std::string id : {"shear_v1", "bite_shear_v1", "adaptive_shear_v1"}) {
    Rig rig(id);
    const Eigen::Isometry3d b = rig.ee->blade_in_tcp();
    EXPECT_EQ(rig.ee->tool_id(), id);
    EXPECT_TRUE(b.linear().isIdentity(1e-12));
    EXPECT_NEAR(b.translation().x(), 0.0, 1e-12);
    EXPECT_NEAR(b.translation().y(), 0.0, 1e-12);
    EXPECT_NEAR(b.translation().z(), -rig.ee->profile().geometry.l_blade, 1e-12);
  }
}

TEST(Plugins, AdaptiveFeasibleAndOvershoot)
{
  Rig rig("adaptive_shear_v1");
  const auto t = bag(0.10, 0.08);
  const auto f = rig.ee->feasible(t, allow_all());
  EXPECT_TRUE(f.ok);
  EXPECT_TRUE(f.sleeve_ok);
  EXPECT_TRUE(f.cut_ok);
  EXPECT_NEAR(f.radial_clearance_m, 0.008, 1e-9);
  EXPECT_NEAR(f.overshoot_m, 0.079, 1e-12);
  EXPECT_TRUE(f.tcp_at_cut.isApprox(t.neck + 0.079 * t.axis, 1e-12));
  // Blade frame placed at tcp_at_cut lands exactly on the neck.
  Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity();
  tcp.translation() = f.tcp_at_cut;
  EXPECT_TRUE((tcp * rig.ee->blade_in_tcp()).translation().isApprox(t.neck, 1e-12));
}

TEST(Plugins, BagWiderThanOpening)
{
  Rig rig("adaptive_shear_v1");
  const auto f = rig.ee->feasible(bag(0.117, 0.05), allow_all());
  EXPECT_FALSE(f.ok);
  EXPECT_EQ(f.failure_code, ee::failure::TOOL_NOT_FEASIBLE);
  EXPECT_EQ(f.reason, "bag_wider_than_opening");

  Rig shear("shear_v1");
  EXPECT_FALSE(shear.ee->feasible(bag(0.10, 0.02), allow_all()).ok);
  Rig bite("bite_shear_v1");
  EXPECT_TRUE(bite.ee->feasible(bag(0.09, 0.02), allow_all()).ok);
}

TEST(Plugins, BagLongerThanInsert)
{
  Rig rig("adaptive_shear_v1");
  const auto f = rig.ee->feasible(bag(0.08, 0.10), allow_all());
  EXPECT_FALSE(f.ok);
  EXPECT_EQ(f.failure_code, ee::failure::TOOL_NOT_FEASIBLE);
  EXPECT_EQ(f.reason, "bag_longer_than_L_insert");
  Rig shear("shear_v1");
  EXPECT_FALSE(shear.ee->feasible(bag(0.05, 0.05), allow_all()).ok);
  EXPECT_TRUE(shear.ee->feasible(bag(0.05, 0.025), allow_all()).ok);
}

TEST(Plugins, BudgetFlags)
{
  Rig rig("adaptive_shear_v1");
  auto b = allow_all();
  b.cut_allowed = false;
  auto f = rig.ee->feasible(bag(0.10, 0.08), b);
  EXPECT_TRUE(f.ok);
  EXPECT_TRUE(f.sleeve_ok);
  EXPECT_FALSE(f.cut_ok);
  EXPECT_EQ(f.failure_code, ee::failure::BUDGET_AXIAL_NEGATIVE);
  b.sleeve_allowed = false;
  b.cut_allowed = true;
  f = rig.ee->feasible(bag(0.10, 0.08), b);
  EXPECT_TRUE(f.ok);
  EXPECT_FALSE(f.sleeve_ok);
  EXPECT_FALSE(f.cut_ok);
  EXPECT_EQ(f.failure_code, ee::failure::BUDGET_RADIAL_NEGATIVE);
}

TEST(Plugins, InvalidGeometry)
{
  Rig rig("adaptive_shear_v1");
  auto t = bag(0.10, 0.08);
  t.axis = Eigen::Vector3d::Zero();
  EXPECT_EQ(rig.ee->feasible(t, allow_all()).failure_code, ee::failure::TOOL_NOT_FEASIBLE);
  t = bag(0.10, 0.08);
  t.d95_m = std::nan("");
  EXPECT_FALSE(rig.ee->feasible(t, allow_all()).ok);
}

TEST(Plugins, ShearRollAvoidsCameraSide)
{
  Rig rig("shear_v1");
  auto t = bag(0.06, 0.02);
  EXPECT_TRUE(rig.ee->roll_constraint(t).full());
  t.avoid_direction = Eigen::Vector3d::UnitX();
  const auto rc = rig.ee->roll_constraint(t);
  EXPECT_FALSE(rc.full());
  EXPECT_NEAR(rc.half_width_rad, M_PI / 3.0, 1e-12);
  EXPECT_TRUE(rc.contains(M_PI));
  EXPECT_FALSE(rc.contains(0.0));
  const Eigen::Matrix3d r = ee::tcp_rotation(t.axis, rc.center_rad);
  EXPECT_TRUE(r.col(0).isApprox(-Eigen::Vector3d::UnitX(), 1e-9));
  for (double roll : rc.samples(5)) {
    EXPECT_TRUE(rc.contains(roll));
  }
  EXPECT_NEAR(ee::wrap_angle(rc.samples(5).front() - rc.center_rad), 0.0, 1e-12);
}

TEST(Plugins, BiteJawsPerpendicularToBranch)
{
  Rig rig("bite_shear_v1");
  auto t = bag(0.09, 0.02);
  EXPECT_TRUE(rig.ee->roll_constraint(t).full());
  t.branch_direction = Eigen::Vector3d::UnitY();
  const auto rc = rig.ee->roll_constraint(t);
  EXPECT_FALSE(rc.full());
  EXPECT_NEAR(rc.period_rad, M_PI, 1e-12);
  EXPECT_TRUE(rc.contains(M_PI / 2.0));
  EXPECT_TRUE(rc.contains(-M_PI / 2.0));   // half-turn symmetric
  EXPECT_FALSE(rc.contains(0.0));
  for (double roll : rc.samples(6)) {
    ASSERT_TRUE(rc.contains(roll));
    const Eigen::Matrix3d r = ee::tcp_rotation(t.axis, roll);
    // Jaw closing direction (TCP +Y) within 20 deg of perpendicular to the branch.
    EXPECT_LE(std::fabs(r.col(1).dot(Eigen::Vector3d::UnitY())), std::sin(M_PI / 9.0) + 1e-9);
  }
}

TEST(Plugins, ShearBladeAwayFromBranchOverridesAvoidDirection)
{
  Rig rig("shear_v1");
  auto t = bag(0.06, 0.02);
  t.branch_direction = Eigen::Vector3d::UnitY();
  t.avoid_direction = Eigen::Vector3d::UnitX();
  const auto rc = rig.ee->roll_constraint(t);
  EXPECT_FALSE(rc.full());
  EXPECT_NEAR(rc.period_rad, 2.0 * M_PI, 1e-12);
  for (double roll : rc.samples(5)) {
    ASSERT_TRUE(rc.contains(roll));
    const Eigen::Matrix3d r = ee::tcp_rotation(t.axis, roll);
    // Blade side (TCP +X) within 60 deg of pointing away from the branch.
    EXPECT_LE(r.col(0).dot(Eigen::Vector3d::UnitY()), -std::cos(M_PI / 3.0) + 1e-9);
  }
  const Eigen::Matrix3d c = ee::tcp_rotation(t.axis, rc.center_rad);
  EXPECT_TRUE(c.col(0).isApprox(-Eigen::Vector3d::UnitY(), 1e-9));
}

TEST(Plugins, BranchAlongAxisFallsBackToFullRoll)
{
  // Branch parallel to the bag axis has no roll component: unconstrained (as unknown).
  auto t = bag(0.06, 0.02);
  t.branch_direction = t.axis;
  EXPECT_TRUE(Rig("bite_shear_v1").ee->roll_constraint(t).full());
  EXPECT_TRUE(Rig("shear_v1").ee->roll_constraint(t).full());
}

TEST(Plugins, BiteBranchTiltedOutOfPlaneUsesProjection)
{
  Rig rig("bite_shear_v1");
  auto t = bag(0.09, 0.02);
  t.branch_direction = Eigen::Vector3d(0.0, 1.0, 1.0).normalized();
  const auto rc = rig.ee->roll_constraint(t);
  ASSERT_FALSE(rc.full());
  const Eigen::Matrix3d c = ee::tcp_rotation(t.axis, rc.center_rad);
  const Eigen::Vector3d b = *t.branch_direction;
  const Eigen::Vector3d proj = (b - b.dot(t.axis) * t.axis).normalized();
  EXPECT_NEAR(std::fabs(c.col(0).dot(proj)), 1.0, 1e-9);
}

TEST(Plugins, AdaptiveRollFree)
{
  Rig rig("adaptive_shear_v1");
  auto t = bag(0.10, 0.08);
  t.branch_direction = Eigen::Vector3d::UnitY();
  t.avoid_direction = Eigen::Vector3d::UnitX();
  EXPECT_TRUE(rig.ee->roll_constraint(t).full());
}

TEST(Plugins, InsertStrategies)
{
  const auto t = bag(0.06, 0.02);
  const auto s = Rig("shear_v1").ee->insert(t);
  EXPECT_EQ(s.mode, ee::InsertMode::LINEAR);
  EXPECT_EQ(s.criterion, ee::ArrivalCriterion::TRAVEL_COMPLETE);
  EXPECT_FALSE(s.requires_force_sensing);
  const auto b = Rig("bite_shear_v1").ee->insert(t);
  EXPECT_EQ(b.criterion, ee::ArrivalCriterion::TRAVEL_AND_THROAT_CONTACT);
  EXPECT_TRUE(b.linear_fallback);
  const auto a = Rig("adaptive_shear_v1").ee->insert(t);
  EXPECT_EQ(a.mode, ee::InsertMode::ADMITTANCE);
  EXPECT_EQ(a.criterion, ee::ArrivalCriterion::CONTACT_FORCE_STABLE);
  EXPECT_TRUE(a.requires_force_sensing);
  EXPECT_TRUE(a.linear_fallback);
  EXPECT_GT(a.contact_force_n, 0.0);
  for (const auto & p : {s, b, a}) {
    EXPECT_GT(p.speed_mps, 0.0);
    EXPECT_LE(p.speed_mps, 0.03);
  }
}

TEST(Plugins, InitializeRejectsBadContext)
{
  auto clock = std::make_shared<peach2_test::FakeClock>();
  ee::EndEffectorContext ctx;
  ctx.profile = ee::load_tool_profile("shear_v1", PEACH2_TOOL_CONFIG_DIR);
  ctx.now_s = [clock]() {return clock->now();};
  ctx.sleep_s = [clock](double dt) {clock->sleep(dt);};
  ee::ShearV1 shear;
  EXPECT_THROW(shear.initialize(ctx), std::invalid_argument);  // no io
  ctx.io = std::make_shared<ee::MockIoBackend>(
    ee::MockIoBackend::Config{}, [clock]() {return clock->now();});
  ctx.pins.feedback_pin = ctx.pins.cmd_pin;
  EXPECT_THROW(shear.initialize(ctx), std::invalid_argument);
  ctx.pins.feedback_pin = 1;
  ee::BiteShearV1 bite;
  EXPECT_THROW(bite.initialize(ctx), std::invalid_argument);  // wrong profile
  EXPECT_NO_THROW(shear.initialize(ctx));
  EXPECT_THROW(bite.prepare(), std::logic_error);
}
