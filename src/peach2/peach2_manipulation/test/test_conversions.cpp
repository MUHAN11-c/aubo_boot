#include <gtest/gtest.h>

#include <cmath>

#include "peach2_end_effector/failure_codes.hpp"
#include "peach2_manipulation/conversions.hpp"

namespace pm = peach2_manipulation;
namespace pe = peach2_end_effector;

namespace
{

peach2_interfaces::msg::TargetModel valid_model()
{
  peach2_interfaces::msg::TargetModel m;
  m.target_id = "target_1";
  m.model_revision = 4;
  m.bottom.valid = true;
  m.bottom.position.x = 0.5;
  m.bottom.position.z = 0.8;
  m.neck.valid = true;
  m.neck.position.x = 0.5;
  m.neck.position.z = 0.88;
  m.axis.z = 2.0;
  m.d95_m = 0.07;
  m.length_m = 0.08;
  m.sigma_lateral95_m = 0.004;
  m.sigma_axial95_m = 0.005;
  return m;
}

}  // namespace

TEST(Conversions, GeometryFromValidModelNormalizesAxis)
{
  const auto g = pm::geometry_from_msg(valid_model());
  ASSERT_TRUE(g.has_value());
  EXPECT_EQ(g->target_id, "target_1");
  EXPECT_NEAR(g->axis.norm(), 1.0, 1e-12);
  EXPECT_NEAR(g->neck.z(), 0.88, 1e-12);
  EXPECT_DOUBLE_EQ(g->d95_m, 0.07);
  EXPECT_FALSE(g->branch_direction.has_value());
  EXPECT_FALSE(g->avoid_direction.has_value());
}

TEST(Conversions, GeometryRejectsInvalidLandmarksAxisAndSize)
{
  auto m = valid_model();
  m.neck.valid = false;
  EXPECT_FALSE(pm::geometry_from_msg(m).has_value());
  m = valid_model();
  m.axis.z = 0.0;
  EXPECT_FALSE(pm::geometry_from_msg(m).has_value());
  m = valid_model();
  m.d95_m = std::nan("");
  EXPECT_FALSE(pm::geometry_from_msg(m).has_value());
  m = valid_model();
  m.d95_m = 0.0;
  EXPECT_FALSE(pm::geometry_from_msg(m).has_value());
}

TEST(Conversions, DecisionFromMsg)
{
  peach2_interfaces::msg::GraspDecision d;
  d.target_id = "t";
  d.tool_id = "adaptive_shear_v1";
  d.model_revision = 7;
  d.valid_until.sec = 100;
  d.valid_until.nanosec = 500000000;
  d.approach_allowed = true;
  d.sleeve_allowed = true;
  d.cut_allowed = false;
  d.radial_margin_m = 0.01;
  d.axial_margin_m = -0.002;
  d.pregrasp_tcp.position.x = 0.4;
  d.pregrasp_tcp.orientation.w = 0.0;
  d.pregrasp_tcp.orientation.x = 1.0;
  d.blade_target.z = 0.9;
  d.insert_travel_m = 0.12;
  d.failure_code = pe::failure::BUDGET_AXIAL_NEGATIVE;
  d.reason = "cut_not_allowed";
  const auto v = pm::decision_from_msg(d);
  EXPECT_EQ(v.revision, 7u);
  EXPECT_DOUBLE_EQ(v.valid_until_s, 100.5);
  EXPECT_TRUE(v.sleeve_allowed);
  EXPECT_FALSE(v.cut_allowed);
  EXPECT_NEAR(v.pregrasp_tcp.translation().x(), 0.4, 1e-12);
  // 180 deg about X flips Z.
  EXPECT_NEAR(v.pregrasp_tcp.linear()(2, 2), -1.0, 1e-12);
  EXPECT_NEAR(v.blade_target.z(), 0.9, 1e-12);
  EXPECT_EQ(v.failure_code, pe::failure::BUDGET_AXIAL_NEGATIVE);
}

TEST(Conversions, PoseWithZeroQuaternionIsIdentityRotation)
{
  geometry_msgs::msg::Pose p;
  p.orientation.w = 0.0;
  const auto iso = pm::pose_from_msg(p);
  EXPECT_TRUE(iso.linear().isApprox(Eigen::Matrix3d::Identity()));
}

TEST(Conversions, ResultToMsgKeepsStagesAndCodes)
{
  pm::CycleResult r;
  r.target_id = "t";
  r.tool_id = "shear_v1";
  r.outcome = pm::Outcome::FAILED;
  r.reached = pm::Reached::INSERTED;
  r.failure_code = pe::failure::RETREAT_FAILED;
  r.reason = "x";
  r.recovery_required = true;
  r.stage_names = {"PREPARE_TOOL", "TRANSIT_STAGING"};
  r.stage_times_s = {0.2, 3.0};
  const auto m = pm::result_to_msg(r);
  EXPECT_EQ(m.outcome, peach2_interfaces::msg::HarvestResult::OUTCOME_FAILED);
  EXPECT_EQ(m.reached, peach2_interfaces::msg::HarvestResult::REACHED_INSERTED);
  EXPECT_EQ(m.failure_code, pe::failure::RETREAT_FAILED);
  EXPECT_TRUE(m.recovery_required);
  ASSERT_EQ(m.stage_names.size(), 2u);
  EXPECT_DOUBLE_EQ(m.stage_times_s[1], 3.0);
}

TEST(Conversions, ToolStateUnknownFeedbackAndNaNCurrent)
{
  pe::ToolStatus s;
  s.tool_id = "shear_v1";
  s.state = pe::ToolState::FAULT;
  s.command_closed = true;
  s.fault_reason = "close_feedback_timeout";
  s.suspected_loopback = true;
  const auto m = pm::tool_state_to_msg(s);
  EXPECT_EQ(m.state, peach2_interfaces::msg::ToolState::FAULT);
  EXPECT_EQ(m.feedback, peach2_interfaces::msg::ToolState::FEEDBACK_UNKNOWN);
  EXPECT_TRUE(m.suspected_loopback);
  EXPECT_TRUE(std::isnan(m.actuator_current_a));
  EXPECT_EQ(m.fault_reason, "close_feedback_timeout");

  s.current_a = 1.25;
  s.feedback_closed = true;
  s.suspected_loopback = false;
  const auto m2 = pm::tool_state_to_msg(s);
  EXPECT_EQ(m2.feedback, peach2_interfaces::msg::ToolState::FEEDBACK_CLOSED);
  EXPECT_FALSE(m2.suspected_loopback);
  EXPECT_FLOAT_EQ(m2.actuator_current_a, 1.25f);

  s.feedback_closed = false;
  EXPECT_EQ(
    pm::tool_state_to_msg(s).feedback, peach2_interfaces::msg::ToolState::FEEDBACK_OPEN);
}

TEST(Conversions, GeometryBranchDirectionOnlyWhenKnown)
{
  auto m = valid_model();
  m.branch_direction.y = 3.0;
  m.branch_direction_known = false;
  EXPECT_FALSE(pm::geometry_from_msg(m)->branch_direction.has_value());

  m.branch_direction_known = true;
  const auto g = pm::geometry_from_msg(m);
  ASSERT_TRUE(g->branch_direction.has_value());
  EXPECT_TRUE(g->branch_direction->isApprox(Eigen::Vector3d::UnitY()));

  // Known but unusable: treated as unknown (full roll), the model itself stays valid.
  m.branch_direction.y = std::nan("");
  const auto g2 = pm::geometry_from_msg(m);
  ASSERT_TRUE(g2.has_value());
  EXPECT_FALSE(g2->branch_direction.has_value());
  m.branch_direction.y = 0.0;
  EXPECT_FALSE(pm::geometry_from_msg(m)->branch_direction.has_value());
}

TEST(Conversions, ResultToMsgCarriesPlanOnly)
{
  pm::CycleResult r;
  r.outcome = pm::Outcome::SKIPPED;
  r.failure_code = pe::failure::NONE;
  r.reason = "planned";
  r.plan_only = true;
  const auto m = pm::result_to_msg(r);
  EXPECT_TRUE(m.plan_only);
  EXPECT_EQ(m.outcome, peach2_interfaces::msg::HarvestResult::OUTCOME_SKIPPED);
  EXPECT_EQ(m.failure_code, pe::failure::NONE);
  EXPECT_EQ(m.reached, peach2_interfaces::msg::HarvestResult::REACHED_NONE);
  r.plan_only = false;
  EXPECT_FALSE(pm::result_to_msg(r).plan_only);
}

TEST(Conversions, RobotStatusAndEnablesKeepReceiptTime)
{
  aubo_msgs::msg::RobotStatus rs;
  rs.drives_powered = 1;
  rs.motion_possible = 1;
  rs.error_code = 3;
  const auto s = pm::robot_status_from_msg(rs, 12.5);
  EXPECT_EQ(s.drives_powered, 1);
  EXPECT_EQ(s.error_code, 3);
  EXPECT_DOUBLE_EQ(s.received_s, 12.5);

  peach2_interfaces::msg::Enables en;
  en.seq = 9;
  en.execution = true;
  en.tool = true;
  const auto e = pm::enables_from_msg(en, 3.0);
  EXPECT_EQ(e.seq, 9u);
  EXPECT_TRUE(e.execution);
  EXPECT_FALSE(e.grasp);
  EXPECT_TRUE(e.tool);
  EXPECT_DOUBLE_EQ(e.received_s, 3.0);
}
