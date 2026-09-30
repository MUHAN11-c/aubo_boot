#include <gtest/gtest.h>

#include "peach2_end_effector/failure_codes.hpp"
#include "peach2_manipulation/command_gate.hpp"

namespace failure = peach2_end_effector::failure;
using peach2_manipulation::ClosedEdge;
using peach2_manipulation::CommandGate;
using peach2_manipulation::EnablesSample;
using peach2_manipulation::GateConfig;
using peach2_manipulation::GateStage;
using peach2_manipulation::RobotStatusSample;

namespace
{

RobotStatusSample healthy(double t)
{
  RobotStatusSample s;
  s.mode = 2;
  s.drives_powered = 1;
  s.motion_possible = 1;
  s.received_s = t;
  return s;
}

EnablesSample en(bool e, bool g, bool t, double at)
{
  EnablesSample s;
  s.execution = e;
  s.grasp = g;
  s.tool = t;
  s.received_s = at;
  return s;
}

CommandGate ready_gate(double t)
{
  CommandGate gate;
  gate.set_active(true);
  gate.on_robot_status(healthy(t));
  gate.on_enables(en(true, true, true, t));
  return gate;
}

}  // namespace

TEST(CommandGate, DefaultClosed)
{
  CommandGate gate;
  const auto v = gate.check(GateStage::TRANSIT, 0.0, true);
  EXPECT_FALSE(v.open);
  EXPECT_EQ(v.failure_code, failure::SAFETY_GATE_CLOSED);
  EXPECT_EQ(v.reason, "not_active");
}

TEST(CommandGate, AllOpenWhenReady)
{
  auto gate = ready_gate(10.0);
  for (auto s : {GateStage::TRANSIT, GateStage::APPROACH, GateStage::CONTACT, GateStage::TOOL,
      GateStage::RETREAT, GateStage::RELEASE, GateStage::TOOL_SAFE})
  {
    EXPECT_TRUE(gate.check(s, 10.1, true).open) << peach2_manipulation::to_string(s);
  }
}

TEST(CommandGate, RobotStatusConditions)
{
  auto gate = ready_gate(10.0);
  auto s = healthy(10.0);
  EXPECT_EQ(gate.check(GateStage::TRANSIT, 10.5, false).reason, "robot_status_stale");
  s.e_stopped = 1;
  gate.on_robot_status(s);
  EXPECT_EQ(gate.check(GateStage::TRANSIT, 10.1, false).reason, "e_stopped");
  s = healthy(10.0);
  s.drives_powered = 0;
  gate.on_robot_status(s);
  EXPECT_EQ(gate.check(GateStage::TRANSIT, 10.1, false).reason, "drives_unpowered");
  s = healthy(10.0);
  s.in_error = 1;
  s.error_code = 7;
  gate.on_robot_status(s);
  const auto v = gate.check(GateStage::TRANSIT, 10.1, false);
  EXPECT_EQ(v.failure_code, failure::ROBOT_NOT_READY);
  EXPECT_EQ(v.reason, "robot_in_error:7");
}

TEST(CommandGate, MotionPossibleOnlyBeforeNewTrajectory)
{
  auto gate = ready_gate(10.0);
  auto s = healthy(10.0);
  s.motion_possible = 0;   // driver reports 0 while a trajectory streams
  s.in_motion = 1;
  gate.on_robot_status(s);
  EXPECT_TRUE(gate.check(GateStage::CONTACT, 10.1, false).open);
  const auto v = gate.check(GateStage::CONTACT, 10.1, true);
  EXPECT_FALSE(v.open);
  EXPECT_EQ(v.reason, "motion_not_possible");
}

TEST(CommandGate, MissingRobotStatus)
{
  CommandGate gate;
  gate.set_active(true);
  gate.on_enables(en(true, true, true, 0.0));
  EXPECT_EQ(gate.check(GateStage::TRANSIT, 0.1, true).reason, "robot_status_missing");
  GateConfig mock;
  mock.require_robot_status = false;
  CommandGate mock_gate(mock);
  mock_gate.set_active(true);
  mock_gate.on_enables(en(true, true, true, 0.0));
  EXPECT_TRUE(mock_gate.check(GateStage::TRANSIT, 0.1, true).open);
}

TEST(CommandGate, HeartbeatTimeoutMeansAllOff)
{
  auto gate = ready_gate(10.0);
  gate.on_robot_status(healthy(14.0));
  const auto e = gate.enables(14.0);
  EXPECT_FALSE(e.heartbeat_ok);
  EXPECT_FALSE(e.execution);
  const auto v = gate.check(GateStage::TRANSIT, 14.0, true);
  EXPECT_FALSE(v.open);
  EXPECT_EQ(v.reason, "enables_heartbeat_lost");
}

TEST(CommandGate, EnablesChainTruncates)
{
  auto gate = ready_gate(10.0);
  gate.on_enables(en(true, false, true, 10.0));   // tool without grasp
  auto e = gate.enables(10.1);
  EXPECT_TRUE(e.execution);
  EXPECT_FALSE(e.grasp);
  EXPECT_FALSE(e.tool);
  EXPECT_TRUE(gate.check(GateStage::APPROACH, 10.1, true).open);
  EXPECT_EQ(gate.check(GateStage::CONTACT, 10.1, true).reason, "grasp_disabled");
  EXPECT_EQ(gate.check(GateStage::TOOL, 10.1, false).reason, "grasp_disabled");
  gate.on_enables(en(false, true, true, 10.0));
  e = gate.enables(10.1);
  EXPECT_FALSE(e.grasp);
  EXPECT_FALSE(e.tool);
  EXPECT_EQ(gate.check(GateStage::RETREAT, 10.1, true).reason, "execution_disabled");
  EXPECT_EQ(gate.check(GateStage::RELEASE, 10.1, false).reason, "execution_disabled");
  gate.on_enables(en(true, true, false, 10.0));
  EXPECT_TRUE(gate.check(GateStage::CONTACT, 10.1, true).open);
  EXPECT_EQ(gate.check(GateStage::TOOL, 10.1, false).reason, "tool_disabled");
  EXPECT_EQ(gate.check(GateStage::RELEASE, 10.1, false).reason, "tool_disabled");
}

TEST(CommandGate, RetreatNeedsOnlyExecution)
{
  auto gate = ready_gate(10.0);
  gate.on_enables(en(true, false, false, 10.0));
  EXPECT_TRUE(gate.check(GateStage::RETREAT, 10.1, true).open);
}

TEST(CommandGate, CancelClosesEverythingButToolSafe)
{
  auto gate = ready_gate(10.0);
  gate.set_cancel(true);
  const auto v = gate.check(GateStage::RETREAT, 10.1, true);
  EXPECT_FALSE(v.open);
  EXPECT_EQ(v.failure_code, failure::CANCELED);
  EXPECT_TRUE(gate.check(GateStage::TOOL_SAFE, 10.1, false).open);
}

TEST(CommandGate, ToolSafeIgnoresEnablesButNotEstop)
{
  auto gate = ready_gate(10.0);
  gate.on_enables(en(false, false, false, 10.0));
  EXPECT_TRUE(gate.check(GateStage::TOOL_SAFE, 10.1, false).open);
  auto s = healthy(10.0);
  s.drives_powered = 0;
  gate.on_robot_status(s);
  EXPECT_TRUE(gate.check(GateStage::TOOL_SAFE, 10.1, false).open);
  s.e_stopped = 1;
  gate.on_robot_status(s);
  EXPECT_FALSE(gate.check(GateStage::TOOL_SAFE, 10.1, false).open);
  EXPECT_FALSE(gate.check(GateStage::TOOL_SAFE, 20.0, false).open);
  gate.set_active(false);
  EXPECT_FALSE(gate.check(GateStage::TOOL_SAFE, 10.1, false).open);
}

TEST(ClosedEdge, FiresOncePerTransition)
{
  ClosedEdge edge;
  EXPECT_FALSE(edge.update(false));
  EXPECT_FALSE(edge.update(true));
  EXPECT_TRUE(edge.update(false));
  EXPECT_FALSE(edge.update(false));
  EXPECT_FALSE(edge.update(true));
  EXPECT_TRUE(edge.update(false));
}
