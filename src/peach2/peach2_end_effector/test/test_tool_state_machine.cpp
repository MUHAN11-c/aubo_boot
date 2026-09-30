#include <gtest/gtest.h>

#include "peach2_end_effector/tool_state_machine.hpp"

using peach2_end_effector::ToolState;
using peach2_end_effector::ToolStateMachine;
using peach2_end_effector::ToolStateMachineConfig;

namespace
{

ToolStateMachine make_sm(double timeout = 1.5, double min_act = 0.03, double energized = 0.0)
{
  ToolStateMachineConfig c;
  c.feedback_timeout_s = timeout;
  c.min_actuation_s = min_act;
  c.max_close_energized_s = energized;
  return ToolStateMachine(c);
}

/// UNKNOWN -> OPEN_CONFIRMED with a real (already open) blade.
void open_up(ToolStateMachine & sm, double t)
{
  sm.feedback(false, t);
  ASSERT_TRUE(sm.begin_command(false, t).accepted);
  sm.end_command(true, t);
  sm.feedback(false, t + 0.01);
  ASSERT_EQ(sm.state(), ToolState::OPEN_CONFIRMED);
}

}  // namespace

TEST(ToolStateMachine, StartsUnknownAndRejectsClose)
{
  auto sm = make_sm();
  EXPECT_EQ(sm.state(), ToolState::UNKNOWN);
  const auto d = sm.begin_command(true, 0.0);
  EXPECT_FALSE(d.accepted);
  EXPECT_EQ(sm.state(), ToolState::UNKNOWN);
}

TEST(ToolStateMachine, FullCycle)
{
  auto sm = make_sm();
  open_up(sm, 0.0);
  ASSERT_TRUE(sm.begin_command(true, 1.0).accepted);
  EXPECT_EQ(sm.state(), ToolState::CLOSING);
  sm.end_command(true, 1.0);
  EXPECT_TRUE(sm.command_closed());
  sm.feedback(false, 1.05);
  EXPECT_EQ(sm.state(), ToolState::CLOSING);
  sm.feedback(true, 1.2);
  EXPECT_EQ(sm.state(), ToolState::CLOSED_CONFIRMED);
  ASSERT_TRUE(sm.begin_command(false, 2.0).accepted);
  EXPECT_EQ(sm.state(), ToolState::OPENING);
  sm.end_command(true, 2.0);
  sm.feedback(true, 2.05);
  EXPECT_EQ(sm.state(), ToolState::OPENING);
  sm.feedback(false, 2.2);
  EXPECT_EQ(sm.state(), ToolState::OPEN_CONFIRMED);
  EXPECT_FALSE(sm.suspected_loopback());
}

TEST(ToolStateMachine, FeedbackDuringWriteIsNotConfirmation)
{
  auto sm = make_sm();
  open_up(sm, 0.0);
  ASSERT_TRUE(sm.begin_command(true, 1.0).accepted);
  sm.feedback(true, 1.0);  // sample while the write is in flight
  EXPECT_EQ(sm.state(), ToolState::CLOSING);
}

TEST(ToolStateMachine, CloseTimeoutFaults)
{
  auto sm = make_sm(1.5);
  open_up(sm, 0.0);
  sm.begin_command(true, 1.0);
  sm.end_command(true, 1.0);
  sm.update(2.4);
  EXPECT_EQ(sm.state(), ToolState::CLOSING);
  sm.update(2.6);
  EXPECT_EQ(sm.state(), ToolState::FAULT);
  EXPECT_EQ(sm.fault_reason(), "close_feedback_timeout");
  EXPECT_TRUE(sm.needs_deenergize());
}

TEST(ToolStateMachine, OpenTimeoutFaults)
{
  auto sm = make_sm(1.0);
  sm.feedback(true, 0.0);
  sm.begin_command(false, 0.0);
  sm.end_command(true, 0.0);
  sm.feedback(true, 0.5);
  sm.update(1.1);
  EXPECT_EQ(sm.state(), ToolState::FAULT);
  EXPECT_EQ(sm.fault_reason(), "open_feedback_timeout");
}

TEST(ToolStateMachine, WriteFailureFaults)
{
  auto sm = make_sm();
  open_up(sm, 0.0);
  sm.begin_command(true, 1.0);
  sm.end_command(false, 1.0);
  EXPECT_EQ(sm.state(), ToolState::FAULT);
  EXPECT_EQ(sm.fault_reason(), "command_write_failed");
  EXPECT_FALSE(sm.command_closed());
}

TEST(ToolStateMachine, ZeroDelayCloseIsLoopback)
{
  auto sm = make_sm();
  open_up(sm, 0.0);
  sm.begin_command(true, 1.0);
  sm.end_command(true, 1.0);
  sm.feedback(true, 1.0);
  EXPECT_EQ(sm.state(), ToolState::FAULT);
  EXPECT_TRUE(sm.suspected_loopback());
  EXPECT_EQ(sm.fault_reason(), "suspected_loopback");
}

TEST(ToolStateMachine, ZeroDelayOpenAfterClosedIsLoopback)
{
  auto sm = make_sm(1.5, 0.03);
  open_up(sm, 0.0);
  sm.begin_command(true, 1.0);
  sm.end_command(true, 1.0);
  sm.feedback(true, 1.2);
  ASSERT_EQ(sm.state(), ToolState::CLOSED_CONFIRMED);
  sm.begin_command(false, 2.0);
  sm.end_command(true, 2.0);
  sm.feedback(false, 2.001);
  EXPECT_EQ(sm.state(), ToolState::FAULT);
  EXPECT_TRUE(sm.suspected_loopback());
}

TEST(ToolStateMachine, UnexpectedFeedbackFaults)
{
  auto sm = make_sm();
  open_up(sm, 0.0);
  sm.feedback(true, 0.5);
  EXPECT_EQ(sm.state(), ToolState::FAULT);
  EXPECT_EQ(sm.fault_reason(), "unexpected_feedback_closed");

  auto sm2 = make_sm();
  open_up(sm2, 0.0);
  sm2.begin_command(true, 1.0);
  sm2.end_command(true, 1.0);
  sm2.feedback(true, 1.2);
  sm2.feedback(false, 1.5);
  EXPECT_EQ(sm2.state(), ToolState::FAULT);
  EXPECT_EQ(sm2.fault_reason(), "unexpected_feedback_open");
}

TEST(ToolStateMachine, CloseWithFeedbackAlreadyClosedFaults)
{
  auto sm = make_sm();
  open_up(sm, 0.0);
  // Feedback flips high without a command being in progress -> FAULT first.
  sm.feedback(true, 0.5);
  EXPECT_FALSE(sm.begin_command(true, 0.6).accepted);
  EXPECT_EQ(sm.state(), ToolState::FAULT);
}

TEST(ToolStateMachine, FaultOnlyLeftByAck)
{
  auto sm = make_sm();
  sm.fault("test");
  EXPECT_EQ(sm.state(), ToolState::FAULT);
  EXPECT_FALSE(sm.begin_command(true, 0.0).accepted);
  // Open is accepted (energy-reducing) but does not leave FAULT.
  EXPECT_TRUE(sm.begin_command(false, 0.0).accepted);
  sm.end_command(true, 0.0);
  sm.feedback(false, 0.5);
  sm.update(10.0);
  EXPECT_EQ(sm.state(), ToolState::FAULT);
  sm.mark_unknown();
  EXPECT_EQ(sm.state(), ToolState::FAULT);
  sm.reset_by_ack();
  EXPECT_EQ(sm.state(), ToolState::UNKNOWN);
  EXPECT_TRUE(sm.fault_reason().empty());
  EXPECT_FALSE(sm.suspected_loopback());
}

TEST(ToolStateMachine, FirstFaultReasonKept)
{
  auto sm = make_sm();
  sm.fault("first");
  sm.fault("second");
  EXPECT_EQ(sm.fault_reason(), "first");
}

TEST(ToolStateMachine, MarkUnknownFromConfirmed)
{
  auto sm = make_sm();
  open_up(sm, 0.0);
  sm.mark_unknown();
  EXPECT_EQ(sm.state(), ToolState::UNKNOWN);
  EXPECT_FALSE(sm.begin_command(true, 1.0).accepted);
}

TEST(ToolStateMachine, CommandInFlightRejected)
{
  auto sm = make_sm();
  ASSERT_TRUE(sm.begin_command(false, 0.0).accepted);
  EXPECT_FALSE(sm.begin_command(false, 0.0).accepted);
}

TEST(ToolStateMachine, CloseEnergizedLimit)
{
  auto sm = make_sm(1.5, 0.03, 2.0);
  open_up(sm, 0.0);
  sm.begin_command(true, 1.0);
  sm.end_command(true, 1.0);
  sm.feedback(true, 1.2);
  sm.update(2.9);
  EXPECT_EQ(sm.state(), ToolState::CLOSED_CONFIRMED);
  sm.update(3.1);
  EXPECT_EQ(sm.state(), ToolState::FAULT);
  EXPECT_EQ(sm.fault_reason(), "close_energized_too_long");
}

TEST(ToolStateMachine, OpenFromClosingAborts)
{
  auto sm = make_sm();
  open_up(sm, 0.0);
  sm.begin_command(true, 1.0);
  sm.end_command(true, 1.0);
  ASSERT_TRUE(sm.begin_command(false, 1.1).accepted);
  sm.end_command(true, 1.1);
  sm.feedback(false, 1.2);
  EXPECT_EQ(sm.state(), ToolState::OPEN_CONFIRMED);
}
