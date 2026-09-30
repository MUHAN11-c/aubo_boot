#include <gtest/gtest.h>

#include <chrono>
#include <string>

#include "peach2_end_effector/failure_codes.hpp"
#include "test_helpers.hpp"

namespace ee = peach2_end_effector;
using Fault = ee::MockIoBackend::Fault;
using peach2_test::Rig;
using std::chrono::milliseconds;

namespace
{

ee::MockIoBackend::Config mock_with(Fault fault)
{
  ee::MockIoBackend::Config c;
  c.fault = fault;
  return c;
}

}  // namespace

TEST(MockFlow, PrepareCutConfirmRelease)
{
  Rig rig;
  const auto p = rig.ee->prepare();
  ASSERT_TRUE(p.ok) << p.reason;
  EXPECT_EQ(rig.ee->status().state, ee::ToolState::OPEN_CONFIRMED);
  EXPECT_EQ(rig.io->write_count(), 1);

  const auto c = rig.ee->cut();
  ASSERT_TRUE(c.ok) << c.reason;
  EXPECT_EQ(rig.ee->status().state, ee::ToolState::CLOSING);
  EXPECT_EQ(rig.io->last_output(0), true);

  const double t0 = rig.clock->now();
  const auto v = rig.ee->confirm_cut(milliseconds(2000));
  ASSERT_TRUE(v.confirmed) << v.reason;
  EXPECT_TRUE(v.feedback_edge);
  EXPECT_FALSE(v.current_checked);
  EXPECT_GE(rig.clock->now() - t0, 0.15 - 1e-9);
  EXPECT_EQ(rig.ee->status().state, ee::ToolState::CLOSED_CONFIRMED);

  const auto r = rig.ee->release();
  ASSERT_TRUE(r.ok) << r.reason;
  EXPECT_EQ(rig.ee->status().state, ee::ToolState::OPEN_CONFIRMED);
  EXPECT_EQ(rig.io->last_output(0), false);
  EXPECT_FALSE(rig.ee->status().suspected_loopback);
}

TEST(MockFlow, PrepareIdempotentWhenOpen)
{
  Rig rig;
  ASSERT_TRUE(rig.ee->prepare().ok);
  const auto again = rig.ee->prepare();
  EXPECT_TRUE(again.ok);
  EXPECT_EQ(again.reason, "prepare_already_open");
  EXPECT_EQ(rig.io->write_count(), 1);
}

TEST(MockFlow, ActiveLowFeedback)
{
  ee::MockIoBackend::Config c;
  c.feedback_active_high = false;
  Rig rig("adaptive_shear_v1", c);
  ASSERT_TRUE(rig.ee->prepare().ok);
  ASSERT_TRUE(rig.ee->cut().ok);
  EXPECT_TRUE(rig.ee->confirm_cut(milliseconds(2000)).confirmed);
}

TEST(MockFlow, CutRequiresConfirmedOpen)
{
  Rig rig;
  const auto c = rig.ee->cut();
  EXPECT_FALSE(c.ok);
  EXPECT_EQ(c.failure_code, ee::failure::TOOL_NOT_OPEN);
  EXPECT_EQ(rig.io->write_count(), 0);
}

TEST(MockFlow, ConfirmWithoutCutIsNotConfirmed)
{
  Rig rig;
  ASSERT_TRUE(rig.ee->prepare().ok);
  const auto v = rig.ee->confirm_cut(milliseconds(200));
  EXPECT_FALSE(v.confirmed);
  EXPECT_EQ(v.failure_code, ee::failure::CUT_NOT_CONFIRMED);
}

TEST(MockFlow, WriteFailureThenAck)
{
  Rig rig("adaptive_shear_v1", mock_with(Fault::WRITE_FAILS));
  const auto p = rig.ee->prepare();
  EXPECT_FALSE(p.ok);
  EXPECT_EQ(p.failure_code, ee::failure::TOOL_COMMAND_FAILED);
  EXPECT_EQ(rig.ee->status().state, ee::ToolState::FAULT);
  rig.io->set_fault(Fault::NONE);
  EXPECT_EQ(rig.ee->prepare().failure_code, ee::failure::TOOL_FAULT);
  rig.ee->reset_by_ack();
  EXPECT_EQ(rig.ee->status().state, ee::ToolState::UNKNOWN);
  EXPECT_TRUE(rig.ee->prepare().ok);
}

TEST(MockFlow, NoFeedbackRefusesToPrepare)
{
  Rig rig("adaptive_shear_v1", mock_with(Fault::NO_FEEDBACK));
  const auto p = rig.ee->prepare();
  EXPECT_FALSE(p.ok);
  EXPECT_EQ(p.failure_code, ee::failure::TOOL_NOT_OPEN);
  EXPECT_EQ(rig.io->write_count(), 0);
}

TEST(MockFlow, StuckClosedPrepareTimesOut)
{
  Rig rig("adaptive_shear_v1", mock_with(Fault::STUCK_CLOSED));
  const double t0 = rig.clock->now();
  const auto p = rig.ee->prepare();
  EXPECT_FALSE(p.ok);
  EXPECT_EQ(p.failure_code, ee::failure::TOOL_NOT_OPEN);
  EXPECT_EQ(rig.ee->status().state, ee::ToolState::FAULT);
  EXPECT_EQ(rig.ee->status().fault_reason, "open_feedback_timeout");
  EXPECT_GE(rig.clock->now() - t0, 1.5);
  EXPECT_LE(rig.clock->now() - t0, 1.7);
}

TEST(MockFlow, StuckOpenCutTimesOutAndAbortOpens)
{
  Rig rig("adaptive_shear_v1", mock_with(Fault::STUCK_OPEN));
  ASSERT_TRUE(rig.ee->prepare().ok);
  ASSERT_TRUE(rig.ee->cut().ok);
  const auto v = rig.ee->confirm_cut(milliseconds(3000));
  EXPECT_FALSE(v.confirmed);
  EXPECT_EQ(v.failure_code, ee::failure::TOOL_FEEDBACK_TIMEOUT);
  EXPECT_EQ(rig.ee->status().state, ee::ToolState::FAULT);
  EXPECT_TRUE(rig.ee->status().command_closed);

  const auto a = rig.ee->abort_safe();
  EXPECT_FALSE(a.ok);
  EXPECT_EQ(a.failure_code, ee::failure::TOOL_FAULT);
  EXPECT_EQ(rig.io->last_output(0), false);  // open level written even in FAULT
  EXPECT_FALSE(rig.ee->status().command_closed);
  EXPECT_EQ(rig.ee->cut().failure_code, ee::failure::TOOL_FAULT);
}

TEST(MockFlow, ConfirmWindowShorterThanActuation)
{
  Rig rig;
  ASSERT_TRUE(rig.ee->prepare().ok);
  ASSERT_TRUE(rig.ee->cut().ok);
  const auto v = rig.ee->confirm_cut(milliseconds(50));
  EXPECT_EQ(v.failure_code, ee::failure::TOOL_FEEDBACK_TIMEOUT);
  EXPECT_EQ(rig.ee->status().state, ee::ToolState::FAULT);
}

TEST(MockFlow, LoopbackDetected)
{
  Rig rig("adaptive_shear_v1", mock_with(Fault::LOOPBACK));
  ASSERT_TRUE(rig.ee->prepare().ok);
  ASSERT_TRUE(rig.ee->cut().ok);
  const auto v = rig.ee->confirm_cut(milliseconds(2000));
  EXPECT_FALSE(v.confirmed);
  EXPECT_EQ(v.failure_code, ee::failure::TOOL_FAULT);
  const auto s = rig.ee->status();
  EXPECT_TRUE(s.suspected_loopback);
  EXPECT_EQ(s.state, ee::ToolState::FAULT);
}

TEST(MockFlow, CurrentSignatureConfirms)
{
  ee::MockIoBackend::Config c;
  c.simulate_current = true;
  c.current_cut_signature = true;
  ee::CurrentSignatureConfig cur;
  cur.enabled = true;
  Rig rig("adaptive_shear_v1", c, cur);
  ASSERT_TRUE(rig.ee->prepare().ok);
  ASSERT_TRUE(rig.ee->cut().ok);
  const auto v = rig.ee->confirm_cut(milliseconds(2000));
  EXPECT_TRUE(v.confirmed) << v.reason;
  EXPECT_TRUE(v.current_checked);
  EXPECT_GT(v.peak_current_a, 1.0);
}

TEST(MockFlow, MissingCurrentSignatureIsCutNotConfirmedThenAbortOpens)
{
  ee::MockIoBackend::Config c;
  c.simulate_current = true;
  c.current_cut_signature = false;
  ee::CurrentSignatureConfig cur;
  cur.enabled = true;
  Rig rig("adaptive_shear_v1", c, cur);
  ASSERT_TRUE(rig.ee->prepare().ok);
  ASSERT_TRUE(rig.ee->cut().ok);
  const auto v = rig.ee->confirm_cut(milliseconds(2000));
  EXPECT_FALSE(v.confirmed);
  EXPECT_TRUE(v.feedback_edge);
  EXPECT_EQ(v.failure_code, ee::failure::CUT_NOT_CONFIRMED);
  // Retry path: open, confirm open, cut again.
  const auto a = rig.ee->abort_safe();
  EXPECT_TRUE(a.ok) << a.reason;
  EXPECT_EQ(rig.ee->status().state, ee::ToolState::OPEN_CONFIRMED);
  EXPECT_TRUE(rig.ee->cut().ok);
}

TEST(MockFlow, CurrentUnavailableWithSignatureRequired)
{
  ee::CurrentSignatureConfig cur;
  cur.enabled = true;
  Rig rig("adaptive_shear_v1", {}, cur);
  ASSERT_TRUE(rig.ee->prepare().ok);
  ASSERT_TRUE(rig.ee->cut().ok);
  const auto v = rig.ee->confirm_cut(milliseconds(2000));
  EXPECT_EQ(v.failure_code, ee::failure::CUT_NOT_CONFIRMED);
  EXPECT_EQ(v.reason, "confirm_cut:current_unavailable");
}

TEST(MockFlow, ReleaseFailsWhenBladeStaysClosed)
{
  Rig rig("adaptive_shear_v1", mock_with(Fault::OPEN_STUCK));
  ASSERT_TRUE(rig.ee->prepare().ok);
  ASSERT_TRUE(rig.ee->cut().ok);
  ASSERT_TRUE(rig.ee->confirm_cut(milliseconds(2000)).confirmed);
  const auto r = rig.ee->release();
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.failure_code, ee::failure::TOOL_NOT_OPEN);
  EXPECT_EQ(rig.ee->status().state, ee::ToolState::FAULT);
}

TEST(MockFlow, MarkUnknownForcesReprepare)
{
  Rig rig;
  ASSERT_TRUE(rig.ee->prepare().ok);
  rig.ee->mark_unknown("robot e-stop observed");
  EXPECT_EQ(rig.ee->status().state, ee::ToolState::UNKNOWN);
  EXPECT_EQ(rig.ee->cut().failure_code, ee::failure::TOOL_NOT_OPEN);
  EXPECT_TRUE(rig.ee->prepare().ok);
  EXPECT_TRUE(rig.ee->cut().ok);
}

TEST(MockFlow, PollDeenergizesAfterFault)
{
  Rig rig("adaptive_shear_v1", mock_with(Fault::STUCK_OPEN));
  ASSERT_TRUE(rig.ee->prepare().ok);
  ASSERT_TRUE(rig.ee->cut().ok);
  for (int i = 0; i < 20; ++i) {
    rig.clock->sleep(0.1);
    rig.ee->poll();
  }
  EXPECT_EQ(rig.ee->status().state, ee::ToolState::FAULT);
  EXPECT_EQ(rig.ee->status().fault_reason, "close_feedback_timeout");
  EXPECT_EQ(rig.io->last_output(0), false);
}

TEST(MockFlow, AllThreePluginsRunTheSameFlow)
{
  for (const std::string id : {"shear_v1", "bite_shear_v1", "adaptive_shear_v1"}) {
    Rig rig(id);
    ASSERT_TRUE(rig.ee->prepare().ok) << id;
    ASSERT_TRUE(rig.ee->cut().ok) << id;
    ASSERT_TRUE(rig.ee->confirm_cut(milliseconds(2000)).confirmed) << id;
    ASSERT_TRUE(rig.ee->release().ok) << id;
  }
}
