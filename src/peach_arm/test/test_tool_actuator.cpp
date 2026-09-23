// 批次3（2026-09-23）：ToolActuator 三轴 + DI 沿 + 保守语义单测。
#include <gtest/gtest.h>

#include <string>

#include "peach_arm/tool_actuator.hpp"

namespace
{

using peach_arm::ToolActuator;
using peach_arm::ToolActuatorState;
using peach_arm::ToolCommandContext;

ToolActuator armed_and_cut()
{
  peach_arm::ToolActuator actuator;
  actuator.setSendIo([](std::string &) {return true;});
  peach_arm::ToolCommandContext ctx;
  ctx.target_id = "t0";
  ctx.contact_transaction_id = "tx0";
  std::string reason;
  actuator.arm(ctx, reason);
  actuator.sendCut(reason);
  return actuator;
}

}  // namespace

TEST(ToolActuator, DiClosedRiseConfirmsCut)
{
  auto actuator = armed_and_cut();
  std::string edge;
  // 先低电平（无沿），再上升沿 → confirm
  ASSERT_FALSE(actuator.ingestToolDi(false, edge));
  ASSERT_TRUE(actuator.ingestToolDi(true, edge));
  EXPECT_EQ(edge, "di:closed-rise");
  // 新沿已见 → confirm 可达且成功
  std::string reason;
  EXPECT_TRUE(actuator.confirmFeedback(true, reason));
  EXPECT_EQ(actuator.state(),
    peach_arm::ToolActuatorState::CUT_FEEDBACK_CONFIRMED);
  const auto axes = actuator.axes();
  EXPECT_EQ(axes.blade, 3u);  // BLADE_CLOSED
}

TEST(ToolActuator, StuckHighDiIsNotNewEdge)
{
  auto actuator = armed_and_cut();
  std::string edge;
  // 首帧即高（早已卡高）：无沿基准→不算新沿；第二帧仍高→无变化
  ASSERT_FALSE(actuator.ingestToolDi(true, edge));
  ASSERT_FALSE(actuator.ingestToolDi(true, edge));
  std::string reason;
  EXPECT_FALSE(actuator.confirmFeedback(true, reason));
}

TEST(ToolActuator, OpenFallReleasesPayloadAxes)
{
  auto actuator = armed_and_cut();
  std::string edge;
  actuator.ingestToolDi(false, edge);
  actuator.ingestToolDi(true, edge);   // closed-rise
  ASSERT_TRUE(actuator.ingestToolDi(false, edge));  // open-fall
  EXPECT_EQ(edge, "di:open-fall");
  const auto axes = actuator.axes();
  EXPECT_EQ(axes.blade, 1u);    // BLADE_OPEN
  EXPECT_EQ(axes.payload, 1u);  // PAYLOAD_ABSENT
  EXPECT_EQ(axes.retention, 1u);  // RETENTION_READY
}

TEST(ToolActuator, CutOncePerTransaction)
{
  auto actuator = armed_and_cut();
  std::string reason;
  // 状态门在前：非 ARMED 态再发 → tool_not_armed（幂等由状态+事务双门保证）
  EXPECT_FALSE(actuator.sendCut(reason));
  EXPECT_EQ(reason, "tool_not_armed");
  // 同事务重 arm 也拒（事务门）
  peach_arm::ToolCommandContext same_ctx;
  same_ctx.contact_transaction_id = "tx0";
  EXPECT_FALSE(actuator.arm(same_ctx, reason));
  EXPECT_EQ(reason, "cut_already_commanded_this_transaction");
}

TEST(ToolActuator, ResetRestoresUnknownAxes)
{
  auto actuator = armed_and_cut();
  actuator.resetSafe();
  EXPECT_EQ(actuator.state(), peach_arm::ToolActuatorState::SAFE);
  const auto axes = actuator.axes();
  EXPECT_EQ(axes.blade, 0u);   // UNKNOWN
  EXPECT_EQ(axes.payload, 0u);
}

TEST(ToolActuator, HarvestNeedsBothEvidences)
{
  EXPECT_FALSE(peach_arm::harvestConfirmed(false, true));
  EXPECT_FALSE(peach_arm::harvestConfirmed(true, false));
  EXPECT_TRUE(peach_arm::harvestConfirmed(true, true));
}
