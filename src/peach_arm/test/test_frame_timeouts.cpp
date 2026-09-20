// W5-3：FrameRateTimeouts 公式对拍（边界值表）。公式自节点 effective* 逐字
// 搬移，本表钉死 EMA 未测得/高低帧率/异常间隔/COLLECTING 各分支的数值。
#include "peach_arm/frame_timeouts.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace
{
peach_arm::FrameRateTimeouts makeDefault()
{
  peach_arm::FrameRateTimeoutConfig config;
  config.assumed_frame_interval_s = 0.4;
  config.frame_wait_s = 4.0;
  config.target_observation_max_age_config_s = 3.0;
  config.reconfirm_wait_s = 6.0;
  config.refined_timeout_s = 30.0;
  return peach_arm::FrameRateTimeouts(config);
}
}  // namespace

TEST(FrameRateTimeouts, FreshNodeUsesAssumedIntervalEstimates)
{
  const auto timeouts = makeDefault();
  EXPECT_DOUBLE_EQ(timeouts.frameIntervalEmaS(), 0.0);
  // EMA 未测得：等帧/再确认/精化窗口用 assumed=0.4 估（4 帧+1s、4 帧+1s、
  // 3 帧+2s）；新鲜度门保持 yaml 回退 3s。
  EXPECT_NEAR(timeouts.waitIntervalS(), 0.4, 1e-12);
  EXPECT_NEAR(timeouts.frameWaitS(), 2.6, 1e-12);
  EXPECT_NEAR(timeouts.targetMaxAgeS(), 3.0, 1e-12);
  EXPECT_NEAR(timeouts.reconfirmWaitS(), 2.6, 1e-12);
  EXPECT_NEAR(timeouts.refinedWaitS(false), 3.2, 1e-12);
  // COLLECTING：配置上限覆盖采集→finalize→refit 全程。
  EXPECT_NEAR(timeouts.refinedWaitS(true), 30.0, 1e-12);
}

TEST(FrameRateTimeouts, CollectingOverridesShortWindowRegardlessOfEma)
{
  auto timeouts = makeDefault();
  timeouts.onTargetFrame(10.0);
  timeouts.onTargetFrame(10.5);
  EXPECT_NEAR(timeouts.frameIntervalEmaS(), 0.5, 1e-12);
  EXPECT_NEAR(timeouts.refinedWaitS(true), 30.0, 1e-12);
}

TEST(FrameRateTimeouts, MeasuredEmaDrivesAllWindows)
{
  auto timeouts = makeDefault();
  timeouts.onTargetFrame(10.0);
  timeouts.onTargetFrame(10.5);  // dt=0.5 → EMA=0.5
  EXPECT_NEAR(timeouts.frameIntervalEmaS(), 0.5, 1e-12);
  EXPECT_NEAR(timeouts.waitIntervalS(), 0.5, 1e-12);
  EXPECT_NEAR(timeouts.frameWaitS(), 3.0, 1e-12);      // 4×0.5+1
  EXPECT_NEAR(timeouts.targetMaxAgeS(), 3.0, 1e-12);   // 自适应 1.75 不得收过回退 3s
  EXPECT_NEAR(timeouts.reconfirmWaitS(), 3.0, 1e-12);  // 4×0.5+1
  EXPECT_NEAR(timeouts.refinedWaitS(false), 3.5, 1e-12);  // 3×0.5+2
}

TEST(FrameRateTimeouts, LowFrameRateWidensUpToCaps)
{
  auto timeouts = makeDefault();
  timeouts.onTargetFrame(10.0);
  timeouts.onTargetFrame(10.5);  // EMA=0.5
  timeouts.onTargetFrame(18.5);  // dt=8 → EMA=0.7×0.5+0.3×8=2.75
  EXPECT_NEAR(timeouts.frameIntervalEmaS(), 2.75, 1e-12);
  EXPECT_NEAR(timeouts.frameWaitS(), 4.0, 1e-12);        // 12 → cap 4s
  EXPECT_NEAR(timeouts.targetMaxAgeS(), 7.375, 1e-12);   // max(3, 2.5×2.75+0.5)
  EXPECT_NEAR(timeouts.reconfirmWaitS(), 6.0, 1e-12);    // 12 → cap 6s
  EXPECT_NEAR(timeouts.refinedWaitS(false), 10.25, 1e-12);  // 3×2.75+2
}

TEST(FrameRateTimeouts, HighFrameRateFloors)
{
  auto timeouts = makeDefault();
  timeouts.onTargetFrame(100.0);
  timeouts.onTargetFrame(100.05);  // EMA=0.05
  EXPECT_NEAR(timeouts.frameWaitS(), 2.0, 1e-12);       // 1.2 → floor 2s
  EXPECT_NEAR(timeouts.targetMaxAgeS(), 3.0, 1e-12);    // 自适应 0.625 → 回退 3s
  EXPECT_NEAR(timeouts.reconfirmWaitS(), 2.0, 1e-12);   // 1.2 → floor 2s
  EXPECT_NEAR(timeouts.refinedWaitS(false), 2.15, 1e-12);  // 3×0.05+2
}

TEST(FrameRateTimeouts, AbnormalIntervalsDoNotEnterEma)
{
  auto timeouts = makeDefault();
  timeouts.onTargetFrame(10.0);
  timeouts.onTargetFrame(10.5);  // EMA=0.5
  // dt=34.5（时钟跳变/暂停后首帧）不进 EMA：
  timeouts.onTargetFrame(45.0);
  EXPECT_NEAR(timeouts.frameIntervalEmaS(), 0.5, 1e-12);
  // dt=0.0005（重复帧）不进 EMA：
  timeouts.onTargetFrame(45.0005);
  EXPECT_NEAR(timeouts.frameIntervalEmaS(), 0.5, 1e-12);
  // 恢复正常帧后继续 0.7/0.3 混合：
  timeouts.onTargetFrame(45.2);  // dt=0.1995
  EXPECT_NEAR(timeouts.frameIntervalEmaS(), 0.7 * 0.5 + 0.3 * 0.1995, 1e-12);
}

TEST(FrameRateTimeouts, FirstFrameDoesNotSetEma)
{
  auto timeouts = makeDefault();
  timeouts.onTargetFrame(10.0);
  EXPECT_DOUBLE_EQ(timeouts.frameIntervalEmaS(), 0.0);
  EXPECT_NEAR(timeouts.frameWaitS(), 2.6, 1e-12);
}

TEST(FrameRateTimeouts, ZeroAssumedFallsBackToConfigCaps)
{
  peach_arm::FrameRateTimeoutConfig config;
  config.assumed_frame_interval_s = 0.0;
  config.frame_wait_s = 4.0;
  config.target_observation_max_age_config_s = 3.0;
  config.reconfirm_wait_s = 6.0;
  config.refined_timeout_s = 30.0;
  const peach_arm::FrameRateTimeouts timeouts(config);
  EXPECT_NEAR(timeouts.waitIntervalS(), 0.0, 1e-12);
  EXPECT_NEAR(timeouts.frameWaitS(), 4.0, 1e-12);
  EXPECT_NEAR(timeouts.targetMaxAgeS(), 3.0, 1e-12);
  EXPECT_NEAR(timeouts.reconfirmWaitS(), 6.0, 1e-12);
  EXPECT_NEAR(timeouts.refinedWaitS(false), 30.0, 1e-12);
}

TEST(FrameRateTimeouts, RefinedFloorFollowsCapWhenCapBelowTwo)
{
  peach_arm::FrameRateTimeoutConfig config;
  config.assumed_frame_interval_s = 0.4;
  config.refined_timeout_s = 1.5;
  const peach_arm::FrameRateTimeouts timeouts(config);
  // 下限 min(2, cap)=1.5：3×0.4+2=3.2 夹到 cap。
  EXPECT_NEAR(timeouts.refinedWaitS(false), 1.5, 1e-12);
}

TEST(FrameRateTimeouts, UpdateConfigRewritesAllWindows)
{
  auto timeouts = makeDefault();
  peach_arm::FrameRateTimeoutConfig config;
  config.assumed_frame_interval_s = 0.2;
  config.frame_wait_s = 5.0;
  config.target_observation_max_age_config_s = 2.0;
  config.reconfirm_wait_s = 8.0;
  config.refined_timeout_s = 20.0;
  timeouts.updateConfig(config);
  EXPECT_NEAR(timeouts.waitIntervalS(), 0.2, 1e-12);
  EXPECT_NEAR(timeouts.frameWaitS(), 2.0, 1e-12);  // 4×0.2+1=1.8 → floor 2
  EXPECT_NEAR(timeouts.targetMaxAgeS(), 2.0, 1e-12);
  EXPECT_NEAR(timeouts.reconfirmWaitS(), 2.0, 1e-12);  // 4×0.2+1=1.8 → floor 2s
  EXPECT_NEAR(timeouts.refinedWaitS(false), 2.6, 1e-12);  // 3×0.2+2
  EXPECT_NEAR(timeouts.refinedWaitS(true), 20.0, 1e-12);
}
