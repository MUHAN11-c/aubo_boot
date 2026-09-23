// 批次5：套入/撤退实测行程判据纯核单测。
#include <gtest/gtest.h>

#include "peach_arm/insert_progress.hpp"

using peach_arm::InsertProgress;
using peach_arm::insertProgressDecision;

TEST(InsertProgress, DoneOnMeasuredTarget)
{
  EXPECT_EQ(
    insertProgressDecision(0.08, 0.08, 0.005, 0.0, 3.0, 0.001, 10.0),
    InsertProgress::DONE);
  EXPECT_EQ(
    insertProgressDecision(0.076, 0.08, 0.005, 0.0, 3.0, 0.001, 10.0),
    InsertProgress::DONE);  // 容差内即到位
}

TEST(InsertProgress, ContinueWhileGaining)
{
  EXPECT_EQ(
    insertProgressDecision(0.03, 0.08, 0.005, 1.0, 3.0, 0.001, 10.0),
    InsertProgress::CONTINUE);
}

TEST(InsertProgress, StallAfterWindowWithoutGain)
{
  EXPECT_EQ(
    insertProgressDecision(0.03, 0.08, 0.005, 3.0, 3.0, 0.001, 10.0),
    InsertProgress::STALLED);
  // 停滞窗未满仍继续
  EXPECT_EQ(
    insertProgressDecision(0.03, 0.08, 0.005, 2.9, 3.0, 0.001, 10.0),
    InsertProgress::CONTINUE);
}

TEST(InsertProgress, DeadlineIsBoundNotProof)
{
  // 未到位且时间尽 → DEADLINE（收口 UNKNOWN，不得当完成）
  EXPECT_EQ(
    insertProgressDecision(0.03, 0.08, 0.005, 1.0, 3.0, 0.001, 0.0),
    InsertProgress::DEADLINE);
  EXPECT_EQ(
    insertProgressDecision(0.03, 0.08, 0.005, 1.0, 3.0, 0.001, -1.0),
    InsertProgress::DEADLINE);
}

TEST(InsertProgress, IncreasingErrorNeverDone)
{
  // 反例：行程更小绝不得判 DONE
  EXPECT_NE(
    insertProgressDecision(0.02, 0.08, 0.005, 0.0, 3.0, 0.001, 10.0),
    InsertProgress::DONE);
}
