#include "peach_arm/staging_candidates.hpp"
#include <gtest/gtest.h>

TEST(StagingCandidates, PreferSmallRollBeforeJointCost)
{
  std::vector<peach_arm::StagingCandidate> values{
    {{}, 0.5, 1.0}, {{}, 0.0, 20.0}, {{}, -0.5, 0.5}};
  peach_arm::sortStagingCandidates(values);
  EXPECT_DOUBLE_EQ(values[0].roll_rad, 0.0);
  EXPECT_DOUBLE_EQ(values[1].roll_rad, -0.5);
}

TEST(StagingCandidates, GuardFailureCanTryNextWithoutMotion)
{
  EXPECT_TRUE(peach_arm::retryStagingCandidate(false, false, false, false));
  EXPECT_FALSE(peach_arm::retryStagingCandidate(true, false, false, false));
  EXPECT_FALSE(peach_arm::retryStagingCandidate(false, true, false, false));
  EXPECT_FALSE(peach_arm::retryStagingCandidate(false, false, true, false));
  EXPECT_FALSE(peach_arm::retryStagingCandidate(false, false, false, true));
}

TEST(StagingCandidates, AbandonRolledLinAfterTwoPlanFailures)
{
  EXPECT_FALSE(peach_arm::abandonRolledLinSearch(0));
  EXPECT_FALSE(peach_arm::abandonRolledLinSearch(1));
  EXPECT_TRUE(peach_arm::abandonRolledLinSearch(2));
  EXPECT_TRUE(peach_arm::abandonRolledLinSearch(3));
}

TEST(StagingCandidates, EqualMagnitudeChoosesJointCostWithoutLosingOtherRolls)
{
  std::vector<peach_arm::StagingCandidate> values{
    {{}, 0.25, 4.0}, {{}, -0.25, 2.0}, {{}, 0.5, 0.1}, {{}, 0.0, 10.0}};
  peach_arm::sortStagingCandidates(values);
  ASSERT_EQ(values.size(), 4U);
  EXPECT_DOUBLE_EQ(values[0].roll_rad, 0.0);
  EXPECT_DOUBLE_EQ(values[1].roll_rad, -0.25);
  EXPECT_DOUBLE_EQ(values[2].roll_rad, 0.25);
  EXPECT_DOUBLE_EQ(values[3].roll_rad, 0.5);
}
