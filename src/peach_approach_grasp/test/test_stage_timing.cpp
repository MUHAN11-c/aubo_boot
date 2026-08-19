// Copyright 2026, aubo_e5_ros2_ws authors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright notice,
//      this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "peach_approach_grasp/stage_timing.hpp"
#include "scoped_timer.hpp"

namespace peach_approach_grasp
{
namespace
{
using Clock = StageTimer::Clock;
constexpr auto t0 = Clock::time_point{};

Clock::time_point at(double seconds)
{
  return t0 + std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(seconds));
}

double elapsedOf(const StageTimer & timer, const std::string & name)
{
  for (const auto & entry : timer.entries()) {
    if (entry.name == name) {
      return entry.elapsed.count();
    }
  }
  return -1.0;
}

std::vector<std::string> namesOf(const StageTimer & timer)
{
  std::vector<std::string> names;
  for (const auto & entry : timer.entries()) {
    names.push_back(entry.name);
  }
  return names;
}
}  // namespace

TEST(StageTiming, FullCycleAccumulatesEveryStageInOrder)
{
  // 完整周期：prepare→observe（跨多个视点状态连续累计）→finalize→reconfirm
  // （抓取前再确认，阶段 E1）→approach_insert→tool→retreat→成功终态收口。
  StageTimer timer;
  timer.start(at(0.0));
  timer.onStateChange(CycleState::PLAN_OBSERVATION, at(1.0));
  timer.onStateChange(CycleState::MOVE_TO_VIEW, at(2.5));
  timer.onStateChange(CycleState::WAIT_FRAME, at(4.0));
  timer.onStateChange(CycleState::FINALIZE, at(5.0));
  timer.onStateChange(CycleState::RECONFIRM, at(6.0));
  timer.onStateChange(CycleState::MTC_APPROACH_INSERT, at(7.0));
  timer.onStateChange(CycleState::ACTUATE_TOOL, at(10.0));
  timer.onStateChange(CycleState::MTC_RETREAT, at(10.5));
  timer.onStateChange(CycleState::SUCCEEDED, at(12.5));

  EXPECT_EQ(
    namesOf(timer),
    (std::vector<std::string>{
      "prepare", "observe", "finalize", "reconfirm", "approach_insert", "tool",
      "retreat"}));
  EXPECT_DOUBLE_EQ(elapsedOf(timer, "prepare"), 1.0);
  EXPECT_DOUBLE_EQ(elapsedOf(timer, "observe"), 4.0);
  EXPECT_DOUBLE_EQ(elapsedOf(timer, "finalize"), 1.0);
  EXPECT_DOUBLE_EQ(elapsedOf(timer, "reconfirm"), 1.0);
  EXPECT_DOUBLE_EQ(elapsedOf(timer, "approach_insert"), 3.0);
  EXPECT_DOUBLE_EQ(elapsedOf(timer, "tool"), 0.5);
  EXPECT_DOUBLE_EQ(elapsedOf(timer, "retreat"), 2.0);
  EXPECT_FALSE(timer.active());
}

TEST(StageTiming, SameStageReentryAccumulatesIntoOneBucket)
{
  // 同名阶段多次进入（finalize 内 degrade 二次置位、预览规划等）累计到同一桶，
  // 且插入序保持首次进入位置。
  StageTimer timer;
  timer.start(at(0.0));
  timer.onStateChange(CycleState::FINALIZE, at(1.0));
  timer.onStateChange(CycleState::MTC_APPROACH_INSERT, at(2.0));
  timer.onStateChange(CycleState::PREVIEW_CONTACT_PLANNING, at(3.0));
  timer.onStateChange(CycleState::FAILED, at(5.0));

  EXPECT_EQ(
    namesOf(timer),
    (std::vector<std::string>{"prepare", "finalize", "approach_insert"}));
  // approach_insert = MTC_APPROACH_INSERT 段 1s + PREVIEW_CONTACT_PLANNING 段 2s。
  EXPECT_DOUBLE_EQ(elapsedOf(timer, "approach_insert"), 3.0);
}

TEST(StageTiming, CancelPathFillsOnlyVisitedStages)
{
  // 取消路径：observe 段中取消，未经历的 finalize/approach_insert 等不出现。
  StageTimer timer;
  timer.start(at(0.0));
  timer.onStateChange(CycleState::MOVE_TO_VIEW, at(0.5));
  timer.onStateChange(CycleState::CANCELED, at(2.0));

  EXPECT_EQ(namesOf(timer), (std::vector<std::string>{"prepare", "observe"}));
  EXPECT_DOUBLE_EQ(elapsedOf(timer, "prepare"), 0.5);
  EXPECT_DOUBLE_EQ(elapsedOf(timer, "observe"), 1.5);
  EXPECT_DOUBLE_EQ(elapsedOf(timer, "finalize"), -1.0);
}

TEST(StageTiming, CloseIsIdempotentAndIgnoresLaterTransitions)
{
  // 终态收口后的一切状态变更被忽略（runCycle finish 与 deactivate 的二次置位
  // 不得污染已收口耗时）；close 幂等。
  StageTimer timer;
  timer.start(at(0.0));
  timer.onStateChange(CycleState::WAIT_FRAME, at(1.0));
  timer.onStateChange(CycleState::FAILED, at(3.0));
  timer.onStateChange(CycleState::IDLE, at(10.0));
  timer.close(at(20.0));

  EXPECT_EQ(namesOf(timer), (std::vector<std::string>{"prepare", "observe"}));
  EXPECT_DOUBLE_EQ(elapsedOf(timer, "observe"), 2.0);
}

TEST(StageTiming, NotStartedIsNoOp)
{
  // 周期未启动（configure 的 setState(IDLE) 等）一切调用为 no-op，条目为空。
  StageTimer timer;
  timer.onStateChange(CycleState::FINALIZE, at(1.0));
  timer.close(at(2.0));
  EXPECT_TRUE(timer.entries().empty());
  EXPECT_FALSE(timer.active());
}

TEST(StageTiming, StageNameProjectionOnlyEmitsLegalNames)
{
  // 投影输出的每个名字都必须落在 action 契约的合法集合内（含 reconfirm 段）。
  const std::vector<std::string> legal(kStageNames.begin(), kStageNames.end());
  EXPECT_EQ(legal.size(), 7u);
  EXPECT_NE(std::find(legal.begin(), legal.end(), "reconfirm"), legal.end());
  const CycleState states[] = {
    CycleState::IDLE, CycleState::PLAN_OBSERVATION, CycleState::MOVE_TO_VIEW,
    CycleState::WAIT_FRAME, CycleState::FINALIZE, CycleState::RECONFIRM,
    CycleState::MTC_APPROACH_INSERT, CycleState::ACTUATE_TOOL, CycleState::MTC_RETREAT,
    CycleState::PREVIEW_CONTACT_PLANNING, CycleState::PREVIEW_READY,
    CycleState::PREVIEW_FAILED, CycleState::PLAN_READY, CycleState::READY_FOR_GRASP,
    CycleState::SUCCEEDED, CycleState::CANCELED, CycleState::FAILED,
    CycleState::RECOVERY_REQUIRED};
  for (const auto state : states) {
    const char * stage = stageForState(state);
    if (stage == nullptr) {
      continue;
    }
    EXPECT_NE(
      std::find(legal.begin(), legal.end(), std::string(stage)), legal.end())
      << "stageForState 输出非法阶段名: " << stage;
  }
  // 映射抽查：观察段三态合一，预览规划并入 approach_insert，再确认独立成段。
  EXPECT_STREQ(stageForState(CycleState::WAIT_FRAME), "observe");
  EXPECT_STREQ(
    stageForState(CycleState::PREVIEW_CONTACT_PLANNING), "approach_insert");
  EXPECT_STREQ(stageForState(CycleState::RECONFIRM), "reconfirm");
  EXPECT_EQ(stageForState(CycleState::SUCCEEDED), nullptr);
  EXPECT_EQ(stageForState(CycleState::IDLE), nullptr);
}

TEST(StageTiming, CallbackTimingRegistryAccumulates)
{
  // ScopedTimer 析构即按标签累计；count/total/max/last 语义钉死。
  CallbackTimingRegistry registry;
  {
    ScopedTimer timer(rclcpp::get_logger("test"), "unit_cb", &registry, 0.0);
    ASSERT_GE(timer.elapsedMs(), 0.0);
  }
  {
    ScopedTimer timer(rclcpp::get_logger("test"), "unit_cb", &registry, 0.0);
  }
  const auto snapshot = registry.snapshot();
  ASSERT_EQ(snapshot.count("unit_cb"), 1u);
  const auto & entry = snapshot.at("unit_cb");
  EXPECT_EQ(entry.count, 2u);
  EXPECT_GE(entry.total_ms, entry.max_ms);
  EXPECT_GE(entry.max_ms, 0.0);
  // JSON 投影包含四个统计键（publishState 随 ~/status 发布）。
  const auto json = registry.toJson();
  ASSERT_TRUE(json.contains("unit_cb"));
  EXPECT_EQ(json["unit_cb"].at("count").get<uint64_t>(), 2u);
  EXPECT_TRUE(json["unit_cb"].contains("total_ms"));
  EXPECT_TRUE(json["unit_cb"].contains("max_ms"));
  EXPECT_TRUE(json["unit_cb"].contains("last_ms"));
}

}  // namespace peach_approach_grasp
