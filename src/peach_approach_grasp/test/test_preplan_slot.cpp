// Copyright 2026, aubo_e5_jazzy_ws authors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
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
//
// MTC 预规划槽（2.13-E3 plan-while-waiting）纯核用例：复用判定（漂移≤阈值
// 复用/超限重规划）、取消丢弃（discard 后迟到 complete 按 stale 忽略）、
// waitReady 落定/超时/取消语义。
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <string>
#include <thread>

#include "peach_approach_grasp/preplan_slot.hpp"

namespace
{
using peach_approach_grasp::PreplanSlot;

const Eigen::Vector3d kAnchor(0.30, 0.10, 0.25);

TEST(PreplanSlot, ReuseWhenDriftWithinThreshold)
{
  // 复用主路径：READY 且漂移 ≤ 阈值（grasp.replan_threshold_m=0.02）。
  PreplanSlot slot;
  ASSERT_TRUE(slot.begin(kAnchor));
  slot.complete(true, "MTC plan ready");
  std::string why;
  EXPECT_TRUE(slot.reusable(kAnchor + Eigen::Vector3d(0.01, 0.0, 0.0), 0.02, why));
  EXPECT_TRUE(slot.reusable(kAnchor, 0.02, why));
}

TEST(PreplanSlot, ReplanWhenDriftExceedsThreshold)
{
  // 漂移超限 → 不复用（调用方内联重规划），原因含漂移量。
  PreplanSlot slot;
  ASSERT_TRUE(slot.begin(kAnchor));
  slot.complete(true, "MTC plan ready");
  std::string why;
  EXPECT_FALSE(slot.reusable(kAnchor + Eigen::Vector3d(0.03, 0.0, 0.0), 0.02, why));
  EXPECT_NE(why.find("重规划"), std::string::npos);
}

TEST(PreplanSlot, NotReusableBeforeReadyOrAfterFailure)
{
  PreplanSlot slot;
  std::string why;
  // 空槽（未启动）。
  EXPECT_FALSE(slot.reusable(kAnchor, 0.02, why));
  // 规划中。
  ASSERT_TRUE(slot.begin(kAnchor));
  EXPECT_FALSE(slot.reusable(kAnchor, 0.02, why));
  EXPECT_EQ(slot.state(), PreplanSlot::State::PLANNING);
  // 规划失败。
  slot.complete(false, "MTC planning failed");
  EXPECT_EQ(slot.state(), PreplanSlot::State::FAILED);
  EXPECT_FALSE(slot.reusable(kAnchor, 0.02, why));
  EXPECT_NE(why.find("预规划失败"), std::string::npos);
}

TEST(PreplanSlot, BeginRejectedWhileBusy)
{
  // PLANNING/READY 状态重复 begin 拒绝（调用方须先 discard）；FAILED/EMPTY
  // 允许重新 begin（失败后可按新几何再启动）。
  PreplanSlot slot;
  ASSERT_TRUE(slot.begin(kAnchor));
  EXPECT_FALSE(slot.begin(kAnchor));
  slot.complete(true, "ok");
  EXPECT_FALSE(slot.begin(kAnchor));
  slot.discard();
  EXPECT_TRUE(slot.begin(kAnchor));
}

TEST(PreplanSlot, DiscardDropsAndIgnoresLateComplete)
{
  // 取消/漂移重算丢弃：discard 后槽空不可复用；迟到的 complete（预规划线程
  // 在 discard 之后才落定）按 stale 忽略，不得复活槽。
  PreplanSlot slot;
  ASSERT_TRUE(slot.begin(kAnchor));
  slot.discard();
  EXPECT_EQ(slot.state(), PreplanSlot::State::EMPTY);
  std::string why;
  EXPECT_FALSE(slot.reusable(kAnchor, 0.02, why));
  slot.complete(true, "late result");
  EXPECT_EQ(slot.state(), PreplanSlot::State::EMPTY);
  EXPECT_FALSE(slot.reusable(kAnchor, 0.02, why));
}

TEST(PreplanSlot, WaitReadySemantics)
{
  std::atomic_bool cancel{false};
  PreplanSlot slot;
  // 空槽立即返回（非 PLANNING）。
  EXPECT_TRUE(slot.waitReady(0.01, cancel));
  // 规划中：后台线程 100ms 后落定 → waitReady 等到 READY。
  ASSERT_TRUE(slot.begin(kAnchor));
  std::thread finisher([&slot]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      slot.complete(true, "ok");
    });
  EXPECT_TRUE(slot.waitReady(5.0, cancel));
  EXPECT_EQ(slot.state(), PreplanSlot::State::READY);
  finisher.join();
}

TEST(PreplanSlot, WaitReadyTimeoutAndCancel)
{
  PreplanSlot slot;
  std::atomic_bool cancel{false};
  ASSERT_TRUE(slot.begin(kAnchor));
  // 超时：50ms 内无落定 → false，状态保持 PLANNING。
  EXPECT_FALSE(slot.waitReady(0.05, cancel));
  EXPECT_EQ(slot.state(), PreplanSlot::State::PLANNING);
  // 取消：置位即返回 false。
  cancel.store(true);
  EXPECT_FALSE(slot.waitReady(5.0, cancel));
  slot.complete(true, "ok");  // 测试收尾，防析构竞态（槽无主线程，仅为语义完整）
}

}  // namespace
