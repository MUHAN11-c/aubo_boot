// Copyright 2026, aubo_e5_ros2_ws authors
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

#include <gtest/gtest.h>

#include <string>

#include "peach_approach_grasp/safety_gate.hpp"

namespace peach_approach_grasp
{
namespace
{
// 注入可控假时钟的样本工厂：默认已收到、时刻 100s、四标志全部可运动。
RobotStatusSample makeReadySample()
{
  RobotStatusSample sample;
  sample.received = true;
  sample.received_s = 100.0;
  sample.e_stopped = false;
  sample.in_error = false;
  sample.drives_powered = true;
  sample.motion_possible = true;
  return sample;
}
}  // namespace

TEST(SafetyGate, RobotReadyFreshnessAndFlagCombinations)
{
  double now_s = 100.5;
  SafetyGate gate(SafetyGateConfig{}, [&now_s]() {return now_s;});
  std::string reason;

  // 从未收到：missing
  RobotStatusSample sample;
  EXPECT_FALSE(gate.robotReady(sample, reason));
  EXPECT_EQ(reason, "robot_status_missing");

  // 新鲜且标志全好：放行
  sample = makeReadySample();
  EXPECT_TRUE(gate.robotReady(sample, reason));

  // 超龄（默认 max_age=1.0s，age 1.2s）：stale
  now_s = 101.2;
  EXPECT_FALSE(gate.robotReady(sample, reason));
  EXPECT_EQ(reason, "robot_status_stale");

  // 急停/断电/不可运动：not_motion_ready
  now_s = 100.5;
  sample.e_stopped = true;
  EXPECT_FALSE(gate.robotReady(sample, reason));
  EXPECT_EQ(reason, "robot_status_not_motion_ready");
  sample = makeReadySample();
  sample.drives_powered = false;
  EXPECT_FALSE(gate.robotReady(sample, reason));
  EXPECT_EQ(reason, "robot_status_not_motion_ready");
  sample = makeReadySample();
  sample.motion_possible = false;
  EXPECT_FALSE(gate.robotReady(sample, reason));
  EXPECT_EQ(reason, "robot_status_not_motion_ready");
}

TEST(SafetyGate, RequireRobotStatusDisabledBypassesEverything)
{
  double now_s = 0.0;
  SafetyGateConfig config;
  config.require_robot_status = false;
  SafetyGate gate(config, [&now_s]() {return now_s;});
  std::string reason = "untouched";
  // 未收到且急停置位也放行
  RobotStatusSample sample;
  sample.e_stopped = true;
  EXPECT_TRUE(gate.robotReady(sample, reason));
  EXPECT_EQ(reason, "untouched");
}

TEST(SafetyGate, TargetReadyIdentityValidityFreshness)
{
  double now_s = 100.5;
  // 显式小阈值：本测试验证新鲜度判定逻辑，与默认值（0.78FPS 相机取 3.0s）解耦。
  SafetyGateConfig config;
  config.target_observation_max_age_s = 1.0;
  SafetyGate gate(config, [&now_s]() {return now_s;});
  std::string reason;

  TargetGateSample sample;
  sample.id = "peach_2";
  sample.valid = true;
  sample.received_s = 100.0;
  // 身份不一致：changed（先于有效性/新鲜度判定）
  EXPECT_FALSE(gate.targetReady(sample, "peach_1", reason));
  EXPECT_EQ(reason, "selected_target_changed");

  // 同 id 但观测无效：not_observed
  sample.id = "peach_1";
  sample.valid = false;
  EXPECT_FALSE(gate.targetReady(sample, "peach_1", reason));
  EXPECT_EQ(reason, "selected_target_not_observed");

  // 有效但超龄：stale
  sample.valid = true;
  now_s = 101.2;
  EXPECT_FALSE(gate.targetReady(sample, "peach_1", reason));
  EXPECT_EQ(reason, "selected_target_stale");

  // 有效且新鲜：放行
  now_s = 100.5;
  EXPECT_TRUE(gate.targetReady(sample, "peach_1", reason));
}

}  // namespace peach_approach_grasp
