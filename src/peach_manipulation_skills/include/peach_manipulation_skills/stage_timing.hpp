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
#ifndef PEACH_MANIPULATION_SKILLS__STAGE_TIMING_HPP_
#define PEACH_MANIPULATION_SKILLS__STAGE_TIMING_HPP_

#include <array>
#include <chrono>
#include <string>
#include <vector>

#include "peach_manipulation_skills/cycle_state.hpp"

namespace peach_manipulation_skills
{
// 周期阶段耗时埋点（重构阶段 C）纯核：CycleState → 阶段名投影与墙钟累计。
// 阶段名集合与 ExecuteTarget.action Result 的 stage_names 契约一一对应
// （test_action_contract 钉死）；名字一律小写串，与 action 注释保持同步。
// prepare：周期受理（onStart/previewContact 放行）到首次进入后续阶段的隐式段；
// reconfirm：抓取前再确认段（阶段 E1，CycleState::RECONFIRM 投影）。
inline constexpr std::array<const char *, 7> kStageNames = {
  "prepare", "observe", "finalize", "reconfirm", "approach_insert", "tool", "retreat"};

// CycleState → 阶段名投影；非阶段态（IDLE/全部终态/plan-only 圆满态）返回
// nullptr，表示"当前不处于任何计时阶段"（StageTimer 据此收口）。
// observe 合并观察段三态（PLAN_OBSERVATION/MOVE_TO_VIEW/WAIT_FRAME）；
// approach_insert 合并 MTC 靠近插入与接触轨迹预览规划（PREVIEW 周期同段计时）。
inline const char * stageForState(CycleState state)
{
  switch (state) {
    case CycleState::PLAN_OBSERVATION:
    case CycleState::MOVE_TO_VIEW:
    case CycleState::WAIT_FRAME:
      return "observe";
    case CycleState::FINALIZE:
      return "finalize";
    case CycleState::RECONFIRM:
      return "reconfirm";
    case CycleState::MTC_APPROACH_INSERT:
    case CycleState::PREVIEW_CONTACT_PLANNING:
      return "approach_insert";
    case CycleState::ACTUATE_TOOL:
      return "tool";
    case CycleState::MTC_RETREAT:
      return "retreat";
    default:
      return nullptr;
  }
}

// 单个阶段的累计耗时（插入序排列，同名阶段多次进入累计到同一桶）。
struct StageDuration
{
  std::string name;
  std::chrono::duration<double> elapsed{0.0};
};

// 阶段墙钟计时器：steady_clock 时间点由调用方注入（节点侧在 setState 的
// cycle_state_ 变更点喂入；测试用合成时间点）。start 进入 prepare 段，
// 每次 onStateChange 把上一段累计进对应桶并切换当前段；投影为 nullptr 的
// 状态（终态/IDLE）使计时收口，此后一切变更被忽略。未 start 前全部 no-op。
// 非线程安全：节点侧一律在 state_mutex_ 内访问。
class StageTimer
{
public:
  using Clock = std::chrono::steady_clock;

  // 开始一个新周期：清空上一周期记录，当前段置为 prepare。
  void start(Clock::time_point now)
  {
    entries_.clear();
    started_ = true;
    closed_ = false;
    last_ = now;
    enterStage("prepare");
  }

  // 状态切换点：累计上一段耗时，切到 state 投影的新段；nullptr 投影即收口。
  void onStateChange(CycleState state, Clock::time_point now)
  {
    if (!started_ || closed_) {
      return;
    }
    accrue(now);
    const char * stage = stageForState(state);
    if (stage == nullptr) {
      closed_ = true;
      current_ = nullptr;
      return;
    }
    enterStage(stage);
  }

  // 显式收口（终局组装 result 时兜底）：把当前段累计完成后关闭计时。
  void close(Clock::time_point now)
  {
    if (!started_ || closed_) {
      return;
    }
    accrue(now);
    closed_ = true;
    current_ = nullptr;
  }

  bool active() const {return started_ && !closed_;}

  // 已历经阶段（含 prepare），按首次进入顺序；未经历的阶段不出现。
  const std::vector<StageDuration> & entries() const {return entries_;}

private:
  void enterStage(const char * stage)
  {
    current_ = stage;
    for (auto & entry : entries_) {
      if (entry.name == stage) {
        return;
      }
    }
    entries_.push_back(StageDuration{stage, std::chrono::duration<double>{0.0}});
  }

  void accrue(Clock::time_point now)
  {
    if (current_ == nullptr || now <= last_) {
      last_ = now > last_ ? now : last_;
      return;
    }
    for (auto & entry : entries_) {
      if (entry.name == current_) {
        entry.elapsed += now - last_;
        break;
      }
    }
    last_ = now;
  }

  bool started_{false};
  bool closed_{false};
  const char * current_{nullptr};
  Clock::time_point last_{};
  std::vector<StageDuration> entries_;
};

}  // namespace peach_manipulation_skills
#endif  // PEACH_MANIPULATION_SKILLS__STAGE_TIMING_HPP_
