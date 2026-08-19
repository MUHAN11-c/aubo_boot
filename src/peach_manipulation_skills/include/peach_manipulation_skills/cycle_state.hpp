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
#ifndef PEACH_MANIPULATION_SKILLS__CYCLE_STATE_HPP_
#define PEACH_MANIPULATION_SKILLS__CYCLE_STATE_HPP_

#include <cstdint>
#include <string>

#include "peach_manipulation_skills/action_contract.hpp"

namespace peach_manipulation_skills
{
// 周期状态枚举：节点内部一律以枚举流转，state_json_ 中的字符串只是发布层投影，
// 终局判定只认枚举，避免字符串拼写漂移导致终态被误分类为 RUNNING。
enum class CycleState
{
  IDLE,
  PLAN_OBSERVATION,
  MOVE_TO_VIEW,
  WAIT_FRAME,
  FINALIZE,
  RECONFIRM,
  MTC_APPROACH_INSERT,
  ACTUATE_TOOL,
  MTC_RETREAT,
  PREVIEW_CONTACT_PLANNING,
  PREVIEW_READY,
  PREVIEW_FAILED,
  PLAN_READY,
  READY_FOR_GRASP,
  SUCCEEDED,
  CANCELED,
  FAILED,
  RECOVERY_REQUIRED
};

// 发布层投影字符串必须与历史状态 JSON 完全一致（dashboard/web 只读消费）。
inline std::string toString(CycleState state)
{
  switch (state) {
    case CycleState::IDLE:
      return "IDLE";
    case CycleState::PLAN_OBSERVATION:
      return "PLAN_OBSERVATION";
    case CycleState::MOVE_TO_VIEW:
      return "MOVE_TO_VIEW";
    case CycleState::WAIT_FRAME:
      return "WAIT_FRAME";
    case CycleState::FINALIZE:
      return "FINALIZE";
    case CycleState::RECONFIRM:
      return "RECONFIRM";
    case CycleState::MTC_APPROACH_INSERT:
      return "MTC_APPROACH_INSERT";
    case CycleState::ACTUATE_TOOL:
      return "ACTUATE_TOOL";
    case CycleState::MTC_RETREAT:
      return "MTC_RETREAT";
    case CycleState::PREVIEW_CONTACT_PLANNING:
      return "PREVIEW_CONTACT_PLANNING";
    case CycleState::PREVIEW_READY:
      return "PREVIEW_READY";
    case CycleState::PREVIEW_FAILED:
      return "PREVIEW_FAILED";
    case CycleState::PLAN_READY:
      return "PLAN_READY";
    case CycleState::READY_FOR_GRASP:
      return "READY_FOR_GRASP";
    case CycleState::SUCCEEDED:
      return "SUCCEEDED";
    case CycleState::CANCELED:
      return "CANCELED";
    case CycleState::FAILED:
      return "FAILED";
    case CycleState::RECOVERY_REQUIRED:
      return "RECOVERY_REQUIRED";
  }
  return "UNKNOWN";
}

// 终局分类：PLAN_READY 与 READY_FOR_GRASP 分别是只规划（plan-only）与
// grasp.enabled=false 两档的圆满终态，必须映射为 SUCCEEDED；其余非终态为 RUNNING。
inline CycleOutcome terminalOutcome(CycleState state)
{
  switch (state) {
    case CycleState::SUCCEEDED:
    case CycleState::PREVIEW_READY:
    case CycleState::PLAN_READY:
    case CycleState::READY_FOR_GRASP:
      return CycleOutcome::SUCCEEDED;
    case CycleState::CANCELED:
      return CycleOutcome::CANCELED;
    case CycleState::FAILED:
    case CycleState::PREVIEW_FAILED:
      return CycleOutcome::FAILED;
    case CycleState::RECOVERY_REQUIRED:
      return CycleOutcome::RECOVERY_REQUIRED;
    default:
      return CycleOutcome::RUNNING;
  }
}

// 周期终局结果：executeAction 据此组装 action Result，不再回读状态字符串。
struct CycleResult
{
  CycleOutcome outcome{CycleOutcome::RUNNING};
  std::string reason;
  bool recovery_required{false};
};

// A13：CycleState → HarvestState.target_phase 投影（ExecuteTarget 反馈携带，
// 编排器据此驱动批次过程线的目标阶段）。取值即 peach_interfaces/HarvestState.msg
// 的 TARGET_* 常量（cycle_action.cpp 有 static_assert 双向钉死，防枚举漂移）。
// 语义约定：质量门在 FINALIZE 内完成（FINALIZING 含验证）；RECONFIRM（阶段 E1
// 抓取前再确认，2.7-RECONFIRM）映射 VALIDATING——它是 finalize 之后、接触段之前
// 的最后一道验证关；
// plan-only 圆满终态（PLAN_READY/READY_FOR_GRASP/PREVIEW_READY）映射 COMPLETING
// （周期收尾、结果即出），CANCELED 映射 IDLE（编排器记账后同回 IDLE，无取消相）。
constexpr uint8_t targetPhase(CycleState state)
{
  switch (state) {
    case CycleState::PLAN_OBSERVATION:
    case CycleState::MOVE_TO_VIEW:
    case CycleState::WAIT_FRAME:
      return 2;  // OBSERVING
    case CycleState::FINALIZE:
      return 3;  // FINALIZING（含质量门验证）
    case CycleState::RECONFIRM:
      return 4;  // VALIDATING（抓取前再确认：新鲜观测+锚点漂移门）
    case CycleState::MTC_APPROACH_INSERT:
    case CycleState::PREVIEW_CONTACT_PLANNING:
      return 5;  // APPROACHING
    case CycleState::ACTUATE_TOOL:
      return 6;  // TOOL_ACTION
    case CycleState::MTC_RETREAT:
      return 7;  // RETREATING
    case CycleState::PLAN_READY:
    case CycleState::READY_FOR_GRASP:
    case CycleState::PREVIEW_READY:
      return 8;  // COMPLETING（plan-only 圆满收尾）
    case CycleState::SUCCEEDED:
      return 9;  // TARGET_SUCCEEDED
    case CycleState::FAILED:
    case CycleState::PREVIEW_FAILED:
    case CycleState::RECOVERY_REQUIRED:
      return 11;  // TARGET_FAILED
    case CycleState::IDLE:
    case CycleState::CANCELED:
    default:
      return 0;  // TARGET_IDLE
  }
}
}  // namespace peach_manipulation_skills
#endif  // PEACH_MANIPULATION_SKILLS__CYCLE_STATE_HPP_
