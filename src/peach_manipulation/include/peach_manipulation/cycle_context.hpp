// 功能：周期上下文。一次 ExecuteTarget/手动周期的全部可变状态（含终局）。
#ifndef PEACH_MANIPULATION__CYCLE_CONTEXT_HPP_
#define PEACH_MANIPULATION__CYCLE_CONTEXT_HPP_

#include <Eigen/Geometry>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <peach_interfaces/msg/pregrasp_verification.hpp>

#include "peach_manipulation/cycle_state.hpp"
#include "peach_manipulation/target_cache.hpp"
#include "peach_manipulation/view_planner.hpp"

namespace peach_manipulation
{

// 周期上下文：一次 ExecuteTarget/手动周期 的全部可变状态。
// 线程规则：action 线程在受理时创建并填充 goal 字段；worker 线程启动后
// 是唯一读写者；跨线程只经 atomics（running/cancel/recovery/pending_outcome/
// execution_enabled/grasp/tool）。周期消亡即整体丢弃——钉残留类 bug 结构性不可能。
struct CycleContext
{
  // goal 身份与模式（创建时一次性写入）
  std::string target_id;        // ExecuteTarget.goal.target_id；手动周期为空
  bool observe_only{false};
  bool pregrasp_only{false};
  bool skip_observation{false};
  bool action_driven{false};
  // 目标/精化快照与观察候选
  std::optional<CachedTarget> target;
  std::optional<CachedRefined> refined;
  std::vector<ViewCandidate> candidates;
  // 接触几何
  Eigen::Isometry3d entry_tip_pose{Eigen::Isometry3d::Identity()};
  double travel_m{0.0};
  // 再确认漂移判定的参考锚点（base 系）：FinalizeAndValidate 出口几何对应的
  // 目标锚点（0.5·(bottom+neck)）；再确认漂移超限重算后更新为最新观测锚点。
  Eigen::Vector3d reference_anchor{Eigen::Vector3d::Zero()};
  // 完成度与终局
  bool pregrasp_verified{false};
  bool sleeve_planned{false};
  bool cut_command_accepted{false};
  bool cut_confirmed{false};
  bool retreat_confirmed{false};
  uint8_t completion_level{0};
  uint32_t failure_code{0};
  peach_interfaces::msg::PregraspVerification pregrasp_msg{};
  std::string contact_transaction_id;
  CycleState terminal_state{CycleState::SUCCEEDED};
  std::string terminal_message;
  std::string failure_reason;
};

}  // namespace peach_manipulation

#endif  // PEACH_MANIPULATION__CYCLE_CONTEXT_HPP_
