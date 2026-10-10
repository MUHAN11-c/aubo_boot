// 功能：周期状态枚举与一次 ExecuteTarget/手动周期的全部可变状态（含终局）。
#ifndef PEACH_MANIPULATION__CYCLE_CONTEXT_HPP_
#define PEACH_MANIPULATION__CYCLE_CONTEXT_HPP_

#include <Eigen/Geometry>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <peach_interfaces/msg/pregrasp_verification.hpp>
#include <rclcpp/rclcpp.hpp>

#include "peach_arm/target_cache.hpp"
#include "peach_arm/view_planner.hpp"

namespace peach_arm
{
/// action 终局分类。终局判定只走枚举，不再从状态字符串反推。
enum class CycleOutcome
{
  RUNNING,             ///< 周期仍在走。
  SUCCEEDED,           ///< 圆满（含 plan-only / PREVIEW_READY）。
  CANCELED,            ///< 被取消。
  FAILED,              ///< 失败（含 PREVIEW_FAILED）。
  RECOVERY_REQUIRED    ///< 接触故障，须人确认后 ACK。
};

/// 周期状态：节点内部一律以枚举流转；state_json_ 字符串只是发布层投影。
enum class CycleState
{
  IDLE,                      ///< 无周期。
  PLAN_OBSERVATION,          ///< 规划观察视点。
  MOVE_TO_VIEW,              ///< 移到观察位。
  WAIT_FRAME,                ///< 等待新鲜观测帧。
  FINALIZE,                  ///< 收口几何与质量门。
  RECONFIRM,                 ///< 抓取前再确认（新鲜观测+漂移门）。
  MTC_APPROACH_INSERT,       ///< 接近并套入。
  ACTUATE_TOOL,              ///< 刀具 IO（剪切）。
  MTC_RETREAT,               ///< 沿轴撤退。
  PREVIEW_CONTACT_PLANNING,  ///< 预览接触规划中。
  PREVIEW_READY,             ///< 预览规划成功（圆满终态）。
  PREVIEW_FAILED,            ///< 预览规划失败。
  PLAN_READY,                ///< plan-only 圆满（未接触）。
  READY_FOR_GRASP,           ///< grasp.enabled=false 时的圆满停点。
  SUCCEEDED,                 ///< 接触周期成功。
  CANCELED,                  ///< 取消。
  FAILED,                    ///< 失败。
  RECOVERY_REQUIRED          ///< 须人确认现场。
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

/// 周期终局结果：executeAction 据此组装 action Result，不再回读状态字符串。
struct CycleResult
{
  CycleOutcome outcome{CycleOutcome::RUNNING};  ///< 终局分类。
  std::string reason;                           ///< 失败/取消原因短句。
  bool recovery_required{false};                ///< True=须 ACK 才能再派。
};

// A13：CycleState → HarvestState.target_phase 投影（ExecuteTarget 反馈携带，
// 编排器据此驱动批次过程线的目标阶段）。取值即 peach_interfaces/HarvestState.msg
// 的 TARGET_* 常量（cycle.cpp 有 static_assert 双向钉死，防枚举漂移）。
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

/// 一次 ExecuteTarget / 手动周期的全部可变状态。
/// 线程：action 受理时创建；worker 启动后是唯一读写者；跨线程只经 atomics。
struct CycleContext
{
  /// ExecuteTarget.goal.target_id；手动周期为空。
  std::string target_id;
  bool observe_only{false};     ///< True=只观察/补视，不接触。
  bool pregrasp_only{false};    ///< True=到预抓取验证后停，不 SetIO。
  bool skip_observation{false}; ///< True=跳过观察段，直接用已有几何。
  bool action_driven{false};    ///< True=来自 ExecuteTarget，非手动预览。
  /// goal.clearance 填写即 present；CONTACT/TOOL 优先走令牌，否则回退 GraspDecision 话题。
  bool clearance_present{false};   ///< goal 是否携带接触许可令牌。
  bool clearance_allowed{false};   ///< 令牌 allowed（只授权套入/剪切）。
  std::string clearance_target_id; ///< 令牌绑定的 target_id。
  rclcpp::Time clearance_valid_until{0, 0, RCL_ROS_TIME};  ///< 过期后不得执行。
  rclcpp::Time clearance_model_stamp{0, 0, RCL_ROS_TIME};  ///< 模型新鲜度戳。
  double clearance_fresh_window_s{0.0};  ///< >0 时启用 stamp 新鲜度复检 [s]。
  double clearance_radial_margin_m{0.0};  ///< 令牌径向余量（批次4：CONTACT 档位门）。
  double clearance_axial_margin_m{0.0};  ///< 令牌轴向余量（批次4：TOOL 档位门）。
  std::optional<CachedTarget> target;    ///< 感知初始几何快照。
  std::optional<CachedRefined> refined;  ///< 重建精化几何快照。
  std::vector<ViewCandidate> candidates; ///< 本周期观察视点队列。
  Eigen::Isometry3d entry_tip_pose{Eigen::Isometry3d::Identity()};  ///< 袋外入口 TCP（base 系）。
  double travel_m{0.0};  ///< 建议插入行程 [m]。
  /// 再确认漂移锚点（base 系）：Finalize 出口 0.5·(bottom+neck)。
  Eigen::Vector3d reference_anchor{Eigen::Vector3d::Zero()};
  bool pregrasp_verified{false};     ///< 预抓取验证通过。
  bool sleeve_planned{false};        ///< 套入轨迹已规划。
  bool sleeve_partial{false};        ///< 套入未走完（部分行程）。
  bool cut_command_accepted{false};  ///< SetIO 已 ACK（≠切断确认）。
  bool cut_confirmed{false};         ///< 剪切确认（现行预留）。
  bool retreat_confirmed{false};     ///< 撤退到位。
  bool imu_follow_session{false};    ///< 自适应接触窗内已 ~/enable，终局须 disable。
  uint8_t completion_level{0};       ///< ExecuteTarget 完成度档。
  uint32_t failure_code{0};          ///< FailureCode.*；0=无失败。
  peach_interfaces::msg::PregraspVerification pregrasp_msg{};  ///< 预抓取验证消息。
  std::string contact_transaction_id;  ///< 接触事务 ID（账本对账）。
  CycleState terminal_state{CycleState::SUCCEEDED};  ///< 计划圆满态（plan-only 可能是 PLAN_READY）。
  std::string terminal_message;  ///< 终局人读短句。
  std::string failure_reason;    ///< 失败原因（稳定短语）。
};

}  // namespace peach_arm

#endif  // PEACH_MANIPULATION__CYCLE_CONTEXT_HPP_
