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
// 行为树节点体与注册：12 个 BT 节点、候选视点 marker 发布、重建帧/精化等待薄壳。
#include <algorithm>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "approach_grasp_node_impl.hpp"
#include "peach_manipulation_skills/grasp_geometry.hpp"
#include "peach_manipulation_skills/protected_zones.hpp"
#include "peach_manipulation_skills/reconfirm_policy.hpp"
#include <tf2_eigen/tf2_eigen.hpp>

namespace peach_manipulation_skills
{
namespace
{
// 跟踪状态枚举 → 中文标签（再确认失败原因文案用；常量为
// PeachTargetObservation.msg 的 tracking_status 枚举，255=缓存未知）。
std::string trackingStatusLabel(uint8_t status)
{
  using Observation = peach_interfaces::msg::PeachTargetObservation;
  switch (status) {
    case Observation::OBSERVED:
      return "OBSERVED";
    case Observation::OCCLUDED:
      return "OCCLUDED(遮挡)";
    case Observation::LOST:
      return "LOST(跟踪丢失)";
    case Observation::INVALID:
      return "INVALID";
    case Observation::OUT_OF_VIEW:
      return "OUT_OF_VIEW(出视野)";
    case Observation::DEPTH_VOID:
      return "DEPTH_VOID(深度空洞)";
    default:
      return "UNKNOWN";
  }
}
}  // namespace
// 数据快照薄壳：统一从 cache_ 取一致性快照，供 BT 节点体与运动接口使用。
QualitySnapshot ApproachGraspNode::qualitySnapshot()
{
  return cache_.qualitySnapshot();
}

std::optional<CachedTarget> ApproachGraspNode::targetSnapshot()
{
  return cache_.targetSnapshot();
}

// 周期生效目标快照（阶段 E 残局抬质量能力端）。
// 为什么按周期分流而不是在执行接受时把 goal 目标写进 selected 缓存：
// 覆盖 target_ 会污染感知驱动的四源调和（selected 身份/精化/决策缓存的 ID
// 一致性全部以 target_ 为准），周期结束后还会留下"假 selected"干扰后续
// 周期；而锁定集锚点缓存是独立数据源，goal 钉入 ID（cycle_target_id_）在
// 受理时已写好，执行体按"本周期生效目标"取快照即可，零侵入既有调和语义。
// FULL/PREVIEW/手动周期（cycle_observe_only_=false）恒退化为 selected 缓存。
std::optional<CachedTarget> ApproachGraspNode::cycleTargetSnapshot()
{
  if (!cycle_target_id_.empty()) {
    auto locked = cache_.lockedTargetSnapshot(cycle_target_id_);
    if (locked) {return locked;}
  }
  return cache_.targetSnapshot();
}

std::vector<Eigen::Vector3d> ApproachGraspNode::observedDirectionsSnapshot()
{
  return cache_.observedDirections();
}

std::optional<CachedRefined> ApproachGraspNode::refinedSnapshot()
{
  return cache_.refinedSnapshot();
}

std::string ApproachGraspNode::graspDecisionTargetSnapshot()
{
  return cache_.graspDecisionTarget();
}

bool ApproachGraspNode::waitForNewView(std::size_t previous_views)
{
  return cache_.waitForNewView(previous_views, effectiveFrameWaitS(), cancel_requested_);
}

bool ApproachGraspNode::waitForFreshTarget(double after_s)
{
  return waitForFreshCycleTarget(after_s, effectiveFrameWaitS());
}

bool ApproachGraspNode::waitForFreshCycleTarget(double after_s, double window_s)
{
  // 数据源随周期生效目标走（同 cycleTargetSnapshot 的分流理由）：
  // OBSERVE_ONLY 等 goal 目标的锁定集锚点新鲜帧，其余等 selected 新鲜帧。
  if (!cycle_target_id_.empty()) {
    return cache_.waitForFreshLockedTarget(
      cycle_target_id_, after_s, window_s, cancel_requested_);
  }
  return cache_.waitForFreshTarget(after_s, window_s, cancel_requested_);
}

bool ApproachGraspNode::waitForRefined(const std::string & target_id)
{
  // 超时按协议 2.7-FINALIZE 的 T(refined) 帧率自适应（effectiveRefinedWaitS）。
  return cache_.waitForRefined(target_id, effectiveRefinedWaitS(), cancel_requested_);
}

void ApproachGraspNode::registerBehaviorTreeNodes()
{
  bt_factory_.registerSimpleAction(
    "PrepareCycle", [this](BT::TreeNode &) {return btPrepareCycle();});
  bt_factory_.registerSimpleCondition(
    "IsPlanOnly", [this](BT::TreeNode &) {
      return execution_enabled_.load() ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    });
  bt_factory_.registerSimpleAction(
    "PlanObservationPreview", [this](BT::TreeNode &) {return btPlanPreview();});
  bt_factory_.registerSimpleAction(
    "AcquireReconstructionViews", [this](BT::TreeNode &) {return btAcquireViews();});
  bt_factory_.registerSimpleAction(
    "FinalizeAndValidate", [this](BT::TreeNode &) {return btFinalizeAndValidate();});
  // 抓取前再确认（2.7-RECONFIRM）：FinalizeAndValidate 之后、MTC 接触段之前的
  // 最后一道验证关（新鲜观测+锚点漂移+摆动平息）。
  bt_factory_.registerSimpleAction(
    "ReconfirmTarget", [this](BT::TreeNode &) {return btReconfirmTarget();});
  bt_factory_.registerSimpleCondition(
    "IsGraspDisabled", [this](BT::TreeNode &) {
      return grasp_enabled_.load() ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    });
  // OBSERVE_ONLY 周期短路条件：FinalizeAndValidate 后直接落 PLAN_READY 终态，
  // 跳过 MTC/工具/撤离段（优先级高于 IsGraspDisabled，见 harvest_tree.xml）。
  bt_factory_.registerSimpleCondition(
    "IsObserveOnly", [this](BT::TreeNode &) {
      return cycle_observe_only_.load() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  bt_factory_.registerSimpleCondition(
    "IsSkipObservation", [this](BT::TreeNode &) {
      return cycle_skip_observation_.load() ?
             BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  bt_factory_.registerSimpleAction(
    "ReportObserveOnly", [this](BT::TreeNode &) {return btReportObserveOnly();});
  bt_factory_.registerSimpleAction(
    "ReportReadyForGrasp", [this](BT::TreeNode &) {return btReportReady();});
  bt_factory_.registerSimpleAction(
    "MTCApproachAndInsert", [this](BT::TreeNode &) {return btMtcApproachAndInsert();});
  bt_factory_.registerSimpleAction(
    "ActuateTool", [this](BT::TreeNode &) {return btActuateTool();});
  bt_factory_.registerSimpleAction(
    "MTCRetreat", [this](BT::TreeNode &) {return btMtcRetreat();});
  bt_factory_.registerSimpleAction(
    "DepositToStation", [this](BT::TreeNode &) {return btDepositToStation();});
  bt_factory_.registerSimpleAction(
    "CompleteTarget", [this](BT::TreeNode &) {return btCompleteTarget();});
}

BT::NodeStatus ApproachGraspNode::btFailure(const std::string & reason)
{
  bt_failure_reason_ = reason;
  setState(CycleState::FAILED, reason);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ApproachGraspNode::btPrepareCycle()
{
  // 周期"黑板"锚定本周期生效目标：OBSERVE_ONLY=锁定集锚点缓存的 goal 目标，
  // 其余=感知 selected（见 cycleTargetSnapshot 注释）。
  cycle_target_ = cycleTargetSnapshot();
  cycle_refined_.reset();
  cycle_candidates_.clear();
  cycle_degraded_grasp_ = false;
  if (!cycle_target_) {
    return btFailure("周期目标（selected/锁定集锚点）在启动后失效");
  }
  // goal 钉死校验（设计文档第 7 节）：action 受理到本快照之间感知若已切换
  // selected，身份不一致即周期失败，由编排按新 selected 重新派发；
  // 手动周期钉入值为空，直接采纳当下快照身份。OBSERVE_ONLY 周期的快照按
  // goal ID 取自锁定集缓存，身份一致由缓存键保证（条目消失走上方空快照
  // 失败分支），本校验恒通过。
  if (!cycle_target_id_.empty() && cycle_target_->id != cycle_target_id_) {
    return btFailure(
      "目标身份变更: goal=" + cycle_target_id_ +
      " 当前 selected=" + cycle_target_->id);
  }
  cycle_target_id_ = cycle_target_->id;
  setState(CycleState::PLAN_OBSERVATION, "行为树生成目标导向主动视点");
  const auto base_from_camera = motion_->lookupTransform(base_frame_, camera_frame_);
  if (!base_from_camera) {
    return btFailure("无法取得当前相机位姿");
  }
  ViewContext view_context;
  view_context.target = cycle_target_->center;
  view_context.current_camera_position = base_from_camera->translation();
  view_context.observed_directions = observedDirectionsSnapshot();
  cycle_candidates_ = view_planner_->generate(view_context);
  publishViewMarkers(cycle_target_->center, cycle_candidates_);
  if (cycle_candidates_.empty()) {
    return btFailure("没有生成可用观察视点");
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ApproachGraspNode::btPlanPreview()
{
  for (const auto & candidate : cycle_candidates_) {
    if (motion_->planOrMoveCamera(
        candidate.camera_pose, "PTP", false, candidate.label, true))
    {
      cycle_terminal_state_ = CycleState::PLAN_READY;
      cycle_terminal_message_ = "行为树只规划预览成功；未发送任何运动";
      return BT::NodeStatus::SUCCESS;
    }
  }
  return btFailure("所有候选观察位姿均不可规划");
}

BT::NodeStatus ApproachGraspNode::btAcquireViews()
{
  // 扫描预算制（2.13-E2 / 2.7-OBSERVE，判定纯核 ScanBudget）：
  //   - 下限保证：有效视点观测（移动到位且收到新鲜目标观测）未达
  //     min_effective_views_ 前不得收口；
  //   - 提前收口：达到下限且质量门允许 finalize 即停；
  //   - 预算自适应：运行预算 = max(scan_time_budget_s_, 2.5×移动成本EMA)；
  //     剩余预算按实测 EMA 换不起一个视点即预测性收口，强制 finalize 走降级链；
  //   - maximum_moves_ 仅兜底（候选规划/移动全失败的极端场景）。
  const ScanBudget scan_budget(ScanBudgetConfig{
      maximum_scan_moves_, min_effective_views_, scan_time_budget_s_});
  int moves = 0;
  int effective_views = 0;
  bool budget_exhausted = false;
  std::vector<std::string> attempted;
  const double scan_start_s = now().seconds();
  while (!cancel_requested_.load()) {
    const GateResult finalize_gate = quality_gate_->readyToFinalize(qualitySnapshot());
    const double elapsed_s = now().seconds() - scan_start_s;
    const ScanVerdict verdict = scan_budget.poll(
      finalize_gate.allowed, moves, effective_views, elapsed_s,
      scan_move_cost_ema_s_);
    if (verdict == ScanVerdict::CONVERGED) {
      break;
    }
    if (verdict == ScanVerdict::BUDGET_EXHAUSTED) {
      budget_exhausted = true;
      RCLCPP_WARN(
        get_logger(),
        "观察预算收口（已耗时 %.1fs / 预算 %.1fs，移动成本EMA %.1fs，"
        "有效视点 %d/%d）：%s，强制 finalize 走降级抓取链",
        elapsed_s, scan_budget.effectiveBudgetS(scan_move_cost_ema_s_),
        scan_move_cost_ema_s_, effective_views, min_effective_views_,
        finalize_gate.reason.c_str());
      break;
    }
    if (verdict == ScanVerdict::MOVES_EXHAUSTED) {
      break;
    }
    const auto current_camera = motion_->lookupTransform(base_frame_, camera_frame_);
    if (!current_camera) {
      return btFailure("扫描中无法取得相机位姿");
    }
    // 每次规划前用缓存中的最新观测锚点：跨视角锚点偏差在近距获得观测后
    // 自动纠偏，避免按拍照位姿的旧锚点把目标指到画面外（stale 主因）。
    // OBSERVE_ONLY 周期取 goal 目标的锁定集锚点（见 cycleTargetSnapshot）。
    const auto latest_target = cycleTargetSnapshot();
    const Eigen::Vector3d scan_center =
      (latest_target && latest_target->id == cycle_target_id_) ?
      latest_target->center : cycle_target_->center;
    ViewContext scan_context;
    scan_context.target = scan_center;
    scan_context.current_camera_position = current_camera->translation();
    scan_context.observed_directions = observedDirectionsSnapshot();
    cycle_candidates_ = view_planner_->generate(scan_context);
    publishViewMarkers(scan_center, cycle_candidates_);
    bool moved = false;
    for (const auto & candidate : cycle_candidates_) {
      if (std::find(attempted.begin(), attempted.end(), candidate.label) != attempted.end()) {
        continue;
      }
      attempted.push_back(candidate.label);
      std::string target_reason;
      if (!cycleTargetReady(cycle_target_id_, target_reason)) {
        const bool stale = target_reason == "selected_target_stale";
        // 短暂闪烁/遮挡：等一个新鲜帧窗口后复核，恢复则继续扫描
        const bool recovered = stale && moves > 0 &&
          waitForFreshTarget(now().seconds()) &&
          cycleTargetReady(cycle_target_id_, target_reason);
        if (recovered) {
          RCLCPP_INFO(get_logger(), "目标观测短暂丢失后已恢复，继续扫描");
        }
        if (stale && moves == 0) {
          // 周期起步目标即 stale（残局视角暂不可见）：凭记忆锚点做获取性
          // 移动，到位后由 waitForFreshTarget 判定是否重新可见
          RCLCPP_INFO(get_logger(), "目标观测暂陈旧，凭记忆锚点执行获取性移动");
        } else if (!recovered) {
          return btFailure("目标身份/可见性安全门失败: " + target_reason);
        }
      }
      const std::size_t before = qualitySnapshot().captured_views;
      setState(
        CycleState::MOVE_TO_VIEW,
        candidate.label + " score=" + std::to_string(candidate.score));
      const std::string planner = moves == 0 ? "PTP" : "LIN";
      // 移动成本计时起点：含规划+执行+到位后等帧（预规划预算估计的实测输入）。
      const double move_start_s = now().seconds();
      if (!motion_->planOrMoveCamera(
          candidate.camera_pose, planner, true, candidate.label, true))
      {
        continue;
      }
      ++moves;
      moved = true;
      // 先等到位后的新鲜目标观测：移动中途被重建接受的帧会让 waitForNewView
      // 立即返回，而感知有效样本仍是移动前的旧帧，直接复核安全门必然 stale
      // （0.78FPS 下到位后首帧约 1.3s 才到，6s 窗口约等 4-5 帧）。
      const double move_done_s = now().seconds();
      setState(CycleState::WAIT_FRAME, "等待到位后新鲜目标观测与重建帧");
      if (!waitForFreshTarget(move_done_s)) {
        // 无新鲜目标观测则本视点必无有效掩膜帧，不再等重建成帧，直接换视点；
        // 本次移动不计有效视点、不进成本 EMA（失败样本会带偏预算估计）。
        RCLCPP_WARN(get_logger(), "视点到达但等待新鲜目标观测超时，换下一视点");
        break;
      }
      ++effective_views;
      if (!waitForNewView(before)) {
        RCLCPP_WARN(get_logger(), "视点到达但等待新重建帧超时");
      }
      // 移动+等帧成本 EMA（0.7/0.3，与 trackFrameInterval 同形状）：预算制
      // （2.13-E2）预测"剩余预算能否再换一个有效视点"的实测输入，跨周期保留。
      const double move_cost_s = now().seconds() - move_start_s;
      scan_move_cost_ema_s_ = scan_move_cost_ema_s_ > 0.0 ?
        0.7 * scan_move_cost_ema_s_ + 0.3 * move_cost_s : move_cost_s;
      break;
    }
    if (!moved) {
      // 候选穷尽：质量门已放行时直接收口 finalize（有效视点下限防的是"没看
      // 够就收"，质量达标说明既有覆盖足够）；未放行才按目标不可达跳过。
      if (finalize_gate.allowed) {
        break;
      }
      // 视角规划不可达：批次侧可按 SKIPPED_UNREACHABLE 直接跳过该目标。
      pending_outcome_.store(ExecuteTarget::Result::SKIPPED_UNREACHABLE);
      return btFailure("剩余候选视点均不可达或规划失败");
    }
  }
  if (cancel_requested_.load()) {
    return BT::NodeStatus::FAILURE;
  }
  const GateResult gate = quality_gate_->readyToFinalize(qualitySnapshot());
  if (!gate.allowed) {
    if (budget_exhausted) {
      // 预算收口已按 SUCCESS 路径带现有覆盖强制 finalize（精化不达标由候选
      // 锚点降级抓取兜底，非极端必抓；降级终局 reason 带 degraded_anchor 可计数）。
      return BT::NodeStatus::SUCCESS;
    }
    // 移动次数上限内采集帧不足/不收敛（含有效视点未达下限）：按目标不可达跳过。
    pending_outcome_.store(ExecuteTarget::Result::SKIPPED_UNREACHABLE);
    return btFailure(
      "达到扫描上限仍未收敛（有效视点 " + std::to_string(effective_views) +
      "/" + std::to_string(min_effective_views_) + "）: " + gate.reason);
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ApproachGraspNode::btFinalizeAndValidate()
{
  if (cycle_observe_only_.load()) {
    return BT::NodeStatus::SUCCESS;
  }
  setState(CycleState::FINALIZE, "等待重建精化几何（BuildTargetModel）");
  const bool refined_arrived = waitForRefined(cycle_target_id_);
  const GateResult gate = quality_gate_->readyToGrasp(qualitySnapshot());
  const bool grasp_ready = refined_arrived && gate.allowed &&
    graspDecisionTargetSnapshot() == cycle_target_id_;
  const auto tip_from_tool = motion_->lookupTransform(tip_frame_, tool_frame_);
  if (!tip_from_tool) {
    return btFailure("无法取得 tip 到 tool 的变换");
  }
  if (grasp_ready) {
    cycle_refined_ = refinedSnapshot();
    if (!cycle_refined_) {
      return btFailure("精化位姿数据不存在");
    }
    // 再确认漂移判定的参考锚点：精化几何对应的目标锚点（底/颈中点）。
    cycle_reference_anchor_ = 0.5 * (cycle_refined_->bottom + cycle_refined_->neck);
    Eigen::Isometry3d entry_tool_pose = Eigen::Isometry3d::Identity();
    entry_tool_pose.translation() = cycle_refined_->entry;
    entry_tool_pose.linear() = ViewPlanner::toolOrientation(
      cycle_refined_->axis, cycle_target_->initial_pose.linear().col(0));
    cycle_entry_tip_pose_ = entry_tool_pose * tip_from_tool->inverse();
    cycle_travel_m_ = insertionTravel(*cycle_refined_);
    if (grasp_hyp_pub_) {
      peach_interfaces::msg::GraspHypothesis hyp;
      hyp.header.stamp = now();
      hyp.header.frame_id = base_frame_;
      hyp.target_id = cycle_target_id_;
      hyp.entry_pose = tf2::toMsg(cycle_entry_tip_pose_);
      hyp.travel_m = static_cast<float>(cycle_travel_m_);
      hyp.rank_score = 1.0f;
      grasp_hyp_pub_->publish(hyp);
    }
    return BT::NodeStatus::SUCCESS;
  }
  // 回退（非极端必抓）：精化未产出或质量门未过时，身份一致的锚点还在就用
  // 感知候选几何降级抓取；连锚点都没有才算极端情况，按 SKIPPED_QUALITY 跳过。
  // OBSERVE_ONLY 周期取 goal 目标的锁定集锚点（见 cycleTargetSnapshot）。
  const auto fallback = cycleTargetSnapshot();
  if (!fallback || fallback->id != cycle_target_id_) {
    pending_outcome_.store(ExecuteTarget::Result::SKIPPED_QUALITY);
    return btFailure(
      "最终抓取质量门失败且无候选锚点可回退: " + gate.reason);
  }
  CachedRefined degraded;
  degraded.id = fallback->id;
  degraded.axis = fallback->initial_axis.normalized();
  degraded.suggested_travel_m = fallback->suggested_travel_m;
  degraded.valid = true;
  // 降级路径无精化 neck/entry 几何，insertionTravel 的几何回退恒钳下限
  // （插入过浅）：suggested_travel_m 无效时直接取量程中点。
  cycle_travel_m_ = degraded.suggested_travel_m > 0.0 ?
    insertionTravel(degraded) :
    std::clamp(
      0.5 * (minimum_travel_m_ + maximum_travel_m_), minimum_travel_m_,
      maximum_travel_m_);
  // 入口点取锚点后方（沿轴后退 行程+袋外余量），不依赖可能缺省的 initial_pose；
  // 袋外预入口余量由 grasp.fallback_standoff_m 参数化（原为硬编码 5cm）。
  degraded.entry = degradedEntryPoint(
    fallback->center, degraded.axis, cycle_travel_m_, fallback_standoff_m_);
  // 降级路径的再确认参考锚点 = 感知候选 center（与本入口几何同源）。
  cycle_reference_anchor_ = fallback->center;
  cycle_refined_ = degraded;
  Eigen::Isometry3d entry_tool_pose = Eigen::Isometry3d::Identity();
  entry_tool_pose.translation() = degraded.entry;
  entry_tool_pose.linear() = ViewPlanner::toolOrientation(
    degraded.axis, fallback->initial_pose.linear().col(0));
  cycle_entry_tip_pose_ = entry_tool_pose * tip_from_tool->inverse();
  cycle_degraded_grasp_ = true;
  RCLCPP_WARN(
    get_logger(), "精化不可用/未过门（%s），回退候选锚点降级抓取 %s",
    gate.reason.c_str(), cycle_target_id_.c_str());
  setState(CycleState::FINALIZE, "精化未达标，按感知候选锚点降级抓取");
  if (grasp_hyp_pub_) {
    peach_interfaces::msg::GraspHypothesis hyp;
    hyp.header.stamp = now();
    hyp.header.frame_id = base_frame_;
    hyp.target_id = cycle_target_id_;
    hyp.entry_pose = tf2::toMsg(cycle_entry_tip_pose_);
    hyp.travel_m = static_cast<float>(cycle_travel_m_);
    hyp.rank_score = 0.5f;
    hyp.diagnostic_flags = {"degraded_anchor"};
    grasp_hyp_pub_->publish(hyp);
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ApproachGraspNode::btReconfirmTarget()
{
  // 抓取前再确认（2.7-RECONFIRM）：FinalizeAndValidate 已产出入口几何
  // （cycle_refined_/cycle_entry_tip_pose_/cycle_travel_m_，本周期"黑板"成员），
  // 但 finalize 耗时必然超过观测新鲜度窗口，接触段前必须用最新观测复核目标
  // 没飘走、没换身份、没在风里持续摆动，再决定放行 MTC。
  //
  // 回退开关（验证期遗留）：allow_stale_anchor=true 时退化为旧"按静态锚点
  // 继续"行为，直接放行；false（默认）时新鲜度由本节点单点把关。
  if (allow_stale_anchor_) {
    RCLCPP_WARN(
      get_logger(),
      "grasp.allow_stale_anchor=true：跳过抓取前再确认，按静态目标锚点继续"
      "（旧行为，验证期遗留）");
    return BT::NodeStatus::SUCCESS;
  }
  if (!cycle_refined_ || !cycle_refined_->valid) {
    pending_outcome_.store(ExecuteTarget::Result::SKIPPED_QUALITY);
    return btFailure("再确认无有效入口几何（FinalizeAndValidate 未产出）");
  }
  setState(
    CycleState::RECONFIRM,
    "抓取前再确认：等待新鲜观测复核身份/锚点漂移/摆动平息");
  // plan-while-waiting（2.13-E3）：再确认等待窗口机械臂静止，用当前锚点后台
  // 预规划 MTC 接近/插入任务（只规划不执行）；再确认通过且锚点漂移 ≤
  // grasp.replan_threshold_m 时由 MTC 节点直接执行预规划解，超限才重规划。
  // allow_stale_anchor 旧行为路径在上方已提前返回，不启动预规划。
  preplan_reuse_ = false;
  launchPreplan();
  ReconfirmPolicy policy(ReconfirmConfig{
      reconfirm_tolerance_m_, reconfirm_max_attempts_, false});
  Eigen::Vector3d reference_anchor = cycle_reference_anchor_;
  // 最近一次新鲜观测样本（REFINED 重算要用其锚点/轴）。
  Eigen::Vector3d latest_anchor = reference_anchor;
  Eigen::Vector3d latest_axis = cycle_refined_->axis;
  while (!cancel_requested_.load()) {
    // 单次尝试窗口：实测帧间隔 EMA 自适应（运行时优先，禁硬编码墙钟）；
    // 摆动等平息留在同一窗口预算内，不因摆动帧重开窗口。
    const double window_s = effectiveReconfirmWaitS();
    const double deadline_s = now().seconds() + window_s;
    double after_s = now().seconds();
    ReconfirmDecision decision;
    while (!cancel_requested_.load()) {
      const double remaining_s = deadline_s - now().seconds();
      bool got_fresh = false;
      if (remaining_s > 1e-3) {
        // 窗口等待谓词对周期生效目标生效：OBSERVE_ONLY 等 goal 目标的锁定集
        // 锚点新鲜帧（当前树经 IsObserveOnly 短路不会走到这里，但谓词按周期
        // 上下文取数，树结构调整后语义仍正确），其余等 selected 新鲜帧。
        got_fresh = waitForFreshCycleTarget(after_s, remaining_s);
      }
      const auto latest = got_fresh ? cycleTargetSnapshot() : std::nullopt;
      if (!latest) {
        // 窗口耗尽/目标失效：累计一次超限（摆动持续整个窗口也走本路径）。
        decision = policy.check(ReconfirmSample::exhausted());
        break;
      }
      after_s = latest->received_s;
      latest_anchor = latest->center;
      latest_axis = latest->initial_axis;
      ReconfirmSample sample;
      sample.fresh = true;
      sample.identity_ok = latest->id == cycle_target_id_;
      sample.swinging = latest->swinging;
      sample.anchor = latest_anchor;
      sample.axis = latest_axis;
      sample.anchor_drift_m = (latest_anchor - reference_anchor).norm();
      decision = policy.check(sample);
      // PENDING 仅出现在摆动等平息路径：留在本窗口预算内等下一帧复核。
      if (decision.verdict != ReconfirmVerdict::PENDING) {
        break;
      }
      setState(CycleState::RECONFIRM, decision.reason);
    }
    if (cancel_requested_.load()) {
      return BT::NodeStatus::FAILURE;
    }
    if (decision.verdict == ReconfirmVerdict::PASS) {
      // 预规划复用判定（2.13-E3）：先等有界落定（再确认窗口通常长于 MTC
      // 规划，多数情况已 READY；等待上限 planning_time_s_ 是 MTC stage 自身
      // 的规划预算，不会超产）；READY 且最终锚点漂移 ≤ replan_threshold_m_
      // 才复用，否则清场走内联重规划（由 btMtcApproachAndInsert 执行）。
      preplan_slot_.waitReady(planning_time_s_, cancel_requested_);
      std::string reuse_why;
      preplan_reuse_ = !cancel_requested_.load() &&
        preplan_slot_.reusable(latest_anchor, replan_threshold_m_, reuse_why);
      if (preplan_reuse_) {
        RCLCPP_INFO(
          get_logger(), "%s；MTC 将复用再确认窗口的预规划解", decision.reason.c_str());
      } else {
        RCLCPP_INFO(
          get_logger(), "%s；预规划不复用（%s），MTC 将内联重规划",
          decision.reason.c_str(), reuse_why.c_str());
        settlePreplanBeforeReplan();
      }
      return BT::NodeStatus::SUCCESS;
    }
    if (decision.verdict == ReconfirmVerdict::REFINED) {
      // 漂移超限重算入口几何后，按旧几何的预规划必然作废：立即清槽（仍在
      // 运行的 plan 结果按 stale 忽略）；不再按新几何重启预规划——重算只
      // 发生一次（协议 2.7-RECONFIRM），MTC 阶段内联重规划即可。
      preplan_slot_.discard();
      // 漂移超限：用最新锚点重算 entry/axis 一次（写回周期"黑板"成员）。
      // 精化路径按 delta 平移保留 entry-bottom 几何关系；降级路径按参数化
      // 袋外余量整体重构入口点。参考锚点随之更新，下一窗口按新锚点复核。
      const Eigen::Vector3d delta = latest_anchor - reference_anchor;
      if (nonzeroFinite(latest_axis)) {
        cycle_refined_->axis = latest_axis.normalized();
      }
      if (cycle_degraded_grasp_) {
        cycle_refined_->entry = degradedEntryPoint(
          latest_anchor, cycle_refined_->axis, cycle_travel_m_, fallback_standoff_m_);
      } else {
        cycle_refined_->entry += delta;
        cycle_refined_->bottom += delta;
        cycle_refined_->neck += delta;
      }
      const auto tip_from_tool = motion_->lookupTransform(tip_frame_, tool_frame_);
      if (!tip_from_tool) {
        return btFailure("再确认重算入口时无法取得 tip 到 tool 的变换");
      }
      Eigen::Isometry3d entry_tool_pose = Eigen::Isometry3d::Identity();
      entry_tool_pose.translation() = cycle_refined_->entry;
      entry_tool_pose.linear() = ViewPlanner::toolOrientation(
        cycle_refined_->axis, cycle_target_->initial_pose.linear().col(0));
      cycle_entry_tip_pose_ = entry_tool_pose * tip_from_tool->inverse();
      reference_anchor = latest_anchor;
      RCLCPP_WARN(get_logger(), "%s", decision.reason.c_str());
      setState(CycleState::RECONFIRM, decision.reason);
      continue;
    }
    if (decision.verdict == ReconfirmVerdict::PENDING) {
      // 窗口耗尽计一次超限后重开窗口再试（reason 已含累计次数）。
      RCLCPP_WARN(get_logger(), "%s", decision.reason.c_str());
      setState(CycleState::RECONFIRM, decision.reason);
      continue;
    }
    // ABORT：放弃本目标，周期终局 SKIPPED_QUALITY；reason 附最近跟踪状态，
    // 便于区分"观测窗口耗尽"是出视野/深度空洞/跟踪丢失中的哪一类。
    std::string reason = decision.reason;
    const auto latest = cycleTargetSnapshot();
    if (latest) {
      reason += "；最近跟踪状态=" + trackingStatusLabel(latest->tracking_status);
    }
    pending_outcome_.store(ExecuteTarget::Result::SKIPPED_QUALITY);
    return btFailure(reason);
  }
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ApproachGraspNode::btReportReady()
{
  cycle_terminal_state_ = CycleState::READY_FOR_GRASP;
  cycle_terminal_message_ = "精化质量通过；grasp.enabled=false，未执行接触动作";
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ApproachGraspNode::btReportObserveOnly()
{
  // OBSERVE_ONLY 圆满终态：观察+精化验证段已完成，reason 标 observe_only；
  // PLAN_READY 经 terminalOutcome 映射 SUCCEEDED 上报编排器。
  cycle_terminal_state_ = CycleState::PLAN_READY;
  cycle_terminal_message_ =
    "observe_only 周期完成：仅观察与精化验证，未执行靠近/抓取/工具动作";
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ApproachGraspNode::btMtcApproachAndInsert()
{
  std::string reason;
  // 运动输出权限（A8）：deactivate 竞态下周期虽已被取消流程拦截，此处再兜底。
  if (!motionOutputAllowed(reason)) {
    return btFailure("MTC 执行被拒绝: " + reason);
  }
  if (!safetyReady(reason)) {
    return btFailure("MTC 执行前机器人安全门失败: " + reason);
  }
  // 环境几何保护区（阶段 F1）：接触段入口点落入任一保护盒（闭区间，含盒
  // 表面）即拒规划，终局 SKIPPED_UNREACHABLE。本检查放在 MTC 规划前单点
  // 执行，天然覆盖全部入口来源——FinalizeAndValidate 精化入口、降级链
  // degradedEntryPoint 入口、ReconfirmTarget 漂移重算入口；静态几何约束
  // 先于目标新鲜度等待判定，快速失败且不消耗观测窗口预算。
  if (cycle_refined_ && cycle_refined_->valid) {
    if (const auto hit = protectedZoneHit(cycle_refined_->entry, protected_zones_)) {
      pending_outcome_.store(ExecuteTarget::Result::SKIPPED_UNREACHABLE);
      return btFailure(
        "入口在保护区（盒#" + std::to_string(*hit) + "，入口点 [" +
        std::to_string(cycle_refined_->entry.x()) + ", " +
        std::to_string(cycle_refined_->entry.y()) + ", " +
        std::to_string(cycle_refined_->entry.z()) + "]），拒绝 MTC 规划");
    }
  }
  if (!cycleTargetReady(cycle_target_id_, reason)) {
    if (reason != "selected_target_stale") {
      return btFailure("MTC 执行前安全门失败: " + reason);
    }
    // finalize 耗时必然超过观测新鲜度窗口：先等一窗新鲜观测再复核；
    // 仍不新鲜时——allow_stale_anchor=false（默认）：新鲜度已由 ReconfirmTarget
    // 单点把关，此处不再二次放行 stale，按质量原因跳过；allow_stale_anchor=true
    // （验证期遗留）：按静态果实处置——锚点来自多视融合/身份记忆，MTC 碰撞
    // 检查兜底，继续执行。
    if (waitForFreshTarget(now().seconds()) &&
      cycleTargetReady(cycle_target_id_, reason))
    {
      RCLCPP_INFO(get_logger(), "MTC 前目标观测已刷新，继续执行");
    } else if (!allow_stale_anchor_) {
      pending_outcome_.store(ExecuteTarget::Result::SKIPPED_QUALITY);
      return btFailure(
        "MTC 执行前目标观测仍陈旧（ReconfirmTarget 后二次把关，"
        "allow_stale_anchor=false 不放行 stale）");
    } else {
      RCLCPP_WARN(
        get_logger(), "MTC 前观测仍陈旧，按静态目标锚点继续（已等待复核）");
    }
  }
  setState(
    CycleState::MTC_APPROACH_INSERT,
    cycle_degraded_grasp_ ?
      "MTC（降级：候选锚点）: OMPL 避障到入口，再沿候选轴直线插入" :
      "MTC: OMPL 避障到入口，再沿精化轴直线插入");
  GraspTaskResult result;
  if (preplan_reuse_) {
    // 预规划复用（2.13-E3）：再确认 PASS 点已判定 READY 且漂移 ≤
    // replan_threshold_m_；直接执行预规划解，省去串行规划段。入口几何自
    // 预规划启动后未变（REFINED 重算已在再确认段清槽，不可能走到这里）。
    preplan_reuse_ = false;
    if (preplan_thread_.joinable()) {
      preplan_thread_.join();
    }
    preplan_slot_.discard();
    setState(CycleState::MTC_APPROACH_INSERT, "MTC 执行预规划解（plan-while-waiting 命中）");
    result = grasp_task_->executePreplannedApproach();
    if (!result.success && !result.execution_started &&
      result.reason.find("无可用预规划解") != std::string::npos)
    {
      // 兜底：槽判定与任务槽不一致（理论上不发生）时退化为内联规划。
      RCLCPP_WARN(get_logger(), "预规划槽命中但任务缺失，退回内联规划");
      result = grasp_task_->approachAndInsert(
        cycle_entry_tip_pose_, cycle_refined_->axis, cycle_travel_m_, true);
    }
  } else {
    // 内联规划前清场：防预规划线程仍占用 GraspTask active 槽造成并发。
    settlePreplanBeforeReplan();
    result = grasp_task_->approachAndInsert(
      cycle_entry_tip_pose_, cycle_refined_->axis, cycle_travel_m_, true);
  }
  if (result.execution_started) {
    contact_recovery_required_.store(true);
  }
  if (!result.success) {
    // 规划阶段失败说明目标不可达；执行已启动后的失败维持 FAILED 分级。
    pending_outcome_.store(
      result.execution_started ?
      ExecuteTarget::Result::FAILED : ExecuteTarget::Result::SKIPPED_UNREACHABLE);
    return btFailure("MTC 接近/插入失败: " + result.reason);
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ApproachGraspNode::btActuateTool()
{
  setState(CycleState::ACTUATE_TOOL, "触发末端工具抓取/切割");
  if (!commandToolClose()) {
    const auto retreat = grasp_task_->retreat(
      cycle_refined_->axis, cycle_travel_m_, true);
    if (retreat.success) {
      contact_recovery_required_.store(false);
    }
    return btFailure(
      "末端工具失败；MTC 紧急撤离结果: " + retreat.reason);
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ApproachGraspNode::btMtcRetreat()
{
  setState(CycleState::MTC_RETREAT, "MTC 沿插入反方向保持直线撤离");
  const auto result = grasp_task_->retreat(
    cycle_refined_->axis, cycle_travel_m_, true);
  if (!result.success) {
    return btFailure("MTC 抓取后撤离失败，需要人工处理: " + result.reason);
  }
  contact_recovery_required_.store(false);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ApproachGraspNode::btDepositToStation()
{
  setState(CycleState::MTC_RETREAT, "转移到卸果站或跳过（M8 未标定）");
  if (deposit_pose_named_target_.empty()) {
    cycle_deposit_ok_ = false;
    cycle_deposit_skipped_m8_ = true;
    cycle_deposit_reason_ = "pending_m8_unload_pose";
    return BT::NodeStatus::SUCCESS;
  }
  if (!motion_) {
    cycle_deposit_ok_ = false;
    cycle_deposit_skipped_m8_ = false;
    cycle_deposit_reason_ = "MoveIt 尚未初始化";
    return btFailure("卸果转移失败: " + cycle_deposit_reason_);
  }
  std::string message;
  const bool ok = motion_->goToPhotoPose(
    deposit_pose_named_target_, execution_enabled_.load(), message);
  cycle_deposit_ok_ = ok;
  cycle_deposit_skipped_m8_ = false;
  cycle_deposit_reason_ = message;
  if (!ok) {
    return btFailure("卸果转移失败: " + message);
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ApproachGraspNode::btCompleteTarget()
{
  // 账本由 peach_task_executor 按 ExecuteTarget 终态写入；此处只结束行为树。
  cycle_terminal_state_ = CycleState::SUCCEEDED;
  cycle_terminal_message_ = "行为树完成主动视觉、MTC 抓取和撤离";
  return BT::NodeStatus::SUCCESS;
}

void ApproachGraspNode::publishViewMarkers(
  const Eigen::Vector3d & target,
  const std::vector<ViewCandidate> & candidates)
{
  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(clear);
  const std::size_t limit = std::min<std::size_t>(candidates.size(), 24U);
  for (std::size_t index = 0; index < limit; ++index) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = now();
    marker.ns = "candidate_views";
    marker.id = static_cast<int>(index);
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    geometry_msgs::msg::Point start;
    start.x = candidates[index].camera_pose.translation().x();
    start.y = candidates[index].camera_pose.translation().y();
    start.z = candidates[index].camera_pose.translation().z();
    geometry_msgs::msg::Point end;
    end.x = target.x();
    end.y = target.y();
    end.z = target.z();
    marker.points = {start, end};
    marker.scale.x = 0.003;
    marker.scale.y = 0.007;
    marker.scale.z = 0.010;
    marker.color.r = static_cast<float>(1.0 - candidates[index].score);
    marker.color.g = static_cast<float>(candidates[index].score);
    marker.color.b = 0.25F;
    marker.color.a = 0.75F;
    array.markers.push_back(marker);
  }
  marker_pub_->publish(array);
}

}  // namespace peach_manipulation_skills
