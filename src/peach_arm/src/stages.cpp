// 功能：显式模式 switch 执行器（阶段函数）。主链阶段：观察、精化验证、再确认、
// 预抓取验证、套入、刀具、原路撤退。不写账本、不调重建 Trigger。接触走
// GraspTask；刀具 IO 只在本文件。阶段调用序列与原 behavior_tree.xml 主树
// 遍历严格同构（映射表见 executeCycle 注释）。
#include "peach_arm/manipulation_skills_node.hpp"
#include "peach_arm/math_utils.hpp"
#include "peach_arm/pregrasp_level.hpp"
#include "peach_arm/pregrasp_residual.hpp"
#include "peach_arm/acm_policy.hpp"
#include "peach_arm/reconfirm_policy.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <peach_interfaces/msg/failure_code.hpp>
#include <peach_interfaces/msg/peach_target_observation.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

using namespace std::chrono_literals;

namespace peach_arm
{
using FailureCode = peach_interfaces::msg::FailureCode;

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

const char * motionStageName(MotionStage stage)
{
  switch (stage) {
    case MotionStage::TRANSIT:
      return "TRANSIT";
    case MotionStage::PREGRASP:
      return "PREGRASP";
    case MotionStage::CONTACT:
      return "CONTACT";
    case MotionStage::TOOL:
      return "TOOL";
  }
  return "TRANSIT";
}
}  // namespace
// 数据快照薄壳：统一从 cache_ 取一致性快照，供阶段函数与运动接口使用。
QualitySnapshot ManipulationSkillsNode::qualitySnapshot()
{
  return cache_.qualitySnapshot();
}

std::optional<CachedTarget> ManipulationSkillsNode::targetSnapshot()
{
  return cache_.targetSnapshot();
}

// 周期生效目标快照（阶段 E 残局抬质量能力端）。
// 为什么按周期分流而不是在执行接受时把 goal 目标写进 selected 缓存：
// 覆盖 target_ 会污染感知驱动的四源调和（selected 身份/精化/决策缓存的 ID
// 一致性全部以 target_ 为准），周期结束后还会留下"假 selected"干扰后续
// 周期；而锁定集锚点缓存是独立数据源，goal 钉入 ID（ctx.target_id）在
// 受理时已写好，执行体按"本周期生效目标"取快照即可，零侵入既有调和语义。
// FULL/PREVIEW/手动周期（target_id 空）恒退化为 selected 缓存。
std::optional<CachedTarget> ManipulationSkillsNode::cycleTargetSnapshot(
  const std::string & target_id)
{
  if (!target_id.empty()) {
    auto locked = cache_.lockedTargetSnapshot(target_id);
    if (locked) {return locked;}
  }
  return cache_.targetSnapshot();
}

std::vector<Eigen::Vector3d> ManipulationSkillsNode::observedDirectionsSnapshot()
{
  return cache_.observedDirections();
}

std::optional<CachedRefined> ManipulationSkillsNode::refinedSnapshot()
{
  return cache_.refinedSnapshot();
}

std::string ManipulationSkillsNode::graspDecisionTargetSnapshot()
{
  return cache_.graspDecisionTarget();
}

bool ManipulationSkillsNode::waitForNewView(std::size_t previous_views)
{
  return cache_.waitForNewView(previous_views, effectiveFrameWaitS(), cancel_requested_);
}

bool ManipulationSkillsNode::waitForNewStation(std::size_t previous_stations)
{
  return cache_.waitForNewStation(
    previous_stations, effectiveFrameWaitS(), cancel_requested_);
}

bool ManipulationSkillsNode::waitForFreshTarget(
  const std::string & target_id, double after_s)
{
  return waitForFreshCycleTarget(target_id, after_s, effectiveFrameWaitS());
}

bool ManipulationSkillsNode::waitForFreshCycleTarget(
  const std::string & target_id, double after_s, double window_s,
  bool live_observation_required)
{
  // 数据源随周期生效目标走（同 cycleTargetSnapshot 的分流理由）：
  // OBSERVE_ONLY 等 goal 目标的锁定集锚点新鲜帧，其余等 selected 新鲜帧。
  // live_observation_required=false 仅再确认：已 OBSERVED 则用当前锚点。
  if (!target_id.empty()) {
    return cache_.waitForFreshLockedTarget(
      target_id, after_s, window_s, cancel_requested_,
      live_observation_required);
  }
  return cache_.waitForFreshTarget(
    after_s, window_s, cancel_requested_, live_observation_required);
}

bool ManipulationSkillsNode::waitForRefined(const std::string & target_id)
{
  // 超时按协议 2.7-FINALIZE 的 T(refined) 帧率自适应（effectiveRefinedWaitS）。
  return cache_.waitForRefined(target_id, effectiveRefinedWaitS(), cancel_requested_);
}

bool ManipulationSkillsNode::failStage(CycleContext & ctx, const std::string & reason)
{
  ctx.failure_reason = reason;
  setState(CycleState::FAILED, reason, ctx.target_id);
  return false;
}

bool ManipulationSkillsNode::failStage(
  CycleContext & ctx, uint8_t outcome, uint32_t failure_code,
  const std::string & reason)
{
  pending_outcome_.store(outcome);
  ctx.failure_code = failure_code;
  return failStage(ctx, reason);
}

Eigen::Isometry3d ManipulationSkillsNode::entryToolPose(
  const Eigen::Vector3d & entry, const Eigen::Vector3d & axis,
  const Eigen::Vector3d & preferred_x)
{
  Eigen::Isometry3d entry_tool_pose = Eigen::Isometry3d::Identity();
  entry_tool_pose.translation() = entry;
  const auto current_tool = motion_->lookupTransform(params_.frames.base, params_.frames.tool);
  entry_tool_pose.linear() = current_tool ?
    alignFrameZ(current_tool->linear(), axis) :
    ViewPlanner::toolOrientation(axis, preferred_x);
  return entry_tool_pose;
}

bool ManipulationSkillsNode::contactEntryGeometry(
  const CachedRefined & refined, const Eigen::Isometry3d & initial_pose,
  Eigen::Isometry3d & entry_tip_pose, double & travel_m, std::string & error)
{
  const Eigen::Isometry3d entry_tool_pose = entryToolPose(
    refined.entry, refined.axis, initial_pose.linear().col(0));
  const auto tip_from_tool = motion_->lookupTransform(params_.frames.tip, params_.frames.tool);
  if (!tip_from_tool) {
    error = "无法取得 tip 到 tool 的变换";
    return false;
  }
  entry_tip_pose = entry_tool_pose * tip_from_tool->inverse();
  travel_m = insertionTravel(refined);
  return true;
}

// 授权矩阵（cycle_support.hpp）的失败包装：GraspDecision 复检未通过
// 沿用 skipped_quality 语义（质量原因跳过，编排器可重派）；其余拒绝
// （权限/安全/取消/使能）按 FAILED 分级。
bool ManipulationSkillsNode::requireStageAuthority(
  CycleContext & ctx, MotionStage stage, const std::string & label)
{
  std::string why;
  if (authorizeStage(ctx, stage, why)) {
    return true;
  }
  const bool decision_recheck_failed =
    (stage == MotionStage::CONTACT || stage == MotionStage::TOOL) &&
    (graspDecisionTargetSnapshot() != ctx.target_id ||
    !qualitySnapshot().grasp_allowed);
  pending_outcome_.store(
    decision_recheck_failed ?
    ExecuteTarget::Result::SKIPPED_QUALITY : ExecuteTarget::Result::FAILED);
  return failStage(
    ctx, label + "被拒绝（" + motionStageName(stage) + "）: " + why);
}

// 阶段调用序列（与原 behavior_tree.xml 主树遍历严格同构；BT 节点→阶段函数
// 映射见包内重构说明）：PrepareCycle →（IsPlanOnly→PlanObservationPreview |
// （IsSkipObservation|ObserveScan）→ QualityValidate →（IsObserveOnly→
// ReportObserveOnly | IsGraspDisabled→ReportReadyForGrasp |（ReconfirmTarget →
// MovePregrasp → VerifyPregrasp →（IsPregraspOnly→HoldPregrasp |
// SleeveCutRetreat 子树））→ CompleteTarget））。VerifyCutHold 空阶段已删
// （W5-12：原实现只落状态不判定；tool 遥测桶由 ActuateCutter 同投影）。
void ManipulationSkillsNode::executeCycle(CycleContext & ctx)
{
  const auto finish = [this, &ctx](CycleState state, const std::string & message) {
      execution_armed_.store(false);
      // 先落终态再解除 running：executeAction 以 running==false 作为周期结束信号，
      // 这样它读到的状态一定是终态而不是上一个中间态。
      setState(state, message, ctx.target_id);
      running_.store(false);
      publishState();
    };
  ctx.failure_reason.clear();
  ctx.terminal_state = CycleState::SUCCEEDED;
  ctx.terminal_message = "周期完成";
  bool ok = false;
  try {
    ok = stagePrepareCycle(ctx);
    if (ok && !execution_enabled_.load()) {
      // PREVIEW 分支（终结，绝不运动）：只规划候选观察位姿。
      ok = stagePlanPreview(ctx);
    } else if (ok) {
      if (!ctx.skip_observation) {
        ok = stageAcquireViews(ctx);
      }
      if (ok) {
        ok = stageFinalizeAndValidate(ctx);
        if (ok) {
          if (ctx.observe_only) {
            stageReportObserveOnly(ctx);
          } else if (!grasp_enabled_.load()) {
            stageReportReady(ctx);
          } else {
            ok = stageReconfirmTarget(ctx);
            if (ok) {ok = stageMovePregrasp(ctx);}
            if (ok) {ok = stageVerifyPregrasp(ctx);}
            if (ok) {
              if (ctx.pregrasp_only) {
                stageHoldPregrasp(ctx);
              } else {
                ok = stagePlanSleeveAndReverseRetreat(ctx);
                if (ok) {ok = stageSleeveLinear(ctx);}
                if (ok) {ok = stageActuateCutter(ctx);}
                if (ok) {ok = stageVerifyCut(ctx);}
                if (ok) {ok = stageExecuteReservedReverseRetreat(ctx);}
                if (ok) {ok = stageReturnHarvestStow(ctx);}
                if (ok) {ok = stageVerifyHarvestOutcome(ctx);}
              }
              if (ok) {stageCompleteTarget(ctx);}
            }
          }
        }
      }
    }
    // 终局分级（顺序即语义）：接触 recovery > 取消 > 成功 > 失败。
    // PREGRASP_ONLY 到位也置 recovery（ACK 前不 Survey），但是产品成功：
    // 不得改写成「接触取消/撤离未确认」再 abort 成 FAILED。
    if (contact_recovery_required_.load() && !(ctx.pregrasp_only && ok)) {
      finish(
        CycleState::RECOVERY_REQUIRED,
        "接触阶段取消或撤离未确认；保持停止，须现场人工撤离后确认恢复");
    } else if (cancel_requested_.load()) {
      finish(CycleState::CANCELED, "用户取消主动视觉/MTC 周期");
    } else if (ok) {
      finish(ctx.terminal_state, ctx.terminal_message);
    } else {
      finish(
        CycleState::FAILED,
        ctx.failure_reason.empty() ? "执行器返回 FAILURE" : ctx.failure_reason);
    }
  } catch (const std::exception & error) {
    if (contact_recovery_required_.load()) {
      finish(
        CycleState::RECOVERY_REQUIRED,
        std::string("接触阶段异常，须现场人工撤离: ") + error.what());
    } else {
      finish(CycleState::FAILED, std::string("周期执行异常: ") + error.what());
    }
  }
}

bool ManipulationSkillsNode::stagePrepareCycle(CycleContext & ctx)
{
  // 周期"黑板"锚定本周期生效目标：OBSERVE_ONLY=锁定集锚点缓存的 goal 目标，
  // 其余=感知 selected（见 cycleTargetSnapshot 注释）。
  ctx.target = cycleTargetSnapshot(ctx.target_id);
  ctx.refined.reset();
  ctx.candidates.clear();
  ctx.pregrasp_verified = false;
  ctx.sleeve_planned = false;
  ctx.cut_command_accepted = false;
  ctx.cut_confirmed = false;
  ctx.retreat_confirmed = false;
  ctx.completion_level = 0;
  ctx.failure_code = FailureCode::NONE;
  ctx.pregrasp_msg = peach_interfaces::msg::PregraspVerification();
  ctx.contact_transaction_id = ctx.target_id + ":contact";
  tool_actuator_.resetSafe();
  if (!ctx.target) {
    return failStage(ctx, "周期目标（selected/锁定集锚点）在启动后失效");
  }
  // goal 钉死校验（ExecuteTarget.goal.target_id）：action 受理到本快照之间感知若已切换
  // selected，身份不一致即周期失败，由编排按新 selected 重新派发；
  // 手动周期钉入值为空，直接采纳当下快照身份。OBSERVE_ONLY 周期的快照按
  // goal ID 取自锁定集缓存，身份一致由缓存键保证（条目消失走上方空快照
  // 失败分支），本校验恒通过。
  if (!ctx.target_id.empty() && ctx.target->id != ctx.target_id) {
    return failStage(
      ctx,
      "目标身份变更: goal=" + ctx.target_id +
      " 当前 selected=" + ctx.target->id);
  }
  ctx.target_id = ctx.target->id;
  setState(CycleState::PLAN_OBSERVATION, "生成目标导向主动视点", ctx.target_id);
  const auto base_from_camera = motion_->lookupTransform(params_.frames.base,
      params_.frames.camera);
  if (!base_from_camera) {
    return failStage(ctx, "无法取得当前相机位姿");
  }
  ViewContext view_context;
  view_context.target = ctx.target->center;
  view_context.current_camera_position = base_from_camera->translation();
  view_context.observed_directions = observedDirectionsSnapshot();
  view_context.bbox_valid = ctx.target->bbox_valid;
  view_context.bbox_x = ctx.target->bbox_x;
  view_context.bbox_y = ctx.target->bbox_y;
  view_context.bbox_w = ctx.target->bbox_w;
  view_context.bbox_h = ctx.target->bbox_h;
  view_context.image_width = ctx.target->image_width;
  view_context.image_height = ctx.target->image_height;
  view_context.neighbor_centers = cache_.lockedNeighborCenters(ctx.target_id);
  view_context.foreground_ratio = ctx.target->foreground_ratio;
  ctx.candidates = view_planner_->generate(view_context);
  publishViewMarkers(ctx.target->center, ctx.candidates);
  if (ctx.candidates.empty()) {
    return failStage(ctx, "没有生成可用观察视点");
  }
  return true;
}

bool ManipulationSkillsNode::stagePlanPreview(CycleContext & ctx)
{
  // OBSERVE_ONLY 必须真走臂采多视角；plan-only 不得伪装成观察成功。
  if (ctx.observe_only) {
    pending_outcome_.store(ExecuteTarget::Result::SKIPPED_QUALITY);
    return failStage(
      ctx,
      "observe_only 需要 execution.enabled=true 才能采多视角；当前为只规划预览");
  }
  // plan-only 结构性只规划：execute=false 恒不发送运动。
  for (const auto & candidate : ctx.candidates) {
    if (motion_->planOrMoveCamera(
        candidate.camera_pose, "LIN", false, candidate.label, false))
    {
      ctx.terminal_state = CycleState::PLAN_READY;
      ctx.terminal_message = "只规划预览成功；未发送任何运动";
      return true;
    }
  }
  pending_outcome_.store(ExecuteTarget::Result::SKIPPED_UNREACHABLE);
  return failStage(ctx, "所有候选观察位姿均不可规划");
}

bool ManipulationSkillsNode::stageAcquireViews(CycleContext & ctx)
{
  // plan-only 防御断言：PREVIEW 分支在 executeCycle 已分流，本阶段不可达
  // plan-only（结构性保证）；仍直接失败，绝不扫描执行。
  if (!execution_enabled_.load()) {
    return failStage(ctx, "plan-only 分支进入观察段（结构性防御，直接失败）");
  }
  // 观察移动授权（TRANSIT 级）：单点判定取代散落的裸使能检查；
  // 逐次执行前运动接口安全门（Active ∧ robotReady）仍逐 move 复核。
  if (!requireStageAuthority(ctx, MotionStage::TRANSIT, "观察段移动")) {
    return false;
  }
  // 扫描停准则（判定纯核 ScanBudget，对齐 Open3D TSDF / NBV）：
  //   - 质量门放行即停，不为凑次数继续运动；
  //   - 否则把 maximum_moves 走完（当前位已采帧；短移补基线）；
  //   - 等帧超时在 waitFor*；时长预算/移动成本 EMA 预测收口已随死分支
  //     删除（W5-12），不参与停准则。
  const ScanBudget scan_budget(ScanBudgetConfig{
      static_cast<int>(params_.scan.maximum_moves)});
  int moves = 0;
  int effective_views = 0;
  std::vector<std::string> attempted;
  const double scan_start_s = now().seconds();
  setState(CycleState::WAIT_FRAME, "当前位采帧，不环绕", ctx.target_id);
  {
    const std::size_t before_stay = qualitySnapshot().captured_views;
    if (waitForFreshTarget(ctx.target_id, scan_start_s) &&
      waitForNewView(before_stay))
    {
      RCLCPP_INFO(
        get_logger(),
        "当前位已采重建帧，不环绕；基线未过再做最多两次最近短移");
    } else {
      RCLCPP_WARN(get_logger(), "当前位采帧未完成，将做一次短 PTP 接近");
    }
  }
  while (!cancel_requested_.load()) {
    const GateResult finalize_gate = quality_gate_->readyToFinalize(qualitySnapshot());
    const ScanVerdict verdict = scan_budget.poll(finalize_gate.allowed, moves);
    if (verdict == ScanVerdict::CONVERGED) {
      break;
    }
    if (verdict == ScanVerdict::MOVES_EXHAUSTED) {
      break;
    }
    const auto current_camera = motion_->lookupTransform(params_.frames.base,
        params_.frames.camera);
    if (!current_camera) {
      return failStage(ctx, "扫描中无法取得相机位姿");
    }
    // 每次规划前用缓存中的最新观测锚点：跨视角锚点偏差在近距获得观测后
    // 自动纠偏，避免按拍照位姿的旧锚点把目标指到画面外（stale 主因）。
    // OBSERVE_ONLY 周期取 goal 目标的锁定集锚点（见 cycleTargetSnapshot）。
    const auto latest_target = cycleTargetSnapshot(ctx.target_id);
    const Eigen::Vector3d scan_center =
      (latest_target && latest_target->id == ctx.target_id) ?
      latest_target->center : ctx.target->center;
    ViewContext scan_context;
    scan_context.target = scan_center;
    scan_context.current_camera_position = current_camera->translation();
    scan_context.observed_directions = observedDirectionsSnapshot();
    if (latest_target && latest_target->id == ctx.target_id) {
      scan_context.bbox_valid = latest_target->bbox_valid;
      scan_context.bbox_x = latest_target->bbox_x;
      scan_context.bbox_y = latest_target->bbox_y;
      scan_context.bbox_w = latest_target->bbox_w;
      scan_context.bbox_h = latest_target->bbox_h;
      scan_context.image_width = latest_target->image_width;
      scan_context.image_height = latest_target->image_height;
    } else if (ctx.target) {
      scan_context.bbox_valid = ctx.target->bbox_valid;
      scan_context.bbox_x = ctx.target->bbox_x;
      scan_context.bbox_y = ctx.target->bbox_y;
      scan_context.bbox_w = ctx.target->bbox_w;
      scan_context.bbox_h = ctx.target->bbox_h;
      scan_context.image_width = ctx.target->image_width;
      scan_context.image_height = ctx.target->image_height;
    }
    scan_context.neighbor_centers = cache_.lockedNeighborCenters(ctx.target_id);
    if (latest_target && latest_target->id == ctx.target_id) {
      scan_context.foreground_ratio = latest_target->foreground_ratio;
    } else if (ctx.target) {
      scan_context.foreground_ratio = ctx.target->foreground_ratio;
    }
    ctx.candidates = view_planner_->generate(scan_context);
    publishViewMarkers(scan_center, ctx.candidates);
    bool moved = false;
    for (const auto & candidate : ctx.candidates) {
      if (std::find(attempted.begin(), attempted.end(), candidate.label) != attempted.end()) {
        continue;
      }
      attempted.push_back(candidate.label);
      std::string target_reason;
      if (!cycleTargetReady(ctx.target_id, target_reason)) {
        const bool stale = target_reason == "selected_target_stale";
        // 短暂闪烁/遮挡：等一个新鲜帧窗口后复核，恢复则继续扫描
        const bool recovered = stale && moves > 0 &&
          waitForFreshTarget(ctx.target_id, now().seconds()) &&
          cycleTargetReady(ctx.target_id, target_reason);
        if (recovered) {
          RCLCPP_INFO(get_logger(), "目标观测短暂丢失后已恢复，继续扫描");
        }
        if (stale && moves == 0) {
          // 周期起步目标即 stale（残局视角暂不可见）：凭记忆锚点做获取性
          // 移动，到位后由 waitForFreshTarget 判定是否重新可见
          RCLCPP_INFO(get_logger(), "目标观测暂陈旧，凭记忆锚点执行获取性移动");
        } else if (!recovered) {
          return failStage(ctx, "目标身份/可见性安全门失败: " + target_reason);
        }
      }
      setState(
        CycleState::MOVE_TO_VIEW,
        candidate.label + " score=" + std::to_string(candidate.score) +
        " travel=" + std::to_string(candidate.travel_m) + "m",
        ctx.target_id);
      // 观察走最近直线：只 Pilz LIN，失败换下一候选；禁止 PTP/OMPL 绕行。
      const std::string planner = "LIN";
      // 移动前记下机位数：到位后同机位连帧不加机位，必须比移动前多一个。
      const std::size_t stations_before_move = observedDirectionsSnapshot().size();
      // 移动成本计时起点：含规划+执行+到位后等帧（预算估计的实测输入）。
      const double move_start_s = now().seconds();
      if (!motion_->planOrMoveCamera(
          candidate.camera_pose, planner, true, candidate.label, false))
      {
        continue;
      }
      ++moves;
      moved = true;
      // 先等到位后的新鲜目标观测：移动中途被重建接受的帧会让 waitForNewView
      // 立即返回，而感知有效样本仍是移动前的旧帧，直接复核安全门必然 stale
      // （0.78FPS 下到位后首帧约 1.3s 才到，6s 窗口约等 4-5 帧）。
      const double move_done_s = now().seconds();
      setState(CycleState::WAIT_FRAME, "等待到位后新鲜目标观测与重建帧", ctx.target_id);
      if (!waitForFreshTarget(ctx.target_id, move_done_s)) {
        // 无新鲜目标观测则本视点必无有效掩膜帧，不再等重建成帧，直接换视点；
        // 本次移动不计有效视点、不进成本 EMA（失败样本会带偏预算估计）。
        RCLCPP_WARN(get_logger(), "视点到达但等待新鲜目标观测超时，换下一视点");
        break;
      }
      // 停走采帧（温室多视：到位静止后再积分）。同机位连帧会加 captured_views
      // 但不加机位；等 view_directions 相对移动前增加，避免 60ms 内收口丢掉下一颗。
      if (!waitForNewStation(stations_before_move)) {
        RCLCPP_WARN(get_logger(), "视点到达但未形成新机位，换下一视点");
        break;
      }
      ++effective_views;
      (void)move_start_s;  // 移动起点仅过程对照（成本 EMA 已随死分支删除）
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
      return failStage(ctx, "剩余候选视点均不可达或规划失败");
    }
  }
  if (cancel_requested_.load()) {
    return false;
  }
  const GateResult gate = quality_gate_->readyToFinalize(qualitySnapshot());
  if (!gate.allowed) {
    // 移动次数上限内采集帧不足/不收敛（含有效视点未达下限）：按目标不可达跳过。
    pending_outcome_.store(ExecuteTarget::Result::SKIPPED_UNREACHABLE);
    return failStage(
      ctx,
      "达到扫描上限仍未收敛（有效视点 " + std::to_string(effective_views) +
      "/" + std::to_string(params_.scan.min_effective_views) + "）: " + gate.reason);
  }
  return true;
}

bool ManipulationSkillsNode::stageFinalizeAndValidate(CycleContext & ctx)
{
  setState(CycleState::FINALIZE, "等待重建精化几何（BuildTargetModel）", ctx.target_id);
  const bool refined_arrived = waitForRefined(ctx.target_id);
  if (ctx.observe_only) {
    if (!refined_arrived) {
      pending_outcome_.store(ExecuteTarget::Result::SKIPPED_QUALITY);
      return failStage(
        ctx, "observe_only 未等到绑定目标的 TSDF/精化几何: " + ctx.target_id);
    }
    return true;
  }
  const QualitySnapshot snapshot = qualitySnapshot();
  const GateResult gate = ctx.pregrasp_only ?
    quality_gate_->readyToApproach(snapshot) :
    quality_gate_->readyToGrasp(snapshot);
  if (!gate.allowed) {
    RCLCPP_WARN(
      get_logger(),
      "grasp quality gate denied: %s (axis_angle_deg=%.2f pregrasp_only=%s)",
      gate.reason.c_str(), snapshot.axis_angle_deg,
      ctx.pregrasp_only ? "true" : "false");
  }
  const bool grasp_ready = refined_arrived && gate.allowed &&
    (ctx.pregrasp_only ||
    graspDecisionTargetSnapshot() == ctx.target_id);
  const auto tip_from_tool = motion_->lookupTransform(params_.frames.tip, params_.frames.tool);
  if (!tip_from_tool) {
    return failStage(ctx, "无法取得 tip 到 tool 的变换");
  }
  if (grasp_ready) {
    ctx.refined = refinedSnapshot();
    if (!ctx.refined) {
      return failStage(ctx, "精化位姿数据不存在");
    }
    // 再确认漂移判定必须和后续新鲜观测用同一套锚点定义（感知底/颈中点）。
    // 若拿 TSDF 中点去比单帧检测中点，现场曾把 ~5cm 的定义差当成果实移动，
    // 再把精化入口整包平移，直线插入在错误位置走不完。
    const auto live_anchor = cycleTargetSnapshot(ctx.target_id);
    ctx.reference_anchor = (live_anchor && live_anchor->valid) ?
      live_anchor->center :
      0.5 * (ctx.refined->bottom + ctx.refined->neck);
    const Eigen::Isometry3d entry_tool_pose = entryToolPose(
      ctx.refined->entry, ctx.refined->axis,
      ctx.target->initial_pose.linear().col(0));
    ctx.entry_tip_pose = entry_tool_pose * tip_from_tool->inverse();
    ctx.travel_m = insertionTravel(*ctx.refined);
    const auto quality = qualitySnapshot();
    RCLCPP_INFO(
      get_logger(),
      "接触几何（核对方向） target=%s entry=[%.3f %.3f %.3f] axis=[%.3f %.3f %.3f] "
      "travel=%.3fm axis_angle_deg=%.2f",
      ctx.target_id.c_str(),
      ctx.refined->entry.x(), ctx.refined->entry.y(), ctx.refined->entry.z(),
      ctx.refined->axis.x(), ctx.refined->axis.y(), ctx.refined->axis.z(),
      ctx.travel_m, quality.axis_angle_deg);
    if (grasp_hyp_pub_) {
      peach_interfaces::msg::GraspHypothesis hyp;
      hyp.header.stamp = now();
      hyp.header.frame_id = params_.frames.base;
      hyp.target_id = ctx.target_id;
      hyp.entry_pose = tf2::toMsg(ctx.entry_tip_pose);
      hyp.travel_m = static_cast<float>(ctx.travel_m);
      hyp.rank_score = 1.0f;
      grasp_hyp_pub_->publish(hyp);
    }
    return true;
  }
  return failStage(
    ctx, ExecuteTarget::Result::SKIPPED_QUALITY,
    FailureCode::DEGRADED_CONTACT_FORBIDDEN,
    ctx.pregrasp_only ?
    ("预抓取缺少融合几何: " + gate.reason) :
    ("GraspDecision.allowed=false，禁止降级接触: " + gate.reason));
}

bool ManipulationSkillsNode::stageReconfirmTarget(CycleContext & ctx)
{
  // 抓取前再确认（2.7-RECONFIRM）：FinalizeAndValidate 已产出入口几何
  // （ctx.refined/entry_tip_pose/travel_m，周期"黑板"字段），但 finalize 耗时
  // 必然超过观测新鲜度窗口，接触段前必须用最新观测复核目标没飘走、没换
  // 身份、没在风里持续摆动，再决定放行接触。
  //
  // 回退开关（验证期遗留）：allow_stale_anchor=true 时退化为旧"按静态锚点
  // 继续"行为，直接放行；false（默认）时新鲜度由本阶段单点把关。
  if (params_.grasp.allow_stale_anchor) {
    RCLCPP_WARN(
      get_logger(),
      "grasp.allow_stale_anchor=true：跳过抓取前再确认，按静态目标锚点继续"
      "（旧行为，验证期遗留）");
    return true;
  }
  if (!ctx.refined || !ctx.refined->valid) {
    pending_outcome_.store(ExecuteTarget::Result::SKIPPED_QUALITY);
    return failStage(ctx, "再确认无有效入口几何（FinalizeAndValidate 未产出）");
  }
  setState(
    CycleState::RECONFIRM,
    "抓取前再确认：等待新鲜观测复核身份/锚点漂移/摆动平息", ctx.target_id);
  ReconfirmPolicy policy(ReconfirmConfig{
      params_.grasp.reconfirm_tolerance_m,
      static_cast<int>(params_.grasp.reconfirm_max_attempts), false});
  Eigen::Vector3d reference_anchor = ctx.reference_anchor;
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
        // 再确认不等 received_s 新戳：锁定目标已是 OBSERVED 时用当前锚点
        // 判漂移（记忆锚点帧仍刷新 updated_s / tracking_status）。
        got_fresh = waitForFreshCycleTarget(
          ctx.target_id, after_s, remaining_s, false);
      }
      const auto latest = got_fresh ? cycleTargetSnapshot(ctx.target_id) : std::nullopt;
      if (!latest) {
        // 窗口耗尽/目标失效：累计一次超限（摆动持续整个窗口也走本路径）。
        decision = policy.check(ReconfirmSample::exhausted());
        break;
      }
      after_s = latest->updated_s > 0.0 ? latest->updated_s : latest->received_s;
      ReconfirmSample sample;
      sample.fresh = true;
      sample.identity_ok = latest->id == ctx.target_id;
      sample.swinging = latest->swinging;
      sample.anchor = latest->center;
      sample.axis = latest->initial_axis;
      sample.anchor_drift_m = (latest->center - reference_anchor).norm();
      decision = policy.check(sample);
      // PENDING 仅出现在摆动等平息路径：留在本窗口预算内等下一帧复核。
      if (decision.verdict != ReconfirmVerdict::PENDING) {
        break;
      }
      setState(CycleState::RECONFIRM, decision.reason, ctx.target_id);
    }
    if (cancel_requested_.load()) {
      return false;
    }
    if (decision.verdict == ReconfirmVerdict::PASS) {
      return true;
    }
    if (decision.verdict == ReconfirmVerdict::REFINED) {
      // 精化路径：多视 TSDF 入口比单帧检测稳，超容差只计一次超限并再开窗，
      // 不把入口平移到跳变锚点（降级重算链已删除，无单帧平移路径）。
      RCLCPP_WARN(
        get_logger(),
        "%s；保留 TSDF 入口，不按单帧平移", decision.reason.c_str());
      setState(
        CycleState::RECONFIRM,
        decision.reason + "；保留精化入口，等观测回到容差", ctx.target_id);
      continue;
    }
    if (decision.verdict == ReconfirmVerdict::PENDING) {
      // 窗口耗尽计一次超限后重开窗口再试（reason 已含累计次数）。
      RCLCPP_WARN(get_logger(), "%s", decision.reason.c_str());
      setState(CycleState::RECONFIRM, decision.reason, ctx.target_id);
      continue;
    }
    // ABORT：放弃本目标，周期终局 SKIPPED_QUALITY；reason 附最近跟踪状态，
    // 便于区分"观测窗口耗尽"是出视野/深度空洞/跟踪丢失中的哪一类。
    std::string reason = decision.reason;
    const auto latest = cycleTargetSnapshot(ctx.target_id);
    if (latest) {
      reason += "；最近跟踪状态=" + trackingStatusLabel(latest->tracking_status);
      reason += " received_s=" + std::to_string(latest->received_s);
      reason += " updated_s=" + std::to_string(latest->updated_s);
    }
    pending_outcome_.store(ExecuteTarget::Result::SKIPPED_QUALITY);
    return failStage(ctx, reason);
  }
  return false;
}

bool ManipulationSkillsNode::stageReportReady(CycleContext & ctx)
{
  ctx.terminal_state = CycleState::READY_FOR_GRASP;
  ctx.terminal_message =
    "精化质量通过；grasp.enabled=false，未执行接触动作";
  return true;
}

bool ManipulationSkillsNode::stageReportObserveOnly(CycleContext & ctx)
{
  // OBSERVE_ONLY 圆满终态：观察+精化验证段已完成，reason 标 observe_only；
  // PLAN_READY 经 terminalOutcome 映射 SUCCEEDED 上报编排器。
  ctx.terminal_state = CycleState::PLAN_READY;
  ctx.terminal_message =
    "observe_only 周期完成：仅观察与精化验证，未执行靠近/抓取/工具动作";
  return true;
}

bool ManipulationSkillsNode::stageMovePregrasp(CycleContext & ctx)
{
  // 预抓取移动授权（PREGRASP 级）。
  if (!requireStageAuthority(ctx, MotionStage::PREGRASP, "到预抓取")) {
    return false;
  }
  setState(CycleState::MTC_APPROACH_INSERT, "拍照位再最短路径到预抓取", ctx.target_id);
  if (!ctx.refined || !grasp_task_) {
    return failStage(ctx, "预抓取无入口几何");
  }
  // 观察停在 look-at。从该姿态直接 LIN/PTP 到预抓取现场常无 IK
  // （08-28 G PTP 0/1；08-31 1405 LIN NO_IK）。拍照位是已知可达的自由空间点。
  std::string photo_msg;
  bool from_photo = motion_ && motion_->goToPhotoPose(
    params_.photo_pose_named_target, execution_enabled_.load(), photo_msg);
  if (from_photo) {
    RCLCPP_INFO(get_logger(), "预抓取从拍照位出发: %s", photo_msg.c_str());
  } else {
    RCLCPP_WARN(
      get_logger(),
      "预抓取未回到拍照位，从当前位规划: %s", photo_msg.c_str());
  }
  const FruitCapsule fruit = fruitCapsuleFor(*ctx.refined);
  startContactGuard();
  auto result = grasp_task_->moveToPregrasp(
    ctx.entry_tip_pose, ctx.refined->axis, fruit, true);
  stopContactGuard();
  if (contactAbortSuspected()) {
    motion_->clearPhotoApproach();
    return failStage(
      ctx, ExecuteTarget::Result::FAILED, FailureCode::SLEEVE_PLAN_FAILED,
      "疑似硬接触（接触检测止损）：接近已取消，须现场确认后 ACK");
  }
  if (!result.success && !result.execution_started && motion_ && !from_photo) {
    if (motion_->goToPhotoPose(
        params_.photo_pose_named_target, execution_enabled_.load(), photo_msg))
    {
      from_photo = true;
      RCLCPP_WARN(
        get_logger(),
        "当前位到预抓取失败（%s），已回拍照位再规划: %s",
        result.reason.c_str(), photo_msg.c_str());
      result = grasp_task_->moveToPregrasp(
        ctx.entry_tip_pose, ctx.refined->axis, fruit, true);
    }
  }
  if (!result.success) {
    if (result.execution_started && motion_) {
      motion_->clearPhotoApproach();
    }
    return failStage(
      ctx, ExecuteTarget::Result::SKIPPED_UNREACHABLE,
      FailureCode::SLEEVE_PLAN_FAILED, "到预抓取失败: " + result.reason);
  }
  if (motion_) {
    if (from_photo) {
      auto approach = grasp_task_->lastApproachTrajectory();
      if (!approach.points.empty()) {
        motion_->rememberPhotoApproach(std::move(approach));
      } else {
        motion_->clearPhotoApproach();
      }
    } else {
      motion_->clearPhotoApproach();
    }
  }
  if (result.execution_started) {
    contact_recovery_required_.store(true);
  }
  ctx.completion_level = std::max(
    ctx.completion_level, ExecuteTarget::Result::LEVEL_PREGRASP_REACHED);
  // CK_AT_STAGING 待 GraspTask 增设 staging 到位回调后补记（staging PTP 与
  // 轴向 LIN 在 moveToPregrasp 单调用内完成，2b 层不可区分）。
  markCheckpoint(ExecuteTarget::Goal::CK_AT_PREGRASP, "到预抓取");
  return true;
}

bool ManipulationSkillsNode::stageVerifyPregrasp(CycleContext & ctx)
{
  setState(CycleState::RECONFIRM, "预抓取停稳验证（不 SetIO）", ctx.target_id);
  if (!ctx.refined || !ctx.refined->valid) {
    return failStage(
      ctx, ExecuteTarget::Result::SKIPPED_QUALITY,
      FailureCode::PREGRASP_RESIDUAL, "预抓取验证无精化几何");
  }
  ctx.pregrasp_msg.target_id = ctx.target_id;
  ctx.pregrasp_msg.tool_profile_id = params_.tool.profile_id;
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    const auto first_axis = motion_->lookupTransform(params_.frames.base, "tool_axis");
    const auto first_mouth = motion_->lookupTransform(params_.frames.base, "sleeve_mouth");
    const auto first_cut = motion_->lookupTransform(params_.frames.base, "cutting_plane");
    std::this_thread::sleep_for(200ms);
    const auto second_axis = motion_->lookupTransform(params_.frames.base, "tool_axis");
    const auto second_mouth = motion_->lookupTransform(params_.frames.base, "sleeve_mouth");
    const auto second_cut = motion_->lookupTransform(params_.frames.base, "cutting_plane");
    if (!first_axis || !second_axis || !first_mouth || !second_mouth ||
      !first_cut || !second_cut)
    {
      return failStage(
        ctx, ExecuteTarget::Result::SKIPPED_QUALITY,
        FailureCode::EXACT_TF_MISSING,
        "预抓取缺少 tool_axis/sleeve_mouth/cutting_plane TF");
    }
    // 残差计算与三阈值判定在 PregraspResidualChecker 纯核（W5-4）；
    // 门限来自 yaml grasp.pregrasp_residual.*（原为 1.5°/2.0°/0.003 硬编码，
    // 部署默认同值），axial 只记录不判定。
    PregraspPoseSample first_sample;
    first_sample.tool_axis = *first_axis;
    first_sample.sleeve_mouth = *first_mouth;
    first_sample.cutting_plane = *first_cut;
    PregraspPoseSample second_sample;
    second_sample.tool_axis = *second_axis;
    second_sample.sleeve_mouth = *second_mouth;
    second_sample.cutting_plane = *second_cut;
    const PregraspThresholds residual_thresholds{
      params_.grasp.pregrasp_residual.frame_consistent_deg,
      params_.grasp.pregrasp_residual.axis_deg,
      params_.grasp.pregrasp_residual.lateral_m};
    const ResidualReport residual = evaluatePregraspResidual(
      residual_thresholds, first_sample, second_sample, ctx.refined->bottom,
      ctx.refined->neck, ctx.refined->axis);
    ctx.pregrasp_msg.frames_consistent = residual.consistent;
    ctx.pregrasp_msg.correction_count = attempt;
    ctx.pregrasp_msg.axis_angle_deg = static_cast<float>(residual.angle_deg);
    ctx.pregrasp_msg.lateral_error_m = static_cast<float>(residual.lateral_m);
    ctx.pregrasp_msg.axial_error_m = static_cast<float>(residual.axial_m);
    ctx.pregrasp_msg.needs_correction =
      (!residual.passed) && residual.consistent;
    ctx.pregrasp_msg.passed = residual.passed;
    if (residual.passed) {
      ctx.pregrasp_verified = true;
      ctx.pregrasp_msg.reason = "pregrasp_verified";
      ctx.completion_level = std::max(
        ctx.completion_level,
        completion_level_after_pregrasp_verify(true));
      return true;
    }
    if (ctx.pregrasp_msg.needs_correction && attempt < 2 && grasp_task_) {
      // 修正回路：第一次到位后按原目标重规划是 no-op；必须从缓存取最新
      // 精化快照（身份必须仍钉在本周期目标）重算 entry/pregrasp 目标——
      // 与 movePregrasp 同一几何构造（alignFrameZ 保留滚转；预抓取=entry
      // 沿轴后撤 approach_along_axis_m，0 时预抓取=入口=袋底；由 GraspTask 内部完成）——再做
      // 增量修正，后续接触段同步用修正后几何。
      const auto latest_refined = refinedSnapshot();
      if (!latest_refined || latest_refined->id != ctx.target_id ||
        !latest_refined->valid)
      {
        RCLCPP_WARN(
          get_logger(),
          "预抓取残差超门但无最新精化快照（身份不符/无效），停止修正");
        break;
      }
      Eigen::Isometry3d entry_tip_pose;
      double travel_m = 0.0;
      std::string entry_error;
      if (!contactEntryGeometry(
          *latest_refined, ctx.target->initial_pose,
          entry_tip_pose, travel_m, entry_error))
      {
        return failStage(
          ctx, ExecuteTarget::Result::SKIPPED_QUALITY,
          FailureCode::EXACT_TF_MISSING,
          "预抓取修正重算入口时" + entry_error);
      }
      ctx.refined = latest_refined;
      ctx.entry_tip_pose = entry_tip_pose;
      ctx.travel_m = travel_m;
      setState(CycleState::RECONFIRM, "预抓取短修正（停—看，不 SetIO）", ctx.target_id);
      const auto fix = grasp_task_->moveToPregrasp(
        ctx.entry_tip_pose, ctx.refined->axis, fruitCapsuleFor(*ctx.refined),
        true);
      if (!fix.success) {
        break;
      }
      continue;
    }
    break;
  }
  if (ctx.pregrasp_only) {
    ctx.pregrasp_msg.reason = "pregrasp_residual_hold";
    ctx.pregrasp_msg.failure_code = FailureCode::PREGRASP_RESIDUAL;
    ctx.completion_level = std::max(
      ctx.completion_level,
      completion_level_after_pregrasp_verify(false));
    RCLCPP_WARN(
      get_logger(),
      "PREGRASP_ONLY 残差未过门仍停在预抓取 angle=%.2fdeg lateral=%.4fm",
      ctx.pregrasp_msg.axis_angle_deg,
      ctx.pregrasp_msg.lateral_error_m);
    return true;
  }
  pending_outcome_.store(ExecuteTarget::Result::SKIPPED_QUALITY);
  ctx.failure_code = FailureCode::PREGRASP_RESIDUAL;
  ctx.pregrasp_msg.failure_code = FailureCode::PREGRASP_RESIDUAL;
  ctx.pregrasp_msg.reason = "pregrasp_residual";
  return failStage(ctx, "预抓取残差未过门");
}

bool ManipulationSkillsNode::stageHoldPregrasp(CycleContext & ctx)
{
  // PREGRASP_ONLY 定位门：停在预抓取，不 PTP harvest_stow、不套入、不 SetIO。
  // 过程线保持靠近（不要写成 RECOVERY_REQUIRED/FAILED）。本阶段只在执行
  // 分支可达（execution 已使能且预抓取已执行/到位），recovery 旗标直接置位：
  // 调度 ACK 前不得 Survey / 下一颗。
  setState(
    CycleState::MTC_APPROACH_INSERT,
    "PREGRASP_ONLY：停在预抓取，不回 harvest_stow，未套入未 SetIO",
    ctx.target_id);
  contact_recovery_required_.store(true);
  RCLCPP_INFO(
    get_logger(),
    "PREGRASP_ONLY 停在预抓取；ACK 后再 Survey / 派下一颗");
  return true;
}

bool ManipulationSkillsNode::stagePlanSleeveAndReverseRetreat(CycleContext & ctx)
{
  // 套入前授权（CONTACT 级）：GraspDecision 复检不通过 → skipped_quality。
  if (!requireStageAuthority(ctx, MotionStage::CONTACT, "套入预规划")) {
    return false;
  }
  setState(CycleState::PREVIEW_CONTACT_PLANNING, "预规划套入与反向撤退", ctx.target_id);
  if (!ctx.refined || !grasp_task_) {
    return failStage(ctx, "套入预规划无几何");
  }
  const auto result = grasp_task_->previewFullContact(
    ctx.entry_tip_pose, ctx.refined->axis, ctx.travel_m,
    fruitCapsuleFor(*ctx.refined));
  if (!result.success) {
    return failStage(
      ctx, ExecuteTarget::Result::SKIPPED_UNREACHABLE,
      FailureCode::SLEEVE_PLAN_FAILED, "套入/撤退预规划失败: " + result.reason);
  }
  ctx.sleeve_planned = true;
  markCheckpoint(ExecuteTarget::Goal::CK_SLEEVE_PLANNED, "套入与反向撤退预规划过门");
  return true;
}

bool ManipulationSkillsNode::stageSleeveLinear(CycleContext & ctx)
{
  // 套入执行前授权（CONTACT 级）：GraspDecision 复检不通过 → skipped_quality。
  if (!requireStageAuthority(ctx, MotionStage::CONTACT, "沿轴套入")) {
    return false;
  }
  setState(CycleState::MTC_APPROACH_INSERT, "沿轴 LIN 套入", ctx.target_id);
  if (grasp_task_) {
    grasp_task_->setContactAcm(ctx.target_id, ContactAcmStage::Sleeve);
  }
  if (!ctx.sleeve_planned || !grasp_task_ || !ctx.refined) {
    return failStage(ctx, "套入前未完成正反向预规划");
  }
  startContactGuard();
  const auto result = grasp_task_->sleeveLinear(
    ctx.refined->axis, ctx.travel_m, true);
  stopContactGuard();
  if (result.execution_started) {
    contact_recovery_required_.store(true);
  }
  if (contactAbortSuspected()) {
    pending_outcome_.store(ExecuteTarget::Result::FAILED);
    ctx.failure_code = FailureCode::SLEEVE_PLAN_FAILED;
    ctx.sleeve_partial = true;
    return failStage(ctx, "疑似硬接触（接触检测止损）：套入已取消，须现场确认后 ACK");
  }
  if (!result.success) {
    pending_outcome_.store(
      result.execution_started ?
      ExecuteTarget::Result::FAILED : ExecuteTarget::Result::SKIPPED_UNREACHABLE);
    ctx.failure_code = FailureCode::SLEEVE_PLAN_FAILED;
    ctx.sleeve_partial = result.execution_started;
    return failStage(ctx, "沿轴套入失败: " + result.reason);
  }
  ctx.sleeve_partial = false;
  ctx.completion_level = std::max(
    ctx.completion_level,
    ExecuteTarget::Result::LEVEL_SLEEVE_COMPLETED);
  markCheckpoint(ExecuteTarget::Goal::CK_SLEEVED, "沿轴套入到位");
  return true;
}

bool ManipulationSkillsNode::stageActuateCutter(CycleContext & ctx)
{
  if (ctx.pregrasp_only) {
    return failStage(ctx, "PREGRASP_ONLY 不得 SetIO");
  }
  if (!tool_enabled_.load()) {
    setState(CycleState::ACTUATE_TOOL, "tool.enabled=false，跳过末端 IO", ctx.target_id);
    ctx.cut_command_accepted = false;
    ctx.cut_confirmed = false;
    return true;
  }
  // 剪切授权（TOOL 级）：GraspDecision 复检不通过 → skipped_quality；
  // SetIO 下发前的再兜底在 commandToolClose（同一授权矩阵）。
  if (!requireStageAuthority(ctx, MotionStage::TOOL, "末端工具剪切")) {
    return false;
  }
  setState(CycleState::ACTUATE_TOOL, "触发末端工具剪切", ctx.target_id);
  std::string reason;
  ToolCommandContext command;
  command.run_id = ctx.target_id;
  command.target_id = ctx.target_id;
  command.contact_transaction_id = ctx.contact_transaction_id;
  if (!tool_actuator_.arm(command, reason) || !tool_actuator_.sendCut(reason)) {
    tool_actuator_.markUnknown();
    ctx.failure_code = FailureCode::TOOL_STATE_UNKNOWN;
    return failStage(
      ctx, "tool_state_unknown: " + reason + "（不自动撤退、不重发）");
  }
  ctx.cut_command_accepted = true;
  markCheckpoint(ExecuteTarget::Goal::CK_CUT_ACCEPTED, "SetIO 受理（≠切断）");
  ctx.completion_level = std::max(
    ctx.completion_level,
    ExecuteTarget::Result::LEVEL_CUT_COMMAND_ACCEPTED);
  // 切断确认保持删除态：SetIO ACK ≠ 切断确认（confirmFeedback 预留，接
  // /aubo_io_controller/io_states 工具 DI 后启用）；verifyHarvestOutcome 按
  // tool∧grasp ∧未确认 → FAILED/CUT_FEEDBACK_TIMEOUT 保守判定。
  ctx.cut_confirmed = false;
  return true;
}

bool ManipulationSkillsNode::stageVerifyCut(CycleContext & ctx)
{
  if (!tool_enabled_.load()) {
    ctx.cut_confirmed = false;
    return true;
  }
  if (!ctx.cut_command_accepted) {
    return failStage(
      ctx, ExecuteTarget::Result::FAILED,
      FailureCode::CUT_COMMAND_FAILED, "无 CUT_COMMAND_ACCEPTED");
  }
  return true;
}

bool ManipulationSkillsNode::stageExecuteReservedReverseRetreat(CycleContext & ctx)
{
  setState(CycleState::MTC_RETREAT, "MTC 沿插入反方向保持直线撤离", ctx.target_id);
  if (grasp_task_) {
    grasp_task_->setContactAcm(ctx.target_id, ContactAcmStage::Retreat);
  }
  // 撤离一律从实际位姿规划（部分套入不逆播完整名义轨迹；ReverseNominal
  // 死分支与恒 WARN 已随 retreat_policy.hpp 删除，W5-12）。
  // 撤离授权经 GraspTask retreat 门（Active∧robotReady∧!cancel∧execution∧
  // grasp，无决策复检——插入后目标常被遮挡/收割后决策翻转，撤离不依赖视觉）。
  const auto result = grasp_task_->retreat(
    ctx.refined->axis, ctx.travel_m + params_.moveit.mtc_approach_along_axis_m, true);
  if (!result.success) {
    ctx.failure_code = FailureCode::RETREAT_FAILED;
    return failStage(ctx, "MTC 抓取后撤离失败，需要人工处理: " + result.reason);
  }
  contact_recovery_required_.store(false);
  ctx.retreat_confirmed = true;
  ctx.completion_level = std::max(
    ctx.completion_level,
    ExecuteTarget::Result::LEVEL_RETREAT_CONFIRMED);
  markCheckpoint(ExecuteTarget::Goal::CK_RETREATED, "原路撤回预抓取");
  return true;
}

bool ManipulationSkillsNode::stageReturnHarvestStow(CycleContext & ctx)
{
  // 回收纳位是接触后的自由空间转移（TRANSIT 级：不再做决策复检）。
  if (!requireStageAuthority(ctx, MotionStage::TRANSIT, "返回 harvest_stow")) {
    return false;
  }
  setState(CycleState::MTC_RETREAT, "返回 harvest_stow", ctx.target_id);
  if (!motion_) {
    return failStage(ctx, "MoveIt 尚未初始化");
  }
  const std::string stow = params_.harvest_stow_named_target.empty() ?
    params_.photo_pose_named_target : params_.harvest_stow_named_target;
  std::string message;
  if (!motion_->goToPhotoPose(stow, execution_enabled_.load(), message)) {
    return failStage(ctx, "返回 harvest_stow 失败: " + message);
  }
  contact_recovery_required_.store(false);
  markCheckpoint(ExecuteTarget::Goal::CK_STOWED, "回收纳位");
  return true;
}

bool ManipulationSkillsNode::stageVerifyHarvestOutcome(CycleContext & ctx)
{
  const bool harvest_ok = harvestConfirmed(ctx.cut_confirmed, ctx.retreat_confirmed);
  if (tool_enabled_.load() && grasp_enabled_.load() && !harvest_ok) {
    pending_outcome_.store(ExecuteTarget::Result::FAILED);
    ctx.failure_code = ctx.retreat_confirmed ?
      FailureCode::CUT_FEEDBACK_TIMEOUT : FailureCode::RETREAT_FAILED;
    return failStage(ctx, "切断或撤退未确认，不得记采摘成功");
  }
  if (harvest_ok) {
    ctx.completion_level = ExecuteTarget::Result::LEVEL_HARVEST_CONFIRMED;
  }
  return true;
}

bool ManipulationSkillsNode::stageCompleteTarget(CycleContext & ctx)
{
  // 账本由 peach_supervisor 按 ExecuteTarget 终态写入；此处只落周期终局。
  ctx.terminal_state = CycleState::SUCCEEDED;
  if (ctx.pregrasp_only) {
    ctx.terminal_message =
      "PREGRASP_ONLY：停在预抓取，未套入未 SetIO；ACK 后再 Survey";
  } else if (ctx.cut_confirmed && ctx.retreat_confirmed) {
    ctx.terminal_message = "切断与撤退均已确认";
  } else {
    ctx.terminal_message =
      "周期完成；tool.enabled=false 或未宣称采摘成功";
  }
  return true;
}

void ManipulationSkillsNode::publishViewMarkers(
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
    marker.header.frame_id = params_.frames.base;
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

}  // namespace peach_arm
