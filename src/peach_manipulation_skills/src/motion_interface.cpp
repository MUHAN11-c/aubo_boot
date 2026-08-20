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
// 节点侧运动相关薄壳：安全门样本组装、Trigger 调用、工具 IO、接触轨迹预览
// 与 go_to_photo_pose 服务回调（周期互斥/recovery 守卫/响应投影）。
// MoveItMotionInterface 实现体见 motion_interface_impl.cpp（独立编译单元，
// 供契约测试直接链接）。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include "approach_grasp_node_impl.hpp"
#include "eigen_conversions.hpp"

namespace peach_manipulation_skills
{

void ApproachGraspNode::onRobotStatus(
  const aubo_msgs::msg::RobotStatus::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(robot_mutex_);
  robot_status_ = *message;
  robot_status_received_ = now();
  robot_status_valid_ = true;
}

double ApproachGraspNode::insertionTravel(const CachedRefined & refined) const
{
  // suggested_travel_m 由感知/重建端结合工具几何给出，是跨包行程契约。
  // 仅为兼容旧记录或异常消息，字段无效时才按几何距离与颈部余量回退。
  double travel = refined.suggested_travel_m;
  if (!std::isfinite(travel) || travel <= 0.0) {
    travel = (refined.neck - refined.entry).norm() - neck_margin_m_;
  }
  return std::clamp(travel, minimum_travel_m_, maximum_travel_m_);
}

bool ApproachGraspNode::safetyReady(std::string & reason)
{
  RobotStatusSample sample;
  {
    std::lock_guard<std::mutex> lock(robot_mutex_);
    sample.received = robot_status_valid_;
    sample.received_s = robot_status_received_.seconds();
    sample.e_stopped = robot_status_.e_stopped != 0;
    sample.in_error = robot_status_.in_error != 0;
    sample.drives_powered = robot_status_.drives_powered != 0;
    sample.motion_possible = robot_status_.motion_possible != 0;
  }
  return safety_gate_->robotReady(sample, reason);
}

bool ApproachGraspNode::cycleTargetReady(
  const std::string & target_id, std::string & reason)
{
  // 安全门样本按周期生效目标取数（同 bt_nodes.cpp cycleTargetSnapshot 的
  // 分流理由）：OBSERVE_ONLY 周期取 goal 目标的锁定集锚点样本（目标门
  // 判其身份/有效性/新鲜度），其余周期取感知 selected 样本。
  const TargetGateSample sample =
    !cycle_target_id_.empty() ?
    cache_.lockedTargetGateSample(target_id) : cache_.targetGateSample();
  return safety_gate_->targetReady(sample, target_id, reason);
}

bool ApproachGraspNode::commandToolClose()
{
  // 工具 IO 是运动类输出（A8）：非 Active 拒绝（纵深防御——正常路径下周期
  // 根本不会在非 Active 启动，此处兜底 deactivate 竞态）。
  std::string motion_reason;
  if (!motionOutputAllowed(motion_reason)) {
    setState(CycleState::FAILED, "工具 IO 被拒绝: " + motion_reason);
    return false;
  }
  if (!tool_enabled_.load()) {
    setState(CycleState::FAILED, "grasp.enabled=true 但 tool.enabled=false");
    return false;
  }
  if (!tool_io_client_->wait_for_service(
      std::chrono::duration<double>(service_timeout_s_)))
  {
    setState(CycleState::FAILED, "末端工具 set_io 服务不可用");
    return false;
  }
  auto request = std::make_shared<aubo_msgs::srv::SetIO::Request>();
  request->fun = static_cast<int8_t>(tool_io_fun_);
  request->pin = static_cast<int8_t>(tool_io_pin_);
  request->state = static_cast<float>(tool_close_state_);
  auto future = tool_io_client_->async_send_request(request);
  if (future.wait_for(std::chrono::duration<double>(service_timeout_s_)) !=
    std::future_status::ready || !future.get()->success)
  {
    setState(CycleState::FAILED, "末端工具关闭命令失败");
    return false;
  }
  return true;
}

void ApproachGraspNode::onPreviewApproachInsert(
  const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
{
  const ScopedTimer timer(get_logger(), "preview_approach_insert", &callback_timing_);
  previewContact(false, response);
}

void ApproachGraspNode::onPreviewFullContact(
  const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
{
  const ScopedTimer timer(get_logger(), "preview_full_contact", &callback_timing_);
  previewContact(true, response);
}

void ApproachGraspNode::previewContact(
  bool include_retreat, Trigger::Response::SharedPtr response)
{
  // 接触轨迹预览是运动类入口（A8）：即便只规划不执行，也非 Active 不放行。
  std::string motion_reason;
  if (!motionOutputAllowed(motion_reason)) {
    response->success = false;
    response->message = motion_reason;
    return;
  }
  bool expected = false;
  if (!running_.compare_exchange_strong(expected, true)) {
    response->success = false;
    response->message = "已有规划或执行周期正在运行";
    return;
  }
  // 预览周期同样在每周期开始时重置终局分级。
  pending_outcome_.store(ExecuteTarget::Result::FAILED);
  // 预览周期（含 action PREVIEW 模式）同样计时：approach_insert 段覆盖
  // PREVIEW_CONTACT_PLANNING（见 stage_timing.hpp 投影注释）。
  startCycleTiming();
  const auto finish = [this, &response](
    bool success, CycleState state, const std::string & message)
    {
      // 先落终态再解除 running，保证 action 侧读到的一定是终态而非中间态。
      setState(state, message);
      running_.store(false);
      publishState();
      response->success = success;
      response->message = message;
    };

  const GateResult gate = quality_gate_->readyToPreviewContact(qualitySnapshot());
  const auto target = targetSnapshot();
  const auto refined = refinedSnapshot();
  if (!gate.allowed) {
    finish(false, CycleState::PREVIEW_FAILED, "接触轨迹预览质量门失败: " + gate.reason);
    return;
  }
  if (!target || !refined || target->id != refined->id ||
    graspDecisionTargetSnapshot() != target->id)
  {
    finish(false, CycleState::PREVIEW_FAILED, "接触轨迹预览目标 ID 或精化几何不一致");
    return;
  }

  // preview 隔离（重构协议阶段 B）：预览不是周期，绝不得写 cycle_target_id_
  // （周期身份钉，由 action 受理/btPrepareCycle 管理）——否则预览过的目标 ID
  // 会污染后续手动周期：btPrepareCycle 的 goal 钉死校验会把"新 selected ≠
  // 预览残留 ID"误判为目标身份变更而失败。本函数内一律用局部 target/refined。
  Eigen::Isometry3d entry_tool_pose = Eigen::Isometry3d::Identity();
  entry_tool_pose.translation() = refined->entry;
  entry_tool_pose.linear() = ViewPlanner::toolOrientation(
    refined->axis, target->initial_pose.linear().col(0));
  const auto tip_from_tool = motion_->lookupTransform(tip_frame_, tool_frame_);
  if (!tip_from_tool) {
    finish(false, CycleState::PREVIEW_FAILED, "无法取得 tip 到 tool 的变换");
    return;
  }
  const Eigen::Isometry3d entry_tip_pose =
    entry_tool_pose * tip_from_tool->inverse();
  const double travel = insertionTravel(*refined);

  setState(
    CycleState::PREVIEW_CONTACT_PLANNING,
    include_retreat ? "MTC 只规划：到入口、直线插入、同轴撤离" :
    "MTC 只规划：到入口、直线插入");
  const GraspTaskResult result = include_retreat ?
    grasp_task_->previewFullContact(entry_tip_pose, refined->axis, travel) :
    grasp_task_->approachAndInsert(entry_tip_pose, refined->axis, travel, false);
  if (!result.success) {
    finish(false, CycleState::PREVIEW_FAILED, "MTC 接触轨迹预览失败: " + result.reason);
    return;
  }
  finish(
    true, CycleState::PREVIEW_READY,
    include_retreat ?
    "完整接触轨迹已发布到 RViz；仅规划，未发送任何运动" :
    "入口与插入轨迹已发布到 RViz；仅规划，未发送任何运动");
}

// 全局拍照位姿：批次编排器在发现/复扫轮次开始前调用，把机械臂送到 SRDF
// 命名状态（默认 global_photo_pose）。与 previewContact 同为 executor 回调内
// 同步规划（execution 使能时含执行），单次调用可能占用数秒——A7 起三个长
// 规划服务（preview_approach_insert / preview_full_contact / go_to_photo_pose）
// 独占 planning_callback_group_（独立互斥组），规划期间只组内排队，默认组的
// 订阅/快捷服务/action 回调照常调度。规划/执行体在 MotionInterfaceBase 实现
// 内，本回调只保留周期互斥、recovery 守卫与响应投影。
void ApproachGraspNode::onGoToPhotoPose(
  const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
{
  const ScopedTimer timer(get_logger(), "go_to_photo_pose", &callback_timing_);
  // 运动输出权限绑定 Active 态（A8）：非 Active 拒绝并给出明确原因。
  std::string motion_reason;
  if (!motionOutputAllowed(motion_reason)) {
    response->success = false;
    response->message = motion_reason;
    return;
  }
  if (!motion_) {
    response->success = false;
    response->message = "MoveIt 尚未初始化";
    return;
  }
  if (contact_recovery_required_.load()) {
    response->success = false;
    response->message =
      "上一周期可能停在接触区；现场人工撤离并确认后调用 acknowledge_recovery";
    return;
  }
  bool expected = false;
  if (!running_.compare_exchange_strong(expected, true)) {
    response->success = false;
    response->message = "已有靠近/抓取周期正在运行，不能移动拍照位姿";
    return;
  }
  // 提前返回路径统一经 finish 解除占用并刷新状态投影。
  const auto finish = [this, &response](bool success, const std::string & message) {
      running_.store(false);
      publishState();
      response->success = success;
      response->message = message;
    };
  std::string message;
  const bool success = motion_->goToPhotoPose(
    photo_pose_named_target_, execution_enabled_.load(), message);
  finish(success, message);
}

}  // namespace peach_manipulation_skills
