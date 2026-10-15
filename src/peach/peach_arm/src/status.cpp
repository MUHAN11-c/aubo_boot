// 功能：技能状态、诊断和完成证据投影。此编译单元不发送运动或工具命令。
#include "peach_arm/manipulation_skills_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

namespace peach_arm
{

void ManipulationSkillsNode::setState(
  CycleState state, const std::string & message, const std::string & target_id)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    // 枚举是唯一权威状态；字符串仅投影进 JSON 供 dashboard/web 只读消费。
    current_state_ = state;
    // 阶段耗时埋点：cycle_state_ 变更点即计时切换点（周期未启动时为 no-op）。
    stage_timer_.onStateChange(state, StageTimer::Clock::now());
    state_json_ = {
      {"state", toString(state)},
      {"message", message},
      {"running", running_.load()},
      {"execution_enabled", execution_enabled_.load()},
      {"execution_armed", execution_armed_.load()},
      {"grasp_enabled", grasp_enabled_.load()},
      {"contact_recovery_required", contact_recovery_required_.load()},
      {"target_id", target_id},
    };
    const QualitySnapshot snapshot = qualitySnapshot();
    state_json_["quality"] = {
      {"selected_target_id", snapshot.selected_target_id},
      {"reconstruction_target_id", snapshot.reconstruction_target_id},
      {"refined_target_id", snapshot.refined_target_id},
      {"captured_views", snapshot.captured_views},
      {"station_count", snapshot.station_count},
      {"max_baseline_deg", snapshot.max_baseline_deg},
      {"mean_nearest_baseline_deg", snapshot.mean_nearest_baseline_deg},
      {"mean_depth_ratio", snapshot.mean_depth_ratio},
      {"refined_rmse_m", snapshot.refined_rmse_m},
      {"refined_inlier_ratio", snapshot.refined_inlier_ratio},
      {"axis_angle_deg", snapshot.axis_angle_deg},
      {"grasp_allowed", snapshot.grasp_allowed},
    };
  }
  publishState();
  // 失败拒因锁存（P1 观测性）：瞬时 reason 此前只在日志，诊断现在可查
  if (state == CycleState::FAILED || state == CycleState::PREVIEW_FAILED) {
    std::lock_guard<std::mutex> denial_lock(last_denial_mutex_);
    last_denial_ = message;
    last_denial_mono_s_ = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  // 失败/须人确认的状态必须以 WARN 起步（P0）：INFO 过滤器下业务失败不可见
  if (state == CycleState::FAILED || state == CycleState::PREVIEW_FAILED ||
    state == CycleState::RECOVERY_REQUIRED)
  {
    RCLCPP_WARN(
      get_logger(), "[%s] %s", toString(state).c_str(), message.c_str());
  } else {
    RCLCPP_INFO(
      get_logger(), "[%s] %s", toString(state).c_str(), message.c_str());
  }
}

void ManipulationSkillsNode::publishState()
{
  // Unconfigured/cleanup 后 status_pub_ 为空（接口尚未创建或已释放）、
  // Inactive 下发布者未激活：状态投影仅内存更新，跳过发布。
  if (!status_pub_ || !status_pub_->is_activated()) {
    return;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_json_["running"] = running_.load();
  state_json_["execution_armed"] = execution_armed_.load();
  state_json_["contact_recovery_required"] = contact_recovery_required_.load();
  // 回调耗时诊断投影（2.16-5）：复用 ~/status 既有通道，不新增话题。
  state_json_["callback_timing"] = callback_timing_.toJson();
  std_msgs::msg::String message;
  message.data = state_json_.dump();
  status_pub_->publish(message);
}

// ---- 诊断双轨任务（W5-10）----
// 级别口径：OK=正常观测；WARN=数据陈旧/降级（周期仍可自行判停）；
// ERROR=通道断流/授权异常。只读投影，不做任何控制决策；不与 ~/status
// 互相替代（web 消费端不动）。

void ManipulationSkillsNode::reportStreamDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  using Status = diagnostic_msgs::msg::DiagnosticStatus;
  const double now_s = now().seconds();
  const double last_arrival = frame_timeouts_.lastArrivalS();
  const double arrival_age_s = last_arrival > 0.0 ? now_s - last_arrival : -1.0;
  const double ema_s = frame_timeouts_.frameIntervalEmaS();
  status.add("frame_interval_ema_s", ema_s);
  status.add("last_arrival_age_s", arrival_age_s);
  status.add("frame_wait_s", frame_timeouts_.frameWaitS());
  status.add("target_max_age_s", frame_timeouts_.targetMaxAgeS());
  status.add("refined_wait_s", frame_timeouts_.refinedWaitS(
      cache_.qualitySnapshot().reconstruction_state == "COLLECTING"));
  // base<-camera 最新 TF 可用性（0 超时探测，不阻塞）：视点生成/接近几何
  // 都依赖这条链，不可用即 ERROR。先核帧名是否已在树上（camera_enabled
  // =false 时相机帧不存在），避免 canTransform 对无效帧名刷 tf2 WARN。
  const auto frames = tf_buffer_.getAllFrameNames();
  const bool base_known =
    std::find(frames.begin(), frames.end(), params_.frames.base) != frames.end();
  const bool camera_known =
    std::find(frames.begin(), frames.end(), params_.frames.camera) != frames.end();
  const bool tf_ok = base_known && camera_known &&
    tf_buffer_.canTransform(
    params_.frames.base, params_.frames.camera, tf2::TimePointZero,
    std::chrono::milliseconds(0));
  status.add("tf_base_to_camera_ok", tf_ok ? 1 : 0);
  if (!camera_known) {
    status.summary(Status::WARN, "相机 TF 帧未发布（camera_enabled=false 或外参未起）");
    return;
  }
  if (!tf_ok) {
    status.summary(Status::ERROR, "base<-camera TF 不可用");
    return;
  }
  if (arrival_age_s < 0.0) {
    status.summary(Status::WARN, "尚未收到目标观测帧");
    return;
  }
  if (arrival_age_s > frame_timeouts_.targetMaxAgeS()) {
    status.summary(Status::WARN, "目标观测流陈旧");
    return;
  }
  status.summary(Status::OK, "观测流与 TF 正常");
}

void ManipulationSkillsNode::reportTargetCacheDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  using Status = diagnostic_msgs::msg::DiagnosticStatus;
  const QualitySnapshot snapshot = qualitySnapshot();
  status.add("reconstruction_state", snapshot.reconstruction_state);
  status.add("data_age_s", snapshot.data_age_s);
  status.add("captured_views",
    static_cast<int>(snapshot.captured_views));
  status.add("station_count", static_cast<int>(snapshot.station_count));
  status.add("selected_target_id", snapshot.selected_target_id);
  status.add("refined_target_id", snapshot.refined_target_id);
  const bool id_consistent =
    snapshot.selected_target_id.empty() ||
    snapshot.refined_target_id.empty() ||
    snapshot.selected_target_id == snapshot.refined_target_id;
  if (!id_consistent) {
    status.summary(Status::WARN, "selected 与精化目标 ID 不一致");
    return;
  }
  status.summary(Status::OK, "目标缓存正常");
}

void ManipulationSkillsNode::reportCallbackTimingDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  using Status = diagnostic_msgs::msg::DiagnosticStatus;
  // TopN 耗时（按 max_ms 降序取前 5；数值口径与 ~/status 的
  // callback_timing 投影一致——复用同一注册表，不新增计时点）。
  auto entries = callback_timing_.snapshot();
  std::vector<std::pair<std::string, CallbackTimingRegistry::Entry>> top(
    entries.begin(), entries.end());
  std::sort(
    top.begin(), top.end(),
    [](const auto & a, const auto & b) {
      return a.second.max_ms > b.second.max_ms;
    });
  std::size_t shown = 0;
  for (const auto & [label, entry] : top) {
    if (shown++ >= 5U) {
      break;
    }
    status.add(label + ".count", static_cast<int>(entry.count));
    status.add(label + ".last_ms", entry.last_ms);
    status.add(label + ".max_ms", entry.max_ms);
  }
  status.summary(Status::OK, "回调耗时 TopN");
}

void ManipulationSkillsNode::reportContactMonitorDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  using Status = diagnostic_msgs::msg::DiagnosticStatus;
  status.add("enabled", contact_detect_config_.enabled ? 1 : 0);
  status.add("baseline_s", contact_detect_config_.baseline_s);
  status.add("slope_threshold", contact_detect_config_.slope_threshold);
  status.add("spike_threshold", contact_detect_config_.spike_threshold);
  status.add("abort_suspected", contactAbortSuspected() ? 1 : 0);
  std::size_t sample_count = 0;
  {
    std::lock_guard<std::mutex> lock(joint_current_mutex_);
    sample_count = joint_current_samples_.size();
  }
  status.add("current_sample_count", static_cast<int>(sample_count));
  if (contactAbortSuspected()) {
    status.summary(Status::ERROR, "疑似硬接触（已取消执行，须现场确认后 ACK）");
    return;
  }
  status.summary(
    Status::OK, contact_detect_config_.enabled ? "接触检测运行中" : "接触检测关闭（仅缓存）");
}

void ManipulationSkillsNode::reportEnablesDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  using Status = diagnostic_msgs::msg::DiagnosticStatus;
  const char * source = enables_external_ ? "console_broadcast" : "local_params";
  double heartbeat_age_s = -1.0;
  if (enables_external_) {
    heartbeat_age_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - enables_last_beat_).count();
  }
  status.add("source", source);
  status.add("heartbeat_age_s", heartbeat_age_s);
  status.add("heartbeat_timeout_s", enables_heartbeat_timeout_s_);
  status.add("execution_enabled", execution_enabled_.load() ? 1 : 0);
  status.add("grasp_enabled", grasp_enabled_.load() ? 1 : 0);
  status.add("tool_enabled", tool_enabled_.load() ? 1 : 0);
  status.add("execution_armed", execution_armed_.load() ? 1 : 0);
  status.add("motion_output_permitted", motion_output_permitted_.load() ? 1 : 0);
  status.add("contact_recovery_required", contact_recovery_required_.load() ? 1 : 0);
  if (enables_external_ && enables_heartbeat_timeout_s_ > 0.0 &&
    heartbeat_age_s > enables_heartbeat_timeout_s_)
  {
    status.summary(Status::WARN, "操作台使能广播心跳超时（看门狗将回落本地参数）");
    return;
  }
  status.summary(Status::OK, std::string("使能正常（源=") + source + "）");
}

// ---- 观测性 P1 增补任务 ----
// robot_status / tool_chain / moveit_availability / last_denial：补齐
// 「柜侧流健康、工具链、move_group 在场、最近拒因」四个只读投影。

void ManipulationSkillsNode::reportRobotStatusDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  using Status = diagnostic_msgs::msg::DiagnosticStatus;
  const bool required = params_.execution.require_robot_status;
  double age_s = -1.0;
  {
    std::lock_guard<std::mutex> lock(robot_mutex_);
    if (robot_status_mono_s_ > 0.0) {
      age_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count() -
        robot_status_mono_s_;
    }
  }
  status.add("required", required ? 1 : 0);
  status.add("valid", robot_status_valid_ ? 1 : 0);
  status.add("age_s", age_s);
  status.add("contract_timeout_s", robot_status_contract_timeout_s_);
  if (!required) {
    status.summary(Status::OK, "mock：不要求 robot_status");
  } else if (!robot_status_valid_) {
    status.summary(Status::ERROR, "从未收到 robot_status（真机要求在线）");
  } else if (age_s > robot_status_contract_timeout_s_) {
    status.summaryf(
      Status::ERROR, "robot_status 断流 %.1fs > %.1fs", age_s,
      robot_status_contract_timeout_s_);
  } else {
    status.summary(Status::OK, "robot_status 新鲜");
  }
}

void ManipulationSkillsNode::reportToolChainDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  using Status = diagnostic_msgs::msg::DiagnosticStatus;
  const ToolAxes axes = tool_actuator_.axes();
  const auto state = tool_actuator_.state();
  status.add("state", static_cast<int>(state));
  status.add("blade", static_cast<int>(axes.blade));
  status.add("retention", static_cast<int>(axes.retention));
  status.add("payload", static_cast<int>(axes.payload));
  status.add("evidence_source", axes.evidence_source);
  status.add("fault_code", axes.fault_code);
  status.add("tool_enabled", tool_enabled_.load() ? 1 : 0);
  if (!axes.fault_code.empty()) {
    status.summary(Status::WARN, "工具链异常: " + axes.fault_code);
  } else {
    status.summary(Status::OK, "工具状态机正常");
  }
}

void ManipulationSkillsNode::reportMoveItAvailabilityDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  using Status = diagnostic_msgs::msg::DiagnosticStatus;
  const bool mgi_ready = move_group_ != nullptr;
  const bool service_ready = planning_scene_client_ != nullptr &&
    planning_scene_client_->service_is_ready();
  status.add("mgi_initialized", mgi_ready ? 1 : 0);
  status.add("get_planning_scene_ready", service_ready ? 1 : 0);
  if (!mgi_ready) {
    status.summary(Status::ERROR, "MoveGroupInterface 未初始化");
  } else if (!service_ready) {
    status.summary(Status::WARN, "move_group 服务未就绪（仍在启动或已退出）");
  } else {
    status.summary(Status::OK, "move_group 在场");
  }
}

void ManipulationSkillsNode::reportLastDenialDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  using Status = diagnostic_msgs::msg::DiagnosticStatus;
  std::lock_guard<std::mutex> lock(last_denial_mutex_);
  status.add("last_denial", last_denial_);
  if (last_denial_.empty()) {
    status.summary(Status::OK, "无失败记录");
    return;
  }
  double age_s = -1.0;
  if (last_denial_mono_s_ > 0.0) {
    age_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count() -
      last_denial_mono_s_;
  }
  status.add("age_s", age_s);
  status.summary(Status::WARN, "最近失败: " + last_denial_);
}

void ManipulationSkillsNode::startCycleTiming()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  stage_timer_.start(StageTimer::Clock::now());
}

void ManipulationSkillsNode::fillStageDurations(
  const std::shared_ptr<ExecuteTarget::Result> & result)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  // 兜底收口：终态 setState 正常已关闭计时；此处防止异常路径漏收。
  stage_timer_.close(StageTimer::Clock::now());
  for (const auto & entry : stage_timer_.entries()) {
    result->stage_names.push_back(entry.name);
    builtin_interfaces::msg::Duration duration;
    const auto nanos =
      std::chrono::duration_cast<std::chrono::nanoseconds>(entry.elapsed).count();
    duration.sec = static_cast<int32_t>(nanos / 1000000000LL);
    duration.nanosec = static_cast<uint32_t>(nanos % 1000000000LL);
    result->stage_durations.push_back(duration);
  }
}

void ManipulationSkillsNode::fillExecuteResults(
  const std::shared_ptr<ExecuteTarget::Result> & result,
  const CycleContext * ctx)
{
  const bool observe_only = ctx && ctx->observe_only;
  const bool pregrasp_only = ctx && ctx->pregrasp_only;
  const bool succeeded =
    result->outcome == ExecuteTarget::Result::SUCCEEDED;
  result->completion_level = ctx ? ctx->completion_level : 0;
  // M3a：受理期拒单（plan mismatch）发生在 ctx 创建之前，读 pending 成员
  // 把码带出（每 goal 于 executeAction 入口复位）；PREVIEW 模式中断路径
  // 同为 !ctx，但该成员恒 0，行为与原状一致。
  result->failure_code = succeeded ?
    0u : (ctx != nullptr ? ctx->failure_code : pending_accept_failure_code_);
  // W7：顶层 bool 镜像已删；cut/retreat/harvest 证据单源 harvest/verification 块。
  const bool harvest_confirmed = harvestConfirmed(
    ctx && ctx->cut_confirmed, ctx && ctx->retreat_confirmed);
  result->pregrasp = ctx ? ctx->pregrasp_msg : peach_interfaces::msg::PregraspVerification{};
  result->harvest.completion_level = result->completion_level;
  result->harvest.commanded =
    grasp_enabled_.load() && !observe_only && !pregrasp_only;
  result->harvest.confirmed = harvest_confirmed;
  result->harvest.grasped = harvest_confirmed;
  result->harvest.reason = result->reason;
  if (!tool_enabled_.load() && result->harvest.commanded) {
    result->harvest.reason += "；tool.enabled=false，跳过末端 IO";
  }
  if (observe_only || pregrasp_only) {
    result->verification.passed = succeeded;
    result->verification.commanded = false;
    result->verification.confirmed =
      pregrasp_only && ctx->pregrasp_verified;
    result->verification.harvest_confirmed = false;
    result->verification.reason = result->reason;
    result->verification.failure_code = result->failure_code;
  } else {
    // FULL：验证随采摘确认走（M8 卸果预留不随 Result 携带）。
    result->verification.passed = result->harvest.grasped;
    result->verification.commanded = result->harvest.commanded;
    result->verification.confirmed = result->harvest.confirmed;
    result->verification.harvest_confirmed = harvest_confirmed;
    result->verification.reason = result->reason;
    result->verification.failure_code = result->failure_code;
  }
  result->outcome_record.target_id = ctx ? ctx->target_id : std::string();
  result->outcome_record.outcome = result->outcome;
  result->outcome_record.reason = result->reason;
}

}  // namespace peach_arm
