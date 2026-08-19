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
#ifndef PEACH_APPROACH_GRASP__TARGET_CACHE_HPP_
#define PEACH_APPROACH_GRASP__TARGET_CACHE_HPP_

#include <Eigen/Geometry>

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "peach_approach_grasp/quality_gate.hpp"
#include "peach_approach_grasp/safety_gate.hpp"

namespace peach_approach_grasp
{

// 几何有效性：分量全有限且非零向量（原节点匿名命名空间实现的原样搬运）。
inline bool nonzeroFinite(const Eigen::Vector3d & value)
{
  return value.allFinite() && value.norm() > 1.0e-6;
}

// 当前选中目标的初始几何（感知观测侧）。
struct CachedTarget
{
  std::string id;
  std::string harvest_run_id;
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  Eigen::Isometry3d initial_pose{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d initial_axis{Eigen::Vector3d::UnitZ()};
  double suggested_travel_m{0.0};
  double received_s{0.0};  // 接收时刻（秒，与注入时钟同源）
  // 最近一帧（含非 OBSERVED 帧）的诊断透传：target_swinging 摆动旗标与
  // tracking_status 原始枚举值（PeachTargetObservation.msg 常量；255=未知），
  // 供抓取前再确认（2.7-RECONFIRM）的摆动等平息与失败原因文案使用。
  bool swinging{false};
  uint8_t tracking_status{255};
  bool valid{false};
};

// 精化几何（重建侧锁存的最终拟合结果）。
struct CachedRefined
{
  std::string id;
  Eigen::Vector3d entry{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bottom{Eigen::Vector3d::Zero()};
  Eigen::Vector3d neck{Eigen::Vector3d::Zero()};
  Eigen::Vector3d axis{Eigen::Vector3d::UnitZ()};
  double suggested_travel_m{0.0};
  bool valid{false};
};

// updateSelectedTarget 输入：observed 由节点按消息字段判定
// （tracking_status==OBSERVED 且 candidate.status!=REJECT），几何有限性由缓存判定。
struct SelectedTargetUpdate
{
  std::string selected_id;
  std::string harvest_run_id;
  bool observed{false};
  Eigen::Vector3d bottom{Eigen::Vector3d::Zero()};
  Eigen::Vector3d neck{Eigen::Vector3d::Zero()};
  Eigen::Vector3d axis{Eigen::Vector3d::Zero()};
  Eigen::Isometry3d entry_pose{Eigen::Isometry3d::Identity()};
  double suggested_travel_m{0.0};
  // 诊断透传（含义见 CachedTarget）：由节点从 diagnostic_flags/tracking_status 提取。
  bool swinging{false};
  uint8_t tracking_status{255};
};

// updateLockedTargets 单目标输入（阶段 E 残局抬质量能力端）：锁定集中一条
// confirmed 观测（含非 selected 目标）的纯值提取；字段语义与
// SelectedTargetUpdate 一致（observed 判定由节点薄壳完成，含 anchor_from_memory
// 记忆锚点帧不算新鲜观测的排除）。
struct LockedTargetUpdate
{
  std::string target_id;
  bool observed{false};
  Eigen::Vector3d bottom{Eigen::Vector3d::Zero()};
  Eigen::Vector3d neck{Eigen::Vector3d::Zero()};
  Eigen::Vector3d axis{Eigen::Vector3d::Zero()};
  Eigen::Isometry3d entry_pose{Eigen::Isometry3d::Identity()};
  double suggested_travel_m{0.0};
  bool swinging{false};
  uint8_t tracking_status{255};
};

// updateReconstructionDiagnostics 输入：节点解析 diagnostics JSON 后的纯值字段。
struct ReconstructionDiagnosticsUpdate
{
  std::string target_id;
  std::string state{"IDLE"};
  std::size_t captured_views{0};
  double max_baseline_deg{0.0};
  double mean_nearest_baseline_deg{0.0};
  double mean_depth_ratio{0.0};
  std::vector<Eigen::Vector3d> view_directions;
};

// updateRefinedPose 输入：clear=true 表示候选数组为空（清精化缓存）。
struct RefinedPoseUpdate
{
  bool clear{false};
  std::string target_id;
  Eigen::Vector3d entry{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bottom{Eigen::Vector3d::Zero()};
  Eigen::Vector3d neck{Eigen::Vector3d::Zero()};
  Eigen::Vector3d axis{Eigen::Vector3d::Zero()};
  double suggested_travel_m{0.0};
  bool accepted{false};
};

// updateRefinedFitting 输入：is_fruit=true 取球拟合指标，否则取柱拟合指标。
struct RefinedFittingUpdate
{
  std::string target_id;
  bool is_fruit{false};
  double sphere_rms_m{0.0};
  double sphere_inlier_ratio{0.0};
  double cylinder_rms_m{0.0};
  double cylinder_inlier_ratio{0.0};
  bool accepted{false};
};

// 目标数据缓存（纯逻辑，零 ROS）：目标观测/精化位姿/精化指标/抓取决策四源的
// ID 一致性调和与快照访问，外加锁定集锚点缓存（id→几何，供 OBSERVE_ONLY 残局
// 抬质量周期受理与执行）。时钟以 std::function 注入（秒），数据经方法传入，
// 不碰 ROS 订阅；内部自带互斥与条件变量，等待语义与原节点 data_cv_ 一致。
class TargetCache
{
public:
  explicit TargetCache(std::function<double()> clock_s);

  // 目标观测调和：ID 冲突时清旧目标/精化/决策缓存；同 ID 的锁存精化结果保留。
  void updateSelectedTarget(const SelectedTargetUpdate & update);
  // 锁定集锚点缓存批量刷新（阶段 E 残局抬质量能力端）：数据源为
  // PeachTargetObservationArray.observations 中全部 confirmed 目标（含非
  // selected；confirmed 过滤与字段提取在节点薄壳完成）。
  //   - target_set_locked=false：锁定集不存在（锁定前 observations 恒空），
  //     清空缓存并复位 run 记钥；
  //   - harvest_run_id 变化：跨批次身份不复用，清空后按新批次重建；
  //   - 单目标刷新语义与 updateSelectedTarget 一致：锚点几何（center/axis/
  //     travel）凡携带即采用，entry_pose/received_s 仅 OBSERVED 有效观测帧
  //     刷新，swinging/tracking_status 诊断透传每帧刷新；本帧缺席的目标保留
  //     最后已知条目（同 selected 缓存的闪烁容忍语义）。
  void updateLockedTargets(
    bool target_set_locked, const std::string & harvest_run_id,
    const std::vector<LockedTargetUpdate> & updates);
  void updateReconstructionDiagnostics(const ReconstructionDiagnosticsUpdate & update);
  // 抓取决策调和；返回 false 表示非当前目标被忽略（节点侧据此记警告）。
  bool updateGraspDecision(const std::string & target_id, bool allowed);
  // 精化位姿调和；返回 false 表示非当前目标被忽略。
  bool updateRefinedPose(const RefinedPoseUpdate & update);
  // 精化拟合指标调和；返回 false 表示非期望目标被忽略。
  bool updateRefinedFitting(const RefinedFittingUpdate & update);

  std::optional<CachedTarget> targetSnapshot() const;
  // 锁定集锚点快照（OBSERVE_ONLY 残局抬质量周期的受理与执行数据源）：
  // 目标不在锁定集或锚点无效（从未携带有效几何）均返回 nullopt——与
  // targetSnapshot 的"无效即空"语义一致；需要区分两种拒绝原因时用
  // lockedTargetGateSample（id 空=不在锁定集，id 命中但 valid=false=锚点缺失）。
  std::optional<CachedTarget> lockedTargetSnapshot(
    const std::string & target_id) const;
  // 锁定集目标的安全门样本：未命中返回空 ID 样本（SafetyGate::targetReady
  // 判身份不匹配拒绝）。
  TargetGateSample lockedTargetGateSample(const std::string & target_id) const;
  std::optional<CachedRefined> refinedSnapshot() const;
  QualitySnapshot qualitySnapshot() const;
  std::string graspDecisionTarget() const;
  std::vector<Eigen::Vector3d> observedDirections() const;
  // 安全门样本（含无效目标 id 与接收时刻，供 SafetyGate::targetReady）。
  TargetGateSample targetGateSample() const;
  // 精化指标的期望 ID：refined 优先、selected 兜底（供忽略警告日志）。
  std::string expectedFittingTargetId() const;

  // 等待新重建帧：谓词满足返回 true，超时或 cancel 置位返回 false。
  bool waitForNewView(
    std::size_t previous_views, double timeout_s, const std::atomic_bool & cancel) const;
  // 等待同 ID 的有效精化位姿：以 refined_.valid 且 ID 匹配为准，
  // fitting 指标单独到达不满足谓词；超时或 cancel 置位返回 false。
  bool waitForRefined(
    const std::string & target_id, double timeout_s,
    const std::atomic_bool & cancel) const;
  // 等待一条 received_s 晚于 after_s 的有效目标观测：视点移动到位后等待
  // 到位后的新鲜帧（移动中途被接受的帧不算），供安全门在新鲜样本上复核。
  bool waitForFreshTarget(
    double after_s, double timeout_s, const std::atomic_bool & cancel) const;
  // waitForFreshTarget 的锁定集版本（OBSERVE_ONLY 周期目标非 selected）：
  // 谓词、超时与取消语义完全相同，只是数据源换成指定 ID 的锁定集锚点条目。
  bool waitForFreshLockedTarget(
    const std::string & target_id, double after_s, double timeout_s,
    const std::atomic_bool & cancel) const;
  // 取消/关停时唤醒全部等待（谓词内的 cancel 负责终结语义）。
  void notifyAll();

private:
  std::function<double()> clock_s_;
  mutable std::mutex mutex_;
  mutable std::condition_variable cv_;
  CachedTarget target_;
  CachedRefined refined_;
  // 锁定集锚点缓存（id → 几何/诊断，复用 CachedTarget）：OBSERVE_ONLY 残局
  // 抬质量周期的受理门与执行体数据源；仅由 updateLockedTargets 维护
  // （locked_run_id_ 为批次记钥，空串=当前无锁定集）。
  std::unordered_map<std::string, CachedTarget> locked_targets_;
  std::string locked_run_id_;
  QualitySnapshot quality_;
  std::vector<Eigen::Vector3d> observed_directions_;
  std::string grasp_decision_target_id_;
  bool diagnostics_seen_{false};
  double diagnostics_received_s_{0.0};
};

}  // namespace peach_approach_grasp

#endif  // PEACH_APPROACH_GRASP__TARGET_CACHE_HPP_
