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
// ID 一致性调和与快照访问。时钟以 std::function 注入（秒），数据经方法传入，
// 不碰 ROS 订阅；内部自带互斥与条件变量，等待语义与原节点 data_cv_ 一致。
class TargetCache
{
public:
  explicit TargetCache(std::function<double()> clock_s);

  // 目标观测调和：ID 冲突时清旧目标/精化/决策缓存；同 ID 的锁存精化结果保留。
  void updateSelectedTarget(const SelectedTargetUpdate & update);
  void updateReconstructionDiagnostics(const ReconstructionDiagnosticsUpdate & update);
  // 抓取决策调和；返回 false 表示非当前目标被忽略（节点侧据此记警告）。
  bool updateGraspDecision(const std::string & target_id, bool allowed);
  // 精化位姿调和；返回 false 表示非当前目标被忽略。
  bool updateRefinedPose(const RefinedPoseUpdate & update);
  // 精化拟合指标调和；返回 false 表示非期望目标被忽略。
  bool updateRefinedFitting(const RefinedFittingUpdate & update);

  std::optional<CachedTarget> targetSnapshot() const;
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
  // 取消/关停时唤醒全部等待（谓词内的 cancel 负责终结语义）。
  void notifyAll();

private:
  std::function<double()> clock_s_;
  mutable std::mutex mutex_;
  mutable std::condition_variable cv_;
  CachedTarget target_;
  CachedRefined refined_;
  QualitySnapshot quality_;
  std::vector<Eigen::Vector3d> observed_directions_;
  std::string grasp_decision_target_id_;
  bool diagnostics_seen_{false};
  double diagnostics_received_s_{0.0};
};

}  // namespace peach_approach_grasp

#endif  // PEACH_APPROACH_GRASP__TARGET_CACHE_HPP_
