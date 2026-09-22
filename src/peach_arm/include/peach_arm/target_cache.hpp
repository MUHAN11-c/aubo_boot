// 功能：当前目标观测/精化/GraspDecision 缓存与 ID 调和。纯核，零 ROS；不写账本。
#ifndef PEACH_MANIPULATION__TARGET_CACHE_HPP_
#define PEACH_MANIPULATION__TARGET_CACHE_HPP_

#include <Eigen/Geometry>

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "peach_arm/model_contract.hpp"
#include "peach_arm/quality_gate.hpp"
#include "peach_arm/safety_gate.hpp"

namespace peach_arm
{

// 几何有效性：分量全有限且非零向量（原节点匿名命名空间实现的原样搬运）。
inline bool nonzeroFinite(const Eigen::Vector3d & value)
{
  return value.allFinite() && value.norm() > 1.0e-6;
}

/// 当前选中目标的初始几何（感知观测侧，base 系）。
struct CachedTarget
{
  std::string id;                 ///< 稳定 target_id。
  std::string harvest_run_id;     ///< 所属批次；空=无批次。
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};           ///< 身份锚点 [m]。
  Eigen::Isometry3d initial_pose{Eigen::Isometry3d::Identity()};  ///< 入口姿态。
  Eigen::Vector3d initial_axis{Eigen::Vector3d::UnitZ()};    ///< 袋底→袋口。
  double suggested_travel_m{0.0};  ///< 视觉建议插入行程 [m]。
  double bag_diameter_upper_m{-1.0};  ///< 感知圆柱直径 [m]；无效 -1。
  Eigen::Vector3d bottom{Eigen::Vector3d::Zero()};  ///< 袋底 [m]（观测携带）.
  Eigen::Vector3d neck{Eigen::Vector3d::Zero()};    ///< 袋口 [m]（观测携带）.
  double received_s{0.0};          ///< 有效 OBSERVED 帧接收时刻 [s]（注入时钟）。
  double updated_s{0.0};           ///< 任意诊断帧到达时刻（含记忆锚点）。
  bool swinging{false};            ///< 感知 target_swinging。
  uint8_t tracking_status{255};    ///< PeachTargetObservation 常量；255=未知。
  bool valid{false};               ///< 几何有限且可用。
  int bbox_x{0};                   ///< 检测框左上 x [px]。
  int bbox_y{0};                   ///< 检测框左上 y [px]。
  int bbox_w{0};                   ///< 检测框宽 [px]。
  int bbox_h{0};                   ///< 检测框高 [px]。
  int image_width{640};            ///< 图像宽 [px]。
  int image_height{480};           ///< 图像高 [px]。
  bool bbox_valid{false};          ///< 框尺寸合法。
  double foreground_ratio{-1.0};   ///< 框内分割占比；无效 -1。
};

/// 精化几何（重建侧锁存）。bag_diameter_upper_m 无效为 -1。
struct CachedRefined
{
  std::string id;  ///< 必须与 selected target_id 一致才采用。
  Eigen::Vector3d entry{Eigen::Vector3d::Zero()};   ///< 袋外入口 [m]。
  Eigen::Vector3d bottom{Eigen::Vector3d::Zero()};  ///< 袋底 [m]。
  Eigen::Vector3d neck{Eigen::Vector3d::Zero()};    ///< 袋口 [m]。
  Eigen::Vector3d axis{Eigen::Vector3d::UnitZ()};   ///< 袋底→袋口。
  double suggested_travel_m{0.0};     ///< 建议插入行程 [m]。
  double bag_diameter_upper_m{-1.0};  ///< 感知圆柱直径 [m]；无效 -1。
  bool valid{false};                  ///< 精化可用。
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
  double bag_diameter_upper_m{-1.0};
  // 诊断透传（含义见 CachedTarget）：由节点从 diagnostic_flags/tracking_status 提取。
  bool swinging{false};
  uint8_t tracking_status{255};
  int bbox_x{0};
  int bbox_y{0};
  int bbox_w{0};
  int bbox_h{0};
  int image_width{640};
  int image_height{480};
  bool bbox_valid{false};
  double foreground_ratio{-1.0};
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
  double bag_diameter_upper_m{-1.0};
  bool swinging{false};
  uint8_t tracking_status{255};
  int bbox_x{0};
  int bbox_y{0};
  int bbox_w{0};
  int bbox_h{0};
  int image_width{640};
  int image_height{480};
  bool bbox_valid{false};
  double foreground_ratio{-1.0};
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
  double bag_diameter_upper_m{-1.0};
  bool accepted{false};
};

// updateRefinedFitting 输入：is_fruit=true 取球拟合指标，否则取柱拟合指标。
struct RefinedFittingUpdate
{
  bool clear{false};
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
  // 只核身份元组与 allowed；不得续签 valid_until（心跳走 diagnostics）。
  bool updateGraspDecision(const std::string & target_id, bool allowed);
  bool updateGraspDecision(
    const ModelIdentity & identity, bool allowed, double valid_until_s = 0.0);
  // 模型快照只在 finalize 路径写入；诊断心跳不得调用。
  void replaceModelSnapshot(const ModelSnapshot & snapshot);
  ModelSnapshot modelSnapshot() const;
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
  // 锁定集里除 exclude_id 外的有效中心（base 系），供观察朝「更多果」走。
  std::vector<Eigen::Vector3d> lockedNeighborCenters(
    const std::string & exclude_id) const;
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
  // 等待新机位：view_directions 聚类代表数增加。同机位停稳连帧会增加
  // captured_views 但不增加机位；NBV 式覆盖要用机位而不是积分帧。
  bool waitForNewStation(
    std::size_t previous_stations, double timeout_s,
    const std::atomic_bool & cancel) const;
  // 等待同 ID 的有效精化位姿：以 refined_.valid 且 ID 匹配为准，
  // fitting 指标单独到达不满足谓词；超时或 cancel 置位返回 false。
  bool waitForRefined(
    const std::string & target_id, double timeout_s,
    const std::atomic_bool & cancel) const;
  // 把锁定集（优先）或 selected 场景观测提升为精化入口，并钉住重建门
  // （state=READY、refined_accept、grasp_allowed）。之后忽略重建诊断/精化/
  // GraspDecision 话题，避免 IDLE 或未融合 allowed=false 冲掉未精化几何。
  // 无有效锚点返回 false。验证路径（quality.allow_unrefined_geometry）。
  bool promoteUnrefinedGeometry(const std::string & target_id);
  // 等待一条 received_s 晚于 after_s 的有效目标观测：视点移动到位后等待
  // 到位后的新鲜帧（移动中途被接受的帧不算），供安全门在新鲜样本上复核。
  bool waitForFreshTarget(
    double after_s, double timeout_s, const std::atomic_bool & cancel,
    bool live_observation_required = true) const;
  // waitForFreshTarget 的锁定集版本（OBSERVE_ONLY 周期目标非 selected）：
  // 谓词、超时与取消语义完全相同，只是数据源换成指定 ID 的锁定集锚点条目。
  // live_observation_required=false：再确认用——锁定目标已是 OBSERVED 时
  // 用当前锚点判漂移，不等 received_s > after_s（记忆锚点帧仍更新 updated_s）。
  bool waitForFreshLockedTarget(
    const std::string & target_id, double after_s, double timeout_s,
    const std::atomic_bool & cancel,
    bool live_observation_required = true) const;
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
  ModelSnapshot model_;
  double model_generated_s_{0.0};
  bool unrefined_hold_{false};
};

}  // namespace peach_arm

#endif  // PEACH_MANIPULATION__TARGET_CACHE_HPP_
