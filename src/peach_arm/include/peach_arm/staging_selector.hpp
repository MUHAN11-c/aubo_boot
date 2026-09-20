// 功能：staging 候选选择纯核（W5-2，自节点 select_goal_joints lambda 抽出）。
// 编排：keep-roll 及 ±30°/±60° 每 roll 一个并行任务 ×（当前 + seeds-1 随机
// 种子）IK 尝试；按关节距离（腕轴加权）+ 滚转惩罚升序取最近 top_n 个候选。
// IK/限位/自碰由回调注入：KDL 互斥锁留节点侧回调内；每 roll 任务仍各持
// 独立 CollisionEnvFCL（节点成员级池，回调按 roll_index 取用）——并发语义
// 与抽取前一致（扫完全部种子再排序，不因并行提前截断）。
// 零 MoveIt 依赖：gtest 用假 IK 回调注入验证排序/权重/top_n/种子路径。
#ifndef PEACH_MANIPULATION__STAGING_SELECTOR_HPP_
#define PEACH_MANIPULATION__STAGING_SELECTOR_HPP_

#include <Eigen/Geometry>

#include <algorithm>
#include <cstddef>
#include <future>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace peach_arm
{

/// staging 候选选择配置（GPL yaml staging.*；默认值=原节点硬编码）。
struct StagingSelectorConfig
{
  int seeds{5};              ///< 每 roll 的 IK 尝试数（attempt 0=当前种子，>0 随机）。
  double wrist_weight{2.5};  ///< 关节名含 "wrist" 的距离权重。
  double roll_penalty{4.0};  ///< 滚转惩罚系数（dist_sq += roll_penalty·roll²）。
  int top_n{5};              ///< 输出候选上限（按 dist_sq 升序取前 top_n）。
};

/// 单个 staging 候选：PTP 落点关节目标 + 对应工具位姿（含滚转）。
struct StagingCandidate
{
  std::map<std::string, double> joints;
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
};

/// 单次 IK/自碰尝试回调：attempt=0 用当前种子，>0 在解空间随机采样种子；
/// 返回可行关节解（实现内已过关节限位与自碰检查），无可行解返回 nullopt。
/// 回调在并行任务线程执行：实现内部自持 KDL 互斥（setFromIK 非线程安全）
/// 与按 roll_index 关联的线程专属碰撞环境。
using StagingIkSolve = std::function<std::optional<std::vector<double>>(
      int roll_index, const Eigen::Isometry3d & pose, int attempt)>;

/// 候选选择主入口。solve 为空 / names 与 current 维度不符 / names 为空
/// 返回空。roll 按 |roll|>1e-12 乘进 keep_roll_pose 姿态（绕工具 Z）。
inline std::vector<StagingCandidate> selectStagingCandidates(
  const StagingSelectorConfig & config,
  const Eigen::Isometry3d & keep_roll_pose,
  const std::vector<double> & current_joints,
  const std::vector<std::string> & joint_names,
  const std::vector<double> & rolls_rad,
  const StagingIkSolve & solve)
{
  if (!solve || joint_names.empty() ||
    current_joints.size() != joint_names.size())
  {
    return {};
  }
  const int attempts = config.seeds > 0 ? config.seeds : 0;
  const std::size_t top_n =
    config.top_n > 0 ? static_cast<std::size_t>(config.top_n) : 0U;
  struct Scored
  {
    double dist_sq;
    StagingCandidate candidate;
  };
  std::mutex scored_mutex;
  std::vector<Scored> scored;
  std::vector<std::future<void>> jobs;
  jobs.reserve(rolls_rad.size());
  for (std::size_t roll_index = 0; roll_index < rolls_rad.size(); ++roll_index) {
    const double roll = rolls_rad[roll_index];
    jobs.push_back(std::async(
        std::launch::async,
        [&, roll_index, roll, attempts]()
        {
          Eigen::Isometry3d pose = keep_roll_pose;
          if (std::abs(roll) > 1.0e-12) {
            pose.linear() = keep_roll_pose.linear() *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ());
          }
          for (int attempt = 0; attempt < attempts; ++attempt) {
            const auto sol = solve(static_cast<int>(roll_index), pose, attempt);
            if (!sol || sol->size() != joint_names.size()) {
              continue;
            }
            // 关节距离（腕轴加权）+ 滚转惩罚：升序取最近构型。
            double dist_sq = 0.0;
            for (std::size_t i = 0; i < sol->size(); ++i) {
              const double d = (*sol)[i] - current_joints[i];
              const double weight =
              joint_names[i].find("wrist") != std::string::npos ?
              config.wrist_weight : 1.0;
              dist_sq += weight * d * d;
            }
            dist_sq += config.roll_penalty * roll * roll;
            StagingCandidate candidate;
            candidate.pose = pose;
            for (std::size_t i = 0; i < joint_names.size(); ++i) {
              candidate.joints[joint_names[i]] = (*sol)[i];
            }
            std::lock_guard<std::mutex> lock(scored_mutex);
            scored.push_back({dist_sq, std::move(candidate)});
          }
        }));
  }
  for (auto & job : jobs) {
    job.get();
  }
  std::sort(
    scored.begin(), scored.end(),
    [](const Scored & a, const Scored & b) {return a.dist_sq < b.dist_sq;});
  std::vector<StagingCandidate> out;
  out.reserve(std::min<std::size_t>(scored.size(), top_n));
  for (const auto & item : scored) {
    if (out.size() >= top_n) {
      break;
    }
    out.push_back(item.candidate);
  }
  return out;
}

}  // namespace peach_arm

#endif  // PEACH_MANIPULATION__STAGING_SELECTOR_HPP_
