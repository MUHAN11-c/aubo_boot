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
#ifndef PEACH_MANIPULATION_SKILLS__VIEW_PLANNER_HPP_
#define PEACH_MANIPULATION_SKILLS__VIEW_PLANNER_HPP_

#include <Eigen/Geometry>

#include <string>
#include <vector>

#include "peach_manipulation_skills/protected_zones.hpp"
#include "peach_manipulation_skills/view_planner_base.hpp"

namespace peach_manipulation_skills
{

struct ViewPlannerConfig
{
  // 默认值以 config/approach_grasp.yaml 为权威源，此处仅为直接构造兜底：
  // 0.40/0.32m 退出深度相机近距盲区，保持跨视角锚点一致。
  double observation_radius_m{0.40};
  double minimum_radius_m{0.32};
  double azimuth_step_deg{12.0};
  double azimuth_limit_deg{36.0};
  double elevation_step_deg{8.0};
  double elevation_limit_deg{16.0};
  double preferred_baseline_deg{15.0};
  double radial_step_m{0.015};
  int candidate_layers{3};
  int views_to_minimum_radius{5};
  // 相机位置 z 下限（base 系）：低于桌面保护平面的视点物理上必然穿桌，
  // 规划必败且白耗 planning_time×attempts，生成阶段直接剔除。
  double min_camera_height_m{0.06};
  // 环境几何保护区（重构计划阶段 F1）：base 系轴对齐盒列表，候选视点的相机
  // 位置落入任一盒（闭区间，含盒表面）即剔除。
  // 与 min_camera_height_m 的关系：protected_zones 是通用的任意盒列表；
  // 桌面保护平面是"z<下限"半空间这一特例的 shortcut（无限大盒无法用一个
  // 有限 AABB 表达，保留独立参数避免配置噪音），两者并存、各自独立生效。
  std::vector<ProtectedZone> protected_zones;
};

// 默认视点规划实现（注册名 spherical_adaptive）：目标中心球面上的离散自适应
// 候选生成与评分。线程安全与生命周期约定见 ViewPlannerBase。
class ViewPlanner : public ViewPlannerBase
{
public:
  explicit ViewPlanner(ViewPlannerConfig config = ViewPlannerConfig());

  // 基类接口：候选生成（纯函数，空列表=无可用视点）。
  std::vector<ViewCandidate> generate(const ViewContext & context) const override;

  // 便捷重载：与 generate(ViewContext) 等价，供既有单测/直调方使用。
  std::vector<ViewCandidate> generate(
    const Eigen::Vector3d & target,
    const Eigen::Vector3d & current_camera_position,
    const std::vector<Eigen::Vector3d> & observed_directions) const;

  static Eigen::Matrix3d lookAtOptical(
    const Eigen::Vector3d & camera_position,
    const Eigen::Vector3d & target,
    const Eigen::Vector3d & world_up = Eigen::Vector3d::UnitZ());

  static Eigen::Matrix3d toolOrientation(
    const Eigen::Vector3d & approach_axis,
    const Eigen::Vector3d & preferred_x);

private:
  ViewPlannerConfig config_;
};

double angleDegrees(const Eigen::Vector3d & first, const Eigen::Vector3d & second);

}  // namespace peach_manipulation_skills

#endif  // PEACH_MANIPULATION_SKILLS__VIEW_PLANNER_HPP_
