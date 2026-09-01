// 功能：spherical_adaptive 视点规划器。最近短步截断，评分以行程最短为主。
#ifndef PEACH_MANIPULATION__VIEW_PLANNER_HPP_
#define PEACH_MANIPULATION__VIEW_PLANNER_HPP_

#include <Eigen/Geometry>

#include <string>
#include <vector>

#include "peach_manipulation/protected_zones.hpp"
#include "peach_manipulation/view_planner_base.hpp"

namespace peach_manipulation
{

struct ViewPlannerConfig
{
  // 默认值以 config/peach_manipulation.yaml 为权威源，此处仅为直接构造兜底。
  // 视点半径用当前相机距，不再贴 observation_radius 球面；本值只作过近时的参考。
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
  // 每步相机直线位移上限：沿当前位→目标视点截断，禁止绕球面走远路。
  double max_camera_step_m{0.15};
  // 相机位置距 base 原点上限；超出则沿视线收进球内，不绕行。
  double workspace_max_reach_m{0.78};
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

// 默认视点规划实现（注册名 spherical_adaptive）：从当前相机沿直线截到
// max_camera_step_m，评分以行程最短为主；朝框内分割更满的方向微偏，不绕球面、
// 不对侧兜圈。线程安全见 ViewPlannerBase。
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

}  // namespace peach_manipulation

#endif  // PEACH_MANIPULATION__VIEW_PLANNER_HPP_
