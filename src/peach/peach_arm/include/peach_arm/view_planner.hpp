// 功能：spherical_adaptive 视点规划器。最近短步截断，评分以行程最短为主。
#ifndef PEACH_MANIPULATION__VIEW_PLANNER_HPP_
#define PEACH_MANIPULATION__VIEW_PLANNER_HPP_

#include <Eigen/Geometry>

#include <string>
#include <vector>

#include "peach_arm/protected_zones.hpp"

namespace peach_arm
{

/// 单个候选视点：相机位姿（base 系，光学坐标约定 +Z 朝目标）与评分/标签。
struct ViewCandidate
{
  Eigen::Isometry3d camera_pose{Eigen::Isometry3d::Identity()};  ///< 相机光学系位姿。
  Eigen::Vector3d direction_target_to_camera{Eigen::Vector3d::UnitX()};  ///< 目标→相机单位向量。
  double radius_m{0.0};               ///< 当前相机距目标 [m]（截步后）。
  double azimuth_deg{0.0};            ///< 方位角 [deg]。
  double elevation_deg{0.0};          ///< 仰角 [deg]。
  double nearest_baseline_deg{0.0};   ///< 与已采方向最近夹角 [deg]。
  double motion_angle_deg{0.0};       ///< 相对当前视线转角 [deg]。
  double travel_m{0.0};               ///< 当前相机到本候选直线距离 [m]。
  double score{0.0};                  ///< 越大越优先（行程短为主）。
  std::string label;                  ///< 诊断标签。
};

/// generate() 的全部输入（纯值）。实现不得读隐式状态。
struct ViewContext
{
  Eigen::Vector3d target{Eigen::Vector3d::Zero()};  ///< 目标锚点 [m]，base 系。
  Eigen::Vector3d current_camera_position{Eigen::Vector3d::Zero()};  ///< 当前相机位置 [m]。
  std::vector<Eigen::Vector3d> observed_directions;  ///< 已采 target→camera 单位向量。
  int image_width{640};   ///< 图像宽 [px]。
  int image_height{480};  ///< 图像高 [px]。
  int bbox_x{0};          ///< 检测框左上 x [px]。
  int bbox_y{0};          ///< 检测框左上 y [px]。
  int bbox_w{0};          ///< 检测框宽 [px]。
  int bbox_h{0};          ///< 检测框高 [px]。
  bool bbox_valid{false}; ///< 框可用。
  double foreground_ratio{-1.0};  ///< 框内分割占比；无效 -1。
  std::vector<Eigen::Vector3d> neighbor_centers;  ///< 邻果中心（朝「更多果」走）。
};

struct ViewPlannerConfig
{
  // 默认值以 config/peach_arm.yaml 为权威源，此处仅为直接构造兜底。
  // 视点半径用当前相机距，不再贴 observation_radius 球面；本值只作过近时的参考。
  double observation_radius_m{0.40};
  double minimum_radius_m{0.32};
  double azimuth_step_deg{12.0};
  double elevation_step_deg{8.0};
  double elevation_limit_deg{0.0};
  double preferred_baseline_deg{12.0};
  double radial_step_m{0.015};
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

// 默认视点规划实现（唯一实现，零第二实现虚基类已删，W5-8；将来需要第二
// 实现时按 AGENTS 走 pluginlib 新缝）：从当前相机沿直线截到
// max_camera_step_m，评分以行程最短为主；朝框内分割更满的方向微偏，不绕
// 球面、不对侧兜圈。generate() 为 const 纯函数语义，只在周期工作线程调用。
class ViewPlanner
{
public:
  explicit ViewPlanner(ViewPlannerConfig config = ViewPlannerConfig());

  // 候选生成（纯函数，空列表=无可用视点；退化输入由实现兜底）。
  std::vector<ViewCandidate> generate(const ViewContext & context) const;

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

}  // namespace peach_arm

#endif  // PEACH_MANIPULATION__VIEW_PLANNER_HPP_
