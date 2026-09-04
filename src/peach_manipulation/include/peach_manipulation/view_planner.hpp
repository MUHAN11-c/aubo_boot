// 功能：spherical_adaptive 视点规划器。最近短步截断，评分以行程最短为主。
#ifndef PEACH_MANIPULATION__VIEW_PLANNER_HPP_
#define PEACH_MANIPULATION__VIEW_PLANNER_HPP_

#include <Eigen/Geometry>

#include <string>
#include <vector>

#include "peach_manipulation/protected_zones.hpp"

namespace peach_manipulation
{

// 单个候选视点：相机位姿（base 系，光学坐标约定 +Z 朝目标）与评分/标签。
struct ViewCandidate
{
  Eigen::Isometry3d camera_pose{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d direction_target_to_camera{Eigen::Vector3d::UnitX()};
  double radius_m{0.0};
  double azimuth_deg{0.0};
  double elevation_deg{0.0};
  double nearest_baseline_deg{0.0};
  double motion_angle_deg{0.0};
  double travel_m{0.0};  // 当前相机到本候选的直线距离（截步后）
  double score{0.0};
  std::string label;
};

// generate() 的全部输入（纯值）：目标锚点、当前相机位置、已采集观察方向
// （target→camera 单位向量，base 系），以及检测/分割可见性（像素框、框内分割占比、邻果）。
// 实现不得读取除此之外的任何隐式状态。
struct ViewContext
{
  Eigen::Vector3d target{Eigen::Vector3d::Zero()};
  Eigen::Vector3d current_camera_position{Eigen::Vector3d::Zero()};
  std::vector<Eigen::Vector3d> observed_directions;
  int image_width{640};
  int image_height{480};
  int bbox_x{0};
  int bbox_y{0};
  int bbox_w{0};
  int bbox_h{0};
  bool bbox_valid{false};
  // 检测框内分割占比（BagFitting.foreground_ratio）；无效 -1。
  double foreground_ratio{-1.0};
  std::vector<Eigen::Vector3d> neighbor_centers;
};

// 视点规划器抽象基类。
// 用途：为目标生成按优先级排序的候选观察视点列表。
// 生命周期：由节点构造期/参数重载时经工厂创建，unique_ptr 独占持有。
// 线程安全：generate() 为 const 纯函数语义，只在周期工作线程调用。
// 可替换性：唯一实现 ViewPlanner（节点直接构造）。
class ViewPlannerBase
{
public:
  virtual ~ViewPlannerBase() = default;

  // 前置：context.target 为有效锚点；observed_directions 可为空（实现应自行
  //   兜底为当前视线方向）。
  // 后置：返回按实现内优先级排序的候选列表；空列表表示无可用视点。
  // 失败语义：不抛异常，以空列表表达"没有生成可用观察视点"。
  virtual std::vector<ViewCandidate> generate(const ViewContext & context) const = 0;
};

struct ViewPlannerConfig
{
  // 默认值以 config/manipulation_parameters.yaml 为权威源，此处仅为直接构造兜底。
  // 视点半径用当前相机距，不再贴 observation_radius 球面；本值只作过近时的参考。
  double observation_radius_m{0.40};
  double minimum_radius_m{0.32};
  double azimuth_step_deg{12.0};
  double azimuth_limit_deg{16.0};
  double elevation_step_deg{8.0};
  double elevation_limit_deg{0.0};
  double preferred_baseline_deg{12.0};
  double radial_step_m{0.015};
  int candidate_layers{1};
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

// 默认视点规划实现：从当前相机沿直线截到 max_camera_step_m，评分以行程最短为主；
// 朝框内分割更满的方向微偏，不绕球面、不对侧兜圈。线程安全见 ViewPlannerBase。
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
