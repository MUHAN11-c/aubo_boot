// 功能：观察视点规划抽象。纯值输入输出，零 ROS。只在周期工作线程调 generate()。
#ifndef PEACH_MANIPULATION__VIEW_PLANNER_BASE_HPP_
#define PEACH_MANIPULATION__VIEW_PLANNER_BASE_HPP_

#include <Eigen/Geometry>

#include <string>
#include <vector>

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

}  // namespace peach_manipulation

#endif  // PEACH_MANIPULATION__VIEW_PLANNER_BASE_HPP_
