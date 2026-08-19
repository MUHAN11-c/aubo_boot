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
// 职责：观察视点规划职责的可替换接口（重构协议 2.14：每个可替换职责一个抽象
// 基类，签名最小化、输入输出为纯数据类型、零 ROS 类型）。
// 契约：实现必须线程无关、可重入——节点只在 BT 工作线程内调用 generate()；
// 实现内部不持可变的跨调用状态。调用端（bt_nodes）只持
// std::unique_ptr<ViewPlannerBase>，经 impl_factory.hpp 按名字创建。
#ifndef PEACH_APPROACH_GRASP__VIEW_PLANNER_BASE_HPP_
#define PEACH_APPROACH_GRASP__VIEW_PLANNER_BASE_HPP_

#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace peach_approach_grasp
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
  double score{0.0};
  std::string label;
};

// generate() 的全部输入（纯值）：目标锚点、当前相机位置与已采集观察方向
// （target→camera 单位向量，base 系）。实现不得读取除此之外的任何隐式状态。
struct ViewContext
{
  Eigen::Vector3d target{Eigen::Vector3d::Zero()};
  Eigen::Vector3d current_camera_position{Eigen::Vector3d::Zero()};
  std::vector<Eigen::Vector3d> observed_directions;
};

// 视点规划器抽象基类。
// 用途：为目标生成按优先级排序的候选观察视点列表。
// 生命周期：由节点构造期/参数重载时经工厂创建，unique_ptr 独占持有。
// 线程安全：generate() 为 const 纯函数语义，只在 BT 工作线程调用。
// 可替换性：注册名见 impl_factory.hpp（默认实现 spherical_adaptive）。
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

}  // namespace peach_approach_grasp

#endif  // PEACH_APPROACH_GRASP__VIEW_PLANNER_BASE_HPP_
