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
#ifndef PEACH_APPROACH_GRASP__VIEW_PLANNER_HPP_
#define PEACH_APPROACH_GRASP__VIEW_PLANNER_HPP_

#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace peach_approach_grasp
{

struct ViewPlannerConfig
{
  double observation_radius_m{0.28};
  double minimum_radius_m{0.20};
  double azimuth_step_deg{12.0};
  double azimuth_limit_deg{36.0};
  double elevation_step_deg{8.0};
  double elevation_limit_deg{16.0};
  double preferred_baseline_deg{15.0};
  double radial_step_m{0.015};
  int candidate_layers{3};
  int views_to_minimum_radius{5};
};

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

class ViewPlanner
{
public:
  explicit ViewPlanner(ViewPlannerConfig config = ViewPlannerConfig());

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

}  // namespace peach_approach_grasp

#endif  // PEACH_APPROACH_GRASP__VIEW_PLANNER_HPP_
