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

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <vector>

#include "peach_approach_grasp/view_planner.hpp"

namespace peach_approach_grasp
{

TEST(ViewPlanner, OpticalAxisLooksAtTarget)
{
  const Eigen::Vector3d camera(0.3, 0.1, 0.2);
  const Eigen::Vector3d target(0.0, 0.0, 0.0);
  const Eigen::Matrix3d rotation = ViewPlanner::lookAtOptical(camera, target);
  EXPECT_NEAR(rotation.determinant(), 1.0, 1.0e-9);
  EXPECT_NEAR(rotation.col(2).dot((target - camera).normalized()), 1.0, 1.0e-9);
}

TEST(ViewPlanner, CandidateScoresAreSortedAndRespectRadius)
{
  ViewPlannerConfig config;
  config.observation_radius_m = 0.3;
  config.minimum_radius_m = 0.25;
  ViewPlanner planner(config);
  const auto candidates = planner.generate(
    Eigen::Vector3d::Zero(), Eigen::Vector3d(0.3, 0.0, 0.0),
    std::vector<Eigen::Vector3d>{Eigen::Vector3d::UnitX()});
  ASSERT_FALSE(candidates.empty());
  for (std::size_t index = 1; index < candidates.size(); ++index) {
    EXPECT_GE(candidates[index - 1].score, candidates[index].score);
  }
  for (const auto & candidate : candidates) {
    EXPECT_GE(candidate.radius_m, 0.25);
  }
}

TEST(ViewPlanner, ToolZAxisFollowsApproachAxis)
{
  const Eigen::Vector3d axis(0.2, -0.4, 0.8);
  const Eigen::Matrix3d rotation = ViewPlanner::toolOrientation(
    axis, Eigen::Vector3d::UnitX());
  EXPECT_NEAR(rotation.col(2).dot(axis.normalized()), 1.0, 1.0e-9);
  EXPECT_NEAR(rotation.determinant(), 1.0, 1.0e-9);
}

TEST(ViewPlanner, AcceptedViewsAdvanceThePreferredRadius)
{
  ViewPlannerConfig config;
  config.observation_radius_m = 0.30;
  config.minimum_radius_m = 0.20;
  config.radial_step_m = 0.05;
  config.candidate_layers = 3;
  config.views_to_minimum_radius = 5;
  ViewPlanner planner(config);
  const Eigen::Vector3d current_camera(0.30, 0.0, 0.0);
  const auto outer = planner.generate(
    Eigen::Vector3d::Zero(), current_camera, {});
  const auto inner = planner.generate(
    Eigen::Vector3d::Zero(), current_camera,
    std::vector<Eigen::Vector3d>(5, Eigen::Vector3d::UnitX()));
  ASSERT_FALSE(outer.empty());
  ASSERT_FALSE(inner.empty());
  EXPECT_GT(outer.front().radius_m, inner.front().radius_m);
}

}  // namespace peach_approach_grasp
