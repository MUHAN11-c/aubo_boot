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

#include "peach_approach_grasp/quality_gate.hpp"

namespace peach_approach_grasp
{

TEST(QualityGate, RequiresIdentityCoverageAndDepth)
{
  QualityGate gate;
  QualitySnapshot snapshot;
  snapshot.selected_target_id = "peach_1";
  snapshot.reconstruction_target_id = "peach_1";
  snapshot.captured_views = 5;
  snapshot.max_baseline_deg = 25.0;
  snapshot.mean_nearest_baseline_deg = 10.0;
  snapshot.mean_depth_ratio = 0.6;
  snapshot.data_age_s = 0.1;
  EXPECT_TRUE(gate.readyToFinalize(snapshot).allowed);

  snapshot.reconstruction_target_id = "peach_2";
  EXPECT_FALSE(gate.readyToFinalize(snapshot).allowed);
}

TEST(QualityGate, RequiresSameRefinedTargetAndFitQuality)
{
  QualityGate gate;
  QualitySnapshot snapshot;
  snapshot.selected_target_id = "peach_1";
  snapshot.reconstruction_target_id = "peach_1";
  snapshot.reconstruction_state = "READY";
  snapshot.refined_target_id = "peach_1";
  snapshot.refined_accept = true;
  snapshot.grasp_allowed = true;
  snapshot.refined_rmse_m = 0.003;
  snapshot.refined_inlier_ratio = 0.6;
  snapshot.data_age_s = 0.1;
  EXPECT_TRUE(gate.readyToGrasp(snapshot).allowed);

  snapshot.data_age_s = 99.0;
  EXPECT_TRUE(gate.readyToPreviewContact(snapshot).allowed);
  EXPECT_FALSE(gate.readyToGrasp(snapshot).allowed);

  snapshot.refined_inlier_ratio = 0.1;
  EXPECT_FALSE(gate.readyToPreviewContact(snapshot).allowed);
}

}  // namespace peach_approach_grasp
