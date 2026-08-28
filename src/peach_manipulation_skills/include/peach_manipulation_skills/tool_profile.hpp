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
#ifndef PEACH_MANIPULATION_SKILLS__TOOL_PROFILE_HPP_
#define PEACH_MANIPULATION_SKILLS__TOOL_PROFILE_HPP_

#include <string>

namespace peach_manipulation_skills
{

// hollow_cylinder_v1 静态档案；与 aubo_description/config 对齐。
struct ToolProfile
{
  std::string profile_id{"hollow_cylinder_v1"};
  std::string version{"1.1"};
  double d_inner{0.104};
  double d_outer{0.120};
  double l_insert{0.200};
  double l_blade{0.0};
  double wall_clearance{0.002};
  double blade_capture_half_width{0.008};
  double fruit_safety_clearance{0.012};
  double axial_safety_margin{0.004};
  double tool_runout95{0.001};
  int io_fun{3};
  int io_pin{0};
  double close_state{1.0};
  double feedback_timeout_s{1.5};
};

inline ToolProfile defaultHollowCylinderV1()
{
  return ToolProfile{};
}

}  // namespace peach_manipulation_skills

#endif  // PEACH_MANIPULATION_SKILLS__TOOL_PROFILE_HPP_
