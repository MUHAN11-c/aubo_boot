// 功能：空心筒刀具静态档案（几何与 IO 针脚）。与 aubo_description 对齐。
#ifndef PEACH_MANIPULATION__TOOL_PROFILE_HPP_
#define PEACH_MANIPULATION__TOOL_PROFILE_HPP_

#include <string>

namespace peach_manipulation
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

}  // namespace peach_manipulation

#endif  // PEACH_MANIPULATION__TOOL_PROFILE_HPP_
