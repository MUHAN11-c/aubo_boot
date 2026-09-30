#pragma once

#include <string>

namespace peach2_end_effector
{

/// geometry_m block of aubo_description/config/<tool_id>.yaml. All values in metres.
struct ToolGeometry
{
  double d_inner{0.0};         ///< functional opening diameter
  double d_outer{0.0};
  double l_insert{0.0};        ///< max bag length (bottom -> neck) the tool can swallow
  double l_blade{0.0};         ///< TCP (mouth) -> blade plane distance along TCP -Z
  double body_length{0.0};
  double body_radius{0.0};
  double wall_clearance{0.0};
};

/// io block. `pin` is the command DO; the feedback DI pin is a node parameter because the
/// profile historically used the same pin for both (P0-3).
struct ToolIo
{
  int fun{3};                  ///< aubo_msgs/SetIO fun (3 = tool digital output)
  int pin{0};
  double close_state{1.0};     ///< SetIO state written to close the blade
  bool pulse{false};
  double feedback_timeout_s{1.5};
};

struct ToolProfile
{
  std::string tool_id;
  std::string calibration_status;
  ToolGeometry geometry;
  ToolIo io;
};

/// Loads and validates one profile file. Throws std::runtime_error with the offending key.
ToolProfile load_tool_profile_file(const std::string & path);

/// `<config_dir>/<tool_id>.yaml`; also checks profile_id == tool_id.
ToolProfile load_tool_profile(const std::string & tool_id, const std::string & config_dir);

}  // namespace peach2_end_effector
