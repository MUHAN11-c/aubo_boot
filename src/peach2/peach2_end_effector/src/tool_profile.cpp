#include "peach2_end_effector/tool_profile.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>
#include <string>

namespace peach2_end_effector
{
namespace
{

double required_length(const YAML::Node & block, const std::string & key, const std::string & where)
{
  const YAML::Node value = block[key];
  if (!value) {
    throw std::runtime_error(where + ": missing geometry_m." + key);
  }
  const double v = value.as<double>();
  if (!std::isfinite(v) || v < 0.0 || v > 1.0) {
    throw std::runtime_error(where + ": geometry_m." + key + " out of range [0, 1] m");
  }
  return v;
}

}  // namespace

ToolProfile load_tool_profile_file(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error(path + ": " + e.what());
  }
  ToolProfile profile;
  if (!root["profile_id"]) {
    throw std::runtime_error(path + ": missing profile_id");
  }
  profile.tool_id = root["profile_id"].as<std::string>();
  if (root["calibration_status"]) {
    profile.calibration_status = root["calibration_status"].as<std::string>();
  }

  const YAML::Node geometry = root["geometry_m"];
  if (!geometry || !geometry.IsMap()) {
    throw std::runtime_error(path + ": missing geometry_m");
  }
  auto & g = profile.geometry;
  g.d_inner = required_length(geometry, "D_inner", path);
  g.d_outer = required_length(geometry, "D_outer", path);
  g.l_insert = required_length(geometry, "L_insert", path);
  g.l_blade = required_length(geometry, "L_blade", path);
  g.body_length = required_length(geometry, "body_length", path);
  g.body_radius = required_length(geometry, "body_radius", path);
  g.wall_clearance = required_length(geometry, "wall_clearance", path);
  if (g.d_inner <= 0.0) {
    throw std::runtime_error(path + ": geometry_m.D_inner must be > 0");
  }

  const YAML::Node io = root["io"];
  if (!io || !io.IsMap()) {
    throw std::runtime_error(path + ": missing io");
  }
  for (const char * key : {"fun", "pin", "close_state", "feedback_timeout_s"}) {
    if (!io[key]) {
      throw std::runtime_error(path + ": missing io." + key);
    }
  }
  profile.io.fun = io["fun"].as<int>();
  profile.io.pin = io["pin"].as<int>();
  profile.io.close_state = io["close_state"].as<double>();
  profile.io.pulse = io["pulse"] ? io["pulse"].as<bool>() : false;
  profile.io.feedback_timeout_s = io["feedback_timeout_s"].as<double>();
  if (!(profile.io.feedback_timeout_s > 0.0 && profile.io.feedback_timeout_s < 30.0)) {
    throw std::runtime_error(path + ": io.feedback_timeout_s out of range (0, 30) s");
  }
  if (profile.io.pin < 0 || profile.io.pin > 15) {
    throw std::runtime_error(path + ": io.pin out of range [0, 15]");
  }
  return profile;
}

ToolProfile load_tool_profile(const std::string & tool_id, const std::string & config_dir)
{
  const std::string path = config_dir + "/" + tool_id + ".yaml";
  ToolProfile profile = load_tool_profile_file(path);
  if (profile.tool_id != tool_id) {
    throw std::runtime_error(
            path + ": profile_id '" + profile.tool_id + "' != requested '" + tool_id + "'");
  }
  return profile;
}

}  // namespace peach2_end_effector
