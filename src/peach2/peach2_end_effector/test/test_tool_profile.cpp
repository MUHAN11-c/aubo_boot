#include <gtest/gtest.h>

#include <fstream>
#include <stdexcept>
#include <string>

#include "peach2_end_effector/tool_profile.hpp"

using peach2_end_effector::load_tool_profile;
using peach2_end_effector::load_tool_profile_file;

namespace
{

const std::string kDir = PEACH2_TOOL_CONFIG_DIR;

std::string write_temp(const std::string & name, const std::string & body)
{
  const std::string path = testing::TempDir() + "/" + name;
  std::ofstream out(path);
  out << body;
  return path;
}

const char * kValid =
  "profile_id: test_tool\n"
  "geometry_m: {D_inner: 0.1, D_outer: 0.2, L_insert: 0.05, L_blade: 0.03,\n"
  "  body_length: 0.1, body_radius: 0.05, wall_clearance: 0.002}\n"
  "io: {fun: 3, pin: 0, close_state: 1.0, pulse: false, feedback_timeout_s: 1.5}\n";

}  // namespace

TEST(ToolProfile, LoadsAllThreeShippedProfiles)
{
  for (const std::string id : {"shear_v1", "bite_shear_v1", "adaptive_shear_v1"}) {
    const auto p = load_tool_profile(id, kDir);
    EXPECT_EQ(p.tool_id, id);
    EXPECT_GT(p.geometry.d_inner, 0.0);
    EXPECT_GT(p.geometry.l_blade, 0.0);
    EXPECT_GT(p.geometry.l_insert, 0.0);
    EXPECT_EQ(p.io.fun, 3);
    EXPECT_DOUBLE_EQ(p.io.feedback_timeout_s, 1.5);
  }
}

TEST(ToolProfile, ShippedValues)
{
  const auto shear = load_tool_profile("shear_v1", kDir);
  EXPECT_DOUBLE_EQ(shear.geometry.d_inner, 0.080);
  EXPECT_DOUBLE_EQ(shear.geometry.l_blade, 0.030);
  const auto bite = load_tool_profile("bite_shear_v1", kDir);
  EXPECT_DOUBLE_EQ(bite.geometry.d_inner, 0.104);
  EXPECT_DOUBLE_EQ(bite.geometry.l_blade, 0.037);
  const auto adaptive = load_tool_profile("adaptive_shear_v1", kDir);
  EXPECT_DOUBLE_EQ(adaptive.geometry.d_inner, 0.120);
  EXPECT_DOUBLE_EQ(adaptive.geometry.l_insert, 0.090);
  EXPECT_DOUBLE_EQ(adaptive.geometry.l_blade, 0.079);
}

TEST(ToolProfile, ValidTempFile)
{
  const auto p = load_tool_profile_file(write_temp("valid.yaml", kValid));
  EXPECT_EQ(p.tool_id, "test_tool");
  EXPECT_DOUBLE_EQ(p.geometry.l_insert, 0.05);
}

TEST(ToolProfile, MissingFileThrows)
{
  EXPECT_THROW(load_tool_profile("no_such_tool", kDir), std::runtime_error);
}

TEST(ToolProfile, IdMismatchThrows)
{
  write_temp("other_name.yaml", kValid);
  EXPECT_THROW(load_tool_profile("other_name", testing::TempDir()), std::runtime_error);
}

TEST(ToolProfile, MissingKeyNamesKey)
{
  const std::string body =
    "profile_id: t\n"
    "geometry_m: {D_inner: 0.1, D_outer: 0.2, L_insert: 0.05,\n"
    "  body_length: 0.1, body_radius: 0.05, wall_clearance: 0.002}\n"
    "io: {fun: 3, pin: 0, close_state: 1.0, feedback_timeout_s: 1.5}\n";
  try {
    load_tool_profile_file(write_temp("missing.yaml", body));
    FAIL() << "expected throw";
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find("L_blade"), std::string::npos);
  }
}

TEST(ToolProfile, RangeChecks)
{
  const std::string bad_len =
    "profile_id: t\n"
    "geometry_m: {D_inner: 0.1, D_outer: 0.2, L_insert: 5.0, L_blade: 0.03,\n"
    "  body_length: 0.1, body_radius: 0.05, wall_clearance: 0.002}\n"
    "io: {fun: 3, pin: 0, close_state: 1.0, feedback_timeout_s: 1.5}\n";
  EXPECT_THROW(load_tool_profile_file(write_temp("bad_len.yaml", bad_len)), std::runtime_error);
  const std::string bad_timeout =
    "profile_id: t\n"
    "geometry_m: {D_inner: 0.1, D_outer: 0.2, L_insert: 0.05, L_blade: 0.03,\n"
    "  body_length: 0.1, body_radius: 0.05, wall_clearance: 0.002}\n"
    "io: {fun: 3, pin: 0, close_state: 1.0, feedback_timeout_s: 0.0}\n";
  EXPECT_THROW(
    load_tool_profile_file(write_temp("bad_to.yaml", bad_timeout)), std::runtime_error);
}
