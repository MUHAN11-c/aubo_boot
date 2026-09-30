#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <string>

#include "peach2_end_effector/end_effector.hpp"
#include "pluginlib/class_loader.hpp"

TEST(PluginlibLoad, AllThreeClassesDeclaredAndLoadable)
{
  pluginlib::ClassLoader<peach2_end_effector::EndEffector> loader(
    "peach2_end_effector", "peach2_end_effector::EndEffector");
  const std::map<std::string, std::string> expected = {
    {"peach2_end_effector::ShearV1", "shear_v1"},
    {"peach2_end_effector::BiteShearV1", "bite_shear_v1"},
    {"peach2_end_effector::AdaptiveShearV1", "adaptive_shear_v1"},
  };
  for (const auto & [name, tool_id] : expected) {
    ASSERT_TRUE(loader.isClassAvailable(name)) << name;
    std::shared_ptr<peach2_end_effector::EndEffector> ee = loader.createSharedInstance(name);
    ASSERT_NE(ee, nullptr);
    EXPECT_EQ(ee->tool_id(), tool_id);
  }
}
