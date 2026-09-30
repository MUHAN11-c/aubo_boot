#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "peach2_manipulation/bag_obstacles.hpp"

namespace pm = peach2_manipulation;
namespace ee = peach2_end_effector;

namespace
{

ee::TargetGeometry model(const std::string & id, double x, double d95 = 0.07)
{
  ee::TargetGeometry g;
  g.target_id = id;
  g.axis = Eigen::Vector3d::UnitZ();
  g.bottom = Eigen::Vector3d(x, 0.0, 0.80);
  g.neck = Eigen::Vector3d(x, 0.0, 0.88);
  g.d95_m = d95;
  g.length_m = 0.08;
  return g;
}

bool has(const std::vector<std::string> & v, const std::string & s)
{
  return std::find(v.begin(), v.end(), s) != v.end();
}

}  // namespace

TEST(BagObstacles, IdPrefix)
{
  EXPECT_EQ(pm::bag_object_id("t7"), "peach_bag_t7");
  EXPECT_TRUE(pm::is_bag_object_id("peach_bag_t7"));
  EXPECT_FALSE(pm::is_bag_object_id("peach_bag_"));
  EXPECT_FALSE(pm::is_bag_object_id("peach_scene_hard_0"));
  EXPECT_FALSE(pm::is_bag_object_id("t7"));
}

TEST(BagObstacles, CapsuleSpansBottomToNeckWithMarginRadius)
{
  auto g = model("t1", 0.5);
  g.neck = Eigen::Vector3d(0.5, 0.06, 0.88);   // tilted bag
  const auto c = pm::bag_capsule(g, 0.02);
  ASSERT_TRUE(c.has_value());
  EXPECT_EQ(c->id, "peach_bag_t1");
  EXPECT_NEAR(c->radius_m, 0.035 + 0.02, 1e-12);
  EXPECT_NEAR(c->length_m(), 0.10, 1e-12);
  EXPECT_TRUE(c->center().isApprox(Eigen::Vector3d(0.5, 0.03, 0.84)));
  const Eigen::Vector3d z = c->orientation() * Eigen::Vector3d::UnitZ();
  EXPECT_TRUE(z.isApprox((g.neck - g.bottom).normalized(), 1e-12));
}

TEST(BagObstacles, UnusableModelsAndMarginsRejected)
{
  auto g = model("t1", 0.5);
  EXPECT_FALSE(pm::bag_capsule(g, -0.01).has_value());
  EXPECT_FALSE(pm::bag_capsule(g, std::nan("")).has_value());
  EXPECT_TRUE(pm::bag_capsule(g, 0.0).has_value());
  g.d95_m = 0.0;
  EXPECT_FALSE(pm::bag_capsule(g, 0.02).has_value());
  g = model("t1", 0.5);
  g.neck = g.bottom;
  EXPECT_FALSE(pm::bag_capsule(g, 0.02).has_value());
  g = model("t1", 0.5);
  g.bottom.x() = std::nan("");
  EXPECT_FALSE(pm::bag_capsule(g, 0.02).has_value());
  g = model("", 0.5);
  EXPECT_FALSE(pm::bag_capsule(g, 0.02).has_value());
}

TEST(BagObstacles, DesiredExcludesOnlyTheCurrentTarget)
{
  const std::vector<ee::TargetGeometry> models = {
    model("t2", 0.6), model("t1", 0.5), model("t3", 0.7)};
  const auto all = pm::desired_bag_capsules(models, 0.02, "");
  ASSERT_EQ(all.size(), 3u);
  EXPECT_EQ(all[0].id, "peach_bag_t1");   // sorted by id
  const auto others = pm::desired_bag_capsules(models, 0.02, "t1");
  ASSERT_EQ(others.size(), 2u);
  for (const auto & c : others) {
    EXPECT_NE(c.target_id, "t1");
  }
  EXPECT_EQ(pm::desired_bag_capsules(models, 0.02, "unknown").size(), 3u);
}

TEST(BagObstacles, DiffAddsUpdatesAndRemoves)
{
  pm::BagObstacleSet set;
  auto desired = pm::desired_bag_capsules({model("t1", 0.5), model("t2", 0.6)}, 0.02, "");
  auto d = set.diff_to(desired);
  EXPECT_EQ(d.upsert.size(), 2u);
  EXPECT_TRUE(d.remove.empty());
  set.commit(d);
  EXPECT_EQ(set.size(), 2u);

  // Unchanged: no-op diff (no scene call needed).
  EXPECT_TRUE(set.diff_to(desired).empty());

  // Moved t2, dropped t1, new t3.
  auto m2 = model("t2", 0.6);
  m2.bottom.z() += 0.01;
  desired = pm::desired_bag_capsules({m2, model("t3", 0.7)}, 0.02, "");
  d = set.diff_to(desired);
  ASSERT_EQ(d.upsert.size(), 2u);
  EXPECT_EQ(d.remove, std::vector<std::string>{"peach_bag_t1"});
  set.commit(d);
  EXPECT_FALSE(set.contains("peach_bag_t1"));
  EXPECT_TRUE(set.contains("peach_bag_t3"));
}

TEST(BagObstacles, ApproachThenContactThenRestore)
{
  // The cycle's scene sequence for target t1 with neighbour t2.
  const std::vector<ee::TargetGeometry> models = {model("t1", 0.5), model("t2", 0.6)};
  pm::BagObstacleSet set;
  set.commit(set.diff_to(pm::desired_bag_capsules(models, 0.02, "")));   // approach
  const auto contact = set.diff_to(pm::desired_bag_capsules(models, 0.02, "t1"));
  EXPECT_TRUE(contact.upsert.empty());
  EXPECT_EQ(contact.remove, std::vector<std::string>{"peach_bag_t1"});
  set.commit(contact);
  EXPECT_FALSE(set.contains("peach_bag_t1"));
  EXPECT_TRUE(set.contains("peach_bag_t2"));
  const auto restore = set.diff_to(pm::desired_bag_capsules(models, 0.02, ""));
  ASSERT_EQ(restore.upsert.size(), 1u);
  EXPECT_EQ(restore.upsert[0].id, "peach_bag_t1");
  EXPECT_TRUE(restore.remove.empty());
}

TEST(BagObstacles, UncommittedDiffIsRetried)
{
  pm::BagObstacleSet set;
  const auto desired = pm::desired_bag_capsules({model("t1", 0.5)}, 0.02, "");
  const auto d = set.diff_to(desired);
  // Scene rejected the change: nothing committed, the same diff comes back.
  EXPECT_EQ(set.diff_to(desired).upsert.size(), d.upsert.size());
  EXPECT_EQ(set.size(), 0u);
}

TEST(BagObstacles, ClearAllRemovesEveryKnownBag)
{
  pm::BagObstacleSet set;
  set.commit(set.diff_to(pm::desired_bag_capsules({model("t1", 0.5), model("t2", 0.6)}, 0.0, "")));
  const auto d = set.clear_all();
  EXPECT_TRUE(d.upsert.empty());
  EXPECT_EQ(d.remove.size(), 2u);
  set.commit(d);
  EXPECT_EQ(set.size(), 0u);
  EXPECT_TRUE(set.clear_all().empty());
}

TEST(BagObstacles, AdoptedLeftoversAreReplacedOrRemoved)
{
  pm::BagObstacleSet set;
  set.adopt({"peach_bag_t1", "peach_bag_old", "peach_scene_hard_3", "table"});
  EXPECT_EQ(set.size(), 2u);
  const auto d = set.diff_to(pm::desired_bag_capsules({model("t1", 0.5)}, 0.02, ""));
  ASSERT_EQ(d.upsert.size(), 1u);   // unknown geometry: always rewritten
  EXPECT_EQ(d.upsert[0].id, "peach_bag_t1");
  EXPECT_EQ(d.remove, std::vector<std::string>{"peach_bag_old"});
  EXPECT_FALSE(has(d.remove, "peach_scene_hard_3"));
  EXPECT_FALSE(has(d.remove, "table"));
}
