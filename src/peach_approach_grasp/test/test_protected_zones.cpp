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
//
// 环境几何保护区（阶段 F1）纯核用例：stride-6 扁平编码解析（空/单个/多个/
// 畸形长度残余组/min>=max/非有限分量丢弃）与点包含判定（盒内/盒外/边界
// 闭区间）。

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <limits>
#include <vector>

#include "peach_approach_grasp/protected_zones.hpp"

namespace peach_approach_grasp
{

TEST(ProtectedZones, ParseEmptyListYieldsNoZones)
{
  const auto result = parseProtectedZones({});
  EXPECT_TRUE(result.zones.empty());
  EXPECT_TRUE(result.issues.empty());
}

TEST(ProtectedZones, ParseSingleZone)
{
  const auto result = parseProtectedZones({0.1, 0.2, 0.3, 0.4, 0.5, 0.6});
  ASSERT_EQ(result.zones.size(), 1U);
  EXPECT_TRUE(result.issues.empty());
  EXPECT_EQ(result.zones[0].min, Eigen::Vector3d(0.1, 0.2, 0.3));
  EXPECT_EQ(result.zones[0].max, Eigen::Vector3d(0.4, 0.5, 0.6));
}

TEST(ProtectedZones, ParseMultipleZonesPreserveOrder)
{
  const auto result = parseProtectedZones(
    {0.0, 0.0, 0.0, 1.0, 1.0, 1.0,
      -1.0, -1.0, -1.0, -0.5, -0.5, -0.5});
  ASSERT_EQ(result.zones.size(), 2U);
  EXPECT_TRUE(result.issues.empty());
  EXPECT_EQ(result.zones[0].max, Eigen::Vector3d(1.0, 1.0, 1.0));
  EXPECT_EQ(result.zones[1].min, Eigen::Vector3d(-1.0, -1.0, -1.0));
}

TEST(ProtectedZones, TrailingIncompleteGroupDroppedButCompleteBoxesKept)
{
  // 长度 8 = 1 个完整盒 + 2 个残余分量：残余组丢弃，完整盒保留（配置笔误
  // 不应拖垮全部保护区）。
  const auto result = parseProtectedZones(
    {0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 5.0, 6.0});
  ASSERT_EQ(result.zones.size(), 1U);
  ASSERT_EQ(result.issues.size(), 1U);
  EXPECT_NE(result.issues[0].find("非 6 倍数"), std::string::npos);
}

TEST(ProtectedZones, MinGreaterEqualMaxDroppedPerBox)
{
  // 盒#0 z 轴 min==max（退化盒）、盒#2 x 轴 min>max：逐盒丢弃，合法盒#1 保留。
  const auto result = parseProtectedZones(
    {0.0, 0.0, 0.5, 1.0, 1.0, 0.5,
      0.0, 0.0, 0.0, 1.0, 1.0, 1.0,
      2.0, 0.0, 0.0, 1.0, 1.0, 1.0});
  ASSERT_EQ(result.zones.size(), 1U);
  EXPECT_EQ(result.zones[0].max, Eigen::Vector3d(1.0, 1.0, 1.0));
  EXPECT_EQ(result.issues.size(), 2U);
}

TEST(ProtectedZones, NonFiniteComponentDropped)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto result = parseProtectedZones({0.0, 0.0, 0.0, nan, 1.0, 1.0});
  EXPECT_TRUE(result.zones.empty());
  ASSERT_EQ(result.issues.size(), 1U);
  EXPECT_NE(result.issues[0].find("非有限"), std::string::npos);
}

TEST(ProtectedZones, HitInsideOutsideAndBoundary)
{
  const auto parsed = parseProtectedZones(
    {0.0, 0.0, 0.0, 1.0, 1.0, 1.0,
      2.0, 2.0, 2.0, 3.0, 3.0, 3.0});
  ASSERT_EQ(parsed.zones.size(), 2U);
  // 空列表快速路径。
  EXPECT_FALSE(protectedZoneHit(Eigen::Vector3d(0.5, 0.5, 0.5), {}));
  // 盒内命中，返回盒序号。
  const auto hit0 = protectedZoneHit(
    Eigen::Vector3d(0.5, 0.5, 0.5), parsed.zones);
  ASSERT_TRUE(hit0.has_value());
  EXPECT_EQ(*hit0, 0U);
  const auto hit1 = protectedZoneHit(
    Eigen::Vector3d(2.5, 2.5, 2.5), parsed.zones);
  ASSERT_TRUE(hit1.has_value());
  EXPECT_EQ(*hit1, 1U);
  // 盒外不命中（间隙与各轴外侧）。
  EXPECT_FALSE(protectedZoneHit(Eigen::Vector3d(1.5, 0.5, 0.5), parsed.zones));
  EXPECT_FALSE(protectedZoneHit(Eigen::Vector3d(-1e-9, 0.5, 0.5), parsed.zones));
  EXPECT_FALSE(protectedZoneHit(Eigen::Vector3d(0.5, 0.5, 3.5), parsed.zones));
  // 闭区间边界：盒表面视为盒内（min 面与 max 面都命中）。
  EXPECT_TRUE(protectedZoneHit(Eigen::Vector3d(0.0, 0.5, 0.5), parsed.zones));
  EXPECT_TRUE(protectedZoneHit(Eigen::Vector3d(1.0, 1.0, 1.0), parsed.zones));
  EXPECT_TRUE(protectedZoneHit(Eigen::Vector3d(3.0, 2.0, 2.5), parsed.zones));
}

}  // namespace peach_approach_grasp
