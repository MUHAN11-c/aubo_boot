// 功能：套入几何纯核护栏测试（对轴、滚转、预抓取、线段距离、工具×果实）。
#include "peach_arm/grasp_geometry.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>
#include <vector>

// 工具筒体尺寸（W5-6 起由 yaml tool.body_* 注入；测试用原硬编码默认值）。
constexpr double kToolLen = 0.200;
constexpr double kToolRad = 0.060;

namespace
{

// 保护区一致性用的小工具位姿点列（inspectToolVsFruit 的 Waypoint 形状）。
struct TestWaypoint
{
  double x, y, z, qx, qy, qz, qw;
};

// 绕 Y 转 -90° 的四元数：工具 Z 映到 -X（开口朝 -X、筒体向 +X 延伸）。
Eigen::Quaterniond toolQuatMinusX()
{
  return Eigen::Quaterniond(
    Eigen::AngleAxisd(-(EIGEN_PI / 2.0), Eigen::Vector3d::UnitY()));
}

// 常用果实胶囊：bottom=(0,0,0)、neck=(0,0,0.10)、轴 +Z、
// 直径 60mm + 膨胀 10mm → 半径 = 0.03+0.01 = 0.04。
peach_arm::FruitCapsule standardFruit()
{
  return peach_arm::fruitCapsuleFrom(
    Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 0.10),
    Eigen::Vector3d::UnitZ(), 0.06, 0.01, 0.10, true);
}

// 断言旋转矩阵仍为正交归一（对轴后不许数值漂移出 SO(3)）。
void expectOrthonormal(const Eigen::Matrix3d & R)
{
  EXPECT_TRUE((R.transpose() * R).isApprox(Eigen::Matrix3d::Identity(), 1e-9));
  EXPECT_NEAR(R.determinant(), 1.0, 1e-9);
}

}  // namespace

// ---------- alignFrameZ：Z 轴对到目标轴，最小旋转不多拧滚转 ----------

TEST(GraspGeometry, AlignFrameZPointsZAlongAxis)
{
  // 绕 X 转 90° 的姿态：Z 指向 +Y；对轴到 +Z 后 Z 必须平行 +Z。
  const Eigen::Matrix3d current =
    Eigen::AngleAxisd((EIGEN_PI / 2.0), Eigen::Vector3d::UnitX()).toRotationMatrix();
  const Eigen::Matrix3d aligned =
    peach_arm::alignFrameZ(current, Eigen::Vector3d::UnitZ());
  EXPECT_NEAR(aligned.col(2).dot(Eigen::Vector3d::UnitZ()), 1.0, 1e-9);
  expectOrthonormal(aligned);
  // 一般斜轴：对轴后 Z 与目标轴夹角为 0（方向随轴，含非单位轴输入）。
  const Eigen::Vector3d axis(0.3, -0.4, 0.5);
  const Eigen::Matrix3d aligned2 = peach_arm::alignFrameZ(current, axis);
  EXPECT_NEAR(
    aligned2.col(2).dot(axis.normalized()), 1.0, 1e-9);
  expectOrthonormal(aligned2);
}

TEST(GraspGeometry, AlignFrameZDegenerateAxisReturnsInput)
{
  // 零轴/非有限轴：无法对轴，原样返回（调用方按兜底处理）。
  const Eigen::Matrix3d current =
    Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()).toRotationMatrix();
  EXPECT_TRUE(
    peach_arm::alignFrameZ(current, Eigen::Vector3d::Zero()).isApprox(current));
}

TEST(GraspGeometry, AlignFrameZRolledKeepsAxisAndRollAngle)
{
  // 对轴后绕工具 Z 滚转：Z 仍指目标轴；X 相对未滚转框转 roll 角。
  const Eigen::Matrix3d current =
    Eigen::AngleAxisd((EIGEN_PI / 2.0), Eigen::Vector3d::UnitX()).toRotationMatrix();
  const Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
  const Eigen::Matrix3d aligned =
    peach_arm::alignFrameZ(current, axis);
  const double roll = EIGEN_PI / 6.0;
  const Eigen::Matrix3d rolled =
    peach_arm::alignFrameZRolled(current, axis, roll);
  EXPECT_NEAR(rolled.col(2).dot(axis), 1.0, 1e-9);
  expectOrthonormal(rolled);
  EXPECT_NEAR(
    rolled.col(0).dot(aligned.col(0)), std::cos(roll), 1e-9);
  EXPECT_NEAR(
    rolled.col(0).dot(aligned.col(1)), std::sin(roll), 1e-9);
  // roll≈0 直接退回对轴结果（避免多余数值扰动）。
  EXPECT_TRUE(peach_arm::alignFrameZRolled(current, axis, 0.0).isApprox(aligned));
}

// ---------- toolRollsRad：keep-roll 优先，只扫 ±30°/±60° ----------

TEST(GraspGeometry, ToolRollsAreFiveLevelsAroundZero)
{
  const std::vector<double> rolls = peach_arm::toolRollsRad();
  ASSERT_EQ(rolls.size(), 5U);
  EXPECT_NEAR(rolls[0], 0.0, 1e-12);
  EXPECT_NEAR(rolls[1], EIGEN_PI / 6.0, 1e-12);
  EXPECT_NEAR(rolls[2], -EIGEN_PI / 6.0, 1e-12);
  EXPECT_NEAR(rolls[3], EIGEN_PI / 3.0, 1e-12);
  EXPECT_NEAR(rolls[4], -EIGEN_PI / 3.0, 1e-12);
}

// ---------- pregraspAlongAxis / pregraspFromEntryKeepRoll ----------

TEST(GraspGeometry, PregraspAlongAxisRetreatsByStandoff)
{
  Eigen::Isometry3d entry = Eigen::Isometry3d::Identity();
  entry.translation() = Eigen::Vector3d(0.3, -0.2, 0.5);
  const Eigen::Isometry3d pregrasp = peach_arm::pregraspAlongAxis(
    entry, Eigen::Vector3d::UnitZ(), 0.03);
  EXPECT_TRUE(pregrasp.translation().isApprox(
      Eigen::Vector3d(0.3, -0.2, 0.47), 1e-12));
  EXPECT_TRUE(pregrasp.linear().isApprox(entry.linear()));
  // 斜轴（非单位长度）也按方向后撤同一 standoff。
  const Eigen::Isometry3d diagonal = peach_arm::pregraspAlongAxis(
    entry, Eigen::Vector3d(0.0, 0.0, 2.0), 0.03);
  EXPECT_TRUE(diagonal.translation().isApprox(
      Eigen::Vector3d(0.3, -0.2, 0.47), 1e-12));
  // 负 standoff 不前插（retreat 夹到 0）。
  const Eigen::Isometry3d hold = peach_arm::pregraspAlongAxis(
    entry, Eigen::Vector3d::UnitZ(), -0.5);
  EXPECT_TRUE(hold.translation().isApprox(entry.translation(), 1e-12));
  // 零轴：无法后撤，原样返回。
  const Eigen::Isometry3d unchanged = peach_arm::pregraspAlongAxis(
    entry, Eigen::Vector3d::Zero(), 0.03);
  EXPECT_TRUE(unchanged.translation().isApprox(entry.translation(), 1e-12));
}

TEST(GraspGeometry, PregraspFromEntryKeepRollAlignsCurrentAttitude)
{
  Eigen::Isometry3d entry = Eigen::Isometry3d::Identity();
  entry.translation() = Eigen::Vector3d(0.1, 0.2, 0.6);
  // 当前 TCP 已 Z 朝 +Z（带一截任意滚转）：keep-roll 后姿态必须原样保留。
  const Eigen::Matrix3d current_R =
    (Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()) *
    Eigen::Matrix3d::Identity());
  const Eigen::Isometry3d pregrasp = peach_arm::pregraspFromEntryKeepRoll(
    entry, current_R, 0.03);
  EXPECT_TRUE(pregrasp.translation().isApprox(
      Eigen::Vector3d(0.1, 0.2, 0.57), 1e-12));
  EXPECT_TRUE(pregrasp.linear().isApprox(current_R, 1e-9));
  // 当前 TCP 姿态 Z 偏轴：对轴后 Z 指向入口 Z（=轴），滚转由最小旋转决定。
  const Eigen::Matrix3d tilted =
    Eigen::AngleAxisd((EIGEN_PI / 2.0), Eigen::Vector3d::UnitX()).toRotationMatrix();
  const Eigen::Isometry3d aligned = peach_arm::pregraspFromEntryKeepRoll(
    entry, tilted, 0.03);
  EXPECT_TRUE(aligned.translation().isApprox(
      Eigen::Vector3d(0.1, 0.2, 0.57), 1e-12));
  EXPECT_NEAR(aligned.linear().col(2).dot(Eigen::Vector3d::UnitZ()), 1.0, 1e-9);
}

// ---------- segmentSegmentDistance：平行/相交/一般位置手算对拍 ----------

TEST(GraspGeometry, SegmentSegmentDistanceHandComputed)
{
  // 平行线段：相距 1 m。
  EXPECT_NEAR(
    peach_arm::segmentSegmentDistance(
      Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0),
      Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(1, 1, 0)),
    1.0, 1e-9);
  // 垂直交错（异面）：最近点对 (0,0,0)-(0,0,1)，距离 1。
  EXPECT_NEAR(
    peach_arm::segmentSegmentDistance(
      Eigen::Vector3d(-1, 0, 0), Eigen::Vector3d(1, 0, 0),
      Eigen::Vector3d(0, -1, 1), Eigen::Vector3d(0, 1, 1)),
    1.0, 1e-9);
  // 相交：距离 0。
  EXPECT_NEAR(
    peach_arm::segmentSegmentDistance(
      Eigen::Vector3d(-1, 0, 0), Eigen::Vector3d(1, 0, 0),
      Eigen::Vector3d(0, -1, 0), Eigen::Vector3d(0, 1, 0)),
    0.0, 1e-9);
  // 双退化（两点）：欧氏距离。
  EXPECT_NEAR(
    peach_arm::segmentSegmentDistance(
      Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0),
      Eigen::Vector3d(0, 0, 5), Eigen::Vector3d(0, 0, 5)),
    5.0, 1e-9);
  // 点到线段：垂足落在线段内。
  EXPECT_NEAR(
    peach_arm::segmentSegmentDistance(
      Eigen::Vector3d(0.5, 0, 3), Eigen::Vector3d(0.5, 0, 3),
      Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0)),
    3.0, 1e-9);
  // 端点 clamp：两段同轴但隔开，最近的是端点对。
  EXPECT_NEAR(
    peach_arm::segmentSegmentDistance(
      Eigen::Vector3d(2, 0, 0), Eigen::Vector3d(3, 0, 0),
      Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0)),
    1.0, 1e-9);
  // 一般异面位置：长竖线段与 x 轴线段，最近 0.5 m。
  EXPECT_NEAR(
    peach_arm::segmentSegmentDistance(
      Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0),
      Eigen::Vector3d(0.5, 10, 0.5), Eigen::Vector3d(0.5, -10, 0.5)),
    0.5, 1e-9);
}

// ---------- fruitRadiusM：下限保护与膨胀 ----------

TEST(GraspGeometry, FruitRadiusFloorAndInflation)
{
  EXPECT_NEAR(peach_arm::fruitRadiusM(0.06, 0.01, 0.10), 0.04, 1e-12);
  // 感知直径过小：落下限 0.025，再加膨胀。
  EXPECT_NEAR(peach_arm::fruitRadiusM(0.01, 0.01, 0.10), 0.035, 1e-12);
  // 无效直径（<=1e-6）用 fallback，负膨胀不缩半径。
  EXPECT_NEAR(peach_arm::fruitRadiusM(0.0, 0.0, 0.10), 0.10, 1e-12);
  EXPECT_NEAR(peach_arm::fruitRadiusM(0.06, -0.5, 0.10), 0.03, 1e-12);
}

// ---------- toolCapsuleClearance：轴向投影不重叠放行，重叠按净距 ----------

TEST(GraspGeometry, ToolCapsuleClearanceClearCase)
{
  // 工具筒体沿 +X 从 (0.2,0,0.05) 延伸到 (0.4,0,0.05)，与果轴最近距 0.2：
  // 净间隙 = 0.2 - 工具半径 0.06 - 果半径 0.04 = 0.10。
  const double clearance = peach_arm::toolCapsuleClearance(
    Eigen::Vector3d(0.2, 0, 0.05), toolQuatMinusX(), standardFruit(),
    kToolLen, kToolRad);
  EXPECT_NEAR(clearance, 0.10, 1e-9);
  // 接触态：工具拉近到轴距 0.05 → 净间隙 -0.05。
  const double hit = peach_arm::toolCapsuleClearance(
    Eigen::Vector3d(0.05, 0, 0.05), toolQuatMinusX(), standardFruit(),
    kToolLen, kToolRad);
  EXPECT_NEAR(hit, -0.05, 1e-9);
  // 轴向投影不重叠（工具整体在果上方）：不判侧撞，间隙无穷。
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(0.0, 0.0, 0.5);
  const double above = peach_arm::toolCapsuleClearance(
    pose.translation(), Eigen::Quaterniond(pose.linear()), standardFruit(),
    kToolLen, kToolRad);
  EXPECT_TRUE(std::isinf(above));
  // 胶囊关闭：恒放行。
  peach_arm::FruitCapsule disabled = standardFruit();
  disabled.enabled = false;
  EXPECT_TRUE(std::isinf(
      peach_arm::toolCapsuleClearance(
        Eigen::Vector3d(0.05, 0, 0.05), toolQuatMinusX(), disabled,
        kToolLen, kToolRad)));
}

// ---------- inspectToolVsFruit：clear/hit 两态与反爬门 ----------

TEST(GraspGeometry, InspectToolVsFruitClearAndHit)
{
  const Eigen::Quaterniond quat = toolQuatMinusX();
  // 远离点列：全间隙为正，allowed 且最小间隙 ≈ 0.10。
  std::vector<TestWaypoint> clear_points = {
    {0.5, 0.0, 0.05, quat.x(), quat.y(), quat.z(), quat.w()},
    {0.2, 0.0, 0.05, quat.x(), quat.y(), quat.z(), quat.w()}};
  const peach_arm::FruitAuditReport clear_report =
    peach_arm::inspectToolVsFruit(
      clear_points, standardFruit(), false,
      kToolLen, kToolRad);
  EXPECT_TRUE(clear_report.allowed);
  EXPECT_NEAR(clear_report.min_clearance_m, 0.10, 1e-9);
  // 接触点列：筒体压到胶囊 → 拒绝，理由含接触关键词。
  std::vector<TestWaypoint> hit_points = {
    {0.5, 0.0, 0.05, quat.x(), quat.y(), quat.z(), quat.w()},
    {0.05, 0.0, 0.05, quat.x(), quat.y(), quat.z(), quat.w()}};
  const peach_arm::FruitAuditReport hit_report =
    peach_arm::inspectToolVsFruit(
      hit_points, standardFruit(), false,
      kToolLen, kToolRad);
  EXPECT_FALSE(hit_report.allowed);
  EXPECT_NE(hit_report.reason.find("工具筒体接触果实胶囊"), std::string::npos);
}

TEST(GraspGeometry, InspectToolVsFruitClimbAuditAndDisabled)
{
  const Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();
  // 反爬门：起点 s=0 → 上限 max(0,0)+0.02；中途 s=0.05 的点判绕行拒绝。
  std::vector<TestWaypoint> climb_points = {
    {0.3, 0.0, 0.0, quat.x(), quat.y(), quat.z(), quat.w()},
    {0.3, 0.0, 0.05, quat.x(), quat.y(), quat.z(), quat.w()}};
  const peach_arm::FruitAuditReport climb =
    peach_arm::inspectToolVsFruit(
      climb_points, standardFruit(), true,
      kToolLen, kToolRad);
  EXPECT_FALSE(climb.allowed);
  EXPECT_NE(climb.reason.find("从果上方绕行"), std::string::npos);
  // 关闭胶囊：恒过。
  peach_arm::FruitCapsule disabled = standardFruit();
  disabled.enabled = false;
  const peach_arm::FruitAuditReport off =
    peach_arm::inspectToolVsFruit(
      climb_points, disabled, true,
      kToolLen, kToolRad);
  EXPECT_TRUE(off.allowed);
  // 点列不足：跳过（allowed）。
  std::vector<TestWaypoint> single = {
    {0.3, 0.0, 0.0, quat.x(), quat.y(), quat.z(), quat.w()}};
  const peach_arm::FruitAuditReport few =
    peach_arm::inspectToolVsFruit(
      single, standardFruit(), true,
      kToolLen, kToolRad);
  EXPECT_TRUE(few.allowed);
}

// ---------- toolSweepHitsFruit：lerp+slerp 扫掠采样命中 ----------

TEST(GraspGeometry, ToolSweepHitsFruitAlongSweep)
{
  const Eigen::Quaterniond quat = toolQuatMinusX();
  Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
  start.linear() = quat.toRotationMatrix();
  start.translation() = Eigen::Vector3d(0.5, 0.0, 0.05);
  // 终点压进胶囊：扫掠中段必穿过接触区 → 命中。
  Eigen::Isometry3d goal = start;
  goal.translation() = Eigen::Vector3d(0.05, 0.0, 0.05);
  EXPECT_TRUE(
    peach_arm::toolSweepHitsFruit(
      start, goal, standardFruit(),
      kToolLen, kToolRad));
  // 两端都远离：不命中。
  Eigen::Isometry3d safe_goal = start;
  safe_goal.translation() = Eigen::Vector3d(0.4, 0.0, 0.05);
  EXPECT_FALSE(
    peach_arm::toolSweepHitsFruit(
      start, safe_goal, standardFruit(),
      kToolLen, kToolRad));
  // 胶囊关闭：扫掠恒不命中。
  peach_arm::FruitCapsule disabled = standardFruit();
  disabled.enabled = false;
  EXPECT_FALSE(
    peach_arm::toolSweepHitsFruit(
      start, goal, disabled,
      kToolLen, kToolRad));
}
