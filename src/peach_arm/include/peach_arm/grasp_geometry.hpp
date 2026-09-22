// 功能：套入入口点纯函数。入口 = 锚点 − 轴·(行程 + standoff)。
// 工具筒体尺寸常量已删（W5-6）：审查函数的 tool_length/tool_radius 一律
// 由调用方注入（GraspTaskConfig.tool_body_*，yaml tool.body_*，与
// tcp.xacro tool_body_link 对齐）。
#ifndef PEACH_MANIPULATION__GRASP_GEOMETRY_HPP_
#define PEACH_MANIPULATION__GRASP_GEOMETRY_HPP_

#include "peach_arm/math_utils.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace peach_arm
{

// 最小旋转：把 current_R 的 Z 转到 axis，滚转跟着走。接触对轴只用这个，
// 不要用检测位拼出的绝对姿态（那会多转一截滚转，远处 LIN 常无 IK）。
inline Eigen::Matrix3d alignFrameZ(
  const Eigen::Matrix3d & current_R, const Eigen::Vector3d & axis)
{
  const Eigen::Vector3d z = current_R.col(2);
  if (!z.allFinite() || z.norm() < 1.0e-9 || !axis.allFinite() ||
    axis.norm() < 1.0e-9)
  {
    return current_R;
  }
  const Eigen::Quaterniond delta =
    Eigen::Quaterniond::FromTwoVectors(z, axis.normalized());
  return (delta * Eigen::Quaterniond(current_R)).normalized().toRotationMatrix();
}

// 对轴后再绕工具 Z 滚转。对应 MTC GenerateGraspPose 的 angle_delta 采样：
// 圆筒刀口方位自由；keep-roll 直线若自碰，换滚转仍走同一条位置弦。
inline Eigen::Matrix3d alignFrameZRolled(
  const Eigen::Matrix3d & current_R,
  const Eigen::Vector3d & axis,
  double roll_rad)
{
  const Eigen::Matrix3d aligned = alignFrameZ(current_R, axis);
  if (std::abs(roll_rad) < 1.0e-12) {
    return aligned;
  }
  return aligned * Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitZ());
}

// 刀口滚转：keep-roll 优先，只扫 ±30°/±60°。更大滚转会让 PTP 把 TCP
// 拧过 90°+（09-11 mock 30 例里出现 180° 姿态行程）。
inline std::vector<double> toolRollsRad()
{
  constexpr double step = static_cast<double>(EIGEN_PI) / 6.0;
  return {0.0, step, -step, 2.0 * step, -2.0 * step};
}

// 预抓取 = 入口沿 −axis 后撤 standoff_m，姿态与入口一致。
inline Eigen::Isometry3d pregraspAlongAxis(
  const Eigen::Isometry3d & entry, const Eigen::Vector3d & axis, double standoff_m)
{
  Eigen::Isometry3d pose = entry;
  if (!axis.allFinite() || axis.norm() < 1.0e-9) {
    return pose;
  }
  const double retreat = standoff_m > 0.0 ? standoff_m : 0.0;
  pose.translation() -= axis.normalized() * retreat;
  return pose;
}

inline constexpr char kHollowCylinderProfileId[] = "hollow_cylinder_v1";
inline constexpr char kAdaptiveCylinderProfileId[] = "adaptive_cylinder_v1";
/// imu_follow insert.speed_m_s 部署默认；peach 用它估等待，不读对方参数。
inline constexpr double kImuFollowInsertSpeedMps = 0.01;

/// 仅自适应圆柱在预抓取→套入→回预抓取窗内走 imu_follow；空心末端永远 false。
inline bool usesImuFollowContact(const std::string & profile_id)
{
  return profile_id == kAdaptiveCylinderProfileId;
}

/// FULL 回收纳命名目标序列：先拍照位（可倒放已过门的接近，不过 transit_max），
/// 再 harvest_stow。二者同名或 stow 空则一跳。禁止从预抓取直 PTP stow——
/// SRDF 里 stow 与 global_photo_pose 已分叉（主要是 wrist2 ≈0.22 rad），
/// goToPhotoPose(stow) 对不上接近起点，会新规划并误撞 6 rad 累计行程门。
inline std::vector<std::string> harvestStowNamedHops(
  const std::string & photo_named,
  const std::string & stow_named)
{
  const std::string photo = photo_named.empty() ?
    std::string("global_photo_pose") : photo_named;
  const std::string stow = stow_named.empty() ? photo : stow_named;
  if (stow == photo) {
    return {photo};
  }
  return {photo, stow};
}

/// 果平面接近折线（零 ROS）。拍照位往往在果上方侧向：在「当前位–袋轴」
/// 平面内 **一跳斜插** 到预抓取下方轴上 staging。模型 TCP Z=开口（法兰
/// Rx(-90°)，零位朝世界 +Z），拍照位 FK 近单位阵；悬挂袋轴亦近 +Z，对轴
/// 夹角约 8°。斜插 keep-roll 对轴（小夹角也走长斜边，避免短沿轴再拧腕）。
/// 禁止把 ±30/±60 刀口滚转叠进笛卡尔。最后沿轴垂直升到预抓取。
struct PlanarApproachHop
{
  enum class Kind
  {
    InPlaneToAxis,  ///< 面内一跳到轴上 staging（keep-roll 对轴）。
    AlongAxis,      ///< 沿轴升到预抓取（垂直果实进入）。
  };
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  Kind kind{Kind::AlongAxis};
};

struct PlanarApproachPath
{
  std::vector<PlanarApproachHop> hops;
  Eigen::Vector3d plane_normal{Eigen::Vector3d::UnitX()};
  double lateral_m{0.0};
  double axial_to_plane_m{0.0};
  /// 当前 TCP Z 与袋轴夹角（弧度）。拍照位开口已近世界 +Z，悬挂袋轴亦近
  /// +Z 时此值约 8°（09-01 现场 / SRDF FK）。
  double z_align_rad{0.0};
};

inline constexpr double kPlanarHopSkipM = 0.005;
/// 果平面折线 LIN 速度/加速度封顶。sim `--velocity 1.0` 会把
/// `velocity_scaling` 提到 1.0；斜插段带着对轴，平移 0.5 m 的时间内要拧
/// 腕，Pilz 按笛卡尔时长参数化会把 wrist1 打到 4–6 rad/s²（限 3），LIN
/// 失败后 PTP 在果高抡弧。yaml 默认 0.10 不受此封顶抬高。
inline constexpr double kPlanarLinVelCap = 0.20;
inline constexpr double kPlanarLinAccelCap = 0.10;
/// 斜插腕轴超限后的降档。151043：acc=0.10 时 tilt/travel_min/near_horizontal
/// wrist1 到 −3.09…−5.16；峰值跟笛卡尔加速度成正比，只降速度不够。
/// 0.04 ≈ 5.16×(0.04/0.10)=2.06 < 3。沿轴段已对轴，用更高档。
inline constexpr double kPlanarLinVelRetry = 0.10;
inline constexpr double kPlanarLinAccelRetry = 0.04;
inline constexpr double kPlanarAlongVelCap = 0.25;
inline constexpr double kPlanarAlongAccelCap = 0.20;
/// 对轴夹角参考：拍照位开口已近世界 +Z 时悬挂袋约 8°。笛卡尔斜插一律
/// keep-roll 对轴（夹角走长斜插）；禁止把 ±30°/±60° 刀口滚转叠进笛卡尔。
/// 小于本值仍走长斜插 keep-roll——保持拍照姿态会把残差留到 0.13 m 沿轴段，
/// wrist1 从 −3.3 恶化到 −7.1（与 delay-align 同类）。
inline constexpr double kPlanarKeepCurrentOriRad =
  15.0 * static_cast<double>(EIGEN_PI) / 180.0;

struct PlanarLinScale
{
  double velocity{kPlanarLinVelCap};
  double acceleration{kPlanarLinAccelCap};
};

/// 斜插用调用方档位（首档 0.20/0.10，失败再 0.10/0.04）；沿轴已对轴走 along_*。
inline PlanarLinScale planarHopLinScale(
  PlanarApproachHop::Kind kind,
  double in_plane_vel,
  double in_plane_acc,
  double along_vel,
  double along_acc)
{
  PlanarLinScale scale;
  if (kind == PlanarApproachHop::Kind::AlongAxis) {
    scale.velocity = along_vel > 0.0 ? along_vel : kPlanarAlongVelCap;
    scale.acceleration = along_acc > 0.0 ? along_acc : kPlanarAlongAccelCap;
    return scale;
  }
  scale.velocity = in_plane_vel > 0.0 ? in_plane_vel : kPlanarLinVelCap;
  scale.acceleration = in_plane_acc > 0.0 ? in_plane_acc : kPlanarLinAccelCap;
  return scale;
}

/// 把折线各跳姿态换成给定线性部。笛卡尔斜插禁止用带刀口滚转的候选姿态
/// 调用——会在 0.5 m LIN 上拧 Rz。PTP 兜底才按候选关节滚转。
inline void applyPlanarApproachOrientation(
  PlanarApproachPath & path, const Eigen::Matrix3d & linear)
{
  for (auto & hop : path.hops) {
    hop.pose.linear() = linear;
  }
}

inline double planarApproachMaxHopM(
  const Eigen::Isometry3d & current, const PlanarApproachPath & path)
{
  Eigen::Vector3d prev = current.translation();
  double max_m = 0.0;
  for (const auto & hop : path.hops) {
    max_m = std::max(max_m, (hop.pose.translation() - prev).norm());
    prev = hop.pose.translation();
  }
  return max_m;
}

inline const char * planarApproachHopName(PlanarApproachHop::Kind kind)
{
  switch (kind) {
    case PlanarApproachHop::Kind::InPlaneToAxis:
      return "lin in plane to bag axis";
    case PlanarApproachHop::Kind::AlongAxis:
      return "lin to on-axis pregrasp";
  }
  return "lin planar hop";
}

inline PlanarApproachPath planarApproachHops(
  const Eigen::Isometry3d & current,
  const Eigen::Isometry3d & entry,
  const Eigen::Vector3d & insertion_axis,
  double along_axis_m,
  double staging_standoff_m)
{
  PlanarApproachPath path;
  if (!insertion_axis.allFinite() || insertion_axis.norm() < 1.0e-9 ||
    staging_standoff_m <= kPlanarHopSkipM ||
    !current.translation().allFinite() || !entry.translation().allFinite())
  {
    return path;
  }
  const Eigen::Vector3d axis = insertion_axis.normalized();
  const Eigen::Isometry3d pregrasp = pregraspAlongAxis(entry, axis, along_axis_m);
  const Eigen::Vector3d staging_p =
    pregrasp.translation() - axis * staging_standoff_m;
  const Eigen::Vector3d current_p = current.translation();
  if ((current_p - pregrasp.translation()).norm() <= kPlanarHopSkipM) {
    return path;
  }
  const Eigen::Vector3d rel = current_p - staging_p;
  const double axial = rel.dot(axis);
  const Eigen::Vector3d lat_vec = rel - axis * axial;
  path.lateral_m = lat_vec.norm();
  path.axial_to_plane_m = axial;
  if (path.lateral_m > 1.0e-9) {
    path.plane_normal = axis.cross(lat_vec).normalized();
  } else {
    path.plane_normal = axis.unitOrthogonal();
  }
  const Eigen::Vector3d current_z = current.linear().col(2).normalized();
  path.z_align_rad = std::acos(
    std::clamp(current_z.dot(axis), -1.0, 1.0));
  const Eigen::Matrix3d aligned = alignFrameZ(current.linear(), axis);
  auto emit = [&](const Eigen::Vector3d & point, PlanarApproachHop::Kind kind) {
      const Eigen::Vector3d prev = path.hops.empty() ?
        current_p : path.hops.back().pose.translation();
      if ((point - prev).norm() <= kPlanarHopSkipM) {
        return;
      }
      PlanarApproachHop hop;
      hop.pose.translation() = point;
      // keep-roll 对轴：FromTwoVectors 只把 Z 转到袋轴，不叠刀口 Rz。
      // 小夹角也放在长斜插上——沿轴 0.13 m 再拧 8° 会打爆 wrist1；保持拍照
      // 姿态同样把残差留到短沿轴（typical −7.15 / 限 3）。
      hop.pose.linear() = aligned;
      hop.kind = kind;
      path.hops.push_back(std::move(hop));
    };
  emit(staging_p, PlanarApproachHop::Kind::InPlaneToAxis);
  emit(pregrasp.translation(), PlanarApproachHop::Kind::AlongAxis);
  return path;
}

/// 插入或回退行程的墙钟等待（速度默认 0.01 m/s + 0.5 s 到位裕量，封顶 25 s）。
inline double imuFollowTravelWaitS(double travel_m, double speed_m_s)
{
  const double speed = speed_m_s > 1.0e-9 ? speed_m_s : kImuFollowInsertSpeedMps;
  const double travel = travel_m > 0.0 ? travel_m : 0.0;
  return std::min(25.0, travel / speed + 0.5);
}

// SELECT / CheckReachability 与 MovePregrasp 同一停位：请求当入口（Z=袋轴），
// 位置沿 −Z 后撤 standoff，姿态 alignFrameZ 保留当前 TCP 滚转。轴无效则原样返回。
inline Eigen::Isometry3d pregraspFromEntryKeepRoll(
  const Eigen::Isometry3d & entry,
  const Eigen::Matrix3d & current_R,
  double standoff_m)
{
  const Eigen::Vector3d axis = entry.linear().col(2);
  Eigen::Isometry3d pose = pregraspAlongAxis(entry, axis, standoff_m);
  if (!axis.allFinite() || axis.norm() < 1.0e-9) {
    return pose;
  }
  pose.linear() = alignFrameZ(current_R, axis);
  return pose;
}

// 套入终点 = 入口沿 +axis 前插 travel_m，姿态与入口一致。负/零行程不前插。
inline Eigen::Isometry3d sleeveAlongAxis(
  const Eigen::Isometry3d & entry, const Eigen::Vector3d & axis, double travel_m)
{
  Eigen::Isometry3d pose = entry;
  if (!axis.allFinite() || axis.norm() < 1.0e-9) {
    return pose;
  }
  const double travel = travel_m > 0.0 ? travel_m : 0.0;
  pose.translation() += axis.normalized() * travel;
  return pose;
}

// SELECT / CheckReachability 套入终点与接触 LIN 同一几何：入口沿袋轴前插
// travel，姿态 alignFrameZ 保留当前 TCP 滚转。轴无效则原样返回。
inline Eigen::Isometry3d sleeveFromEntryKeepRoll(
  const Eigen::Isometry3d & entry,
  const Eigen::Matrix3d & current_R,
  double travel_m)
{
  const Eigen::Vector3d axis = entry.linear().col(2);
  Eigen::Isometry3d pose = sleeveAlongAxis(entry, axis, travel_m);
  if (!axis.allFinite() || axis.norm() < 1.0e-9) {
    return pose;
  }
  pose.linear() = alignFrameZ(current_R, axis);
  return pose;
}

// 接触行程夹到 [min, max]；未给/非正则用 min（与 yaml grasp.min/max 对齐）。
inline double clampInsertionTravel(double suggested_m, double min_m, double max_m)
{
  const double lo = min_m > 0.0 ? min_m : 0.0;
  const double hi = max_m > lo ? max_m : lo;
  if (suggested_m <= 1.0e-6) {
    return lo;
  }
  return std::clamp(suggested_m, lo, hi);
}

// ExecuteTarget.mode / profile 数值与 IDL 对齐。profile=0 是默认 HOLD，
// 不得把 mode=FULL 盖成停预抓取（旧客户端不填 profile）。PROFILE_FULL
// 强制套入。PREVIEW / OBSERVE_ONLY 不被默认 HOLD 改成预抓取停。
constexpr uint8_t kExecutePreview = 0;
constexpr uint8_t kExecuteObserveOnly = 1;
constexpr uint8_t kExecuteFull = 2;
constexpr uint8_t kExecutePregraspOnly = 3;
constexpr uint8_t kProfilePregraspHold = 0;
constexpr uint8_t kProfileFull = 1;

inline bool pregraspOnlyFromGoal(uint8_t mode, uint8_t profile)
{
  if (profile == kProfileFull) {
    return false;
  }
  if (mode == kExecuteFull) {
    return false;
  }
  return mode == kExecutePregraspOnly;
}

/// SELECT / CheckReachability 失败码优先级：预抓取 IK → 套入终点 IK → 沿轴笛卡尔。
inline const char * sleeveReachabilityCode(
  bool pregrasp_ik, bool sleeve_ik, bool sleeve_cartesian)
{
  if (!pregrasp_ik) {
    return "no_ik";
  }
  if (!sleeve_ik) {
    return "sleeve_no_ik";
  }
  if (!sleeve_cartesian) {
    return "sleeve_no_cartesian";
  }
  return "";
}

// 果实胶囊（2026-09-14 约束重设计）：感知拟合圆柱 bottom→neck 的有限段
// + 逐目标半径（直径/2 + 固定膨胀）。保护果实的口径是「工具有限圆柱不
// 与本胶囊在轴向投影重叠区内相交」；从下方接近由反爬规则（锚定果底
// 平面）另行保证。
// diameter<=0（感知无效）时用 fallback_radius_m 保守值，调用方告警。
// 半无限 BagKeepout 已删（G 弦时代产物；2026-09-10 探针）。
struct FruitCapsule
{
  Eigen::Vector3d bottom{Eigen::Vector3d::Zero()};  ///< 果底中心（胶囊段起点）[m]，base 系。
  Eigen::Vector3d neck{Eigen::Vector3d::Zero()};    ///< 果颈中心（胶囊段终点）[m]，base 系。
  Eigen::Vector3d axis{Eigen::Vector3d::UnitZ()};   ///< 果轴单位向量（bottom→neck）。
  double radius_m{0.05};  ///< 胶囊半径 [m]（含膨胀；工具间隙据此扣减）。
  bool enabled{true};     ///< false=审查整体关闭（感知无效时调用方决定）。
};

/// 果实胶囊保护半径：diameter>0 取半径，否则回退 fallback；下限 0.025 m
/// 防感知给极小直径把保护半径归零，另加固定膨胀。
inline double fruitRadiusM(
  double diameter_m, double inflation_m, double fallback_radius_m)
{
  const double base = diameter_m > 1.0e-6 ? diameter_m / 2.0 : fallback_radius_m;
  return std::max(0.025, base) + std::max(0.0, inflation_m);
}

/// 由感知拟合量组装 FruitCapsule；轴退化（近零）按 +Z 兜底，保证审查
/// 几何恒有效（NaN/零轴进审查会把所有点判同侧）。
inline FruitCapsule fruitCapsuleFrom(
  const Eigen::Vector3d & bottom, const Eigen::Vector3d & neck,
  const Eigen::Vector3d & axis, double diameter_m, double inflation_m,
  double fallback_radius_m, bool enabled)
{
  FruitCapsule fruit;
  fruit.bottom = bottom;
  fruit.neck = neck;
  fruit.axis = axis.norm() > 1.0e-9 ? axis.normalized() : Eigen::Vector3d::UnitZ();
  fruit.radius_m = fruitRadiusM(diameter_m, inflation_m, fallback_radius_m);
  fruit.enabled = enabled;
  return fruit;
}

/// 胶囊审查是否整体关闭：enabled=false、半径非正或任一几何量非有限时
/// 调用方按「关闭」处理（返回无穷间隙），不产生误判拒发。
inline bool fruitCapsuleDisabled(const FruitCapsule & fruit)
{
  return !fruit.enabled || fruit.radius_m <= 1.0e-6 ||
         !fruit.axis.allFinite() || !fruit.bottom.allFinite() ||
         !fruit.neck.allFinite();
}

// 点相对果底平面的轴向/径向（反爬与诊断用；s=0 在果底）。
inline void axialRadial(
  const Eigen::Vector3d & point, const FruitCapsule & fruit,
  double & axial_m, double & radial_m)
{
  const Eigen::Vector3d delta = point - fruit.bottom;
  axial_m = delta.dot(fruit.axis);
  radial_m = (delta - axial_m * fruit.axis).norm();
}

// 线段-线段最近距离（clamp 到两端，退化共线取端点距）。
// d(s,t)²=|r+s·d1−t·d2|² 的驻点解两步 clamp：先解 s 并 clamp，再回代解 t
// 并 clamp，最后对 s 复核一次（审计用途，近似即够）。
inline double segmentSegmentDistance(
  const Eigen::Vector3d & p1, const Eigen::Vector3d & q1,
  const Eigen::Vector3d & p2, const Eigen::Vector3d & q2)
{
  const Eigen::Vector3d d1 = q1 - p1;
  const Eigen::Vector3d d2 = q2 - p2;
  const Eigen::Vector3d r = p1 - p2;
  const double a = d1.squaredNorm();
  const double b = d1.dot(d2);
  const double c = d2.squaredNorm();
  const double f = r.dot(d1);
  const double g = r.dot(d2);
  const double denom = a * c - b * b;
  double s = 0.0;
  double t = 0.0;
  if (a <= 1.0e-12 && c <= 1.0e-12) {
    return r.norm();
  }
  if (a <= 1.0e-12) {
    t = std::clamp(g / c, 0.0, 1.0);
  } else if (c <= 1.0e-12) {
    s = std::clamp(-f / a, 0.0, 1.0);
  } else {
    s = denom > 1.0e-12 ?
      std::clamp((b * g - c * f) / denom, 0.0, 1.0) : 0.0;
    t = std::clamp((b * s + g) / c, 0.0, 1.0);
    s = std::clamp((b * t - f) / a, 0.0, 1.0);
  }
  return (r + s * d1 - t * d2).norm();
}

/// 工具筒体尾点（TCP 沿 −Z 退 tool_length）：有限圆柱段的另一端，
/// 胶囊间隙按 TCP→尾点线段计算。
inline Eigen::Vector3d toolTailPoint(
  const Eigen::Vector3d & tcp, const Eigen::Quaterniond & tcp_quat,
  double tool_length_m)
{
  const Eigen::Vector3d tool_z = tcp_quat.normalized() * Eigen::Vector3d::UnitZ();
  return tcp - tool_z * tool_length_m;
}

/// 两轴向区间（果底→果颈 vs 工具首尾投影）是否重叠（含 1e-9 容差）：
/// 不重叠则径向接触不伤果，直接判无穷间隙。
inline bool axialRangesOverlap(double a0, double a1, double b0, double b1)
{
  const double amin = std::min(a0, a1);
  const double amax = std::max(a0, a1);
  const double bmin = std::min(b0, b1);
  const double bmax = std::max(b0, b1);
  return amin <= bmax + 1.0e-9 && bmin <= amax + 1.0e-9;
}

/// 工具有限圆柱 vs 果实胶囊的径向间隙 [m]：轴向投影不重叠或审查关闭
/// 返回 +∞；否则为线段距 − 工具半径 − 果半径（<=0 即接触）。
inline double toolCapsuleClearance(
  const Eigen::Vector3d & tcp, const Eigen::Quaterniond & tcp_quat,
  const FruitCapsule & fruit, double tool_length_m, double tool_radius_m)
{
  if (fruitCapsuleDisabled(fruit)) {
    return std::numeric_limits<double>::infinity();
  }
  const Eigen::Vector3d tail = toolTailPoint(tcp, tcp_quat, tool_length_m);
  double tool_s0 = 0.0;
  double tool_s1 = 0.0;
  double fruit_s0 = 0.0;
  double fruit_s1 = 0.0;
  double unused = 0.0;
  axialRadial(tcp, fruit, tool_s0, unused);
  axialRadial(tail, fruit, tool_s1, unused);
  axialRadial(fruit.bottom, fruit, fruit_s0, unused);
  axialRadial(fruit.neck, fruit, fruit_s1, unused);
  if (!axialRangesOverlap(tool_s0, tool_s1, fruit_s0, fruit_s1)) {
    return std::numeric_limits<double>::infinity();
  }
  return segmentSegmentDistance(tcp, tail, fruit.bottom, fruit.neck) -
         tool_radius_m - fruit.radius_m;
}

/// 工具×果实胶囊逐点审查结果。
struct FruitAuditReport
{
  bool allowed{true};  ///< 是否放行（接触胶囊或反爬超限即 false）。
  std::string reason{"果实胶囊审查通过"};  ///< 拒发原因（含量测值，人读）。
  double min_clearance_m{std::numeric_limits<double>::infinity()};  ///< 全程最小间隙 [m]（含通过时的观测值）。
};

// audit_climb=false 只查工具×果实胶囊接触（staging 转移首段 PTP 弧用：
// 拍照位本就在果上方，锚定起点的反爬门会把关节弧 2–4 cm 拱高误判绕行）。
// =true 另查反爬：TCP 的 s 不得超过 max(本段起点 s, 0)+2 cm（锚定果底）。
// 套入/撤退段不走本审查（任务性穿果）。
template<typename Waypoint>
inline FruitAuditReport inspectToolVsFruit(
  const std::vector<Waypoint> & points, const FruitCapsule & fruit,
  bool audit_climb, double tool_length_m, double tool_radius_m)
{
  FruitAuditReport report;
  if (fruitCapsuleDisabled(fruit)) {
    report.reason = "果实胶囊审查关闭";
    return report;
  }
  if (points.size() < 2U) {
    report.reason = "果实胶囊审查：点列不足，跳过";
    return report;
  }
  double start_s = 0.0;
  double start_r = 0.0;
  axialRadial(
    Eigen::Vector3d(points.front().x, points.front().y, points.front().z),
    fruit, start_s, start_r);
  (void)start_r;
  const double s_max = (start_s > 0.0 ? start_s : 0.0) + 0.02;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const Eigen::Vector3d tcp(points[i].x, points[i].y, points[i].z);
    const Eigen::Quaterniond quat(
      points[i].qw, points[i].qx, points[i].qy, points[i].qz);
    double axial_m = 0.0;
    double radial_m = 0.0;
    axialRadial(tcp, fruit, axial_m, radial_m);
    if (audit_climb && axial_m > s_max) {
      report.allowed = false;
      std::ostringstream reason;
      reason << "从果上方绕行 s=" << axial_m << "m > 上限 " << s_max << "m";
      report.reason = reason.str();
      return report;
    }
    const double clearance = toolCapsuleClearance(
      tcp, quat, fruit, tool_length_m, tool_radius_m);
    report.min_clearance_m = std::min(report.min_clearance_m, clearance);
    if (clearance <= 0.0) {
      report.allowed = false;
      std::ostringstream reason;
      reason << "工具筒体接触果实胶囊 s=" << axial_m << "m r=" << radial_m
             << "m 间隙=" << clearance << "m (果半径 " << fruit.radius_m << "m)";
      report.reason = reason.str();
      return report;
    }
  }
  return report;
}

// 直连 LIN 资格用：从 start 到 goal 的工具扫掠（位置 lerp + 姿态 slerp）
// 是否接触果实胶囊。姿态用两端四元数（调用方给对齐后姿态）。
inline bool toolSweepHitsFruit(
  const Eigen::Isometry3d & start, const Eigen::Isometry3d & goal,
  const FruitCapsule & fruit, double tool_length_m, double tool_radius_m,
  std::size_t samples = 24U)
{
  if (fruitCapsuleDisabled(fruit)) {
    return false;
  }
  const Eigen::Quaterniond q0(start.linear());
  const Eigen::Quaterniond q1(goal.linear());
  const std::size_t count = samples < 2U ? 2U : samples;
  struct W
  {
    double x, y, z, qx, qy, qz, qw;
  };
  std::vector<W> points;
  points.reserve(count + 1U);
  for (std::size_t i = 0; i <= count; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(count);
    const Eigen::Vector3d p =
      (1.0 - t) * start.translation() + t * goal.translation();
    const Eigen::Quaterniond q = q0.slerp(t, q1);
    points.push_back({p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w()});
  }
  const FruitAuditReport report = inspectToolVsFruit(
    points, fruit, false, tool_length_m, tool_radius_m);
  return !report.allowed;
}

}  // namespace peach_arm
#endif  // PEACH_MANIPULATION__GRASP_GEOMETRY_HPP_
