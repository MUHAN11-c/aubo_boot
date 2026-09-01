// 功能：环境保护区 AABB（base_link，米）。参数 stride-6：[xmin,ymin,zmin,xmax,ymax,zmax]*N。
// 零 ROS。视点落入盒内则剔除；也用于 MTC 场景 keepout。
#ifndef PEACH_MANIPULATION__PROTECTED_ZONES_HPP_
#define PEACH_MANIPULATION__PROTECTED_ZONES_HPP_

#include <Eigen/Geometry>

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace peach_manipulation
{

// 单个保护区：base 系轴对齐盒，min/max 为两个对角点。
// 包含判定采用闭区间（盒表面视为盒内）：相机/入口贴着盒面与穿入盒内同样
// 危险，边界不留缝隙。
struct ProtectedZone
{
  Eigen::Vector3d min{Eigen::Vector3d::Zero()};
  Eigen::Vector3d max{Eigen::Vector3d::Zero()};
};

// parseProtectedZones 输出：合法盒 + 丢弃原因（调用方 WARN，不炸节点）。
struct ProtectedZoneParseResult
{
  std::vector<ProtectedZone> zones;
  std::vector<std::string> issues;
};

// 解析 stride-6。长度非 6 倍数：完整盒保留，残余丢弃。单盒非有限或 min>=max：丢该盒。
inline ProtectedZoneParseResult parseProtectedZones(const std::vector<double> & flat)
{
  ProtectedZoneParseResult result;
  const std::size_t complete = flat.size() / 6;
  if (flat.size() % 6 != 0) {
    result.issues.push_back(
      "长度 " + std::to_string(flat.size()) + " 非 6 倍数，末尾 " +
      std::to_string(flat.size() - complete * 6) + " 个分量的残余组已丢弃");
  }
  for (std::size_t i = 0; i < complete; ++i) {
    ProtectedZone zone;
    zone.min = Eigen::Vector3d(flat[i * 6], flat[i * 6 + 1], flat[i * 6 + 2]);
    zone.max = Eigen::Vector3d(flat[i * 6 + 3], flat[i * 6 + 4], flat[i * 6 + 5]);
    if (!zone.min.allFinite() || !zone.max.allFinite()) {
      result.issues.push_back(
        "盒#" + std::to_string(i) + " 含非有限分量，已丢弃");
      continue;
    }
    if ((zone.min.array() >= zone.max.array()).any()) {
      result.issues.push_back(
        "盒#" + std::to_string(i) + " 存在 min>=max 的轴（空盒/退化盒），已丢弃");
      continue;
    }
    result.zones.push_back(zone);
  }
  return result;
}

// 闭区间含表面；空列表恒 nullopt。命中返回盒序号。
inline std::optional<std::size_t> protectedZoneHit(
  const Eigen::Vector3d & point, const std::vector<ProtectedZone> & zones)
{
  for (std::size_t i = 0; i < zones.size(); ++i) {
    if ((point.array() >= zones[i].min.array()).all() &&
      (point.array() <= zones[i].max.array()).all())
    {
      return i;
    }
  }
  return std::nullopt;
}

}  // namespace peach_manipulation

#endif  // PEACH_MANIPULATION__PROTECTED_ZONES_HPP_
