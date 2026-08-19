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
// 职责：环境几何保护区（重构计划阶段 F1）纯核——base_link 系轴对齐盒（AABB）
// 列表的参数解析与点包含判定，零 ROS 依赖、纯函数、线程无关。
// 输入：scan.protected_zones 参数（double 扁平数组，stride-6 编码
//   [xmin,ymin,zmin,xmax,ymax,zmax]*N，单位米）。
// 输出：合法盒列表 + 逐条丢弃原因（节点侧 WARN）；点命中判定（供视点生成
//   剔除与 MTC 接触段入口剔除两处调用端共用同一谓词，保证剔除语义一致）。
#ifndef PEACH_APPROACH_GRASP__PROTECTED_ZONES_HPP_
#define PEACH_APPROACH_GRASP__PROTECTED_ZONES_HPP_

#include <Eigen/Geometry>

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace peach_approach_grasp
{

// 单个保护区：base 系轴对齐盒，min/max 为两个对角点。
// 包含判定采用闭区间（盒表面视为盒内）：相机/入口贴着盒面与穿入盒内同样
// 危险，边界不留缝隙。
struct ProtectedZone
{
  Eigen::Vector3d min{Eigen::Vector3d::Zero()};
  Eigen::Vector3d max{Eigen::Vector3d::Zero()};
};

// parseProtectedZones 输出：合法盒与逐条丢弃原因（调用方逐条 WARN，不炸节点）。
struct ProtectedZoneParseResult
{
  std::vector<ProtectedZone> zones;
  std::vector<std::string> issues;
};

// 解析 scan.protected_zones 扁平数组（stride-6）。
// 失败语义（协议：告警并丢弃该盒，不炸节点）：
//   - 长度非 6 倍数：末尾不完整残余组整体丢弃（记一条 issue），前面的完整
//     盒照常解析——配置笔误不应拖垮全部保护区；
//   - 单盒含非有限分量或任一轴 min>=max（空盒/退化盒/大小写反）：该盒丢弃，
//     其余盒不受影响。
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

// 点是否落入任一保护盒（闭区间，含盒表面）；命中返回盒序号供原因文案使用。
// 空保护区列表恒返回 nullopt（默认配置零开销快速路径）。
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

}  // namespace peach_approach_grasp

#endif  // PEACH_APPROACH_GRASP__PROTECTED_ZONES_HPP_
