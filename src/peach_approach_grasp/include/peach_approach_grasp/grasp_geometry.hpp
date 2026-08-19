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
#ifndef PEACH_APPROACH_GRASP__GRASP_GEOMETRY_HPP_
#define PEACH_APPROACH_GRASP__GRASP_GEOMETRY_HPP_

#include <Eigen/Geometry>

namespace peach_approach_grasp
{

// 降级/重算的入口点构造（纯函数）：入口点 = 锚点 − 轴·(行程 + standoff)。
// standoff 是袋外预入口余量（管进袋前净空），与入袋后越过袋口的颈部余量
// neck_margin 互补：一个在 entry 构造里后退，一个在 insertionTravel 里前送，
// 两者作用于行程两端，不叠加也不互相替代。axis 为零向量时按原样返回 NaN 传播
// 防护交给调用方（调用前必须保证 nonzeroFinite(axis)）。
inline Eigen::Vector3d degradedEntryPoint(
  const Eigen::Vector3d & anchor, const Eigen::Vector3d & axis,
  double travel_m, double standoff_m)
{
  return anchor - axis.normalized() * (travel_m + standoff_m);
}

}  // namespace peach_approach_grasp
#endif  // PEACH_APPROACH_GRASP__GRASP_GEOMETRY_HPP_
