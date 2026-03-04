#include "coordinate_transforms/core/point_transform.hpp"
#include <cmath>

namespace coordinate_transforms {
namespace core {

// 手系变换矩阵：右手系↔左手系。右手系 X×Y=Z，左手系 X×Y=-Z。
// 公式: 仅旋转时用反射矩阵，如 M = diag(1, 1, -1) 使 Z 取反，即 [x,y,z]' -> [x,y,-z]。
Matrix3 handedness_transform(Handedness from, Handedness to) {
  if (from == to) {
    return {1,0,0, 0,1,0, 0,0,1};
  }
  return {1,0,0, 0,1,0, 0,0,-1};
}

Matrix3 right_to_left() {
  return handedness_transform(Handedness::Right, Handedness::Left);
}

Matrix3 left_to_right() {
  return handedness_transform(Handedness::Left, Handedness::Right);
}

// 旋转: p' = R*p（行优先 R[0..8] 对应 3×3）。
static Point3 apply_R(const Point3& p, const Matrix3& R) {
  return {
    R[0]*p[0] + R[1]*p[1] + R[2]*p[2],
    R[3]*p[0] + R[4]*p[1] + R[5]*p[2],
    R[6]*p[0] + R[7]*p[1] + R[8]*p[2]
  };
}

// 刚体变换: p' = T*[p;1] 取前三维，即 p' = R*p + t。
static Point3 apply_T(const Point3& p, const Matrix4& T) {
  return {
    T[0]*p[0] + T[1]*p[1] + T[2]*p[2] + T[3],
    T[4]*p[0] + T[5]*p[1] + T[6]*p[2] + T[7],
    T[8]*p[0] + T[9]*p[1] + T[10]*p[2] + T[11]
  };
}

// 单点变换: p' = T*p（齐次），等价 p' = R*p + t。
Point3 transform_point(const Point3& p, const Matrix4& T) {
  return apply_T(p, T);
}

// 单点变换: p' = R*p + t。
Point3 transform_point(const Point3& p, const Matrix3& R, const Point3& t) {
  Point3 rp = apply_R(p, R);
  return {rp[0]+t[0], rp[1]+t[1], rp[2]+t[2]};
}

// 方向向量变换: v' = R*v（仅旋转，无平移）；可选单位化 v' := v'/|v'|。
Point3 transform_direction(const Point3& v, const Matrix3& R, bool normalize) {
  Point3 out = apply_R(v, R);
  if (normalize) {
    double n = std::sqrt(out[0]*out[0] + out[1]*out[1] + out[2]*out[2]);
    if (n > 1e-10) { out[0]/=n; out[1]/=n; out[2]/=n; }
  }
  return out;
}

Point3 transform_direction(const Point3& v, const Matrix4& T, bool normalize) {
  Matrix3 R = {T[0],T[1],T[2], T[4],T[5],T[6], T[8],T[9],T[10]};
  return transform_direction(v, R, normalize);
}

// 多点变换: 对每点 p_i 做 p'_i = T*p_i（齐次），即 P' = (T * P^T)^T 取前 3 列。
std::vector<Point3> transform_points(const std::vector<Point3>& P, const Matrix4& T) {
  std::vector<Point3> out(P.size());
  for (size_t i = 0; i < P.size(); ++i) out[i] = apply_T(P[i], T);
  return out;
}

// 位姿/架变换: 原点 origin' = T_B_A*origin；姿态 new_R = R_B_A * R（矩阵乘）。
// 即完整架 T_old = [R|origin; 0 0 0 1]，变换后 T_new = T_B_A * T_old。
void transform_pose(const Point3& origin, const Matrix3& R, const Matrix4& T_B_A,
                    Point3& new_origin, Matrix3& new_R) {
  new_origin = apply_T(origin, T_B_A);
  Matrix3 R_B_A = {T_B_A[0],T_B_A[1],T_B_A[2], T_B_A[4],T_B_A[5],T_B_A[6], T_B_A[8],T_B_A[9],T_B_A[10]};
  new_R[0] = R_B_A[0]*R[0] + R_B_A[1]*R[3] + R_B_A[2]*R[6];
  new_R[1] = R_B_A[0]*R[1] + R_B_A[1]*R[4] + R_B_A[2]*R[7];
  new_R[2] = R_B_A[0]*R[2] + R_B_A[1]*R[5] + R_B_A[2]*R[8];
  new_R[3] = R_B_A[3]*R[0] + R_B_A[4]*R[3] + R_B_A[5]*R[6];
  new_R[4] = R_B_A[3]*R[1] + R_B_A[4]*R[4] + R_B_A[5]*R[7];
  new_R[5] = R_B_A[3]*R[2] + R_B_A[4]*R[5] + R_B_A[5]*R[8];
  new_R[6] = R_B_A[6]*R[0] + R_B_A[7]*R[3] + R_B_A[8]*R[6];
  new_R[7] = R_B_A[6]*R[1] + R_B_A[7]*R[4] + R_B_A[8]*R[7];
  new_R[8] = R_B_A[6]*R[2] + R_B_A[7]*R[5] + R_B_A[8]*R[8];
}

// 完整架变换: T_new = T_B_A * T_old（4×4 矩阵乘）。同一刚体变换作用到架的原点与姿态。
Matrix4 transform_frame(const Matrix4& T_old, const Matrix4& T_B_A) {
  Matrix4 out{};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) {
      out[i*4+j] = 0;
      for (int k = 0; k < 4; ++k)
        out[i*4+j] += T_B_A[i*4+k] * T_old[k*4+j];
    }
  return out;
}

// 多方向向量: 每轴 v'_i = R*v_i，可选单位化。
std::vector<Point3> transform_directions(const std::vector<Point3>& V, const Matrix3& R, bool normalize) {
  std::vector<Point3> out(V.size());
  for (size_t i = 0; i < V.size(); ++i) out[i] = transform_direction(V[i], R, normalize);
  return out;
}

// 直线（一点+方向）变换: point' = T*point；direction' = R*direction 并单位化（方向仅旋转）。
void transform_line(const Point3& point, const Point3& direction, const Matrix4& T,
                    Point3& point_out, Point3& direction_out) {
  point_out = apply_T(point, T);
  Matrix3 R = {T[0],T[1],T[2], T[4],T[5],T[6], T[8],T[9],T[10]};
  direction_out = transform_direction(direction, R, true);
}

}  // namespace core
}  // namespace coordinate_transforms
