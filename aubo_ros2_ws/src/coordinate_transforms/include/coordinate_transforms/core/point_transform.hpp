#ifndef COORDINATE_TRANSFORMS_CORE_POINT_TRANSFORM_HPP
#define COORDINATE_TRANSFORMS_CORE_POINT_TRANSFORM_HPP

#include <array>
#include <vector>

namespace coordinate_transforms {
namespace core {

using Point3 = std::array<double, 3>;
using Matrix3 = std::array<double, 9>;
using Matrix4 = std::array<double, 16>;

enum class Handedness { Right, Left };

// --- Handedness ---
/** 3x3 reflection matrix: right <-> left (e.g. flip Z: diag(1,1,-1)). */
Matrix3 handedness_transform(Handedness from, Handedness to);
Matrix3 right_to_left();
Matrix3 left_to_right();

// --- Single point ---
Point3 transform_point(const Point3& p, const Matrix4& T);
Point3 transform_point(const Point3& p, const Matrix3& R, const Point3& t);

// --- Direction (rotation only) ---
Point3 transform_direction(const Point3& v, const Matrix3& R, bool normalize = true);
Point3 transform_direction(const Point3& v, const Matrix4& T, bool normalize = true);

// --- Batch points ---
std::vector<Point3> transform_points(const std::vector<Point3>& P, const Matrix4& T);

// --- Frame / pose: origin + rotation ---
void transform_pose(const Point3& origin, const Matrix3& R, const Matrix4& T_B_A,
                    Point3& new_origin, Matrix3& new_R);
/** Full 4x4 frame: T_new = T_B_A * T_old. */
Matrix4 transform_frame(const Matrix4& T_old, const Matrix4& T_B_A);

// --- Multiple directions ---
std::vector<Point3> transform_directions(const std::vector<Point3>& V, const Matrix3& R, bool normalize = true);

// --- Line (point + direction) ---
void transform_line(const Point3& point, const Point3& direction, const Matrix4& T,
                    Point3& point_out, Point3& direction_out);

}  // namespace core
}  // namespace coordinate_transforms

#endif
