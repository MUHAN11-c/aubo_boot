/**
 * Export sample 3D points and 2D projection to PCD (and optionally CSV for 2D).
 * No OpenCV dependency. With OpenCV, use opencv_process_data for image/PNG.
 * Output dir: COORD_TF_OUTPUT_DIR env or "./coord_tf_output".
 */
#include "coordinate_transforms/core/coordinate_systems.hpp"
#include "coordinate_transforms/core/rotation_conversion.hpp"
#include <fstream>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <string>
#include <sys/stat.h>

static std::string get_output_dir() {
  const char* env = std::getenv("COORD_TF_OUTPUT_DIR");
  if (env && env[0] != '\0') return std::string(env);
  return "coord_tf_output";
}

int main() {
  std::string out_dir = get_output_dir();
  mkdir(out_dir.c_str(), 0755);

  coordinate_transforms::core::CameraIntrinsics K;
  K.fx = 500.0;
  K.fy = 500.0;
  K.cx = 320.0;
  K.cy = 240.0;

  std::vector<coordinate_transforms::core::Point3> P_3d = {
    {0.1, 0.0, 1.0},
    {0.0, 0.1, 1.0},
    {-0.05, -0.05, 0.8}
  };

  auto uv = coordinate_transforms::core::project_3d_to_2d_batch(P_3d, K);

  std::string pcd_path = out_dir + "/coord_tf_sample.pcd";
  std::ofstream pcd(pcd_path);
  pcd << "# .PCD - Point Cloud Data\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n";
  pcd << "WIDTH " << P_3d.size() << "\nHEIGHT 1\nPOINTS " << P_3d.size() << "\nDATA ascii\n";
  for (const auto& p : P_3d)
    pcd << p[0] << " " << p[1] << " " << p[2] << "\n";
  pcd.close();
  std::cout << "Wrote " << pcd_path << " (" << P_3d.size() << " points)\n";

  std::string csv_path = out_dir + "/coord_tf_sample_2d.csv";
  std::ofstream csv(csv_path);
  csv << "u,v\n";
  for (const auto& p : uv)
    csv << p[0] << "," << p[1] << "\n";
  csv.close();
  std::cout << "Wrote " << csv_path << "\n";

  return 0;
}
