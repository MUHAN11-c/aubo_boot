/**
 * Optional: with OpenCV, draw 2D points and reprojection error, save to PNG/YAML.
 * Built only when find_package(OpenCV) succeeds.
 * Output dir: COORD_TF_OUTPUT_DIR env or "./coord_tf_output".
 */
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "coordinate_transforms/core/coordinate_systems.hpp"
#include <iostream>
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

  cv::Mat img(480, 640, CV_8UC3, cv::Scalar(40, 40, 40));

  coordinate_transforms::core::CameraIntrinsics K;
  K.fx = 500.0;
  K.fy = 500.0;
  K.cx = 320.0;
  K.cy = 240.0;

  coordinate_transforms::core::Point3 p3d = {0.1, 0.0, 1.0};
  auto uv_proj = coordinate_transforms::core::project_3d_to_2d(p3d, K);
  coordinate_transforms::core::Point2 obs = {uv_proj[0] + 5.0, uv_proj[1] + 5.0};

  cv::circle(img, cv::Point2d(obs[0], obs[1]), 5, cv::Scalar(0, 0, 255), -1);
  cv::circle(img, cv::Point2d(uv_proj[0], uv_proj[1]), 5, cv::Scalar(0, 255, 0), -1);
  cv::line(img, cv::Point2d(obs[0], obs[1]), cv::Point2d(uv_proj[0], uv_proj[1]), cv::Scalar(255, 255, 0), 1);

  std::string png_path = out_dir + "/coord_tf_reprojection.png";
  cv::imwrite(png_path, img);
  std::cout << "Wrote " << png_path << " (observed=red, projected=green, line=error)\n";

  std::string yaml_path = out_dir + "/coord_tf_process_data.yaml";
  cv::FileStorage fs(yaml_path, cv::FileStorage::WRITE);
  fs << "fx" << K.fx << "fy" << K.fy << "cx" << K.cx << "cy" << K.cy;
  fs << "point_3d" << std::vector<double>{p3d[0], p3d[1], p3d[2]};
  fs << "observed_uv" << std::vector<double>{obs[0], obs[1]};
  fs << "projected_uv" << std::vector<double>{uv_proj[0], uv_proj[1]};
  fs.release();
  std::cout << "Wrote " << yaml_path << "\n";

  return 0;
}
