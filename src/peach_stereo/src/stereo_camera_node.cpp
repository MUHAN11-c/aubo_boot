// peach_stereo：PS800-E1 主机侧单图案立体深度相机节点（2026-09-17 首版）
//
// 数据链路（对应 docs/testing-log.md 09-16 五个条目的验证结论）：
//   解锁激光(关自动控制+拉功率) → 彩色640x480 + 双目IR 1280x960 同帧组采集(~13.7 组/s)
//   → stereoRectify 校正 → SGBM(半分辨率) → 深度(校正左系, float mm)
//   → TYMapDepthImageToColorCoordinate 配准到彩色几何
//   → 发布与 percipio_camera 同构话题（深度 uint16 × 0.25mm，感知零改动）
//
// 配准网格的为什么（09-20 真机复核，勿再"修复"）：SDK 深度→3D 按所传标定的针孔解释
// 输入网格（libtyimgproc 反汇编：忽略畸变、内参按分辨率折算）。曾按纯针孔推理把 z 反
// 校正回原始左 IR 网格再喂 calibL——同帧组真 SDK 三方实测（testing-log 09-20）偏离
// vendor 链 +23px，重投影到 DEPTH_CAM 标定网格 +26px；z_rect 直接喂 calibL 最贴
// （+8px）：设备把校正旋转折进了 DEPTH_CAM 标定的主点/焦距（cx=605 vs 647、f=1045
// vs 1104），两套网格几何近似同构、设备内部真实网格不可从标定推导。手眼/感知按
// percipio 链标定，与 vendor 同构即系统正确——保持直接配准，禁止加配准 warp；
// +8px 残差（≈10mm@0.6m）由"切前端重做手眼标定"吸收。
//
// 关键坑（勿重蹈）：
//   - 标定结构体是 float32：必须 Mat(...,CV_32F,ptr).convertTo(x,CV_64F)，直接 CV_64F 包装=NaN
//   - 彩色不设分辨率时默认 2560x1920 yuyv（9.8MB/帧），会把帧组拖到 ~2.5 组/s
//   - 激光设置跨连接自动复位（auto=1/power=50）；曝光值跨连接残留
//   - 不要用 image_transport 发 16UC1 / bgr8 混流：Jazzy 加载全部插件且无
//     enable_pub_plugins；harvest catch-all 一订，jpeg 编深度、compressedDepth
//     编彩色，每帧 ERROR
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <camera_calibration_parsers/parse.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/header.hpp>

#include "TYApi.h"
#include "TYCoordinateMapper.h"

namespace peach_stereo {

class StereoCameraNode : public rclcpp::Node {
 public:
  explicit StereoCameraNode(const rclcpp::NodeOptions& options)
      : Node("peach_stereo_camera_node", options) {
    declare_parameters();
    device_ip_ = get_parameter("device_ip").as_string();
    camera_name_ = get_parameter("camera_name").as_string();
    color_mode_ = get_parameter("color_mode").as_string();
    ir_exposure_ = static_cast<int32_t>(get_parameter("ir_exposure").as_int());
    laser_power_ = static_cast<int32_t>(get_parameter("laser_power").as_int());
    num_disp_ = static_cast<int>(get_parameter("sgbm.num_disparities").as_int());
    block_ = static_cast<int>(get_parameter("sgbm.block_size").as_int());
    uniq_ = static_cast<int>(get_parameter("sgbm.uniqueness_ratio").as_int());
    proc_scale_ = static_cast<double>(get_parameter("sgbm.processing_scale").as_double());
    const std::string mode_str = get_parameter("sgbm.mode").as_string();
    if (mode_str == "3way") {
      sgbm_mode_ = cv::StereoSGBM::MODE_SGBM_3WAY;
    } else if (mode_str == "sgbm") {
      sgbm_mode_ = cv::StereoSGBM::MODE_SGBM;
    } else if (mode_str == "hh") {
      sgbm_mode_ = cv::StereoSGBM::MODE_HH;
    } else if (mode_str == "hh4") {
      sgbm_mode_ = cv::StereoSGBM::MODE_HH4;
    } else {
      RCLCPP_FATAL(get_logger(), "sgbm.mode must be 3way/sgbm/hh/hh4, got '%s'",
                   mode_str.c_str());
      throw std::runtime_error("invalid sgbm.mode");
    }
    avg_k_ = std::max(1, static_cast<int>(get_parameter("avg_k").as_int()));
    median_ksize_ = static_cast<int>(get_parameter("median_ksize").as_int());
    if (median_ksize_ != 0 && median_ksize_ != 3 && median_ksize_ != 5) {
      RCLCPP_FATAL(get_logger(), "median_ksize must be 0/3/5, got %d",
                   median_ksize_);
      throw std::runtime_error("invalid median_ksize");
    }
    temporal_k_ = static_cast<int>(get_parameter("temporal_k").as_int());
    if (temporal_k_ != 1 && temporal_k_ != 3 && temporal_k_ != 5) {
      RCLCPP_FATAL(get_logger(), "temporal_k must be 1/3/5, got %d", temporal_k_);
      throw std::runtime_error("invalid temporal_k");
    }

    color_frame_ = camera_name_ + "_color_frame";
    color_optical_ = camera_name_ + "_color_optical_frame";
    depth_frame_ = camera_name_ + "_depth_frame";
    depth_optical_ = camera_name_ + "_depth_optical_frame";
    link_frame_ = camera_name_ + "_link";

    // 彩色内参文件覆盖（与 percipio 前端共用同一份标定 yaml；空=设备内参折算）
    const std::string info_file = get_parameter("color_camera_info_file").as_string();
    if (!info_file.empty()) {
      std::string calib_name;
      if (camera_calibration_parsers::readCalibration(
              info_file, calib_name, color_info_override_)) {
        info_override_ = true;
        RCLCPP_INFO(get_logger(), "color camera_info override from %s (camera=%s)",
                    info_file.c_str(), calib_name.c_str());
      } else {
        RCLCPP_ERROR(get_logger(),
                     "failed to parse color_camera_info_file '%s', "
                     "falling back to device intrinsic", info_file.c_str());
      }
    }

    // 话题名与 percipio_camera 同构（namespace=camera）。只发 raw，避免 Jazzy
    // image_transport 全插件被 catch-all 订到后交叉编码刷 ERROR。
    color_pub_ = create_publisher<sensor_msgs::msg::Image>("color/image_raw", 10);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>("depth/image_raw", 10);
    color_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "color/camera_info", 10);
    depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "depth/camera_info", 10);
    points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "depth_registered/points", 10);
    static_tf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    if (!openDevice()) {
      RCLCPP_FATAL(get_logger(), "open device %s failed", device_ip_.c_str());
      throw std::runtime_error("open device failed");
    }
    running_.store(true);
    capture_thread_ = std::thread([this] { captureLoop(); });
  }

  ~StereoCameraNode() override {
    running_.store(false);
    if (capture_thread_.joinable()) capture_thread_.join();
    closeDevice();
  }

 private:
  void declare_parameters() {
    declare_parameter<std::string>("device_ip", "169.254.10.110");
    declare_parameter<std::string>("camera_name", "camera");
    declare_parameter<std::string>("color_mode", "640x480");
    declare_parameter<std::string>("color_camera_info_file", "");
    declare_parameter<int>("ir_exposure", 990);
    declare_parameter<int>("laser_power", 100);
    declare_parameter<int>("sgbm.num_disparities", 128);
    declare_parameter<int>("sgbm.block_size", 5);
    declare_parameter<int>("sgbm.uniqueness_ratio", 6);
    declare_parameter<double>("sgbm.processing_scale", 0.5);
    declare_parameter<std::string>("sgbm.mode", "hh4");
    declare_parameter<int>("avg_k", 1);
    declare_parameter<int>("median_ksize", 3);
    declare_parameter<int>("temporal_k", 1);
  }

  // 按 IP 枚举设备（不依赖 percipio_camera 的 Utils.hpp/TYThread）
  bool openDevice() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    TYInitLib();
    TYUpdateInterfaceList();  // 必须先刷新，否则 ETH 接口不进列表（selectDevice 同款顺序）
    TY_INTERFACE_INFO ifaces[16];
    uint32_t n_if = 0;
    TY_STATUS s = TYGetInterfaceList(ifaces, 16, &n_if);
    RCLCPP_INFO(get_logger(), "interface list: status=%d count=%u", s, n_if);
    if (s != TY_STATUS_OK) return false;
    for (uint32_t i = 0; i < n_if && dev_ == nullptr; i++) {
      if (ifaces[i].type != TY_INTERFACE_ETHERNET) continue;
      TY_INTERFACE_HANDLE ih;
      TY_STATUS os = TYOpenInterface(ifaces[i].id, &ih);
      if (os != TY_STATUS_OK) {
        RCLCPP_WARN(get_logger(), "open iface %s failed: %d", ifaces[i].id, os);
        continue;
      }
      uint32_t n_dev = 0;
      TYUpdateDeviceList(ih);  // GigE 广播发现设备（selectDevice 同款；缺这步设备数为 0）
      TYGetDeviceNumber(ih, &n_dev);
      RCLCPP_INFO(get_logger(), "iface %s: %u device(s)", ifaces[i].id, n_dev);
      if (n_dev == 0) { TYCloseInterface(ih); continue; }
      std::vector<TY_DEVICE_BASE_INFO> devs(n_dev);
      if (TYGetDeviceList(ih, devs.data(), n_dev, &n_dev) != TY_STATUS_OK) {
        TYCloseInterface(ih);
        continue;
      }
      for (uint32_t d = 0; d < n_dev; d++) {
        RCLCPP_INFO(get_logger(), "  dev %s model %s ip %s", devs[d].id,
                    devs[d].modelName, devs[d].netInfo.ip);
        if (device_ip_.empty() ||
            device_ip_ == std::string(devs[d].netInfo.ip)) {
          TY_STATUS ds = TYOpenDevice(ih, devs[d].id, &dev_);
          RCLCPP_INFO(get_logger(), "open device %s: status=%d", devs[d].id, ds);
          if (ds == TY_STATUS_OK) {
            iface_ = ih;
            break;
          }
        }
      }
      if (dev_ == nullptr) TYCloseInterface(ih);
    }
    if (dev_ == nullptr) return false;

    // 设备时钟同步到宿主：时间戳=纪元微秒，直接作 ROS 时间发布
    TYSetEnum(dev_, TY_COMPONENT_DEVICE, TY_ENUM_TIME_SYNC_TYPE, TY_TIME_SYNC_TYPE_HOST);

    // 标定：左 IR（深度参考系）/右 IR/彩色；右 IR 与彩色外参均为相对左 IR
    TYGetStruct(dev_, TY_COMPONENT_IR_CAM_LEFT, TY_STRUCT_CAM_CALIB_DATA, &calib_l_, sizeof(calib_l_));
    TYGetStruct(dev_, TY_COMPONENT_IR_CAM_RIGHT, TY_STRUCT_CAM_CALIB_DATA, &calib_r_, sizeof(calib_r_));
    TYGetStruct(dev_, TY_COMPONENT_RGB_CAM, TY_STRUCT_CAM_CALIB_DATA, &calib_c_, sizeof(calib_c_));

    setupRectification();
    if (!setupColorMode()) {
      return false;
    }

    // 解锁激光（独立 IR 模式下投射器自动控制不点亮——09-16 三续结论）
    TYSetBool(dev_, TY_COMPONENT_LASER, TY_BOOL_LASER_AUTO_CTRL, false);
    TYSetInt(dev_, TY_COMPONENT_LASER, TY_INT_LASER_POWER, laser_power_);
    TYSetInt(dev_, TY_COMPONENT_IR_CAM_LEFT, TY_INT_EXPOSURE_TIME, ir_exposure_);
    TYSetInt(dev_, TY_COMPONENT_IR_CAM_RIGHT, TY_INT_EXPOSURE_TIME, ir_exposure_);

    if (TYEnableComponents(dev_, TY_COMPONENT_RGB_CAM |
                                   TY_COMPONENT_IR_CAM_LEFT |
                                   TY_COMPONENT_IR_CAM_RIGHT) != TY_STATUS_OK) {
      RCLCPP_ERROR(get_logger(), "enable RGB+IR components failed");
      return false;
    }

    uint32_t frame_size = 0;
    TYGetFrameBufferSize(dev_, &frame_size);
    buf_[0].resize(frame_size);
    buf_[1].resize(frame_size);
    TYEnqueueBuffer(dev_, buf_[0].data(), frame_size);
    TYEnqueueBuffer(dev_, buf_[1].data(), frame_size);
    if (TYStartCapture(dev_) != TY_STATUS_OK) {
      RCLCPP_ERROR(get_logger(), "start capture failed");
      return false;
    }
    publishStaticTf();
    return true;
  }

  void closeDevice() {
    if (dev_ == nullptr) return;
    TYStopCapture(dev_);
    TYClearBufferQueue(dev_);
    // 礼貌复位（连接关闭本也会自动复位，双保险）
    TYSetBool(dev_, TY_COMPONENT_LASER, TY_BOOL_LASER_AUTO_CTRL, true);
    TYSetInt(dev_, TY_COMPONENT_LASER, TY_INT_LASER_POWER, 50);
    TYCloseDevice(dev_);
    TYCloseInterface(iface_);
    dev_ = nullptr;
    RCLCPP_INFO(get_logger(), "device closed, laser restored");
  }

  void setupRectification() {
    cv::Mat K1, K2, D1, D2, E;
    cv::Mat(3, 3, CV_32F, calib_l_.intrinsic.data).convertTo(K1, CV_64F);
    cv::Mat(3, 3, CV_32F, calib_r_.intrinsic.data).convertTo(K2, CV_64F);
    cv::Mat(1, 8, CV_32F, calib_l_.distortion.data).convertTo(D1, CV_64F);
    cv::Mat(1, 8, CV_32F, calib_r_.distortion.data).convertTo(D2, CV_64F);
    cv::Mat(4, 4, CV_32F, calib_r_.extrinsic.data).convertTo(E, CV_64F);
    cv::Mat R = E(cv::Rect(0, 0, 3, 3)).clone();
    cv::Mat T = E(cv::Rect(3, 0, 1, 3)).clone();

    ir_size_ = cv::Size(calib_l_.intrinsicWidth, calib_l_.intrinsicHeight);
    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(K1, D1, K2, D2, ir_size_, R, T, R1, R2, P1, P2, Q,
                      cv::CALIB_ZERO_DISPARITY, 0);
    f_rect_ = P1.at<double>(0, 0);
    baseline_ = -P2.at<double>(0, 3) / P1.at<double>(0, 0);
    proc_size_ = cv::Size(cvRound(ir_size_.width * proc_scale_),
                          cvRound(ir_size_.height * proc_scale_));
    f_proc_ = f_rect_ * proc_scale_;
    cv::initUndistortRectifyMap(K1, D1, R1, P1, ir_size_, CV_32FC1, map_lx_, map_ly_);
    cv::initUndistortRectifyMap(K2, D2, R2, P2, ir_size_, CV_32FC1, map_rx_, map_ry_);
    cv::Mat r1vec;
    cv::Rodrigues(R1, r1vec);
    RCLCPP_INFO(get_logger(),
                "rectify: R1=%.2fdeg f=%.1fpx B=%.1fmm proc=%dx%d numDisp=%d med=%d mode=%d tk=%d",
                cv::norm(r1vec) * 180.0 / CV_PI, f_proc_, baseline_,
                proc_size_.width, proc_size_.height, num_disp_, median_ksize_,
                sgbm_mode_, temporal_k_);

    // uniqueness 10→6：09-20 同帧组扫描（simul 3 组），覆盖 +2.7pp 且精度门内；
    // 门余量与备选 b7 见 reports/2026-09-20-camera-image-analysis/sgbm-sweep.md
    // mode hh4：09-20 端到端矩阵（12 变体 vs 设备 18 图案 D 链，同帧组三门）帕累托
    // 最优——精度 8.75→7.50mm、鬼影 1.65→1.48%、粗糙度 1.03→0.94mm（低于设备链
    // 1.08）、覆盖 −0.34pp 门内，匹配 10.9→52.7ms 全率发布；HH 同质量但慢 2 倍，
    // 3way/sgbm 保留为参数档。时域 k3 与全分辨率档记档未落地（率÷3/粗糙度劣化），
    // 见 reports/2026-09-20-e2e-stereo-optimal/
    sgbm_ = cv::StereoSGBM::create(0, num_disp_, block_, 200, 3200, 5, 31, uniq_, 100, 2,
                                   sgbm_mode_);
  }

  // 彩色分辨率档位：从设备枚举的 image mode 里挑 "WxH" 匹配项（防默认 2560x1920）。
  // 失败必须拒启：fallback 档位与实际帧尺寸不符会让配准/camera_info/点云全链错位。
  bool setupColorMode() {
    uint32_t count = 0;
    if (TYGetEnumEntryCount(dev_, TY_COMPONENT_RGB_CAM, TY_ENUM_IMAGE_MODE, &count)
        != TY_STATUS_OK) {
      RCLCPP_ERROR(get_logger(), "enumerate color image modes failed");
      return false;
    }
    std::vector<TY_ENUM_ENTRY> entries(count);
    uint32_t filled = 0;
    if (TYGetEnumEntryInfo(dev_, TY_COMPONENT_RGB_CAM, TY_ENUM_IMAGE_MODE,
                           entries.data(), count, &filled) != TY_STATUS_OK) {
      RCLCPP_ERROR(get_logger(), "read color image modes failed");
      return false;
    }
    for (uint32_t i = 0; i < filled; i++) {
      std::string desc(entries[i].description);
      if (desc.find(color_mode_) != std::string::npos &&
          desc.find("yuyv") != std::string::npos) {
        TYSetEnum(dev_, TY_COMPONENT_RGB_CAM, TY_ENUM_IMAGE_MODE, entries[i].value);
        color_size_ = cv::Size(TYImageWidth(entries[i].value), TYImageHeight(entries[i].value));
        RCLCPP_INFO(get_logger(), "color mode set: %s", desc.c_str());
        return true;
      }
    }
    RCLCPP_FATAL(get_logger(), "color mode %s not found", color_mode_.c_str());
    return false;
  }

  void publishStaticTf() {
    // 与 percipio 同构的静态链：link→{color,depth}_frame（identity）→各 optical（光学旋转）。
    // depth 帧必须发布：peach_arm.yaml 技能侧以 camera_depth_optical_frame 生成视点姿态，
    // 缺帧会 TF 查询失败。各 stream frame 相对 link 均为 identity+同一光学旋转（几何等价）。
    geometry_msgs::msg::TransformStamped link_to_color, color_to_optical;
    geometry_msgs::msg::TransformStamped link_to_depth, depth_to_optical;
    auto stamp = now();
    tf2::Quaternion q;
    q.setRPY(-M_PI / 2, 0.0, -M_PI / 2);

    link_to_color.header.stamp = stamp;
    link_to_color.header.frame_id = link_frame_;
    link_to_color.child_frame_id = color_frame_;
    color_to_optical.header.stamp = stamp;
    color_to_optical.header.frame_id = color_frame_;
    color_to_optical.child_frame_id = color_optical_;
    color_to_optical.transform.rotation.x = q.getX();
    color_to_optical.transform.rotation.y = q.getY();
    color_to_optical.transform.rotation.z = q.getZ();
    color_to_optical.transform.rotation.w = q.getW();

    const std::string depth_frame = depth_frame_;
    const std::string depth_optical = depth_optical_;
    link_to_depth.header.stamp = stamp;
    link_to_depth.header.frame_id = link_frame_;
    link_to_depth.child_frame_id = depth_frame;
    depth_to_optical.header.stamp = stamp;
    depth_to_optical.header.frame_id = depth_frame;
    depth_to_optical.child_frame_id = depth_optical;
    depth_to_optical.transform.rotation.x = q.getX();
    depth_to_optical.transform.rotation.y = q.getY();
    depth_to_optical.transform.rotation.z = q.getZ();
    depth_to_optical.transform.rotation.w = q.getW();

    static_tf_->sendTransform(
        {link_to_color, color_to_optical, link_to_depth, depth_to_optical});
  }

  std::unique_ptr<sensor_msgs::msg::CameraInfo> makeCameraInfo(
      const rclcpp::Time& stamp, const std::string& frame_id) const {
    auto msg = std::make_unique<sensor_msgs::msg::CameraInfo>();
    msg->header.stamp = stamp;
    msg->header.frame_id = frame_id;
    msg->width = color_size_.width;
    msg->height = color_size_.height;
    if (info_override_) {
      // 与 percipio 同语义：保留流分辨率，标定字段整组取自文件
      msg->distortion_model = color_info_override_.distortion_model;
      msg->d = color_info_override_.d;
      msg->k = color_info_override_.k;
      msg->r = color_info_override_.r;
      msg->p = color_info_override_.p;
      return msg;
    }
    const double sx = color_size_.width / static_cast<double>(calib_c_.intrinsicWidth);
    const double sy = color_size_.height / static_cast<double>(calib_c_.intrinsicHeight);
    msg->k[0] = calib_c_.intrinsic.data[0] * sx;
    msg->k[2] = calib_c_.intrinsic.data[2] * sx;
    msg->k[4] = calib_c_.intrinsic.data[4] * sy;
    msg->k[5] = calib_c_.intrinsic.data[5] * sy;
    msg->k[8] = 1.0;
    msg->p[0] = msg->k[0];
    msg->p[2] = msg->k[2];
    msg->p[5] = msg->k[4];
    msg->p[6] = msg->k[5];
    msg->p[10] = 1.0;
    return msg;
  }

  void captureLoop() {
    auto t_start = std::chrono::steady_clock::now();
    int groups = 0, published = 0;
    int acc_frames = 0;
    cv::Mat z_sum, z_cnt;

    while (running_.load() && rclcpp::ok()) {
      TY_FRAME_DATA frame;
      TY_STATUS s = TYFetchFrame(dev_, &frame, 2000);
      if (s != TY_STATUS_OK) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "fetch frame timeout");
        continue;
      }
      groups++;
      const uint8_t* color = nullptr;
      const uint8_t* ir_l = nullptr;
      const uint8_t* ir_r = nullptr;
      int cw = 0, ch = 0;
      uint64_t ts_us = 0;
      for (int k = 0; k < frame.validCount; k++) {
        if (frame.image[k].status != TY_STATUS_OK) continue;
        const auto& img = frame.image[k];
        if (img.componentID == TY_COMPONENT_RGB_CAM) {
          color = static_cast<const uint8_t*>(img.buffer);
          cw = img.width; ch = img.height;
          ts_us = img.timestamp ? img.timestamp : ts_us;
        } else if (img.componentID == TY_COMPONENT_IR_CAM_LEFT) {
          ir_l = static_cast<const uint8_t*>(img.buffer);
          ts_us = img.timestamp ? img.timestamp : ts_us;
        } else if (img.componentID == TY_COMPONENT_IR_CAM_RIGHT) {
          ir_r = static_cast<const uint8_t*>(img.buffer);
        }
      }
      if (color && ir_l && ir_r && ts_us) {
        rclcpp::Time stamp(static_cast<int64_t>(ts_us) * 1000);
        cv::Mat z;
        if (computeDepth(ir_l, ir_r, z_sum, z_cnt, acc_frames, z)) {
          publishGroup(stamp, color, cw, ch, z);
          published++;
        }
      }
      TYEnqueueBuffer(dev_, frame.userBuffer, frame.bufferSize);

      if (groups % 100 == 0) {
        double dt = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t_start).count();
        RCLCPP_INFO(get_logger(), "groups=%d published=%d rate=%.1f gps", groups, published,
                    groups / dt);
      }
    }
  }

  // SGBM 主链路：校正→半分辨率匹配→深度(校正左系, mm)。
  // avg_k>1 时做 k 帧有效值均值（0=无效不进均值，除数为逐像素有效计数）。
  bool computeDepth(const uint8_t* ir_l, const uint8_t* ir_r,
                    cv::Mat & z_sum, cv::Mat & z_cnt, int & acc_frames,
                    cv::Mat & z_out) {
    cv::Mat L(ir_size_, CV_8UC1, const_cast<uint8_t*>(ir_l));
    cv::Mat R(ir_size_, CV_8UC1, const_cast<uint8_t*>(ir_r));
    cv::Mat Lr, Rr;
    cv::remap(L, Lr, map_lx_, map_ly_, cv::INTER_LINEAR);
    cv::remap(R, Rr, map_rx_, map_ry_, cv::INTER_LINEAR);
    if (proc_scale_ != 1.0) {
      cv::resize(Lr, Lr, proc_size_, 0, 0, cv::INTER_LINEAR);
      cv::resize(Rr, Rr, proc_size_, 0, 0, cv::INTER_LINEAR);
    }
    cv::Mat disp;
    sgbm_->compute(Lr, Rr, disp);
    disp.convertTo(disp, CV_32F, 1.0 / 16.0);

    cv::Mat z(proc_size_, CV_32FC1);
    for (int y = 0; y < proc_size_.height; y++) {
      const float* dr = disp.ptr<float>(y);
      float* zr = z.ptr<float>(y);
      for (int x = 0; x < proc_size_.width; x++) {
        zr[x] = dr[x] > 1.f ? static_cast<float>(f_proc_ * baseline_ / dr[x]) : 0.f;
      }
    }

    // 有效值中值（09-20 round4）：中心有效才输出、无效邻居不进中值——纯去噪不补洞。
    // 3x3 过同帧组三门（谷底 8.75 vs 9.00、鬼影 +0.04 门内、覆盖不变）且点云局部平面
    // 粗糙度 −12%（1.17→1.03mm，低于设备链 1.08）；5x5/联合双边/引导滤波被精度门或
    // 粗糙度证伪，见 reports/2026-09-20-camera-image-analysis/pointcloud-quality.md。
    if (median_ksize_ >= 3) {
      const int r = median_ksize_ / 2;
      cv::Mat zm = z.clone();
      for (int y = 0; y < proc_size_.height; y++) {
        const float* src = z.ptr<float>(y);
        float* dst = zm.ptr<float>(y);
        for (int x = 0; x < proc_size_.width; x++) {
          if (src[x] <= 0.f) continue;
          float nb[25];
          int n = 0;
          for (int dy = -r; dy <= r; dy++) {
            const int yy = y + dy;
            if (yy < 0 || yy >= proc_size_.height) continue;
            const float* s2 = z.ptr<float>(yy);
            for (int dx = -r; dx <= r; dx++) {
              const int xx = x + dx;
              if (xx < 0 || xx >= proc_size_.width) continue;
              if (s2[xx] > 0.f) nb[n++] = s2[xx];
            }
          }
          std::nth_element(nb, nb + n / 2, nb + n);
          dst[x] = nb[n / 2];
        }
      }
      z = zm;
    }

    if (avg_k_ > 1) {
      if (acc_frames == 0) {
        z_sum = cv::Mat::zeros(proc_size_, CV_32FC1);
        z_cnt = cv::Mat::zeros(proc_size_, CV_32FC1);
      }
      cv::add(z_sum, z, z_sum);                          // 无效处 z=0，累加无害
      cv::add(z_cnt, cv::Scalar(1.0), z_cnt, z > 0.0f);  // 仅有效像素计数
      acc_frames++;
      if (acc_frames < avg_k_) return false;
      cv::divide(z_sum, z_cnt, z, 1.0);                  // 计数为 0 处商=0（保持无效）
      acc_frames = 0;
    }
    // 注意：z 保持校正左系网格直接进 SDK 配准（见文件头 09-20 复核），勿反校正。
    z_out = z;
    return true;
  }

  void publishGroup(const rclcpp::Time& stamp, const uint8_t* color, int cw, int ch,
                    const cv::Mat& z) {
    cv::Mat yuyv(ch, cw, CV_8UC2, const_cast<uint8_t*>(color));
    cv::Mat bgr;
    cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUY2);

    cv::Mat depth16(proc_size_, CV_16UC1, cv::Scalar(0));
    for (int y = 0; y < proc_size_.height; y++) {
      const float* zr = z.ptr<float>(y);
      uint16_t* dr = depth16.ptr<uint16_t>(y);
      for (int x = 0; x < proc_size_.width; x++) {
        float v = zr[x];
        if (v > 200.f && v < 16000.f) {
          int q = cvRound(v / 0.25f);
          dr[x] = (q > 0 && q < 0xFFFF) ? static_cast<uint16_t>(q) : 0;
        }
      }
    }

    cv::Mat reg(color_size_, CV_16UC1, cv::Scalar(0));
    TY_STATUS ms = TYMapDepthImageToColorCoordinate(
        &calib_l_, static_cast<uint32_t>(proc_size_.width),
        static_cast<uint32_t>(proc_size_.height),
        reinterpret_cast<const uint16_t*>(depth16.data),
        &calib_c_, static_cast<uint32_t>(color_size_.width),
        static_cast<uint32_t>(color_size_.height),
        reinterpret_cast<uint16_t*>(reg.data), 0.25f);
    if (ms != TY_STATUS_OK) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "depth->color registration failed: %d, drop frame", ms);
      return;
    }

    // 滑窗时域中值（09-21 数据驱动轮）：在配准后的彩色网格深度上做 k 帧逐像素
    // 有效中值——源 13.7gps 帧盈余换深度质量，每帧照常发布（不除率，区别于
    // avg_k 批式 Z 域均值）。中值是序统计量、与单调变换可交换（域无关，round4
    // 已证），顺带稳定配准闪烁；参考 librealsense temporal filter 的多帧融合
    // 设计。置信度=采样数占比×窗内取值一致性，与发布深度/点云逐像素对齐。
    if (temporal_k_ > 1) {
      treg_.push_back(reg.clone());
      if (treg_.size() > static_cast<size_t>(temporal_k_)) {
        treg_.erase(treg_.begin());
      }
      const int nbuf = static_cast<int>(treg_.size());
      cv::Mat fused(color_size_, CV_16UC1, cv::Scalar(0));
      tconf_.create(color_size_, CV_32FC1);
      tconf_.setTo(0.0f);
      uint16_t vals[8];
      for (int y = 0; y < color_size_.height; y++) {
        const uint16_t* rows[8];
        for (int b = 0; b < nbuf; b++) rows[b] = treg_[b].ptr<uint16_t>(y);
        uint16_t* fr = fused.ptr<uint16_t>(y);
        float* cr = tconf_.ptr<float>(y);
        for (int x = 0; x < color_size_.width; x++) {
          int n = 0;
          for (int b = 0; b < nbuf; b++) {
            if (rows[b][x] > 0) vals[n++] = rows[b][x];
          }
          if (n == 0) continue;
          std::nth_element(vals, vals + n / 2, vals + n);
          const uint16_t med = vals[n / 2];
          fr[x] = med;
          uint16_t lo = med, hi = med;
          for (int i = 0; i < n; i++) {
            if (vals[i] < lo) lo = vals[i];
            if (vals[i] > hi) hi = vals[i];
          }
          const float med_mm = std::max(med * 0.25f, 1.0f);
          cr[x] = (static_cast<float>(n) / static_cast<float>(temporal_k_)) *
                  (1.0f / (1.0f + (hi - lo) * 0.25f / med_mm));
        }
      }
      reg = fused;
    }

    auto color_msg = cv_bridge::CvImage(
        std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, bgr).toImageMsg();
    color_msg->header.stamp = stamp;
    color_msg->header.frame_id = color_optical_;
    color_pub_->publish(*color_msg);

    auto depth_msg = cv_bridge::CvImage(
        std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_16UC1, reg).toImageMsg();
    depth_msg->header.stamp = stamp;
    depth_msg->header.frame_id = depth_optical_;
    depth_pub_->publish(*depth_msg);

    color_info_pub_->publish(makeCameraInfo(stamp, color_optical_));
    depth_info_pub_->publish(makeCameraInfo(stamp, depth_optical_));
    publishRegisteredCloud(stamp, bgr, reg);
  }

  void publishRegisteredCloud(const rclcpp::Time& stamp, const cv::Mat& bgr,
                              const cv::Mat& depth16) {
    if (points_pub_->get_subscription_count() == 0) {
      return;
    }
    if (bgr.size() != depth16.size() || bgr.type() != CV_8UC3) {
      return;
    }
    auto info = makeCameraInfo(stamp, depth_optical_);
    const double fx = info->k[0];
    const double fy = info->k[4];
    const double cx = info->k[2];
    const double cy = info->k[5];
    if (fx <= 0.0 || fy <= 0.0) {
      return;
    }

    auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    cloud->header.stamp = stamp;
    cloud->header.frame_id = depth_optical_;
    cloud->height = 1;
    cloud->is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(*cloud);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    // 置信度字段（temporal_k>1 时为时域一致性，见 publishGroup；否则恒 1）
    sensor_msgs::msg::PointField cf;
    cf.name = "confidence";
    cf.datatype = sensor_msgs::msg::PointField::FLOAT32;
    cf.count = 1;
    // byString 语义：xyz 分支 pad 到 16、"rgb" 槽落在 16（点云迭代器 r/g/b/a
    // 子字节模型）。confidence 必须排在其后（offset 20），point_step 沿用
    // byString 隐式的 24——写 16/20 会与 rgb 重叠互相覆写（09-21 实测线上
    // rgb@16==confidence@16，颜色被置信度覆写损坏）。
    cf.offset = 20;
    cloud->fields.push_back(cf);
    cloud->point_step = 24;
    const int max_n = depth16.rows * depth16.cols;
    modifier.resize(static_cast<size_t>(max_n));
    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*cloud, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*cloud, "b");
    sensor_msgs::PointCloud2Iterator<float> iter_cf(*cloud, "confidence");
    size_t n = 0;
    for (int v = 0; v < depth16.rows; v++) {
      const uint16_t* dr = depth16.ptr<uint16_t>(v);
      const cv::Vec3b* cr = bgr.ptr<cv::Vec3b>(v);
      for (int u = 0; u < depth16.cols; u++) {
        const uint16_t q = dr[u];
        if (q == 0) {
          continue;
        }
        const float z_m = static_cast<float>(q) * 0.00025f;
        *iter_x = static_cast<float>((u - cx) * z_m / fx);
        *iter_y = static_cast<float>((v - cy) * z_m / fy);
        *iter_z = z_m;
        *iter_b = cr[u][0];
        *iter_g = cr[u][1];
        *iter_r = cr[u][2];
        *iter_cf = (temporal_k_ > 1 && !tconf_.empty())
                       ? tconf_.at<float>(v, u) : 1.0f;
        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_r;
        ++iter_g;
        ++iter_b;
        ++iter_cf;
        ++n;
      }
    }
    cloud->width = static_cast<uint32_t>(n);
    modifier.resize(n);
    points_pub_->publish(std::move(cloud));
  }

  // ---- 成员 ----
  std::string device_ip_, camera_name_, color_mode_;
  int32_t ir_exposure_{990}, laser_power_{100};
  int num_disp_{128}, block_{5}, avg_k_{1}, uniq_{6}, median_ksize_{3};
  int sgbm_mode_{cv::StereoSGBM::MODE_SGBM_3WAY};
  int temporal_k_{1};
  std::vector<cv::Mat> treg_;  // 滑窗时域中值的配准深度环形缓冲（CV_16UC1）
  cv::Mat tconf_;              // 逐像素置信度（与发布深度同彩色网格）
  double proc_scale_{0.5};
  std::string color_frame_, color_optical_, depth_frame_, depth_optical_, link_frame_;

  TY_INTERFACE_HANDLE iface_{nullptr};
  TY_DEV_HANDLE dev_{nullptr};
  TY_CAMERA_CALIB_INFO calib_l_{}, calib_r_{}, calib_c_{};
  cv::Size ir_size_{1280, 960}, proc_size_{640, 480}, color_size_{640, 480};
  double f_rect_{0}, f_proc_{0}, baseline_{0};
  cv::Mat map_lx_, map_ly_, map_rx_, map_ry_;
  cv::Ptr<cv::StereoSGBM> sgbm_;
  std::vector<uint8_t> buf_[2];
  std::atomic<bool> running_{false};
  std::thread capture_thread_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  sensor_msgs::msg::CameraInfo color_info_override_;
  bool info_override_{false};
};

}  // namespace peach_stereo

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<peach_stereo::StereoCameraNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
