// peach_stereo：PS800-E1 主机侧单图案立体深度相机节点（2026-09-17 首版）
//
// 数据链路（对应 docs/testing-log.md 09-16 五个条目的验证结论）：
//   解锁激光(关自动控制+拉功率) → 彩色640x480 + 双目IR 1280x960 同帧组采集(~13.7 组/s)
//   → stereoRectify 校正 → SGBM(半分辨率) → 深度(左IR系, float mm)
//   → TYMapDepthImageToColorCoordinate 配准到彩色几何
//   → 发布与 percipio_camera 同构话题（深度 uint16 × 0.25mm，感知零改动）
//
// 关键坑（勿重蹈）：
//   - 标定结构体是 float32：必须 Mat(...,CV_32F,ptr).convertTo(x,CV_64F)，直接 CV_64F 包装=NaN
//   - 彩色不设分辨率时默认 2560x1920 yuyv（9.8MB/帧），会把帧组拖到 ~2.5 组/s
//   - 激光设置跨连接自动复位（auto=1/power=50）；曝光值跨连接残留
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
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
    proc_scale_ = static_cast<double>(get_parameter("sgbm.processing_scale").as_double());
    avg_k_ = std::max(1, static_cast<int>(get_parameter("avg_k").as_int()));
    publish_debug_ = get_parameter("publish_debug_image").as_bool();

    color_frame_ = camera_name_ + "_color_frame";
    color_optical_ = camera_name_ + "_color_optical_frame";
    link_frame_ = camera_name_ + "_link";

    color_pub_ = image_transport::create_publisher(this, "color/image_raw");
    depth_pub_ = image_transport::create_publisher(this, "depth/image_raw");
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("color/camera_info", 10);
    static_tf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    if (publish_debug_) {
      debug_pub_ = image_transport::create_publisher(this, "depth/debug_color");
    }

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
    declare_parameter<int>("ir_exposure", 990);
    declare_parameter<int>("laser_power", 100);
    declare_parameter<int>("sgbm.num_disparities", 128);
    declare_parameter<int>("sgbm.block_size", 5);
    declare_parameter<double>("sgbm.processing_scale", 0.5);
    declare_parameter<int>("avg_k", 1);
    declare_parameter<bool>("publish_debug_image", true);
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
    setupColorMode();

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
    RCLCPP_INFO(get_logger(), "rectify: f=%.1fpx B=%.1fmm proc=%dx%d numDisp=%d",
                f_proc_, baseline_, proc_size_.width, proc_size_.height, num_disp_);

    sgbm_ = cv::StereoSGBM::create(0, num_disp_, block_, 200, 3200, 5, 31, 10, 100, 2,
                                   cv::StereoSGBM::MODE_SGBM_3WAY);
  }

  void setupColorMode() {
    // 彩色分辨率档位：从设备枚举的 image mode 里挑 "WxH" 匹配项（防默认 2560x1920）
    uint32_t count = 0;
    if (TYGetEnumEntryCount(dev_, TY_COMPONENT_RGB_CAM, TY_ENUM_IMAGE_MODE, &count) != TY_STATUS_OK)
      return;
    std::vector<TY_ENUM_ENTRY> entries(count);
    uint32_t filled = 0;
    if (TYGetEnumEntryInfo(dev_, TY_COMPONENT_RGB_CAM, TY_ENUM_IMAGE_MODE,
                           entries.data(), count, &filled) != TY_STATUS_OK)
      return;
    for (uint32_t i = 0; i < filled; i++) {
      std::string desc(entries[i].description);
      if (desc.find(color_mode_) != std::string::npos &&
          desc.find("yuyv") != std::string::npos) {
        TYSetEnum(dev_, TY_COMPONENT_RGB_CAM, TY_ENUM_IMAGE_MODE, entries[i].value);
        color_size_ = cv::Size(TYImageWidth(entries[i].value), TYImageHeight(entries[i].value));
        RCLCPP_INFO(get_logger(), "color mode set: %s", desc.c_str());
        return;
      }
    }
    RCLCPP_WARN(get_logger(), "color mode %s not found, using device default", color_mode_.c_str());
  }

  void publishStaticTf() {
    geometry_msgs::msg::TransformStamped link_to_frame, frame_to_optical;
    auto stamp = now();
    link_to_frame.header.stamp = stamp;
    link_to_frame.header.frame_id = link_frame_;
    link_to_frame.child_frame_id = color_frame_;
    frame_to_optical.header.stamp = stamp;
    frame_to_optical.header.frame_id = color_frame_;
    frame_to_optical.child_frame_id = color_optical_;
    // 与 percipio 一致的光学系旋转 setRPY(-pi/2, 0, -pi/2)
    tf2::Quaternion q;
    q.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
    frame_to_optical.transform.rotation.x = q.getX();
    frame_to_optical.transform.rotation.y = q.getY();
    frame_to_optical.transform.rotation.z = q.getZ();
    frame_to_optical.transform.rotation.w = q.getW();
    static_tf_->sendTransform({link_to_frame, frame_to_optical});
  }

  std::unique_ptr<sensor_msgs::msg::CameraInfo> makeCameraInfo(const rclcpp::Time& stamp) const {
    auto msg = std::make_unique<sensor_msgs::msg::CameraInfo>();
    msg->header.stamp = stamp;
    msg->header.frame_id = color_optical_;
    msg->width = color_size_.width;
    msg->height = color_size_.height;
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
    cv::Mat z_acc;
    auto last_dbg = std::chrono::steady_clock::now();

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
        if (computeDepth(ir_l, ir_r, z_acc, acc_frames, z)) {
          publishGroup(stamp, color, cw, ch, z, last_dbg);
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

  // SGBM 主链路：校正→半分辨率匹配→深度(左IR系, mm)。avg_k>1 时做 k 帧均值
  bool computeDepth(const uint8_t* ir_l, const uint8_t* ir_r,
                    cv::Mat& z_acc, int& acc_frames, cv::Mat& z_out) {
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

    if (avg_k_ > 1) {
      if (z_acc.empty() || acc_frames == 0) {
        z_acc = cv::Mat::zeros(proc_size_, CV_32FC1);
        acc_frames = 0;
      }
      cv::add(z_acc, z, z_acc);
      acc_frames++;
      if (acc_frames < avg_k_) return false;
      z_acc /= static_cast<double>(avg_k_);
      z = z_acc.clone();
      acc_frames = 0;
    }
    z_out = z;
    return true;
  }

  void publishGroup(const rclcpp::Time& stamp, const uint8_t* color, int cw, int ch,
                    const cv::Mat& z, std::chrono::steady_clock::time_point& last_dbg) {
    // 彩色 yuyv→bgr
    cv::Mat yuyv(ch, cw, CV_8UC2, const_cast<uint8_t*>(color));
    cv::Mat bgr;
    cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUY2);

    // 深度量化为 0.25mm 单位 uint16（与 percipio depth_scale_unit=0.25 对齐）
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

    // 配准到彩色几何（厂商 SDK 主机侧配准；输出同为 0.25mm 单位）
    cv::Mat reg(color_size_, CV_16UC1, cv::Scalar(0));
    TYMapDepthImageToColorCoordinate(
        &calib_l_, static_cast<uint32_t>(proc_size_.width),
        static_cast<uint32_t>(proc_size_.height),
        reinterpret_cast<const uint16_t*>(depth16.data),
        &calib_c_, static_cast<uint32_t>(color_size_.width),
        static_cast<uint32_t>(color_size_.height),
        reinterpret_cast<uint16_t*>(reg.data), 0.25f);

    auto color_msg = cv_bridge::CvImage(
        std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, bgr).toImageMsg();
    color_msg->header.stamp = stamp;
    color_msg->header.frame_id = color_optical_;
    color_pub_.publish(color_msg);

    auto depth_msg = cv_bridge::CvImage(
        std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_16UC1, reg).toImageMsg();
    depth_msg->header.stamp = stamp;
    depth_msg->header.frame_id = color_optical_;
    depth_pub_.publish(depth_msg);

    info_pub_->publish(makeCameraInfo(stamp));

    if (publish_debug_ &&
        std::chrono::steady_clock::now() - last_dbg > std::chrono::milliseconds(500)) {
      last_dbg = std::chrono::steady_clock::now();
      cv::Mat vis(color_size_, CV_8UC1, cv::Scalar(0));
      for (int y = 0; y < color_size_.height; y++) {
        const uint16_t* dr = reg.ptr<uint16_t>(y);
        uint8_t* vr = vis.ptr<uint8_t>(y);
        for (int x = 0; x < color_size_.width; x++) {
          float mm = dr[x] * 0.25f;
          vr[x] = (mm > 200.f && mm < 1500.f)
              ? static_cast<uint8_t>((mm - 200.f) / 1300.f * 255.f) : 0;
        }
      }
      cv::Mat jet;
      cv::applyColorMap(vis, jet, cv::COLORMAP_JET);
      auto dbg = cv_bridge::CvImage(
          std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, jet).toImageMsg();
      dbg->header.stamp = stamp;
      dbg->header.frame_id = color_optical_;
      debug_pub_.publish(dbg);
    }
  }

  // ---- 成员 ----
  std::string device_ip_, camera_name_, color_mode_;
  int32_t ir_exposure_{990}, laser_power_{100};
  int num_disp_{128}, block_{5}, avg_k_{1};
  double proc_scale_{0.5};
  bool publish_debug_{true};
  std::string color_frame_, color_optical_, link_frame_;

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

  image_transport::Publisher color_pub_;
  image_transport::Publisher depth_pub_;
  image_transport::Publisher debug_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
};

}  // namespace peach_stereo

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<peach_stereo::StereoCameraNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
