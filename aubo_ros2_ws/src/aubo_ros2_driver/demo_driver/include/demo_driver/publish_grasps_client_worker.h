/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_PUBLISH_GRASPS_CLIENT_WORKER_H_
#define DEMO_DRIVER_PUBLISH_GRASPS_CLIENT_WORKER_H_

#include <array>
#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <ivg_interfaces/srv/set_robot_io.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace demo_driver
{

/** 笛卡尔路径单段：沿 axis 轴移动 offset 米（与放置多段路径一致） */
struct CartesianSegment
{
  char axis;
  double offset;
};

/**
 * @brief GraspNet 抓取放置 Worker 节点（独立 rclcpp::Node，不继承 MoveitGripperIoBase）
 *
 * 【对外】run / runOneCycle、窗口与采集控制、关机钩子；实现见 publish_grasps_client_worker.cpp 分部注释。
 * 【流程】滑窗选优 → tip→EEF → MoveIt（关节/命名位姿/笛卡尔）→ 夹爪 IO → B 点放置 → 回 A 点安全位（SRDF camera_pose）。
 *
 * 订阅 grasp_poses_base（PoseArray）；循环由 loop_control(SetBool) 或 auto_start_loop 驱动。
 */
class PublishGraspsClientWorker : public rclcpp::Node
{
public:
  explicit PublishGraspsClientWorker(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~PublishGraspsClientWorker() = default;

  static std::shared_ptr<PublishGraspsClientWorker> create(const rclcpp::NodeOptions& options);

  bool waitForServices(std::chrono::seconds timeout);

  bool run();

  /** 单次抓取放置周期，任一步失败返回 false */
  bool runOneCycle();

  /** 加锁清空窗口与 _latest_grasp_poses */
  void clearGraspWindow();

  /** 阻塞等待窗口达到 min_groups_before_pick，超时返回 false */
  bool waitForGraspWindowReady();
  /** 调用感知节点采集控制服务（true 开始采集，false 结束采集） */
  bool requestGraspCapture(bool enable);
  /** 循环抓取控制服务（true 开启循环，false 当前周期结束后退出） */
  void handleLoopControl(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  /** 从窗口选优，prefer_vertical 时取 verticalityScore 最高；返回 (tag, pose) 或 nullopt */
  std::optional<std::pair<std::string, geometry_msgs::msg::Pose>> selectBestFromWindow();

  /**
   * 将 gripper_tip 位姿变换为 end_effector 目标位姿
   * @param pose gripper_tip 在 base_link 下的抓取位姿
   * @param transform_local 抓取局部坐标系下的 4x4 变换，沿 z 轴 -grasp_z_offset
   * @return end_effector 在 base_link 下的目标位姿
   */
  geometry_msgs::msg::Pose applyTransformationToPose(const geometry_msgs::msg::Pose& pose,
                                                     const Eigen::Matrix4d& transform_local);

  /** 构建 gripper_tip -> end_effector 局部变换（沿 z 轴 -grasp_z_offset） */
  Eigen::Matrix4d buildGraspToEndEffectorTransform();

  /** 对 end_effector 目标位姿执行抓取接近（4 点笛卡尔） */
  bool runGraspMotion(const geometry_msgs::msg::Pose& target_pose);

  /** 优雅退出：回 A 点安全位、开夹爪 */
  void onShutdown();

  /** 请求退出：SIGINT 时调用，设置 shutdown_requested_ 使主循环尽快退出 */
  void requestShutdown();
  /** 是否已请求退出 */
  bool isShutdownRequested() const { return shutdown_requested_.load(); }

private:
  // 感知几何、MoveIt 封装与周期内部步骤；调用顺序以 runOneCycle 为准
  /**
   * 4 点笛卡尔抓取接近，fraction<1 或点数>cartesian_max_points 直接返回 false，不回退
   * @param pose_ee end_effector 在 base_link 下的目标位姿（gripper_tip 补偿后）
   * @param height_above 抓取点上方安全高度 (m)
   * @param vel 笛卡尔轨迹速度缩放 [0~1]
   * @param acc 加速度缩放 [0~1]
   */
  bool runGraspApproach(const geometry_msgs::msg::Pose& pose_ee, double height_above, float vel, float acc);

  /**
   * GraspNet 预测抓取专属修正：网络抓取系与实物夹爪绕局部 Z 常差 180°，ori * (0,0,1,0)，位置不变。
   * 仅用于 grasp_poses 等 GraspNet 输出；其它位姿来源勿调用。
   */
  geometry_msgs::msg::Pose applyGraspZFlip180(const geometry_msgs::msg::Pose& pose);

  /** 垂直度得分 [0,1]：抓取 Z 轴与 [0,0,-1] 点积绝对值，越大越垂直 */
  double verticalityScore(const geometry_msgs::msg::Pose& pose);

  /** 工业惯例：每次规划前同步当前状态并设置速度/加速度缩放（关节与笛卡尔均适用） */
  void preparePlanningState(float velocity_factor, float acceleration_factor);

  void initMoveGroup();
  bool setGripperIo(int32_t io_index, bool high);
  /** 移动到安全位 A 点（命名目标 camera_pose） */
  bool moveToHome(float velocity_factor, float acceleration_factor);
  /** 步骤 9：关节空间运动至放置 B 点（固定 6 轴角，与 manipulator 关节顺序一致） */
  bool moveToPlacePointB(float velocity_factor, float acceleration_factor);
  bool runArcPath(char axis, double offset, float velocity_factor, float acceleration_factor);
  bool runArcPathSequence(const std::vector<CartesianSegment>& segments, float velocity_factor,
                          float acceleration_factor);

  void graspPosesCallback(geometry_msgs::msg::PoseArray::ConstSharedPtr msg);

  /** 打印当前末端位姿与规划组关节（rad/deg）；MoveIt 未就绪或无 eef 时静默返回 */
  void logCurrentEefPoseAndJoints(const char* stage_label);

  /**
   * 发布周期状态到 status_topic（JSON 格式）
   * @param cycle_count 当前周期数
   * @param success_count 成功次数
   * @param fail_count 失败次数
   * @param last_step 上次失败步骤 1~10，成功为 0
   * @param is_running 主循环是否在运行
   */
  void publishStatus(int cycle_count, int success_count, int fail_count, int last_step, bool is_running);

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr grasp_poses_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr grasp_capture_client_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr loop_control_service_;

  std::deque<geometry_msgs::msg::PoseArray> grasp_groups_window_;
  geometry_msgs::msg::PoseArray latest_grasp_poses_;
  std::mutex window_mutex_;

  /** 选优策略：true 取垂直度最高抓取，false 取最新组第一个 */
  bool prefer_vertical_;
  /** gripper_tip 相对 end_effector 的 z 轴补偿 (m)，沿 approach 方向反向平移 */
  double grasp_z_offset_;
  /** 抓取点上方安全高度 (m)，笛卡尔路径先到该高度再垂直下降 */
  double height_above_;
  /** 速度缩放 [0~1]：关节规划、moveToHome、笛卡尔（经 preparePlanningState + 笛卡尔时间缩放）共用 */
  float joint_velocity_scaling_;
  /** 加速度缩放 [0~1]，与 joint_velocity_scaling_ 共用场景一致 */
  float joint_acceleration_scaling_;
  /** 抓取位姿话题名，与 graspnet_demo_points_node 发布一致 */
  std::string grasp_poses_topic_;
  /** 等待窗口就绪超时 (s)，超时返回 false */
  double wait_poses_timeout_sec_;
  /** 感知节点采集控制服务名（SetBool: true 启动, false 停止） */
  std::string grasp_capture_service_name_;
  /** 采集控制服务等待超时 (s) */
  double grasp_capture_service_timeout_sec_;
  /** Worker 循环控制服务名（SetBool: true 开启循环, false 当前周期结束后退出） */
  std::string loop_control_service_name_;
  /** 启动后是否立即进入循环；false 时等待服务开启 */
  bool auto_start_loop_{ false };
  /** 滑动窗口大小：缓存最近 N 组 PoseArray */
  size_t grasp_window_size_;
  /** 至少积累 M 组后再选优，避免单次识别噪声 */
  size_t min_groups_before_pick_;
  /** Aubo 夹爪 IO pin 号，与 GripperSwapWorker 一致 */
  int32_t gripper_io_index_;
  /** 抓取后沿 Z 轴抬起高度 (m)，正为向上 */
  double lift_offset_;
  /** 放置：先回安全位再沿 y/x/z 做笛卡尔偏移 (m) */
  double place_offset_y_;
  double place_offset_z_;
  /** 关节空间轨迹与笛卡尔轨迹相互切换时，在「上一段执行结束 → 下一段规划/执行开始」之间的延时 (s)；0 关闭 */
  double joint_cartesian_switch_delay_sec_;
  /** 每周期结束后的等待时间 (s)，成功与失败后均执行 */
  double cycle_delay_sec_;
  /** 失败后额外等待 (s)，避免连续失败时过于频繁重试 */
  double fail_retry_delay_sec_;
  /** 最大周期数，-1 表示无限循环 */
  int max_cycles_;
  /** 状态监控话题，发布 cycle_count、success_count、fail_count 等 JSON */
  std::string status_topic_;
  /** 抓取接近笛卡尔轨迹点数上限，超过则拒绝执行（即使规划 100%%） */
  int cartesian_max_points_;

  std::atomic<bool> shutdown_requested_{ false };
  /** 运行态：是否允许开始新周期 */
  std::atomic<bool> loop_enabled_{ false };
  /** 停机请求：收到 false 后当前周期完成即退出 */
  std::atomic<bool> stop_after_cycle_{ false };
  /** 当前是否处于 runOneCycle 内 */
  std::atomic<bool> cycle_in_progress_{ false };
  int cycle_count_{ 0 };
  int success_count_{ 0 };
  int fail_count_{ 0 };

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Client<ivg_interfaces::srv::SetRobotIO>::SharedPtr aubo_set_io_client_;

  static constexpr double kZMinLimit = 0.2;
  static const int32_t kGripperIoIndex;
};

}  // namespace demo_driver

#endif  // DEMO_DRIVER_PUBLISH_GRASPS_CLIENT_WORKER_H_
