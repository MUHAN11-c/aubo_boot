/*
 * Software License Agreement (BSD License)
 * Copyright (c) 2017-2018, AUBO Robotics
 *
 * ROS2 轨迹控制器 —— 借鉴 ros2_control joint_trajectory_controller 设计。
 *
 * 职责:
 *   1. 接收 FollowJointTrajectory Action 目标
 *   2. 使用 5 次多项式插值生成 200Hz 轨迹点流
 *   3. 通过 HardwareInterface::writeTrajectoryPoints() 下发给机器人
 *   4. 目标容差检查和 Action 状态机管理
 *
 * SDK 模式冲突规则 (重要):
 *   - 本控制器工作在 TCP2CAN 模式下 (conn_control_ 处于透传模式)
 *   - TCP2CAN 模式开启时，SDK 的运动 API (robotServiceJointMove/LineMove 等) 不可用
 *   - 如需使用 SDK 运动 API，需先停止本控制器并 leaveTcp2CanbusMode()
 *   - Dashboard 节点的运动服务在调用前会检查 TCP2CAN 状态并自动切换
 *   - conn_status_ (普通模式) 的状态查询/IO/事件回调不受 TCP2CAN 状态影响
 *
 * 线程模型:
 *   - Action 回调: MultiThreadedExecutor 线程池 (互斥回调组)
 *   - update() 控制循环: ROS2 wall timer, 200Hz (独立回调组)
 *   - 与 HardwareInterface 的交互: write() 在主循环中, read() 按需
 *
 * 使用方式:
 *   1. 构造时传入 AuboHardwareInterface 引用 (必须先 init + enterTcp2CanbusMode)
 *   2. 调用 configure() 创建 Action 服务器和定时器
 *   3. 由 Executor 驱动 —— 无需手动 spin
 *   4. 停用时调用 deactivate() 取消当前目标
 */

#ifndef AUBO_DRIVER_ROS2_JOINT_TRAJECTORY_CONTROLLER_H_
#define AUBO_DRIVER_ROS2_JOINT_TRAJECTORY_CONTROLLER_H_

#include <memory>
#include <vector>
#include <string>
#include <atomic>
#include <queue>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "aubo_driver_ros2/aubo_hardware_interface.h"
#include "aubo_driver_ros2/readerwriterqueue.h"

namespace aubo_driver {

/**
 * 关节轨迹控制器 —— 5 次多项式插值 + FollowJointTrajectory Action。
 *
 * 插值算法从 aubo_robot_simulator_ros2 (Python) 移植，
 * 保留 PORTING_MOTION_FIX.md 中验证有效的全部修复:
 *   - 轨迹边界混合平滑 (根因 A)
 *   - 段间 C1 连续性强制
 *   - 壁钟时序补偿 (整数步数控制)
 */
class JointTrajectoryController : public rclcpp::Node {
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

    /**
     * 构造函数。
     * @param hw  已初始化并进入 TCP2CAN 模式的 HardwareInterface 引用
     * @param joint_names 关节名称列表 (默认: aubo i 系列标准名称)
     * @param options 节点选项
     */
    explicit JointTrajectoryController(
        std::shared_ptr<AuboHardwareInterface> hw,
        const std::vector<std::string>& joint_names = {
            "shoulder_joint", "upperArm_joint", "foreArm_joint",
            "wrist1_joint", "wrist2_joint", "wrist3_joint"},
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    ~JointTrajectoryController() override;

    /**
     * 配置并启动控制器 (创建 Action 服务器和 200Hz 定时器)。
     * 必须在 HardwareInterface::enterTcp2CanbusMode() 成功后调用。
     */
    void configure();

    /**
     * 停用控制器 —— 取消当前目标并停止轨迹流。
     * 可用于模式切换前 (例如从 TCP2CAN 切换到 SDK 运动 API)。
     */
    void deactivate();

    /**
     * 检查是否有活跃的运动目标。
     * @return true 表示正在执行轨迹
     */
    bool isActive() const { return has_active_goal_; }

private:
    // ========================================================================
    // Action 回调 (在 trajectory_cb_group_ 中执行)
    // ========================================================================

    /** 收到新 Goal 时调用 —— 校验关节名称、接受或拒绝 */
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal);

    /** 收到取消请求时调用 */
    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandle> goal_handle);

    /** Goal 被接受后调用 —— 准备轨迹执行 */
    void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);

    // ========================================================================
    // 控制循环 (200Hz, 在 update_cb_group_ 中执行)
    // ========================================================================

    /**
     * 200Hz 控制循环 —— 每个周期:
     *   1. 取当前等待时间对应的插值点
     *   2. 发送到 HardwareInterface
     *   3. 检查目标是否到达
     */
    void update();

    // ========================================================================
    // 插值算法 (从 Python simulator 移植)
    // ========================================================================

    /**
     * 5 次多项式插值 —— 在 [last, curr] 之间按时间 t 插值。
     *
     * 系数公式与 ROS1 C++ MotionControllerSimulator 和 ROS2 Python
     * aubo_robot_simulator_ros2 完全一致。
     *
     * @param last      段起点 (位置/速度/加速度)
     * @param curr      段终点 (位置/速度/加速度)
     * @param t         段内时间 [0, T]
     * @param T         段总时长
     * @param positions[out]  输出 6 关节位置 (rad)
     * @param velocities[out] 输出 6 关节速度 (rad/s)
     * @param accelerations[out] 输出 6 关节加速度 (rad/s²)
     */
    static void quinticInterpolate(
        const trajectory_msgs::msg::JointTrajectoryPoint& last,
        const trajectory_msgs::msg::JointTrajectoryPoint& curr,
        double t, double T,
        double positions[6], double velocities[6], double accelerations[6]);

    /**
     * 边界混合过渡 —— 当新轨迹到达时，从当前实际位置平滑过渡到
     * 新轨迹的第一个点，避免位置瞬跳。
     *
     * 这是 PORTING_MOTION_FIX.md 中根因 A 的修复。
     *
     * @param current_joints  当前实际关节位置 (6 个)
     * @param first_point     新轨迹的第一个目标点
     * @param blend_points[out] 输出的混合过渡点 (最多 30 个 @200Hz)
     * @return 混合过渡点数 (0 表示不需要混合)
     */
    int blendToFirstPoint(
        const double current_joints[6],
        const trajectory_msgs::msg::JointTrajectoryPoint& first_point,
        std::vector<aubo_robot_namespace::wayPoint_S>& blend_points);

    // ========================================================================
    // 状态机和容差检查
    // ========================================================================

    /** 检查当前关节位置是否在目标容差范围内 */
    bool withinGoalConstraints(
        const double current[6],
        const trajectory_msgs::msg::JointTrajectory& traj) const;

    /** 取消当前活跃目标 (停止轨迹流) */
    void abortActiveGoal();

    /** 重映射轨迹关节名称到控制器期望的顺序 */
    trajectory_msgs::msg::JointTrajectory remapJointNames(
        const trajectory_msgs::msg::JointTrajectory& traj) const;

    // ========================================================================
    // 成员变量
    // ========================================================================

    // 硬件接口
    std::shared_ptr<AuboHardwareInterface> hw_;

    // Action 服务器
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

    // 回调组 (Action 回调与 update 定时器隔离)
    rclcpp::CallbackGroup::SharedPtr trajectory_cb_group_;
    rclcpp::CallbackGroup::SharedPtr update_cb_group_;

    // 200Hz 控制循环定时器
    rclcpp::TimerBase::SharedPtr update_timer_;

    // 关节配置
    std::vector<std::string> joint_names_;
    static constexpr int kNJoint = 6;

    // Action 状态
    std::shared_ptr<GoalHandle> active_goal_;
    bool has_active_goal_{false};

    // 当前轨迹缓存
    trajectory_msgs::msg::JointTrajectory current_traj_;
    size_t current_point_index_{0};
    double trajectory_start_time_{0.0};  // 当前轨迹段起始墙钟时间

    // 发送线程
    void sendLoop();
    std::thread send_thread_;
    std::atomic<bool> send_running_{false};
    std::vector<aubo_robot_namespace::wayPoint_S> precomputed_;
    size_t precomputed_idx_{0};
    trajectory_msgs::msg::JointTrajectory goal_target_;

    // 上一段终点 (用于 C1 连续性)
    trajectory_msgs::msg::JointTrajectoryPoint last_goal_point_;
    bool has_last_goal_{false};

    // 容差参数
    double goal_tolerance_{0.02};        // rad, 与 ROS2 trajectory_action 一致
    int goal_hold_count_{0};            // 连续容差满足计数
    static constexpr int kGoalHoldRequired = 5;  // 需要连续 5 帧

    // 节拍控制 (壁钟时序补偿)
    double update_period_{0.005};       // 200Hz = 5ms
    double wall_clock_start_{0.0};      // 当前轨迹段起始 perf_counter

    // 发布器
    rclcpp::Publisher<control_msgs::action::FollowJointTrajectory_Feedback>::SharedPtr feedback_pub_;
};

}  // namespace aubo_driver

#endif  // AUBO_DRIVER_ROS2_JOINT_TRAJECTORY_CONTROLLER_H_
