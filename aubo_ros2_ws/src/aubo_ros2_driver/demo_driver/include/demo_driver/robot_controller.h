#ifndef DEMO_DRIVER_ROBOT_CONTROLLER_H_
#define DEMO_DRIVER_ROBOT_CONTROLLER_H_

#include <moveit/move_group_interface/move_group_interface.h>
#include <ivg_interfaces/srv/set_robot_io.hpp>
#include <rclcpp/rclcpp.hpp>
#include <array>
#include <memory>
#include <string>
#include <vector>

namespace demo_driver
{

/** 笛卡尔路径段: 沿指定轴移动 offset 米 */
struct CartesianSegment {
    char axis;       // 'x'/'y'/'z'
    double offset;   // 偏移量 (m)
};

/**
 * RobotController — 组合模式, 封装 MoveIt 运动 + Aubo IO 控制。
 *
 * Worker 节点持有 RobotController 成员, 通过成员函数调用,
 * 消除 gripper_swap / execute_grasp / publish_grasps 三处的重复代码。
 *
 * IO 语义统一: setGripper(pin, open) — open=true 打开夹爪。
 * 笛卡尔路径委托 MoveGroupInterface::computeCartesianPath，默认开启碰撞检测喵~
 */
class RobotController {
public:
    explicit RobotController(rclcpp::Node* owner,
                             const std::string& planning_group = "manipulator");

    /** 两阶段初始化: 构造后必须调用 init() 才能使用运动/IO 功能 */
    bool init();

    // ── 运动 ──
    bool moveToHome(float vel = 0.5f, float acc = 0.5f);
    bool moveToJoints(const std::array<double, 6>& joints, float vel, float acc);
    bool moveToPose(const geometry_msgs::msg::Pose& target, float vel, float acc);
    bool moveToPosition(double x, double y, double z, float vel, float acc);
    bool moveCartesianZ(double offset_m, float vel, float acc);
    bool moveCartesianPath(const std::vector<CartesianSegment>& segments, float vel, float acc);
    bool moveCartesianStraight(const geometry_msgs::msg::Pose& target, float vel, float acc);
    /// 批量笛卡尔路径规划+执行, waypoints 为完整路径点序列
    /// 内部调用 computeCartesianPath, fraction >= 0.95 则 execute
    bool executeCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints,
                              float vel, float acc);

    // ── 笛卡尔 slerp 工具 ──
    /// 四元数球面最短路径插值, t∈[0,1]
    static geometry_msgs::msg::Quaternion slerp(
        const geometry_msgs::msg::Quaternion& q0,
        const geometry_msgs::msg::Quaternion& q1, double t);
    /// 生成 from→to 的笛卡尔 waypoints (位置线性 + 朝向 slerp)
    static std::vector<geometry_msgs::msg::Pose> interpolateCartesian(
        const geometry_msgs::msg::Pose& from,
        const geometry_msgs::msg::Pose& to, int steps);

    // ── IO ──
    bool setGripper(int pin, bool open);    // open=true 打开
    bool setQuickSwap(int pin, bool lock);  // lock=true 锁紧

    // ── 查询 ──
    geometry_msgs::msg::Pose getCurrentPose();
    std::vector<double> getCurrentJoints();
    geometry_msgs::msg::Pose jointsToPose(const std::array<double, 6>& joints);
    std::string getEndEffectorLink() const;
    void setEndEffectorLink(const std::string& link);

    // ── 配置 ──
    void setVelocityScaling(float v);
    void setAccelerationScaling(float a);
    void setEefStep(double step) { eef_step_ = step; }
    void setZMinLimit(double limit) { z_min_limit_ = limit; }
    void setCartRetryParams(int retries, double wait_sec) {
        max_retries_ = retries; retry_wait_sec_ = wait_sec;
    }

    // 直接访问 MoveGroup (用于需要 plan()+execute() 分开的场景)
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> moveGroup() { return move_group_; }

private:
    rclcpp::Node* node_;
    std::string planning_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Client<ivg_interfaces::srv::SetRobotIO>::SharedPtr io_client_;
    std::string eef_link_;

    // 可配置参数
    double eef_step_{0.015};
    double z_min_limit_{0.2};
    int max_retries_{3};
    double retry_wait_sec_{0.1};

    geometry_msgs::msg::Pose currentPoseInternal();
};

}  // namespace demo_driver
#endif
