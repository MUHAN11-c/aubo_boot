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
    bool moveCartesianZ(double offset_m, float vel, float acc);
    bool moveCartesianPath(const std::vector<CartesianSegment>& segments, float vel, float acc);

    // ── IO ──
    bool setGripper(int pin, bool open);    // open=true 打开
    bool setQuickSwap(int pin, bool lock);  // lock=true 锁紧

    // ── 查询 ──
    geometry_msgs::msg::Pose getCurrentPose();
    std::vector<double> getCurrentJoints();
    std::string getEndEffectorLink() const;

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
    double retry_wait_sec_{0.5};

    geometry_msgs::msg::Pose currentPoseInternal();
};

}  // namespace demo_driver
#endif
