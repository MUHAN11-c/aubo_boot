/*
 * RobotController — 组合模式封装 MoveIt 运动 + Aubo IO。
 * 消除 gripper_swap / execute_grasp / publish_grasps 三处的重复代码。
 * IO 语义统一: setGripper(pin, open=true) 打开夹爪。
 * moveCartesianPath：单次 computeCartesianPath(..., avoid_collisions=true) + 执行喵~
 */
#include "demo_driver/robot_controller.h"
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <geometry_msgs/msg/pose.h>
#include <chrono>
#include <cmath>
#include <thread>

namespace demo_driver
{

RobotController::RobotController(rclcpp::Node* owner, const std::string& planning_group)
    : node_(owner), planning_group_(planning_group)
{
    // 两阶段初始化: 构造只存指针，init() 才建 MoveGroupInterface。
    // MoveGroupInterface 构造需要 owner->shared_from_this()，
    // 但 enable_shared_from_this 的 weak_ptr 在 shared_ptr 构造完成后才初始化。
    move_group_ = nullptr;
}

bool RobotController::init()
{
    if (move_group_) return true;  // 已初始化
    if (!node_) return false;
    try {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node_->shared_from_this(), planning_group_);
        move_group_->allowReplanning(true);
        move_group_->setMaxVelocityScalingFactor(0.5);
        move_group_->setMaxAccelerationScalingFactor(0.5);
        eef_link_ = move_group_->getEndEffectorLink();

        io_client_ = node_->create_client<ivg_interfaces::srv::SetRobotIO>("/set_robot_io");
        return true;
    } catch (...) {
        return false;
    }
}

// =====================================================================
// 运动
// =====================================================================

bool RobotController::moveToHome(float vel, float acc)
{
    if (!move_group_) return false;
    move_group_->setStartStateToCurrentState();
    setVelocityScaling(vel);
    setAccelerationScaling(acc);
    move_group_->setNamedTarget("camera_pose");
    const auto result = move_group_->move();
    return result == moveit::core::MoveItErrorCode::SUCCESS;
}

bool RobotController::moveToJoints(const std::array<double, 6>& joints, float vel, float acc)
{
    if (!move_group_) return false;
    move_group_->setStartStateToCurrentState();
    setVelocityScaling(vel);
    setAccelerationScaling(acc);
    move_group_->setJointValueTarget(std::vector<double>(joints.begin(), joints.end()));
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = move_group_->plan(plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) return false;
    const auto execute_result = move_group_->execute(plan);
    return execute_result == moveit::core::MoveItErrorCode::SUCCESS;
}

bool RobotController::moveToPose(const geometry_msgs::msg::Pose& target,
                                 float vel, float acc)
{
    if (!move_group_) return false;
    move_group_->setStartStateToCurrentState();
    setVelocityScaling(vel);
    setAccelerationScaling(acc);
    move_group_->setPoseTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        return false;
    return move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

bool RobotController::moveCartesianZ(double offset_m, float vel, float acc)
{
    return moveCartesianPath({{'z', offset_m}}, vel, acc);
}

bool RobotController::moveCartesianPath(const std::vector<CartesianSegment>& segments,
                                        float vel, float acc)
{
    if (!move_group_ || segments.empty()) return false;
    setVelocityScaling(vel);
    setAccelerationScaling(acc);

    auto start = currentPoseInternal();
    std::vector<geometry_msgs::msg::Pose> waypoints{start};

    for (const auto& seg : segments) {
        auto next = waypoints.back();
        switch (seg.axis) {
            case 'x': next.position.x += seg.offset; break;
            case 'y': next.position.y += seg.offset; break;
            case 'z': next.position.z += seg.offset; break;
        }
        if (next.position.z < z_min_limit_) next.position.z = z_min_limit_;
        waypoints.push_back(next);
    }
    waypoints.erase(waypoints.begin());  // 去掉起点

    for (int attempt = 0; attempt < max_retries_; ++attempt) {
        moveit_msgs::msg::RobotTrajectory traj;
        moveit_msgs::msg::MoveItErrorCodes error_code;
        double fraction =
            move_group_->computeCartesianPath(waypoints, eef_step_, 0.0, traj, true, &error_code);
        if (fraction < 0.99) {
            if (attempt < max_retries_ - 1)
                std::this_thread::sleep_for(std::chrono::duration<double>(retry_wait_sec_));
            continue;
        }
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = traj;
        return move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }
    return false;
}

bool RobotController::moveCartesianStraight(const geometry_msgs::msg::Pose& target,
                                            float vel, float acc)
{
    if (!move_group_) return false;
    setVelocityScaling(vel);
    setAccelerationScaling(acc);

    double saved_step = eef_step_;
    eef_step_ = 0.002;

    // 手动生成 slerp waypoints, 保证最短旋转路径
    auto start = move_group_->getCurrentPose(eef_link_).pose;
    int steps = 20;  // SLERP waypoints 数量: 保证姿态过渡平滑
    auto waypoints = interpolateCartesian(start, target, steps);

    bool success = false;
    for (int attempt = 0; attempt < max_retries_; ++attempt) {
        moveit_msgs::msg::RobotTrajectory traj;
        moveit_msgs::msg::MoveItErrorCodes error_code;
        double fraction =
            move_group_->computeCartesianPath(waypoints, eef_step_, 0.0, traj, true, &error_code);
        if (fraction < 0.99) {
            RCLCPP_WARN(node_->get_logger(),
                "CartesianStraight 规划 fraction=%.3f (第%d/%d次), 重试...",
                fraction, attempt + 1, max_retries_);
            if (attempt < max_retries_ - 1)
                std::this_thread::sleep_for(std::chrono::duration<double>(retry_wait_sec_));
            continue;
        }
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = traj;
        success = (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) break;
        RCLCPP_WARN(node_->get_logger(),
            "CartesianStraight 执行失败 (第%d/%d次), 重试...",
            attempt + 1, max_retries_);
        if (attempt < max_retries_ - 1)
            std::this_thread::sleep_for(std::chrono::duration<double>(retry_wait_sec_));
    }

    eef_step_ = saved_step;
    return success;
}

// ═══════════════════════════════════════════════════════════════════════════
// executeCartesianPath: 批量笛卡尔路径规划+执行
// ═══════════════════════════════════════════════════════════════════════════

bool RobotController::executeCartesianPath(
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    float vel, float acc)
{
    if (!move_group_ || waypoints.empty()) return false;
    setVelocityScaling(vel);
    setAccelerationScaling(acc);

    // 与 moveCartesianStraight 一致: 设为 0.002 防止 MoveIt2 跳过密集 waypoints
    double saved_step = eef_step_;
    eef_step_ = 0.002;

    for (int attempt = 0; attempt < max_retries_; ++attempt) {
        moveit_msgs::msg::RobotTrajectory traj;
        moveit_msgs::msg::MoveItErrorCodes error_code;
        double fraction =
            move_group_->computeCartesianPath(waypoints, eef_step_, 0.0, traj, true, &error_code);
        if (fraction < 0.95) {
            RCLCPP_WARN(node_->get_logger(),
                "executeCartesianPath 规划 fraction=%.3f (第%d/%d次), 重试...",
                fraction, attempt + 1, max_retries_);
            if (attempt < max_retries_ - 1)
                std::this_thread::sleep_for(std::chrono::duration<double>(retry_wait_sec_));
            continue;
        }
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = traj;
        bool ok = (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (ok) { eef_step_ = saved_step; return true; }
        RCLCPP_WARN(node_->get_logger(),
            "executeCartesianPath 执行失败 (第%d/%d次), 重试...",
            attempt + 1, max_retries_);
        if (attempt < max_retries_ - 1)
            std::this_thread::sleep_for(std::chrono::duration<double>(retry_wait_sec_));
    }

    eef_step_ = saved_step;
    return false;
}

// ═══════════════════════════════════════════════════════════════════════════
// slerp: 四元数球面最短路径插值
// ═══════════════════════════════════════════════════════════════════════════

geometry_msgs::msg::Quaternion RobotController::slerp(
    const geometry_msgs::msg::Quaternion& q0,
    const geometry_msgs::msg::Quaternion& q1, double t)
{
    // 点积 → cos(θ)
    double dot = q0.w * q1.w + q0.x * q1.x + q0.y * q1.y + q0.z * q1.z;

    // q 和 -q 表示同一旋转, 选同号半球保证最短路径
    geometry_msgs::msg::Quaternion q1f = q1;
    if (dot < 0.0) {
        dot = -dot;
        q1f.w = -q1f.w; q1f.x = -q1f.x; q1f.y = -q1f.y; q1f.z = -q1f.z;
    }

    // 夹角极小 → 线性插值避免除零
    const double eps = 0.9995;
    geometry_msgs::msg::Quaternion r;
    if (dot > eps) {
        double s = 1.0 - t;
        r.w = s * q0.w + t * q1f.w;
        r.x = s * q0.x + t * q1f.x;
        r.y = s * q0.y + t * q1f.y;
        r.z = s * q0.z + t * q1f.z;
        // 归一化
        double n = std::sqrt(r.w*r.w + r.x*r.x + r.y*r.y + r.z*r.z);
        r.w /= n; r.x /= n; r.y /= n; r.z /= n;
        return r;
    }

    double theta = std::acos(dot);
    double sin_theta = std::sin(theta);
    double s0 = std::sin((1.0 - t) * theta) / sin_theta;
    double s1 = std::sin(t * theta) / sin_theta;
    r.w = s0 * q0.w + s1 * q1f.w;
    r.x = s0 * q0.x + s1 * q1f.x;
    r.y = s0 * q0.y + s1 * q1f.y;
    r.z = s0 * q0.z + s1 * q1f.z;
    return r;
}

// ═══════════════════════════════════════════════════════════════════════════
// interpolateCartesian: 位置线性 + 朝向 slerp 生成 waypoints
// ═══════════════════════════════════════════════════════════════════════════

std::vector<geometry_msgs::msg::Pose> RobotController::interpolateCartesian(
    const geometry_msgs::msg::Pose& from,
    const geometry_msgs::msg::Pose& to, int steps)
{
    std::vector<geometry_msgs::msg::Pose> waypoints;
    for (int i = 1; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        geometry_msgs::msg::Pose p;
        p.position.x = from.position.x + t * (to.position.x - from.position.x);
        p.position.y = from.position.y + t * (to.position.y - from.position.y);
        p.position.z = from.position.z + t * (to.position.z - from.position.z);
        p.orientation = slerp(from.orientation, to.orientation, t);
        waypoints.push_back(p);
    }
    return waypoints;
}

// =====================================================================
// IO
// =====================================================================

bool RobotController::setGripper(int pin, bool open)
{
    if (!io_client_) { RCLCPP_WARN(node_->get_logger(), "setGripper: io_client_ 未初始化"); return false; }
    if (!io_client_->wait_for_service(std::chrono::seconds(3))) return false;
    auto req = std::make_shared<ivg_interfaces::srv::SetRobotIO::Request>();
    req->io_type  = "digital_output";
    req->io_index = pin;
    req->value    = open ? 1.0 : 0.0;
    auto future = io_client_->async_send_request(req);
    return future.wait_for(std::chrono::seconds(10)) == std::future_status::ready &&
           future.get()->success;
}

bool RobotController::setQuickSwap(int pin, bool lock)
{
    return setGripper(pin, lock);
}

// =====================================================================
// 查询
// =====================================================================

geometry_msgs::msg::Pose RobotController::getCurrentPose()
{
    if (!move_group_) { RCLCPP_WARN(node_->get_logger(), "getCurrentPose: move_group_ 未初始化"); return geometry_msgs::msg::Pose(); }
    return move_group_->getCurrentPose(eef_link_).pose;
}

std::vector<double> RobotController::getCurrentJoints()
{
    if (!move_group_) { RCLCPP_WARN(node_->get_logger(), "getCurrentJoints: move_group_ 未初始化"); return {}; }
    std::vector<double> jv;
    move_group_->getCurrentState()->copyJointGroupPositions(
        move_group_->getCurrentState()->getRobotModel()->getJointModelGroup(
            move_group_->getName()), jv);
    return jv;
}

geometry_msgs::msg::Pose RobotController::jointsToPose(const std::array<double, 6>& joints)
{
    if (!move_group_) { RCLCPP_WARN(node_->get_logger(), "jointsToPose: move_group_ 未初始化"); return geometry_msgs::msg::Pose(); }
    auto robot_model = move_group_->getRobotModel();
    auto jmg = robot_model->getJointModelGroup(move_group_->getName());
    moveit::core::RobotState state(robot_model);
    state.setJointGroupPositions(jmg, std::vector<double>(joints.begin(), joints.end()));
    state.update();
    const auto& t = state.getGlobalLinkTransform(eef_link_);
    geometry_msgs::msg::Pose p;
    p.position.x = t.translation().x();
    p.position.y = t.translation().y();
    p.position.z = t.translation().z();
    Eigen::Quaterniond q(t.rotation());
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();
    return p;
}

std::string RobotController::getEndEffectorLink() const
{
    return eef_link_;
}

void RobotController::setEndEffectorLink(const std::string& link)
{
    eef_link_ = link;
    move_group_->setEndEffectorLink(link);
}

// =====================================================================
// 配置
// =====================================================================

void RobotController::setVelocityScaling(float v) { if (move_group_) move_group_->setMaxVelocityScalingFactor(v); }
void RobotController::setAccelerationScaling(float a) { if (move_group_) move_group_->setMaxAccelerationScalingFactor(a); }

geometry_msgs::msg::Pose RobotController::currentPoseInternal()
{
    if (!move_group_) return geometry_msgs::msg::Pose();
    return move_group_->getCurrentPose(eef_link_).pose;
}

}  // namespace demo_driver
