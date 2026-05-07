/*
 * AuboDashboardNode — 管理所有非实时 SDK 功能的服务节点。
 *
 * SDK 模式冲突规则 (必须遵守):
 *   ┌──────────────────────────────────────────────────────────────┐
 *   │ TCP2CAN 模式 (轨迹流) 与 SDK 运动 API 互斥:                  │
 *   │   - TCP2CAN ON  → robotServiceJointMove/LineMove 等不可用   │
 *   │   - 使用 SDK 运动 API 前必须先 LeaveTcp2CanbusMode()         │
 *   │   - 恢复轨迹流前需要 EnterTcp2CanbusMode()                   │
 *   ├──────────────────────────────────────────────────────────────┤
 *   │ 运动中的安全约束:                                           │
 *   │   - 上电/刹车/关机命令不能在运动中调用 (先 Stop)            │
 *   │   - 碰撞后必须先 CollisionRecover 才能发新的运动指令         │
 *   │   - 工具/动力学参数在运动中修改会被忽略                     │
 *   │   - IsBolck=true 会阻塞数秒 → Dashboard 统一用 IsBolck=false │
 *   ├──────────────────────────────────────────────────────────────┤
 *   │ 线程安全:                                                   │
 *   │   - conn_status_ 用于状态查询和配置 (一个线程串行使用)      │
 *   │   - SDK 回调在 SDK 内部线程执行 → 回调中不调用 SDK API      │
 *   └──────────────────────────────────────────────────────────────┘
 *
 * 设计参考: UR DashboardClientROS (独立进程, 非 RT 管理服务)
 */

#ifndef AUBO_DRIVER_ROS2_AUBO_DASHBOARD_NODE_H_
#define AUBO_DRIVER_ROS2_AUBO_DASHBOARD_NODE_H_

#include <memory>
#include <string>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "aubo_driver_ros2/aubo_hardware_interface.h"
#include "aubo_driver_ros2/AuboRobotMetaType.h"
#include "aubo_driver_ros2/serviceinterface.h"

// 自定义服务类型
#include <demo_interface/srv/set_robot_io.hpp>
#include <demo_interface/srv/move_joint.hpp>
#include <demo_interface/srv/move_line.hpp>
#include <demo_interface/srv/teach_start.hpp>
#include <demo_interface/srv/set_collision_class.hpp>
#include <demo_interface/srv/set_tool_kinematics.hpp>
#include <demo_interface/srv/set_tool_voltage.hpp>
#include <aubo_msgs/srv/get_fk.hpp>
#include <aubo_msgs/srv/get_ik.hpp>
#include <aubo_msgs/srv/set_payload.hpp>

namespace aubo_driver {

/**
 * Dashboard 节点 —— 管理机械臂电源、运动、配置、运动学、诊断。
 *
 * 使用方式:
 *   1. 启动 dashboard_node 可执行文件
 *   2. 调用 /aubo/startup 服务完成机器人上电+松刹车+设置碰撞等级
 *   3. 使用 /aubo/move_joint 等进行直接运动控制 (自动处理 TCP2CAN 切换)
 *   4. 使用 /aubo/set_payload 等配置工具参数
 *   5. 使用 /aubo/get_fk 等进行运动学计算
 *   6. 关闭时调用 /aubo/shutdown
 *
 * Lifecycle 状态:
 *   UNCONFIGURED → on_configure (Login ×2) → INACTIVE
 *   INACTIVE     → on_activate   (Startup)  → ACTIVE
 *   ACTIVE       → on_deactivate (Stop)     → INACTIVE
 *   INACTIVE     → on_cleanup    (Logout)   → UNCONFIGURED
 */
class AuboDashboardNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit AuboDashboardNode(const rclcpp::NodeOptions& options =
                               rclcpp::NodeOptions());
    ~AuboDashboardNode() override;

    // Lifecycle 回调
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State&) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State&) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State&) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State&) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State&) override;

private:
    // ================================================================
    // 系统管理服务 (Power Control)
    // ================================================================

    /** 启动机器人: 上电 + 松刹车 + 设置碰撞等级 + 动力学参数 */
    void onStartup(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> resp);

    /** 关机 */
    void onShutdown(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> resp);

    /** 松刹车 */
    void onBrakeRelease(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> resp);

    /** 停止运动 */
    void onStop(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                std::shared_ptr<std_srvs::srv::Trigger::Response> resp);

    /** 快速停止 */
    void onFastStop(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> resp);

    /** 碰撞恢复 */
    void onCollisionRecover(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> resp);

    // ================================================================
    // 运动控制服务 (Motion Control)
    // ================================================================

    /** 关节运动: robotServiceJointMove */
    void onMoveJoint(const std::shared_ptr<demo_interface::srv::MoveJoint::Request> req,
                     std::shared_ptr<demo_interface::srv::MoveJoint::Response> resp);

    /** 直线运动: robotServiceLineMove */
    void onMoveLine(const std::shared_ptr<demo_interface::srv::MoveLine::Request> req,
                    std::shared_ptr<demo_interface::srv::MoveLine::Response> resp);

    /** 进入教学模式 */
    void onTeachStart(const std::shared_ptr<demo_interface::srv::TeachStart::Request> req,
                      std::shared_ptr<demo_interface::srv::TeachStart::Response> resp);

    /** 退出教学模式 */
    void onTeachStop(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> resp);

    // ================================================================
    // 配置服务 (Configuration)
    // ================================================================

    /** 设置碰撞等级 */
    void onSetCollisionClass(
        const std::shared_ptr<demo_interface::srv::SetCollisionClass::Request> req,
        std::shared_ptr<demo_interface::srv::SetCollisionClass::Response> resp);

    /** 设置工具动力学参数 (payload) */
    void onSetPayload(
        const std::shared_ptr<aubo_msgs::srv::SetPayload::Request> req,
        std::shared_ptr<aubo_msgs::srv::SetPayload::Response> resp);

    /** 设置工具运动学参数 (TCP) */
    void onSetToolKinematics(
        const std::shared_ptr<demo_interface::srv::SetToolKinematics::Request> req,
        std::shared_ptr<demo_interface::srv::SetToolKinematics::Response> resp);

    /** 设置工具电源电压 */
    void onSetToolVoltage(
        const std::shared_ptr<demo_interface::srv::SetToolVoltage::Request> req,
        std::shared_ptr<demo_interface::srv::SetToolVoltage::Response> resp);

    // ================================================================
    // IO 控制服务
    // ================================================================

    void onSetIO(const std::shared_ptr<demo_interface::srv::SetRobotIO::Request> req,
                 std::shared_ptr<demo_interface::srv::SetRobotIO::Response> resp);

    // ================================================================
    // 运动学服务 (Kinematics)
    // ================================================================

    void onGetFK(const std::shared_ptr<aubo_msgs::srv::GetFK::Request> req,
                 std::shared_ptr<aubo_msgs::srv::GetFK::Response> resp);

    void onGetIK(const std::shared_ptr<aubo_msgs::srv::GetIK::Request> req,
                 std::shared_ptr<aubo_msgs::srv::GetIK::Response> resp);

    // ================================================================
    // 诊断服务 (Diagnostics)
    // ================================================================

    /** 获取设备信息 */
    void onGetRobotInfo(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> resp);

    /** 获取关节状态 */
    void onGetJointStatus(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> resp);

    /** 获取安全配置 */
    void onGetSafetyConfig(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> resp);

    // ================================================================
    // 内部辅助
    // ================================================================

    /** 确保 TCP2CAN 模式已退出 (供运动 API 调用前检查) */
    bool ensureTcp2CanOff();

    /** 恢复 TCP2CAN 模式 (与 ensureTcp2CanOff 配对) */
    void restoreTcp2CanIfNeeded();

    /** 设置 Trigger 响应为成功 */
    static void setSuccess(std::shared_ptr<std_srvs::srv::Trigger::Response> resp,
                           const std::string& msg = "ok");

    /** 设置 Trigger 响应为失败 */
    static void setFailure(std::shared_ptr<std_srvs::srv::Trigger::Response> resp,
                           const std::string& msg);

    // 硬件接口 (拥有独立的 ServiceInterface 连接)
    std::unique_ptr<AuboHardwareInterface> hw_;
    std::mutex sdk_mutex_;  // 保护 SDK 调用串行化 (ServiceInterface 非线程安全)

    // 服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_startup_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_shutdown_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_brake_release_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_stop_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_fast_stop_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_collision_recover_;
    rclcpp::Service<demo_interface::srv::MoveJoint>::SharedPtr srv_move_joint_;
    rclcpp::Service<demo_interface::srv::MoveLine>::SharedPtr srv_move_line_;
    rclcpp::Service<demo_interface::srv::TeachStart>::SharedPtr srv_teach_start_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_teach_stop_;
    rclcpp::Service<demo_interface::srv::SetCollisionClass>::SharedPtr srv_set_collision_class_;
    rclcpp::Service<aubo_msgs::srv::SetPayload>::SharedPtr srv_set_payload_;
    rclcpp::Service<demo_interface::srv::SetToolKinematics>::SharedPtr srv_set_tool_kinematics_;
    rclcpp::Service<demo_interface::srv::SetToolVoltage>::SharedPtr srv_set_tool_voltage_;
    rclcpp::Service<demo_interface::srv::SetRobotIO>::SharedPtr srv_set_io_;
    rclcpp::Service<aubo_msgs::srv::GetFK>::SharedPtr srv_get_fk_;
    rclcpp::Service<aubo_msgs::srv::GetIK>::SharedPtr srv_get_ik_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_get_robot_info_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_get_joint_status_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_get_safety_config_;

    // 配置参数
    std::string server_host_{"127.0.0.1"};
    int server_port_{8899};
    int collision_class_{6};
    bool tcp2can_was_active_{false};  // 记录运动 API 调用前 TCP2CAN 状态
};

}  // namespace aubo_driver

#endif  // AUBO_DRIVER_ROS2_AUBO_DASHBOARD_NODE_H_
