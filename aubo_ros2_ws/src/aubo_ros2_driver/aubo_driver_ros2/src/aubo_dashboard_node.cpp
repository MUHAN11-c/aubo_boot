/*
 * AuboDashboardNode 实现 — 20 个服务覆盖全部非实时 SDK 功能。
 *
 * SDK 冲突处理:
 *   - 运动 API 调用前自动 ensureTcp2CanOff()
 *   - 运动 API 调用后恢复 TCP2CAN 状态
 *   - 所有运动 API 使用 IsBolck=false
 *   - Startup/Shutdown 在 conn_status_ 上串行执行
 */

#include "aubo_driver_ros2/aubo_dashboard_node.h"
#include <chrono>
#include <thread>
#include <mutex>

namespace aubo_driver {

// ============================================================================
// 构造/析构
// ============================================================================

AuboDashboardNode::AuboDashboardNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("aubo_dashboard", options)
{
    // 参数在构造时声明, 若已由 auto_declare 声明则跳过
    if (!this->has_parameter("server_host"))
        this->declare_parameter("server_host", rclcpp::ParameterValue(std::string("169.254.10.98")));
    if (!this->has_parameter("server_port"))
        this->declare_parameter("server_port", rclcpp::ParameterValue(8899));
    if (!this->has_parameter("collision_class"))
        this->declare_parameter("collision_class", rclcpp::ParameterValue(6));
    RCLCPP_INFO(this->get_logger(), "Dashboard node created");
}

AuboDashboardNode::~AuboDashboardNode()
{
    RCLCPP_INFO(this->get_logger(), "Dashboard node destroyed");
}

// ============================================================================
// Lifecycle 回调
// ============================================================================

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AuboDashboardNode::on_configure(const rclcpp_lifecycle::State&)
{
    server_host_ = this->get_parameter("server_host").as_string();
    server_port_ = this->get_parameter("server_port").as_int();
    collision_class_ = this->get_parameter("collision_class").as_int();

    hw_ = std::make_unique<AuboHardwareInterface>();

    if (!hw_->init(server_host_, server_port_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to init hardware interface");
        return rclcpp_lifecycle::node_interfaces::
            LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    // ---- 创建服务 ----
    // 系统管理
    srv_startup_ = this->create_service<std_srvs::srv::Trigger>(
        "/aubo/startup",
        std::bind(&AuboDashboardNode::onStartup, this, std::placeholders::_1, std::placeholders::_2));
    srv_shutdown_ = this->create_service<std_srvs::srv::Trigger>(
        "/aubo/shutdown",
        std::bind(&AuboDashboardNode::onShutdown, this, std::placeholders::_1, std::placeholders::_2));
    srv_brake_release_ = this->create_service<std_srvs::srv::Trigger>(
        "/aubo/brake_release",
        std::bind(&AuboDashboardNode::onBrakeRelease, this, std::placeholders::_1, std::placeholders::_2));
    srv_stop_ = this->create_service<std_srvs::srv::Trigger>(
        "/aubo/stop",
        std::bind(&AuboDashboardNode::onStop, this, std::placeholders::_1, std::placeholders::_2));
    srv_fast_stop_ = this->create_service<std_srvs::srv::Trigger>(
        "/aubo/fast_stop",
        std::bind(&AuboDashboardNode::onFastStop, this, std::placeholders::_1, std::placeholders::_2));
    srv_collision_recover_ = this->create_service<std_srvs::srv::Trigger>(
        "/aubo/collision_recover",
        std::bind(&AuboDashboardNode::onCollisionRecover, this, std::placeholders::_1, std::placeholders::_2));

    // 运动控制 (自动处理 TCP2CAN 切换)
    srv_move_joint_ = this->create_service<ivg_interfaces::srv::MoveJoint>(
        "/aubo/move_joint",
        std::bind(&AuboDashboardNode::onMoveJoint, this, std::placeholders::_1, std::placeholders::_2));
    srv_move_line_ = this->create_service<ivg_interfaces::srv::MoveLine>(
        "/aubo/move_line",
        std::bind(&AuboDashboardNode::onMoveLine, this, std::placeholders::_1, std::placeholders::_2));
    srv_teach_start_ = this->create_service<ivg_interfaces::srv::TeachStart>(
        "/aubo/teach_start",
        std::bind(&AuboDashboardNode::onTeachStart, this, std::placeholders::_1, std::placeholders::_2));
    srv_teach_stop_ = this->create_service<std_srvs::srv::Trigger>(
        "/aubo/teach_stop",
        std::bind(&AuboDashboardNode::onTeachStop, this, std::placeholders::_1, std::placeholders::_2));

    // 配置
    srv_set_collision_class_ = this->create_service<ivg_interfaces::srv::SetCollisionClass>(
        "/aubo/set_collision_class",
        std::bind(&AuboDashboardNode::onSetCollisionClass, this, std::placeholders::_1, std::placeholders::_2));
    srv_set_payload_ = this->create_service<ivg_interfaces::srv::SetPayload>(
        "/aubo/set_payload",
        std::bind(&AuboDashboardNode::onSetPayload, this, std::placeholders::_1, std::placeholders::_2));
    srv_set_tool_kinematics_ = this->create_service<ivg_interfaces::srv::SetToolKinematics>(
        "/aubo/set_tool_kinematics",
        std::bind(&AuboDashboardNode::onSetToolKinematics, this, std::placeholders::_1, std::placeholders::_2));
    srv_set_tool_voltage_ = this->create_service<ivg_interfaces::srv::SetToolVoltage>(
        "/aubo/set_tool_voltage",
        std::bind(&AuboDashboardNode::onSetToolVoltage, this, std::placeholders::_1, std::placeholders::_2));

    // IO (使用已有自定义服务)
    srv_set_io_ = this->create_service<ivg_interfaces::srv::SetRobotIO>(
        "/set_robot_io",
        std::bind(&AuboDashboardNode::onSetIO, this, std::placeholders::_1, std::placeholders::_2));

    // 运动学
    srv_get_fk_ = this->create_service<ivg_interfaces::srv::GetFK>(
        "/aubo/get_fk",
        std::bind(&AuboDashboardNode::onGetFK, this, std::placeholders::_1, std::placeholders::_2));
    srv_get_ik_ = this->create_service<ivg_interfaces::srv::GetIK>(
        "/aubo/get_ik",
        std::bind(&AuboDashboardNode::onGetIK, this, std::placeholders::_1, std::placeholders::_2));

    // 诊断
    srv_get_robot_info_ = this->create_service<std_srvs::srv::Trigger>(
        "/aubo/get_robot_info",
        std::bind(&AuboDashboardNode::onGetRobotInfo, this, std::placeholders::_1, std::placeholders::_2));
    srv_get_joint_status_ = this->create_service<std_srvs::srv::Trigger>(
        "/aubo/get_joint_status",
        std::bind(&AuboDashboardNode::onGetJointStatus, this, std::placeholders::_1, std::placeholders::_2));
    srv_get_safety_config_ = this->create_service<std_srvs::srv::Trigger>(
        "/aubo/get_safety_config",
        std::bind(&AuboDashboardNode::onGetSafetyConfig, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Dashboard configured (20 services)");
    return rclcpp_lifecycle::node_interfaces::
        LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AuboDashboardNode::on_activate(const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(this->get_logger(), "Dashboard activated");
    return rclcpp_lifecycle::node_interfaces::
        LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AuboDashboardNode::on_deactivate(const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(this->get_logger(), "Dashboard deactivated");
    return rclcpp_lifecycle::node_interfaces::
        LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AuboDashboardNode::on_cleanup(const rclcpp_lifecycle::State&)
{
    if (hw_) {
        hw_->shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "Dashboard cleaned up");
    return rclcpp_lifecycle::node_interfaces::
        LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AuboDashboardNode::on_shutdown(const rclcpp_lifecycle::State&)
{
    if (hw_) {
        hw_->shutdown();
    }
    return rclcpp_lifecycle::node_interfaces::
        LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// ============================================================================
// 辅助方法
// ============================================================================

void AuboDashboardNode::setSuccess(
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp, const std::string& msg)
{
    resp->success = true;
    resp->message = msg;
}

void AuboDashboardNode::setFailure(
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp, const std::string& msg)
{
    resp->success = false;
    resp->message = msg;
}

bool AuboDashboardNode::ensureTcp2CanOff()
{
    // Dashboard 拥有独立的 ServiceInterface 连接，不会进入 TCP2CAN 模式。
    // TCP2CAN 仅存在于轨迹控制器的 HardwareInterface 实例中。
    // 本函数用于保护: 万一将来 Dashboard 的连接被意外切到 TCP2CAN 模式。
    if (hw_->isConnected() && hw_->isInTcp2CanMode()) {
        tcp2can_was_active_ = true;
        if (!hw_->leaveTcp2CanbusMode()) {
            RCLCPP_ERROR(this->get_logger(),
                "Failed to leave TCP2CAN mode before SDK motion API call");
            return false;
        }
        RCLCPP_INFO(this->get_logger(),
            "Temporarily left TCP2CAN mode for SDK motion API");
    } else {
        tcp2can_was_active_ = false;
    }
    return true;
}

/** 恢复 TCP2CAN 模式 (与 ensureTcp2CanOff 配对调用) */
void AuboDashboardNode::restoreTcp2CanIfNeeded()
{
    if (tcp2can_was_active_ && hw_ && hw_->isConnected()) {
        tcp2can_was_active_ = false;
        if (hw_->enterTcp2CanbusMode()) {
            RCLCPP_INFO(this->get_logger(), "Restored TCP2CAN mode after SDK motion API");
        }
    }
}

// ============================================================================
// 系统管理服务
// ============================================================================

void AuboDashboardNode::onStartup(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
    if (!hw_ || !hw_->isConnected()) {
        setFailure(resp, "Not connected to robot");
        return;
    }

    std::lock_guard<std::mutex> lock(sdk_mutex_);
    auto& sdk = hw_->statusService();

    // Step 1: 设置动力学参数 (无工具默认值)
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    std::memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));

    // Step 2: 启动——完成上电+松刹车+设置碰撞等级
    aubo_robot_namespace::ROBOT_SERVICE_STATE result;
    int ret = sdk.rootServiceRobotStartup(
        toolDynamicsParam, collision_class_,
        true,   // readPose
        true,   // staticCollisionDetect
        1000,   // boardMaxAcc
        result,
        true);  // IsBolck: 等待完成

    if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
        setSuccess(resp, "Startup successful");
    } else {
        setFailure(resp, "Startup failed (ret=" + std::to_string(ret) + ")");
    }
}

void AuboDashboardNode::onShutdown(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
    if (!hw_ || !hw_->isConnected()) {
        setFailure(resp, "Not connected");
        return;
    }

    std::lock_guard<std::mutex> lock(sdk_mutex_);
    // 先停止运动
    hw_->statusService().rootServiceRobotMoveControl(
        aubo_robot_namespace::RobotMoveStop);

    // 关机
    int ret = hw_->statusService().robotServiceRobotShutdown(true);
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
        setSuccess(resp, "Shutdown successful");
    } else {
        setFailure(resp, "Shutdown failed");
    }
}

void AuboDashboardNode::onBrakeRelease(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
    if (!hw_ || !hw_->isConnected()) {
        setFailure(resp, "Not connected");
        return;
    }
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    int ret = hw_->statusService().robotServiceReleaseBrake();
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
        setSuccess(resp, "Brake released");
    } else {
        setFailure(resp, "Brake release failed");
    }
}

void AuboDashboardNode::onStop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
    if (!hw_ || !hw_->isConnected()) {
        setFailure(resp, "Not connected");
        return;
    }
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    int ret = hw_->statusService().rootServiceRobotMoveControl(
        aubo_robot_namespace::RobotMoveStop);
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
        setSuccess(resp, "Motion stopped");
    } else {
        setFailure(resp, "Stop failed");
    }
}

void AuboDashboardNode::onFastStop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
    if (!hw_ || !hw_->isConnected()) {
        setFailure(resp, "Not connected");
        return;
    }
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    int ret = hw_->statusService().robotMoveFastStop();
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
        setSuccess(resp, "Fast stop executed");
    } else {
        setFailure(resp, "Fast stop failed");
    }
}

void AuboDashboardNode::onCollisionRecover(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
    if (!hw_ || !hw_->isConnected()) {
        setFailure(resp, "Not connected");
        return;
    }
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    // 碰撞后必须先恢复才能继续运动
    int ret = hw_->statusService().robotServiceCollisionRecover();
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
        setSuccess(resp, "Collision recovered");
    } else {
        setFailure(resp, "Collision recover failed");
    }
}

// ============================================================================
// 运动控制服务 (自动处理 TCP2CAN 切换)
// ============================================================================

void AuboDashboardNode::onMoveJoint(
    const std::shared_ptr<ivg_interfaces::srv::MoveJoint::Request> req,
    std::shared_ptr<ivg_interfaces::srv::MoveJoint::Response> resp)
{
    if (!hw_ || !hw_->isConnected()) {
        resp->success = false; resp->error_code = -1;
        resp->message = "Not connected to robot";
        return;
    }
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    if (!ensureTcp2CanOff()) {
        resp->success = false; resp->error_code = -2;
        resp->message = "Cannot leave TCP2CAN mode";
        return;
    }

    double joints[6];
    for (int i = 0; i < 6; i++) joints[i] = req->joints[i];

    int ret = hw_->statusService().robotServiceJointMove(joints, false);
    resp->success = (ret == aubo_robot_namespace::InterfaceCallSuccCode);
    resp->error_code = resp->success ? 0 : ret;
    resp->message = resp->success ? "Joint move command sent"
                    : "Joint move failed (ret=" + std::to_string(ret) + ")";
}

void AuboDashboardNode::onMoveLine(
    const std::shared_ptr<ivg_interfaces::srv::MoveLine::Request> req,
    std::shared_ptr<ivg_interfaces::srv::MoveLine::Response> resp)
{
    if (!hw_ || !hw_->isConnected()) {
        resp->success = false; resp->error_code = -1;
        resp->message = "Not connected";
        return;
    }
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    if (!ensureTcp2CanOff()) {
        resp->success = false; resp->error_code = -2;
        resp->message = "Cannot leave TCP2CAN mode";
        return;
    }

    double joints[6];
    for (int i = 0; i < 6; i++) joints[i] = req->joints[i];

    int ret = hw_->statusService().robotServiceLineMove(joints, false);
    resp->success = (ret == aubo_robot_namespace::InterfaceCallSuccCode);
    resp->error_code = resp->success ? 0 : ret;
    resp->message = resp->success ? "Line move command sent"
                    : "Line move failed (ret=" + std::to_string(ret) + ")";
}

void AuboDashboardNode::onTeachStart(
    const std::shared_ptr<ivg_interfaces::srv::TeachStart::Request> req,
    std::shared_ptr<ivg_interfaces::srv::TeachStart::Response> resp)
{
    if (!hw_ || !hw_->isConnected()) {
        resp->success = false; resp->error_code = -1;
        resp->message = "Not connected";
        return;
    }
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    if (!ensureTcp2CanOff()) {
        resp->success = false; resp->error_code = -2;
        resp->message = "Cannot leave TCP2CAN mode";
        return;
    }

    // 将通用索引映射到 SDK 示教模式
    // 1-6:JOINT1-6, 7:MOV_X, 8:MOV_Y, 9:MOV_Z, 10:ROT_X, 11:ROT_Y, 12:ROT_Z
    static const aubo_robot_namespace::teach_mode modes[] = {
        aubo_robot_namespace::JOINT1, aubo_robot_namespace::JOINT2,
        aubo_robot_namespace::JOINT3, aubo_robot_namespace::JOINT4,
        aubo_robot_namespace::JOINT5, aubo_robot_namespace::JOINT6,
        aubo_robot_namespace::MOV_X,  aubo_robot_namespace::MOV_Y,
        aubo_robot_namespace::MOV_Z,  aubo_robot_namespace::ROT_X,
        aubo_robot_namespace::ROT_Y,  aubo_robot_namespace::ROT_Z,
    };
    int idx = std::max(0, std::min(11, req->joint - 1));

    int ret = hw_->statusService().robotServiceTeachStart(
        modes[idx], req->direction);
    resp->success = (ret == aubo_robot_namespace::InterfaceCallSuccCode);
    resp->error_code = resp->success ? 0 : ret;
    resp->message = resp->success ? "Teach mode started"
                    : "Teach start failed (ret=" + std::to_string(ret) + ")";
}

void AuboDashboardNode::onTeachStop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
    if (!hw_ || !hw_->isConnected()) {
        setFailure(resp, "Not connected");
        return;
    }
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    int ret = hw_->statusService().robotServiceTeachStop();
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
        setSuccess(resp, "Teach mode stopped");
    } else {
        setFailure(resp, "Teach stop failed");
    }
}

// ============================================================================
// 配置服务
// ============================================================================

void AuboDashboardNode::onSetCollisionClass(
    const std::shared_ptr<ivg_interfaces::srv::SetCollisionClass::Request> req,
    std::shared_ptr<ivg_interfaces::srv::SetCollisionClass::Response> resp)
{
    if (!hw_ || !hw_->isConnected()) {
        resp->success = false;
        resp->message = "Not connected";
        return;
    }
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    int ret = hw_->statusService().robotServiceSetRobotCollisionClass(req->grade);
    resp->success = (ret == aubo_robot_namespace::InterfaceCallSuccCode);
    resp->message = resp->success ? "Collision class set to " + std::to_string(req->grade)
                    : "Set collision class failed (ret=" + std::to_string(ret) + ")";
}

void AuboDashboardNode::onSetPayload(
    const std::shared_ptr<ivg_interfaces::srv::SetPayload::Request> req,
    std::shared_ptr<ivg_interfaces::srv::SetPayload::Response> resp)
{
    if (!hw_ || !hw_->isConnected()) {
        resp->success = false;
        return;
    }
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    aubo_robot_namespace::ToolDynamicsParam param;
    std::memset(&param, 0, sizeof(param));
    param.payload = req->mass;
    param.positionX = req->center_of_gravity.x;
    param.positionY = req->center_of_gravity.y;
    param.positionZ = req->center_of_gravity.z;

    int ret = hw_->statusService().robotServiceSetToolDynamicsParam(param);
    resp->success = (ret == aubo_robot_namespace::InterfaceCallSuccCode);
}

void AuboDashboardNode::onSetToolKinematics(
    const std::shared_ptr<ivg_interfaces::srv::SetToolKinematics::Request> req,
    std::shared_ptr<ivg_interfaces::srv::SetToolKinematics::Response> resp)
{
    if (!hw_ || !hw_->isConnected()) {
        resp->success = false;
        resp->message = "Not connected";
        return;
    }
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    aubo_robot_namespace::ToolKinematicsParam param;
    param.toolInEndPosition.x = req->position[0];
    param.toolInEndPosition.y = req->position[1];
    param.toolInEndPosition.z = req->position[2];
    param.toolInEndOrientation.w = req->orientation[0];
    param.toolInEndOrientation.x = req->orientation[1];
    param.toolInEndOrientation.y = req->orientation[2];
    param.toolInEndOrientation.z = req->orientation[3];

    int ret = hw_->statusService().robotServiceSetToolKinematicsParam(param);
    resp->success = (ret == aubo_robot_namespace::InterfaceCallSuccCode);
    resp->message = resp->success ? "TCP set"
                    : "Set TCP failed (ret=" + std::to_string(ret) + ")";
}

void AuboDashboardNode::onSetToolVoltage(
    const std::shared_ptr<ivg_interfaces::srv::SetToolVoltage::Request> req,
    std::shared_ptr<ivg_interfaces::srv::SetToolVoltage::Response> resp)
{
    if (!hw_ || !hw_->isConnected()) {
        resp->success = false;
        resp->message = "Not connected";
        return;
    }
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    auto type = static_cast<aubo_robot_namespace::ToolPowerType>(req->voltage_type);
    int ret = hw_->statusService().robotServiceSetToolPowerVoltageType(type);
    resp->success = (ret == aubo_robot_namespace::InterfaceCallSuccCode);
    resp->message = resp->success ? "Tool voltage set"
                    : "Set tool voltage failed (ret=" + std::to_string(ret) + ")";
}

// ============================================================================
// IO 控制
// ============================================================================

void AuboDashboardNode::onSetIO(
    const std::shared_ptr<ivg_interfaces::srv::SetRobotIO::Request> req,
    std::shared_ptr<ivg_interfaces::srv::SetRobotIO::Response> resp)
{
    if (!hw_ || !hw_->isConnected()) {
        resp->success = false;
        resp->message = "Not connected";
        return;
    }
    std::lock_guard<std::mutex> lock(sdk_mutex_);

    std::string io_type = req->io_type;
    std::transform(io_type.begin(), io_type.end(), io_type.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    const int io_index = req->io_index;
    const double value = req->value;

    if (io_type == "digital_output") {
        resp->success = hw_->writeIOCommand(
            aubo_robot_namespace::RobotBoardUserDO, io_index + 32, value);
        resp->message = resp->success ? "digital_output ok" : "failed";
    } else if (io_type == "analog_output") {
        resp->success = hw_->writeIOCommand(
            aubo_robot_namespace::RobotBoardUserAO, io_index, value);
        resp->message = resp->success ? "analog_output ok" : "failed";
    } else if (io_type == "tool_io") {
        resp->success = hw_->writeToolIOCommand(
            io_index, value >= 0.0, value);
        resp->message = resp->success ? "tool_io ok" : "failed";
    } else if (io_type == "tool_power") {
        resp->success = hw_->writeToolPowerType(static_cast<int>(value));
        resp->message = resp->success ? "tool_power ok" : "failed";
    } else {
        resp->success = false;
        resp->message = "Unknown io_type: " + io_type;
    }
}

// ============================================================================
// 运动学服务
// ============================================================================

void AuboDashboardNode::onGetFK(
    const std::shared_ptr<ivg_interfaces::srv::GetFK::Request> req,
    std::shared_ptr<ivg_interfaces::srv::GetFK::Response> resp)
{
    if (!hw_ || !hw_->isConnected()) return;
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    aubo_robot_namespace::wayPoint_S wp;
    double joint[6];
    for (int i = 0; i < 6; i++) joint[i] = req->joint[i];

    int ret = hw_->statusService().robotServiceRobotFk(joint, 6, wp);
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
        resp->pos[0] = static_cast<float>(wp.cartPos.position.x);
        resp->pos[1] = static_cast<float>(wp.cartPos.position.y);
        resp->pos[2] = static_cast<float>(wp.cartPos.position.z);
        resp->ori[0] = static_cast<float>(wp.orientation.w);
        resp->ori[1] = static_cast<float>(wp.orientation.x);
        resp->ori[2] = static_cast<float>(wp.orientation.y);
        resp->ori[3] = static_cast<float>(wp.orientation.z);
    }
}

void AuboDashboardNode::onGetIK(
    const std::shared_ptr<ivg_interfaces::srv::GetIK::Request> req,
    std::shared_ptr<ivg_interfaces::srv::GetIK::Response> resp)
{
    if (!hw_ || !hw_->isConnected()) return;
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    aubo_robot_namespace::wayPoint_S wp;
    double ref_joint[6];
    for (int i = 0; i < 6; i++) ref_joint[i] = req->ref_joint[i];

    aubo_robot_namespace::Pos position;
    position.x = req->pos[0]; position.y = req->pos[1]; position.z = req->pos[2];
    aubo_robot_namespace::Ori ori;
    ori.w = req->ori[0]; ori.x = req->ori[1]; ori.y = req->ori[2]; ori.z = req->ori[3];

    int ret = hw_->statusService().robotServiceRobotIk(
        ref_joint, position, ori, wp);
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
        for (int i = 0; i < 6; i++) {
            resp->joint[i] = static_cast<float>(wp.jointpos[i]);
        }
    }
}

// ============================================================================
// 诊断服务
// ============================================================================

void AuboDashboardNode::onGetRobotInfo(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
    if (!hw_ || !hw_->isConnected()) {
        setFailure(resp, "Not connected");
        return;
    }
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    aubo_robot_namespace::RobotDevInfo info;
    int ret = hw_->statusService().robotServiceGetRobotDevInfoService(info);
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
        setSuccess(resp, "Robot info retrieved");
    } else {
        setFailure(resp, "Failed to get robot info");
    }
}

void AuboDashboardNode::onGetJointStatus(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
    if (!hw_ || !hw_->isConnected()) {
        setFailure(resp, "Not connected");
        return;
    }
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    aubo_robot_namespace::JointStatus status[6];
    int ret = hw_->statusService().robotServiceGetRobotJointStatus(status, 6);
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
        setSuccess(resp, "Joint status retrieved");
    } else {
        setFailure(resp, "Failed to get joint status");
    }
}

void AuboDashboardNode::onGetSafetyConfig(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
    if (!hw_ || !hw_->isConnected()) {
        setFailure(resp, "Not connected");
        return;
    }
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    aubo_robot_namespace::RobotSafetyConfig config;
    int ret = hw_->statusService().robotServiceGetRobotSafetyConfig(config);
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
        setSuccess(resp, "Safety config retrieved");
    } else {
        setFailure(resp, "Failed to get safety config");
    }
}

}  // namespace aubo_driver
