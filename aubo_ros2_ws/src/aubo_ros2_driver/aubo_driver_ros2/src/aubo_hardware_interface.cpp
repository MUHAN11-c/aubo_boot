/*
 * Software License Agreement (BSD License)
 * Copyright (c) 2017-2018, AUBO Robotics
 * Ported to ROS2: Hardware Interface implementation.
 */

#include "aubo_driver_ros2/aubo_hardware_interface.h"
#include <cstring>
#include <thread>
#include <chrono>

namespace aubo_driver {

AuboHardwareInterface::AuboHardwareInterface()
{
    std::memset(last_joints_, 0, sizeof(last_joints_));
}

AuboHardwareInterface::~AuboHardwareInterface()
{
    if (connected_) {
        shutdown();
    }
}

// ============================================================================
// 生命周期接口
// ============================================================================

bool AuboHardwareInterface::init(
    const std::string& server_host, int server_port, int max_retries)
{
    // ---------- 连接 1: conn_control_ (后续进入 TCP2CAN 模式) ----------
    int ret1 = aubo_robot_namespace::InterfaceCallSuccCode;
    int count = 0;
    do {
        count++;
        ret1 = conn_control_.robotServiceLogin(
            server_host.c_str(), server_port, "aubo", "123456");
    } while (ret1 != aubo_robot_namespace::InterfaceCallSuccCode
             && count < max_retries);

    if (ret1 != aubo_robot_namespace::InterfaceCallSuccCode) {
        RCLCPP_ERROR(rclcpp::get_logger("aubo_hw"),
            "conn_control_ login failed after %d retries", max_retries);
        return false;
    }

    // ---------- 连接 2: conn_status_ (保持普通模式，用于状态查询+IO+回调) ----------
    count = 0;
    do {
        count++;
        ret1 = conn_status_.robotServiceLogin(
            server_host.c_str(), server_port, "aubo", "123456");
    } while (ret1 != aubo_robot_namespace::InterfaceCallSuccCode
             && count < max_retries);

    if (ret1 != aubo_robot_namespace::InterfaceCallSuccCode) {
        RCLCPP_ERROR(rclcpp::get_logger("aubo_hw"),
            "conn_status_ login failed after %d retries", max_retries);
        conn_control_.robotServiceLogout();
        return false;
    }

    // 检测是否为真实机器人
    bool real_exist = false;
    int ret2 = conn_status_.robotServiceGetIsRealRobotExist(real_exist);
    if (ret2 == aubo_robot_namespace::InterfaceCallSuccCode) {
        RCLCPP_INFO(rclcpp::get_logger("aubo_hw"),
            real_exist ? "Real robot detected" : "Simulation mode (no real robot)");
    }

    connected_ = true;
    RCLCPP_INFO(rclcpp::get_logger("aubo_hw"),
        "Hardware interface initialized (host=%s, port=%d)", server_host.c_str(), server_port);
    return true;
}

bool AuboHardwareInterface::enterTcp2CanbusMode()
{
    if (!connected_) return false;

    int ret = conn_control_.robotServiceEnterTcp2CanbusMode();
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
        tcp2can_mode_ = true;
        RCLCPP_INFO(rclcpp::get_logger("aubo_hw"), "Entered TCP2CAN mode");
        return true;
    }

    // 重试: 先退出再进入
    if (ret == aubo_robot_namespace::ErrCode_ResponseReturnError) {
        conn_control_.robotServiceLeaveTcp2CanbusMode();
        ret = conn_control_.robotServiceEnterTcp2CanbusMode();
        if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
            tcp2can_mode_ = true;
            RCLCPP_INFO(rclcpp::get_logger("aubo_hw"), "Entered TCP2CAN mode (retry ok)");
            return true;
        }
    }

    RCLCPP_WARN(rclcpp::get_logger("aubo_hw"),
        "Failed to enter TCP2CAN mode (ret=%d)", ret);
    return false;
}

bool AuboHardwareInterface::leaveTcp2CanbusMode()
{
    if (!tcp2can_mode_) return true;

    int ret = conn_control_.robotServiceLeaveTcp2CanbusMode();
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
        tcp2can_mode_ = false;
        RCLCPP_INFO(rclcpp::get_logger("aubo_hw"), "Left TCP2CAN mode");
        return true;
    }
    return false;
}

void AuboHardwareInterface::shutdown()
{
    connected_ = false;
    if (tcp2can_mode_) {
        leaveTcp2CanbusMode();
    }
    conn_control_.robotServiceLogout();
    conn_status_.robotServiceLogout();
    RCLCPP_INFO(rclcpp::get_logger("aubo_hw"), "Hardware interface shutdown");
}

// ============================================================================
// 实时路径接口
// ============================================================================

bool AuboHardwareInterface::readJointState(double joints[6], double velocity[6])
{
    if (!connected_) return false;

    aubo_robot_namespace::wayPoint_S wp;
    int ret = conn_status_.robotServiceGetCurrentWaypointInfo(wp);

    if (ret != aubo_robot_namespace::InterfaceCallSuccCode) {
        if (ret == aubo_robot_namespace::ErrCode_SocketDisconnect) {
            connected_ = false;
            RCLCPP_ERROR(rclcpp::get_logger("aubo_hw"), "Socket disconnect in readJointState");
        }
        return false;
    }

    for (int i = 0; i < 6; i++) {
        joints[i] = wp.jointpos[i];
    }

    // 速度估算 (基于前后两次位置差分 / 时间差)
    if (velocity) {
        auto now = std::chrono::steady_clock::now();
        if (has_last_joints_) {
            double dt = std::chrono::duration<double>(now - last_read_time_).count();
            if (dt > 0.001) {  // 至少 1ms 间隔避免除零
                for (int i = 0; i < 6; i++) {
                    velocity[i] = (joints[i] - last_joints_[i]) / dt;
                }
            } else {
                std::memset(velocity, 0, 6 * sizeof(double));
            }
        } else {
            std::memset(velocity, 0, 6 * sizeof(double));
        }
        last_read_time_ = now;
    }

    for (int i = 0; i < 6; i++) {
        last_joints_[i] = joints[i];
    }
    has_last_joints_ = true;

    return true;
}

bool AuboHardwareInterface::readDiagnosis(int& rib_buffer_size)
{
    if (!connected_) return false;

    // 使用 conn_control_ (TCP2CAN 连接) 查询 RIB
    // 注: 在 TCP2CAN 模式下，同一连接上查诊断可避免缓存过时问题
    // 参考 PORTING_MOTION_FIX.md Fix12: 使用局部变量避免数据竞争
    aubo_robot_namespace::RobotDiagnosis diag;
    int ret = conn_control_.robotServiceGetRobotDiagnosisInfo(diag);

    if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
        rib_buffer_size = diag.macTargetPosDataSize;
        return true;
    }
    return false;
}

bool AuboHardwareInterface::readDiagnosisOnStatus(int& rib_buffer_size)
{
    if (!connected_) return false;
    aubo_robot_namespace::RobotDiagnosis diag;
    int ret = conn_status_.robotServiceGetRobotDiagnosisInfo(diag);
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
        rib_buffer_size = diag.macTargetPosDataSize;
        return true;
    }
    return false;
}

bool AuboHardwareInterface::writeTrajectoryPoints(
    const std::vector<aubo_robot_namespace::wayPoint_S>& waypoints)
{
    if (!connected_ || waypoints.empty()) return false;

    // 直接调用 SDK 批量发送接口
    // 流量控制和批量策略由上层 publishWaypointToRobot 循环负责
    int ret = conn_control_.robotServiceSetRobotPosData2Canbus(waypoints);
    return (ret == aubo_robot_namespace::InterfaceCallSuccCode);
}

bool AuboHardwareInterface::writeTrajectoryPoint(const double joint_angles[6])
{
    if (!connected_) return false;

    double joints[6];
    for (int i = 0; i < 6; i++) joints[i] = joint_angles[i];

    int ret = conn_control_.robotServiceSetRobotPosData2Canbus(joints);
    return (ret == aubo_robot_namespace::InterfaceCallSuccCode);
}

// ============================================================================
// 异步路径接口
// ============================================================================

bool AuboHardwareInterface::readSafetyIOStatus(
    bool& emergency_stopped, bool& protective_stopped)
{
    if (!connected_) {
        emergency_stopped = true;
        protective_stopped = true;
        return false;
    }

    std::vector<aubo_robot_namespace::RobotIoDesc> di_vec;
    std::vector<aubo_robot_namespace::RobotIoType> io_type;
    io_type.push_back(aubo_robot_namespace::RobotBoardControllerDI);

    int ret = conn_status_.robotServiceGetBoardIOStatus(io_type, di_vec);
    if (ret != aubo_robot_namespace::InterfaceCallSuccCode) return false;

    double digitalIn[30] = {0};
    for (size_t i = 0; i < di_vec.size(); i++) {
        if (di_vec[i].ioAddr < 30) {
            digitalIn[di_vec[i].ioAddr] = di_vec[i].ioValue;
        }
    }

    // 真实机器人模式下:
    //   digitalIn[0] / digitalIn[8] = 0 → 紧急停止
    //   digitalIn[1] / digitalIn[9] = 0 → 防护停止
    emergency_stopped  = (digitalIn[0] == 0 || digitalIn[8] == 0);
    protective_stopped = (digitalIn[1] == 0 || digitalIn[9] == 0);

    return true;
}

bool AuboHardwareInterface::readFullIOStatus(
    ivg_interfaces::msg::RobotIOStatus& io_msg)
{
    if (!connected_) return false;

    // 辅助函数: 从 IO 名称解析 pin 号
    auto parse_io_pin = [](const std::string& name, int fallback) -> int {
        std::string digits;
        for (char ch : name) {
            if (std::isdigit(static_cast<unsigned char>(ch))) {
                digits.push_back(ch);
            }
        }
        return digits.empty() ? fallback : std::atoi(digits.c_str());
    };

    std::vector<aubo_robot_namespace::RobotIoDesc> sv_in, sv_out;
    std::vector<aubo_robot_namespace::RobotIoType> io_type_in, io_type_out;

    // --- User DI/DO ---
    io_type_in.push_back(aubo_robot_namespace::RobotBoardUserDI);
    io_type_out.push_back(aubo_robot_namespace::RobotBoardUserDO);
    conn_status_.robotServiceGetBoardIOStatus(io_type_in, sv_in);
    conn_status_.robotServiceGetBoardIOStatus(io_type_out, sv_out);

    for (size_t i = 6; i < sv_in.size(); i++) {
        int pin = parse_io_pin(sv_in[i].ioName, static_cast<int>(i - 6));
        if (pin >= 0) {
            if (io_msg.digital_inputs.size() <= static_cast<size_t>(pin))
                io_msg.digital_inputs.resize(pin + 1, false);
            io_msg.digital_inputs[pin] = (sv_in[i].ioValue != 0);
        }
    }
    for (size_t i = 0; i < sv_out.size(); i++) {
        int pin = parse_io_pin(sv_out[i].ioName, static_cast<int>(i));
        if (pin >= 0) {
            if (io_msg.digital_outputs.size() <= static_cast<size_t>(pin))
                io_msg.digital_outputs.resize(pin + 1, false);
            io_msg.digital_outputs[pin] = (sv_out[i].ioValue != 0);
        }
    }
    sv_in.clear(); sv_out.clear();
    io_type_in.clear(); io_type_out.clear();

    // --- User AI/AO ---
    io_type_in.push_back(aubo_robot_namespace::RobotBoardUserAI);
    io_type_out.push_back(aubo_robot_namespace::RobotBoardUserAO);
    conn_status_.robotServiceGetBoardIOStatus(io_type_in, sv_in);
    conn_status_.robotServiceGetBoardIOStatus(io_type_out, sv_out);
    for (size_t i = 0; i < sv_in.size(); i++) {
        int pin = static_cast<int>(sv_in[i].ioAddr);
        if (pin >= 0) {
            if (io_msg.analog_inputs.size() <= static_cast<size_t>(pin))
                io_msg.analog_inputs.resize(pin + 1, 0.0f);
            io_msg.analog_inputs[pin] = static_cast<float>(sv_in[i].ioValue);
        }
    }
    for (size_t i = 0; i < sv_out.size(); i++) {
        int pin = static_cast<int>(sv_out[i].ioAddr);
        if (pin >= 0) {
            if (io_msg.analog_outputs.size() <= static_cast<size_t>(pin))
                io_msg.analog_outputs.resize(pin + 1, 0.0f);
            io_msg.analog_outputs[pin] = static_cast<float>(sv_out[i].ioValue);
        }
    }
    sv_in.clear(); sv_out.clear();

    // --- Tool IO ---
    conn_status_.robotServiceGetAllToolDigitalIOStatus(sv_in);
    conn_status_.robotServiceGetAllToolAIStatus(sv_out);
    for (size_t i = 0; i < sv_in.size(); i++) {
        int pin = static_cast<int>(sv_in[i].ioAddr);
        if (pin >= 0) {
            if (io_msg.tool_io_status.digital_outputs.size() <= static_cast<size_t>(pin))
                io_msg.tool_io_status.digital_outputs.resize(pin + 1, false);
            io_msg.tool_io_status.digital_outputs[pin] = (sv_in[i].ioValue != 0);
        }
    }
    for (size_t i = 0; i < sv_out.size(); i++) {
        int pin = static_cast<int>(sv_out[i].ioAddr);
        if (pin >= 0) {
            if (io_msg.tool_io_status.analog_inputs.size() <= static_cast<size_t>(pin))
                io_msg.tool_io_status.analog_inputs.resize(pin + 1, 0.0f);
            io_msg.tool_io_status.analog_inputs[pin] = static_cast<float>(sv_out[i].ioValue);
        }
    }

    return true;
}

bool AuboHardwareInterface::writeIOCommand(
    aubo_robot_namespace::RobotIoType type, int addr, double value)
{
    if (!connected_) return false;
    int ret = conn_status_.robotServiceSetBoardIOStatus(type, addr, value);
    return (ret == aubo_robot_namespace::InterfaceCallSuccCode);
}

bool AuboHardwareInterface::writeToolIOCommand(
    int io_index, bool is_output, double value)
{
    if (!connected_) return false;

    auto addr = static_cast<aubo_robot_namespace::ToolDigitalIOAddr>(io_index);

    if (!is_output) {
        // 设为输入模式
        int ret = conn_status_.robotServiceSetToolDigitalIOType(
            addr, aubo_robot_namespace::IO_IN);
        return (ret == aubo_robot_namespace::InterfaceCallSuccCode);
    }

    // 设为输出模式并写入值
    int ret = conn_status_.robotServiceSetToolDigitalIOType(
        addr, aubo_robot_namespace::IO_OUT);
    if (ret != aubo_robot_namespace::InterfaceCallSuccCode) return false;

    auto status = (value > 0.5) ? aubo_robot_namespace::IO_STATUS_VALID
                                : aubo_robot_namespace::IO_STATUS_INVALID;
    ret = conn_status_.robotServiceSetToolDOStatus(addr, status);
    return (ret == aubo_robot_namespace::InterfaceCallSuccCode);
}

bool AuboHardwareInterface::writeToolPowerType(int power_type)
{
    if (!connected_) return false;
    auto type = static_cast<aubo_robot_namespace::ToolPowerType>(power_type);
    int ret = conn_status_.robotServiceSetToolPowerVoltageType(type);
    return (ret == aubo_robot_namespace::InterfaceCallSuccCode);
}

// ============================================================================
// SDK 回调接口
// ============================================================================

void AuboHardwareInterface::registerCallbacks(
    EventCallback event_cb,
    JointStateCallback joint_cb,
    WaypointCallback waypoint_cb,
    SpeedCallback speed_cb)
{
    event_cb_ = std::move(event_cb);
    joint_cb_ = std::move(joint_cb);
    waypoint_cb_ = std::move(waypoint_cb);
    speed_cb_ = std::move(speed_cb);

    // -------- RobotEventCallback (默认启用，不可关闭) --------
    if (event_cb_) {
        conn_status_.robotServiceRegisterRobotEventInfoCallback(
            &AuboHardwareInterface::onRobotEvent, this);
        RCLCPP_INFO(rclcpp::get_logger("aubo_hw"),
            "Registered RobotEventCallback");
    }

    // -------- RealTimeJointStatusCallback --------
    if (joint_cb_) {
        conn_status_.robotServiceSetRealTimeJointStatusPush(true);
        conn_status_.robotServiceRegisterRealTimeJointStatusCallback(
            &AuboHardwareInterface::onJointStatus, this);
        RCLCPP_INFO(rclcpp::get_logger("aubo_hw"),
            "Registered RealTimeJointStatusCallback");
    }

    // -------- RealTimeRoadPointCallback --------
    if (waypoint_cb_) {
        conn_status_.robotServiceSetRealTimeRoadPointPush(true);
        conn_status_.robotServiceRegisterRealTimeRoadPointCallback(
            &AuboHardwareInterface::onRoadPoint, this);
        RCLCPP_INFO(rclcpp::get_logger("aubo_hw"),
            "Registered RealTimeRoadPointCallback");
    }

    // -------- RealTimeEndSpeedCallback --------
    if (speed_cb_) {
        conn_status_.robotServiceSetRealTimeEndSpeedPush(true);
        conn_status_.robotServiceRegisterRealTimeEndSpeedCallback(
            &AuboHardwareInterface::onEndSpeed, this);
        RCLCPP_INFO(rclcpp::get_logger("aubo_hw"),
            "Registered RealTimeEndSpeedCallback");
    }
}

// ============================================================================
// 内部 SDK 回调处理 (静态函数，在 SDK 内部线程执行)
// 约束: 不能调用 SDK API，只能做原子写入 + 日志
// ============================================================================

void AuboHardwareInterface::onRobotEvent(
    const aubo_robot_namespace::RobotEventInfo* info, void* arg)
{
    auto* self = static_cast<AuboHardwareInterface*>(arg);
    if (!info || !self->event_cb_) return;

    // 直接转发给用户回调
    // 用户回调中应只做原子写入 + 日志，避免阻塞 SDK 内部线程
    self->event_cb_(static_cast<int>(info->eventType),
                    info->eventCode, info->eventContent);

    // 内置处理: socket 断开 → 标记未连接
    if (info->eventType == aubo_robot_namespace::RobotEvent_socketDisconnected) {
        self->connected_ = false;
    }
}

void AuboHardwareInterface::onJointStatus(
    const aubo_robot_namespace::JointStatus* status, int size, void* arg)
{
    auto* self = static_cast<AuboHardwareInterface*>(arg);
    if (!status || !self->joint_cb_) return;

    JointFull d;
    for (int i = 0; i < std::min(size, 6); i++) {
        d.pos[i]  = static_cast<double>(status[i].jointPosJ);
        d.vel[i]  = static_cast<double>(status[i].jointSpeedMoto);
        d.cur[i]  = static_cast<double>(status[i].jointCurrentI);
        d.vol[i]  = static_cast<double>(status[i].jointCurVol);
        d.temp[i] = static_cast<double>(status[i].jointCurTemp);
        d.tgt_cur[i] = static_cast<double>(status[i].jointTagCurrentI);
        d.tgt_vel[i] = static_cast<double>(status[i].jointTagSpeedMoto);
        d.tgt_pos[i] = static_cast<double>(status[i].jointTagPosJ);
        d.err[i]  = static_cast<int>(status[i].jointErrorNum);
    }
    self->joint_cb_(d);
}

void AuboHardwareInterface::onRoadPoint(
    const aubo_robot_namespace::wayPoint_S* wp, void* arg)
{
    auto* self = static_cast<AuboHardwareInterface*>(arg);
    if (!wp || !self->waypoint_cb_) return;
    self->waypoint_cb_(*wp);
}

void AuboHardwareInterface::onEndSpeed(double speed, void* arg)
{
    auto* self = static_cast<AuboHardwareInterface*>(arg);
    if (!self->speed_cb_) return;
    self->speed_cb_(speed);
}

}  // namespace aubo_driver
