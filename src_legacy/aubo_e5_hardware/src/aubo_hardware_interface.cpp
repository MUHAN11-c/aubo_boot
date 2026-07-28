/*
 * Jazzy AUBO Hardware Interface implementation.
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

// ══════════════════════════════════════════════════════════════════════
// Lifecycle
// ══════════════════════════════════════════════════════════════════════

bool AuboHardwareInterface::init(
    const std::string& server_host, int server_port, bool need_control,
    int max_retries)
{
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;
    int count = 0;

    // conn_control_ — TCP2CAN trajectory streaming（status-only 模式跳过 login）
    if (need_control) {
        {
            std::lock_guard<std::mutex> lock(control_mutex_);
            do {
                count++;
                ret = conn_control_.robotServiceLogin(
                    server_host.c_str(), server_port, "aubo", "123456");
                if (ret != aubo_robot_namespace::InterfaceCallSuccCode && count < max_retries)
                    std::this_thread::sleep_for(std::chrono::seconds(1));
            } while (ret != aubo_robot_namespace::InterfaceCallSuccCode && count < max_retries);
        }

        if (ret != aubo_robot_namespace::InterfaceCallSuccCode) {
            RCLCPP_ERROR(rclcpp::get_logger("aubo_hw"),
                "conn_control_ login failed after %d retries (ret=%d)", max_retries, ret);
            return false;
        }
        control_logged_in_ = true;
    }

    // conn_status_ — status query + IO + callbacks
    count = 0;
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        do {
            count++;
            ret = conn_status_.robotServiceLogin(
                server_host.c_str(), server_port, "aubo", "123456");
            if (ret != aubo_robot_namespace::InterfaceCallSuccCode && count < max_retries)
                std::this_thread::sleep_for(std::chrono::seconds(1));
        } while (ret != aubo_robot_namespace::InterfaceCallSuccCode && count < max_retries);
    }

    if (ret != aubo_robot_namespace::InterfaceCallSuccCode) {
        RCLCPP_ERROR(rclcpp::get_logger("aubo_hw"),
            "conn_status_ login failed after %d retries", max_retries);
        if (control_logged_in_) {
            std::lock_guard<std::mutex> lock(control_mutex_);
            conn_control_.robotServiceLogout();
            control_logged_in_ = false;
        }
        return false;
    }

    // 经公有方法查询（内部自取 status_mutex_），init 此处未持锁
    bool real_exist = false;
    getIsRealRobotExist(real_exist);
    RCLCPP_INFO(rclcpp::get_logger("aubo_hw"),
        real_exist ? "Real robot detected" : "Simulation mode (no real robot)");

    connected_ = true;
    RCLCPP_INFO(rclcpp::get_logger("aubo_hw"),
        "Hardware interface initialized (host=%s, port=%d)", server_host.c_str(), server_port);
    return true;
}

bool AuboHardwareInterface::enterTcp2CanbusMode()
{
    if (!connected_) return false;

    int ret;
    bool retried = false;
    {
        // 整个 leave+enter 重试序列一把锁（序列内无 sleep/rclcpp 调用）
        std::lock_guard<std::mutex> lock(control_mutex_);
        ret = conn_control_.robotServiceEnterTcp2CanbusMode();
        if (ret == aubo_robot_namespace::ErrCode_ResponseReturnError) {
            retried = true;
            conn_control_.robotServiceLeaveTcp2CanbusMode();
            ret = conn_control_.robotServiceEnterTcp2CanbusMode();
        }
    }

    if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
        tcp2can_mode_ = true;
        RCLCPP_INFO(rclcpp::get_logger("aubo_hw"),
            retried ? "Entered TCP2CAN mode (retry ok)" : "Entered TCP2CAN mode");
        return true;
    }

    RCLCPP_WARN(rclcpp::get_logger("aubo_hw"),
        "Failed to enter TCP2CAN mode (ret=%d)", ret);
    return false;
}

bool AuboHardwareInterface::leaveTcp2CanbusMode()
{
    if (!tcp2can_mode_) return true;

    int ret;
    {
        std::lock_guard<std::mutex> lock(control_mutex_);
        ret = conn_control_.robotServiceLeaveTcp2CanbusMode();
    }

    if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
        tcp2can_mode_ = false;
        RCLCPP_INFO(rclcpp::get_logger("aubo_hw"), "Left TCP2CAN mode");
        return true;
    }
    return false;
}

bool AuboHardwareInterface::stopMotion()
{
    if (!connected_ || !tcp2can_mode_) return false;

    int ret;
    {
        std::lock_guard<std::mutex> lock(control_mutex_);
        ret = conn_control_.rootServiceRobotMoveControl(
            aubo_robot_namespace::RobotMoveStop);
    }

    if (ret == aubo_robot_namespace::ErrCode_RequestTimeout) {
        RCLCPP_WARN(rclcpp::get_logger("aubo_hw"),
            "RobotMoveStop timeout, retrying after 20ms");
        std::this_thread::sleep_for(std::chrono::milliseconds(20));  // 不持锁 sleep
        std::lock_guard<std::mutex> lock(control_mutex_);
        ret = conn_control_.rootServiceRobotMoveControl(
            aubo_robot_namespace::RobotMoveStop);
    }

    if (ret != aubo_robot_namespace::InterfaceCallSuccCode) {
        RCLCPP_WARN(rclcpp::get_logger("aubo_hw"),
            "RobotMoveStop failed (ret=%d), trying robotMoveFastStop", ret);
        std::lock_guard<std::mutex> lock(control_mutex_);
        ret = conn_control_.robotMoveFastStop();
    }

    if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
        RCLCPP_INFO(rclcpp::get_logger("aubo_hw"), "Motion stopped");
        return true;
    }

    RCLCPP_ERROR(rclcpp::get_logger("aubo_hw"), "stopMotion failed (ret=%d)", ret);
    return false;
}

void AuboHardwareInterface::shutdown()
{
    connected_ = false;
    if (tcp2can_mode_) {
        // 公有方法自取 control_mutex_；本路径未持锁，无重入
        leaveTcp2CanbusMode();
    }
    if (control_logged_in_) {
        std::lock_guard<std::mutex> lock(control_mutex_);
        conn_control_.robotServiceLogout();
        control_logged_in_ = false;
    }
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        if (push_joint_ok_) {
            conn_status_.robotServiceSetRealTimeJointStatusPush(false);
            conn_status_.robotServiceRegisterRealTimeJointStatusCallback(
                nullptr, nullptr);
            push_joint_ok_ = false;
        }
        if (push_waypoint_ok_) {
            conn_status_.robotServiceSetRealTimeRoadPointPush(false);
            conn_status_.robotServiceRegisterRealTimeRoadPointCallback(
                nullptr, nullptr);
            push_waypoint_ok_ = false;
        }
        if (push_speed_ok_) {
            conn_status_.robotServiceSetRealTimeEndSpeedPush(false);
            conn_status_.robotServiceRegisterRealTimeEndSpeedCallback(
                nullptr, nullptr);
            push_speed_ok_ = false;
        }
        conn_status_.robotServiceLogout();
    }
    RCLCPP_INFO(rclcpp::get_logger("aubo_hw"), "Hardware interface shutdown");
}

// ══════════════════════════════════════════════════════════════════════
// RT path
// ══════════════════════════════════════════════════════════════════════

bool AuboHardwareInterface::readJointState(double joints[6], double velocity[6])
{
    if (!connected_) return false;

    aubo_robot_namespace::JointStatus status[6]{};
    int ret;
    {
        // 整个「读取 + 速度微分」为一个临界区：
        // 同时保护 conn_status_ 与 last_joints_ 微分状态
        std::lock_guard<std::mutex> lock(status_mutex_);
        ret = conn_status_.robotServiceGetRobotJointStatus(status, 6);

        if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
            for (int i = 0; i < 6; i++) {
                joints[i] = status[i].jointPosJ;
            }

            if (velocity) {
                auto now = std::chrono::steady_clock::now();
                if (has_last_joints_) {
                    double dt = std::chrono::duration<double>(now - last_read_time_).count();
                    if (dt > 0.001) {
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
        } else if (ret == aubo_robot_namespace::ErrCode_SocketDisconnect) {
            connected_ = false;
        }
    }

    if (ret != aubo_robot_namespace::InterfaceCallSuccCode) {
        if (ret == aubo_robot_namespace::ErrCode_SocketDisconnect) {
            RCLCPP_ERROR(rclcpp::get_logger("aubo_hw"),
                "Socket disconnect in readJointState");
        }
        return false;
    }

    return true;
}

bool AuboHardwareInterface::readDiagnosis(int& rib_buffer_size)
{
    if (!connected_) return false;

    aubo_robot_namespace::RobotDiagnosis diag;
    int ret;
    {
        std::lock_guard<std::mutex> lock(control_mutex_);
        ret = conn_control_.robotServiceGetRobotDiagnosisInfo(diag);
    }

    if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
        rib_buffer_size = diag.macTargetPosDataSize;
        // H12: 写入原子缓存仅供展示/诊断；
        // sendLoop 门控必须用当轮实时查询值，禁止回喂该缓存（Fix14 教训）
        rib_cache_.store(diag.macTargetPosDataSize);
        static bool capacity_logged = false;
        if (!capacity_logged && diag.macTargetPosBufferSize > 0) {
            capacity_logged = true;
            RCLCPP_INFO(rclcpp::get_logger("aubo_hw"),
                "RIB total capacity: %d slots", diag.macTargetPosBufferSize);
        }
        return true;
    }
    return false;
}

bool AuboHardwareInterface::getIsRealRobotExist(bool& real_exist)
{
    // init() 在置 connected_ 之前调用本方法，故此处不检查 connected_
    std::lock_guard<std::mutex> lock(status_mutex_);
    int ret = conn_status_.robotServiceGetIsRealRobotExist(real_exist);
    return (ret == aubo_robot_namespace::InterfaceCallSuccCode);
}

bool AuboHardwareInterface::writeTrajectoryPoints(
    const std::vector<aubo_robot_namespace::wayPoint_S>& waypoints)
{
    if (!connected_ || waypoints.empty()) return false;
    std::lock_guard<std::mutex> lock(control_mutex_);
    int ret = conn_control_.robotServiceSetRobotPosData2Canbus(waypoints);
    return (ret == aubo_robot_namespace::InterfaceCallSuccCode);
}

bool AuboHardwareInterface::writeTrajectoryPoint(const double joint_angles[6])
{
    if (!connected_) return false;
    double joints[6];
    for (int i = 0; i < 6; i++) joints[i] = joint_angles[i];
    std::lock_guard<std::mutex> lock(control_mutex_);
    int ret = conn_control_.robotServiceSetRobotPosData2Canbus(joints);
    return (ret == aubo_robot_namespace::InterfaceCallSuccCode);
}

// ══════════════════════════════════════════════════════════════════════
// Async IO path
// ══════════════════════════════════════════════════════════════════════

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

    int ret;
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        ret = conn_status_.robotServiceGetBoardIOStatus(io_type, di_vec);
    }
    if (ret != aubo_robot_namespace::InterfaceCallSuccCode) return false;

    double digitalIn[30] = {0};
    for (size_t i = 0; i < di_vec.size(); i++) {
        if (di_vec[i].ioAddr < 30) {
            digitalIn[di_vec[i].ioAddr] = di_vec[i].ioValue;
        }
    }

    emergency_stopped  = (digitalIn[0] == 0 || digitalIn[8] == 0);
    protective_stopped = (digitalIn[1] == 0 || digitalIn[9] == 0);

    return true;
}


bool AuboHardwareInterface::writeIOCommand(
    aubo_robot_namespace::RobotIoType type, int addr, double value)
{
    if (!connected_) return false;
    std::lock_guard<std::mutex> lock(status_mutex_);
    int ret = conn_status_.robotServiceSetBoardIOStatus(type, addr, value);
    return (ret == aubo_robot_namespace::InterfaceCallSuccCode);
}

bool AuboHardwareInterface::writeToolIOCommand(
    int io_index, bool is_output, double value)
{
    if (!connected_) return false;

    // 多次 conn_status_ 调用为一个序列，入口一把锁（无 sleep/rclcpp 调用）
    std::lock_guard<std::mutex> lock(status_mutex_);

    auto addr = static_cast<aubo_robot_namespace::ToolDigitalIOAddr>(io_index);

    if (!is_output) {
        int ret = conn_status_.robotServiceSetToolDigitalIOType(
            addr, aubo_robot_namespace::IO_IN);
        return (ret == aubo_robot_namespace::InterfaceCallSuccCode);
    }

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
    std::lock_guard<std::mutex> lock(status_mutex_);
    int ret = conn_status_.robotServiceSetToolPowerVoltageType(type);
    return (ret == aubo_robot_namespace::InterfaceCallSuccCode);
}

// ══════════════════════════════════════════════════════════════════════
// SDK callbacks
// ══════════════════════════════════════════════════════════════════════

void AuboHardwareInterface::registerCallbacks(
    EventCallback event_cb,
    JointStateCallback joint_cb,
    WaypointCallback waypoint_cb,
    SpeedCallback speed_cb)
{
    {
        std::lock_guard<std::mutex> lock(status_mutex_);

        event_cb_ = std::move(event_cb);
        joint_cb_ = std::move(joint_cb);
        waypoint_cb_ = std::move(waypoint_cb);
        speed_cb_ = std::move(speed_cb);

        if (event_cb_) {
            conn_status_.robotServiceRegisterRobotEventInfoCallback(
                &AuboHardwareInterface::onRobotEvent, this);
        }

        // 推送（注册在 conn_status_ 普通模式连接上，与 conn_control_ 的
        // TCP2CAN 分属两条 TCP）：2026-07-24 网卡驱动修复后复测——同连接
        // 推送+TCP2CAN 并发 300s 零失败、推送 ~28Hz 不断链（修复前同连接
        // 22-28s 必断，系网卡问题而非固件，docs/nic_driver_incident.md）；
        // 跨连接只会更安全。三推送全注册（旧栈经验：JointStatus+RoadPoint+
        // EndSpeed 组合保活）。推送关闭在 shutdown() 中按 *_ok_ 标志配对。
        if (joint_cb_) {
            conn_status_.robotServiceSetRealTimeJointStatusPush(true);
            conn_status_.robotServiceRegisterRealTimeJointStatusCallback(
                &AuboHardwareInterface::onJointStatus, this);
            push_joint_ok_ = true;
        }
        if (waypoint_cb_) {
            conn_status_.robotServiceSetRealTimeRoadPointPush(true);
            conn_status_.robotServiceRegisterRealTimeRoadPointCallback(
                &AuboHardwareInterface::onRoadPoint, this);
            push_waypoint_ok_ = true;
        }
        if (speed_cb_) {
            conn_status_.robotServiceSetRealTimeEndSpeedPush(true);
            conn_status_.robotServiceRegisterRealTimeEndSpeedCallback(
                &AuboHardwareInterface::onEndSpeed, this);
            push_speed_ok_ = true;
        }

    }

    // 日志放锁外（禁止持锁跨 rclcpp 调用）
    if (event_cb_) {
        RCLCPP_INFO(rclcpp::get_logger("aubo_hw"), "Registered RobotEventCallback");
    }
    RCLCPP_INFO(rclcpp::get_logger("aubo_hw"),
        "Telemetry pushes enabled on conn_status_ (joint=%d roadpoint=%d speed=%d); 20Hz polling kept as redundant path",
        (int)push_joint_ok_, (int)push_waypoint_ok_, (int)push_speed_ok_);
}

void AuboHardwareInterface::onRobotEvent(
    const aubo_robot_namespace::RobotEventInfo* info, void* arg)
{
    auto* self = static_cast<AuboHardwareInterface*>(arg);
    if (!info || !self->event_cb_) return;

    self->event_cb_(static_cast<int>(info->eventType),
                    info->eventCode, info->eventContent);

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
