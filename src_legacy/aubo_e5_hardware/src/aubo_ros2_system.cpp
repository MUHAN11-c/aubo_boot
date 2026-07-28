/*
 * AUBO E5 ros2_control SystemInterface 插件实现（Phase 2）。
 * 设计/参数依据: doc/REFACTORING.md §3/§3.8/§四 Phase 2/§8 实测数据。
 * sendLoop 流控逐字移植自 src/joint_trajectory_controller.cpp:395-554，
 * 唯一改动: 目标水位带 [60,120] + 冻结检测兜底 (§3.6 修订清单)。
 */

#include "aubo_driver_ros2/aubo_ros2_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <thread>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace aubo_driver {

namespace {
const rclcpp::Logger kLogger = rclcpp::get_logger("aubo_ros2_system");
}  // namespace

AuboRos2System::~AuboRos2System()
{
    // 兜底: 生命周期正常路径已处理，这里防止异常退出遗留线程/连接
    stopThreads();
    if (hw_) {
        hw_->shutdown();
    }
}

// ══════════════════════════════════════════════════════════════════════
// Lifecycle
// ══════════════════════════════════════════════════════════════════════

AuboRos2System::CallbackReturn AuboRos2System::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params)
{
    const auto& info = params.hardware_info;

    // URDF <ros2_control><hardware> 内 <param>（无 generate_parameter_library，
    // 首版直接读 hardware_parameters，见 REFACTORING §3.8 裁剪说明）
    auto it_ip = info.hardware_parameters.find("robot_ip");
    if (it_ip != info.hardware_parameters.end()) robot_ip_ = it_ip->second;
    auto it_server_host = info.hardware_parameters.find("server_host");
    if (it_server_host != info.hardware_parameters.end()) robot_ip_ = it_server_host->second;
    auto it_port = info.hardware_parameters.find("port");
    if (it_port != info.hardware_parameters.end()) robot_port_ = std::stoi(it_port->second);
    auto it_server_port = info.hardware_parameters.find("server_port");
    if (it_server_port != info.hardware_parameters.end()) robot_port_ = std::stoi(it_server_port->second);
    auto it_motion = info.hardware_parameters.find("allow_motion_commands");
    if (it_motion != info.hardware_parameters.end()) allow_motion_commands_ = (it_motion->second == "true");
    auto it_batch = info.hardware_parameters.find("tcp2can_batch_max");
    if (it_batch != info.hardware_parameters.end()) {
        tcp2can_batch_max_ = std::clamp(std::stoi(it_batch->second), 1, 32);
    }

    if (info.joints.size() != kNJoint) {
        RCLCPP_ERROR(kLogger, "Expected %d joints, got %zu", kNJoint, info.joints.size());
        return CallbackReturn::ERROR;
    }

    for (const auto& joint : info.joints) {
        bool has_pos_cmd = false, has_pos_state = false, has_vel_state = false;
        for (const auto& ci : joint.command_interfaces) {
            if (ci.name == hardware_interface::HW_IF_POSITION) has_pos_cmd = true;
        }
        for (const auto& si : joint.state_interfaces) {
            if (si.name == hardware_interface::HW_IF_POSITION) has_pos_state = true;
            if (si.name == hardware_interface::HW_IF_VELOCITY) has_vel_state = true;
        }
        if (!has_pos_cmd || !has_pos_state || !has_vel_state) {
            RCLCPP_ERROR(kLogger,
                "Joint '%s' must have position command + position/velocity state interfaces",
                joint.name.c_str());
            return CallbackReturn::ERROR;
        }
        joint_names_.push_back(joint.name);
    }

    hw_pos_cmd_.fill(0.0);
    hw_pos_state_.fill(0.0);
    hw_vel_state_.fill(0.0);
    joint_min_.fill(-3.05);
    joint_max_.fill(3.05);

    RCLCPP_INFO(kLogger, "on_init: robot_ip=%s port=%d joints=%zu tcp2can_batch_max=%d",
        robot_ip_.c_str(), robot_port_, joint_names_.size(), tcp2can_batch_max_);
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
AuboRos2System::on_export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> ifs;
    ifs.reserve(kNJoint * 2);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    for (int i = 0; i < kNJoint; i++) {
        ifs.push_back(std::make_shared<hardware_interface::StateInterface>(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_pos_state_[i]));
        ifs.push_back(std::make_shared<hardware_interface::StateInterface>(
            joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_vel_state_[i]));
    }
#pragma GCC diagnostic pop
    return ifs;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
AuboRos2System::on_export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface::SharedPtr> ifs;
    ifs.reserve(kNJoint);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    for (int i = 0; i < kNJoint; i++) {
        ifs.push_back(std::make_shared<hardware_interface::CommandInterface>(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_pos_cmd_[i]));
    }
#pragma GCC diagnostic pop
    return ifs;
}

AuboRos2System::CallbackReturn AuboRos2System::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    hw_ = std::make_unique<AuboHardwareInterface>();
    // need_control=true: login×2 (conn_control_ + conn_status_)
    if (!hw_->init(robot_ip_, robot_port_, allow_motion_commands_)) {
        RCLCPP_ERROR(kLogger, "on_configure: hw init failed (%s:%d)",
            robot_ip_.c_str(), robot_port_);
        hw_.reset();
        return CallbackReturn::FAILURE;
    }

    // 事件回调：断连检测 + 安全事件闩锁（H6: SDK 内部线程，只允许原子写 + 日志）
    hw_->registerCallbacks(
        [this](int event_type, int event_code, const std::string& content) {
            namespace arn = aubo_robot_namespace;
            if (event_type == arn::RobotEvent_socketDisconnected) {
                RCLCPP_ERROR(kLogger,
                    "Robot socket disconnected (code=%d): %s — read() will report ERROR",
                    event_code, content.c_str());
                // TODO(Phase C): 断连自动重连——重 login 有 ~6s 冷却期 (ret=10003)，
                // 必须退避重试 (1s 起步, ≥8s 窗口)，见 REFACTORING §8.1
                return;
            }
            switch (event_type) {
            case arn::RobotEvent_softEmergency:
            case arn::RobotEvent_remoteEmergencyStop:
            case arn::RobotEvent_collision:
            case arn::RobotEvent_jointError:
            case arn::RobotEvent_armCanbusError:
            case arn::RobotEvent_robotControllerError:
            case arn::RobotEvent_MacDataInterruptWarning:
                safety_fault_.store(true, std::memory_order_relaxed);
                fault_cause_.store(8, std::memory_order_relaxed);
                RCLCPP_ERROR(kLogger,
                    "SAFETY EVENT type=%d code=%d: %s — safety_fault_ latched",
                    event_type, event_code, content.c_str());
                break;
            default:
                break;
            }
        },
        // 关节状态推送（SDK 内部线程，只写原子——H6）：33Hz 位置+真实速度
        // （jointSpeedMoto，优于轮询差分），与 20Hz 轮询互为冗余——任一路径
        // 更新都会刷新新鲜度。2026-07-24 网卡驱动修复后并发长测（300s
        // 同连接推送+TCP2CAN 零断链）验证通过后在 hw 层启用注册。
        // 关节状态推送（SDK 内部线程，只写原子——H6）：33Hz 位置。
        // 速度不用 jointSpeedMoto——2026-07-24 晚 speed_check 实测其为
        // 电机侧整数量化值（静止时 0/±2 跳变，非关节 rad/s），直接喂给
        // JTC 会让样条初始速度条件错误、开局冲刺偏出走廊（cause=2 闩锁，
        // 快照实锤 upperArm 单周期 +0.043 rad）。速度改为对推送位置做
        // 33Hz 差分（与轮询差分同源语义，频率更高）。
        [this](const AuboHardwareInterface::JointFull& js) {
            static bool first_push_logged = false;
            if (!first_push_logged) {
                first_push_logged = true;
                RCLCPP_INFO(kLogger, "first JointStatus push received (33Hz path alive)");
            }
            // 推送差分速度（本 lambda 只被 SDK 推送线程调用，无需原子）
            static double last_pos[kNJoint] = {0};
            static auto last_t = std::chrono::steady_clock::now();
            static bool has_last = false;
            const auto now = std::chrono::steady_clock::now();
            for (int i = 0; i < kNJoint; ++i) {
                double vel = 0.0;
                if (has_last) {
                    const double dt =
                        std::chrono::duration<double>(now - last_t).count();
                    if (dt > 0.001) vel = (js.pos[i] - last_pos[i]) / dt;
                }
                state_pos_[i].store(js.pos[i], std::memory_order_relaxed);
                state_vel_[i].store(vel, std::memory_order_relaxed);
                last_pos[i] = js.pos[i];
            }
            last_t = now;
            has_last = true;
            state_valid_.store(true, std::memory_order_relaxed);
            state_ok_ns_.store(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count(),
                std::memory_order_relaxed);
        },
        // RoadPoint 推送：与旧栈对齐注册，主要用于连接保活。
        // 2026-07-22 断连排查：旧栈（RoadPoint+JointStatus+EndSpeed 三推送全注册）
        // 连接 90s+ 稳定；只注册 JointStatus 时 ~15s 被服务器主动断连。
        // 注意：wayPoint_S 是"路点"（目标）语义，不能当作实际位置反馈，
        // 这里只刷新新鲜度，不写 state_pos_（位置以 JointStatus 推送/轮询为准）。
        [this](const aubo_robot_namespace::wayPoint_S& /*wp*/) {
            state_ok_ns_.store(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count(),
                std::memory_order_relaxed);
        },
        // EndSpeed 推送（占位对齐旧栈，数据暂不消费）
        [](double /*speed*/) {});

    RCLCPP_INFO(kLogger, "on_configure: logged in (%s:%d)", robot_ip_.c_str(), robot_port_);
    return CallbackReturn::SUCCESS;
}

AuboRos2System::CallbackReturn AuboRos2System::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    if (!hw_ || !hw_->isConnected()) {
        RCLCPP_ERROR(kLogger, "on_activate: hardware not connected");
        return CallbackReturn::FAILURE;
    }
    if (allow_motion_commands_) {
    if (!hw_->enterTcp2CanbusMode()) {
        RCLCPP_ERROR(kLogger, "on_activate: enterTcp2CanbusMode failed");
        return CallbackReturn::FAILURE;
    }

    // 清残留 RIB（2026-07-22 probe8 事故教训：重入 TCP2CAN 时 RIB 不自动清空，
    // 上次会话异常退出（如 stopMotion 失败）留下的残点会在进入后立即执行）
    hw_->stopMotion();
    {
        int rib = -1;
        if (hw_->readDiagnosis(rib) && rib > 0) {
            RCLCPP_WARN(kLogger,
                "on_activate: RIB residual %d slots after stopMotion — waiting for drain", rib);
            // 总超时上限 10s（TCP2CAN 空闲期诊断查询可能逐个 1s 超时，
            // 逐次等待最坏 ~55s 不可接受；排空率 ~4.7ms/点，1200 槽全满也仅 ~5.6s）
            const auto t_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
            while (rib > 0 && std::chrono::steady_clock::now() < t_deadline) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                hw_->readDiagnosis(rib);
            }
            if (rib > 0) {
                RCLCPP_ERROR(kLogger, "on_activate: RIB residual %d not drained in 10s, refuse to activate", rib);
                return CallbackReturn::FAILURE;
            }
        }
    }
    }

    // 首点防跳 (§3.4, UR 同款): command 种子值 = 当前实际位置。
    // 双读校验（2026-07-22 probe8 事故教训：单次读在超时/遥测冻结下可能拿到
    // 陈旧/全零假值，以此为种子会命令机械臂跑向错误位置）：连读两次间隔 150ms，
    // 两次都成功且各关节偏差 <0.005 rad 才接受；种子接近全零时高声告警
    // （遥测冻结的典型签名是 J2–J6 恰为 0.0000，见 §8.6），由操作员把关。
    double pos[kNJoint], vel[kNJoint], pos2[kNJoint], vel2[kNJoint];
    if (!hw_->readJointState(pos, vel)) {
        RCLCPP_ERROR(kLogger, "on_activate: readJointState failed, refuse to activate");
        return CallbackReturn::FAILURE;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    if (!hw_->readJointState(pos2, vel2)) {
        RCLCPP_ERROR(kLogger, "on_activate: second readJointState failed, refuse to activate");
        return CallbackReturn::FAILURE;
    }
    int near_zero = 0;
    for (int i = 0; i < kNJoint; i++) {
        if (std::fabs(pos2[i] - pos[i]) > 0.005) {
            RCLCPP_ERROR(kLogger,
                "on_activate: joint %d mismatch %.4f rad between seed reads — telemetry stale or robot moving, refuse to activate",
                i, std::fabs(pos2[i] - pos[i]));
            return CallbackReturn::FAILURE;
        }
        if (std::fabs(pos2[i]) < 1e-4) near_zero++;
    }
    if (near_zero >= 5) {
        RCLCPP_WARN(kLogger,
            "on_activate: seed pose near all-zeros (%.4f %.4f %.4f %.4f %.4f %.4f) — "
            "若示教器显示机械臂不在零位，说明遥测冻结，请重启系统！",
            pos2[0], pos2[1], pos2[2], pos2[3], pos2[4], pos2[5]);
    }
    for (int i = 0; i < kNJoint; i++) {
        hw_pos_cmd_[i] = pos2[i];
        hw_pos_state_[i] = pos2[i];
        hw_vel_state_[i] = vel2[i];
        state_pos_[i].store(pos2[i]);
        state_vel_[i].store(vel2[i]);
    }
    std::array<double, kNJoint> seed{};
    for (int i = 0; i < kNJoint; ++i) seed[i] = pos2[i];
    double seed_progress = 0.0;
    if (allow_motion_commands_ && !commandOnAllowedPath(seed, seed_progress)) {
        RCLCPP_ERROR(kLogger, "on_activate: actual pose is outside the authorized home-camera path");
        hw_->stopMotion();
        hw_->leaveTcp2CanbusMode();
        return CallbackReturn::FAILURE;
    }
    last_path_progress_ = seed_progress;
    path_direction_ = 0;
    state_ok_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
    state_valid_.store(true);

    // 清队列残留 + 复位错误/冻结状态
    std::array<double, kNJoint> dump;
    while (setpoint_queue_.try_dequeue(dump)) {}
    queue_overflow_.store(false);
    command_fault_.store(false);
    traj_fault_.store(false);
    safety_fault_.store(false);
    fault_cause_.store(0);
    rej_seq_.store(0);
    setpoint_seq_.store(0);
    last_enqueue_ns_.store(0);
    last_written_cmd_ = seed;
    has_last_written_cmd_ = true;

    // 启动内部线程 (sender: conn_control_ 发点; status: conn_status_ 轮询)
    threads_running_.store(true);
    if (allow_motion_commands_) sender_thread_ = std::thread(
        &AuboRos2System::senderLoop, this);
    status_thread_ = std::thread(&AuboRos2System::statusLoop, this);

    RCLCPP_INFO(kLogger, "on_activate: TCP2CAN + threads started (seed pos=[%.4f %.4f %.4f %.4f %.4f %.4f])",
        pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
    return CallbackReturn::SUCCESS;
}

AuboRos2System::CallbackReturn AuboRos2System::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    stopThreads();
    if (hw_ && allow_motion_commands_) {
        // H10: 先停运动清 RIB (按 ~4.7ms/点消费率排空, §8.8)，再 leave
        hw_->stopMotion();
        hw_->leaveTcp2CanbusMode();
    }
    // 闩锁复位：现场恢复（dashboard /aubo/collision_recover + /aubo/startup）后
    // 重新 activate 才允许清零（与 H8"碰撞须先 Recover"一致）
    traj_fault_.store(false);
    safety_fault_.store(false);
    command_fault_.store(false);
    RCLCPP_INFO(kLogger, "on_deactivate: threads stopped, left TCP2CAN, fault latches reset");
    return CallbackReturn::SUCCESS;
}

AuboRos2System::CallbackReturn AuboRos2System::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    if (hw_) {
        hw_->shutdown();   // logout conn_control_ + conn_status_ (H10)
        hw_.reset();
    }
    state_valid_.store(false);
    traj_fault_.store(false);
    safety_fault_.store(false);
    command_fault_.store(false);
    RCLCPP_INFO(kLogger, "on_cleanup: logged out");
    return CallbackReturn::SUCCESS;
}

AuboRos2System::CallbackReturn AuboRos2System::on_shutdown(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    stopThreads();
    if (hw_) {
        if (allow_motion_commands_) hw_->stopMotion();
        hw_->shutdown();
        hw_.reset();
    }
    return CallbackReturn::SUCCESS;
}

void AuboRos2System::stopThreads()
{
    threads_running_.store(false);
    if (sender_thread_.joinable()) sender_thread_.join();
    if (status_thread_.joinable()) status_thread_.join();
}

// ══════════════════════════════════════════════════════════════════════
// RT path — 零 SDK 调用、零日志、零堆分配
// ══════════════════════════════════════════════════════════════════════

bool AuboRos2System::commandOnAllowedPath(
    const std::array<double, kNJoint>& command, double& progress) const noexcept
{
    double numerator = 0.0;
    double denominator = 0.0;
    for (int i = 0; i < kNJoint; ++i) {
        if (!std::isfinite(command[i]) || command[i] < joint_min_[i] ||
            command[i] > joint_max_[i]) return false;
        // 包围盒约束（2026-07-25 由"直线管道"改为"关节区间盒"）：每关节
        // 限在 home 与 camera_pose 的区间 ±kPathTolerance 内——忠实实现
        // "只在两个授权位姿之间运动"：放行 OMPL/RRTConnect 的合法曲率
        // （实测肩部偏离直线 0.0515 rad 被旧管道误杀），仍拦截 ≫0.1 rad
        // 级的 SDK 假数据/错误轨迹。方向/反向检测仍用线投影进度。
        const double lo = std::min(home_pose_[i], camera_pose_[i]) - kPathTolerance;
        const double hi = std::max(home_pose_[i], camera_pose_[i]) + kPathTolerance;
        if (command[i] < lo || command[i] > hi) return false;
        const double axis = camera_pose_[i] - home_pose_[i];
        numerator += (command[i] - home_pose_[i]) * axis;
        denominator += axis * axis;
    }
    progress = numerator / denominator;
    if (progress < -kEndpointTolerance || progress > 1.0 + kEndpointTolerance) return false;
    progress = std::clamp(progress, 0.0, 1.0);
    return true;
}

hardware_interface::return_type AuboRos2System::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // RT path — 队满/断连/轨迹级故障/安全闩锁 → ERROR (零 SDK 调用、零日志、零堆分配)
    if (queue_overflow_.load(std::memory_order_relaxed) ||
        traj_fault_.load(std::memory_order_relaxed) ||
        safety_fault_.load(std::memory_order_relaxed) ||
        command_fault_.load(std::memory_order_relaxed)) {
        return hardware_interface::return_type::ERROR;
    }
    // 断连 (HI onRobotEvent 置 connected_=false)：已灌 RIB 的点控制器侧
    // 仍会执行 (§3.3 问题 4)，报 ERROR 让 JTC 中止当前 goal
    if (hw_ && !hw_->isConnected()) {
        return hardware_interface::return_type::ERROR;
    }
    // 原子缓存 → state interfaces (value_ptr 绑定，框架直接读 hw_*_state_)
    if (state_valid_.load(std::memory_order_relaxed)) {
        for (int i = 0; i < kNJoint; i++) {
            hw_pos_state_[i] = state_pos_[i].load(std::memory_order_relaxed);
            hw_vel_state_[i] = state_vel_[i].load(std::memory_order_relaxed);
        }
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type AuboRos2System::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    if (!allow_motion_commands_) return hardware_interface::return_type::OK;
    if (!threads_running_.load(std::memory_order_relaxed)) {
        return hardware_interface::return_type::OK;
    }

    // 反馈失新处理 (2026-07-24 平滑性实测修订；当日下午阈值 5s→12s)：
    // - >kFeedbackFaultNs(12s) 且轨迹执行中 → 闩锁 traj_fault_（RIB 排空后
    //   轨迹长期失明，必须中止）；
    // - >12s 空闲 → 安全 hold，不闩锁；
    // - 0.5-12s 短暂失新 → 继续入队。授权路径是 home↔camera 单一直线走廊，
    //   样条预规划在走廊内、速度有钳制、RIB 有水位门控，短暂失明期间继续
    //   流式不会跑出走廊；而暂停-恢复会让流式速率塌缩到 ~15 pts/s，
    //   机械臂必然跟丢样条被 JTC 中止（实测两次）。
    const int64_t ok_ns = state_ok_ns_.load(std::memory_order_relaxed);
    const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    const int64_t feedback_age = ok_ns > 0 ? now_ns - ok_ns : kFeedbackFaultNs + 1;
    if (feedback_age > kFeedbackFaultNs) {
        const int64_t since_enqueue =
            now_ns - last_enqueue_ns_.load(std::memory_order_relaxed);
        if (since_enqueue < kMotionRecentNs) {
            traj_fault_.store(true, std::memory_order_relaxed);
            fault_cause_.store(1, std::memory_order_relaxed);
        }
        return hardware_interface::return_type::OK;
    }

    double raw_progress = 0.0;
    if (!commandOnAllowedPath(hw_pos_cmd_, raw_progress)) {
        command_fault_.store(true, std::memory_order_relaxed);
        fault_cause_.store(2, std::memory_order_relaxed);
        rej_seq_.fetch_add(1, std::memory_order_relaxed);
        rej_cmd_ = hw_pos_cmd_;
        rej_ref_ = last_written_cmd_;
        rej_progress_ = raw_progress;
        rej_site_ = 1;
        rej_seq_.fetch_add(1, std::memory_order_relaxed);
        return hardware_interface::return_type::OK;
    }

    // 轨迹激活判据 (§3.3 问题 2): 任一关节 setpoint 相对上周期变化 >1e-6
    // 才入队——hold 期 (JTC 无活动 goal 时输出恒定 setpoint) 不喂点，
    // 避免 RIB 被 hold 点占满导致真轨迹起始延迟 + 抖动
    bool changed = !has_last_written_cmd_;
    for (int i = 0; i < kNJoint && !changed; i++) {
        if (std::fabs(hw_pos_cmd_[i] - last_written_cmd_[i]) > 1e-6) changed = true;
    }
    if (!changed) {
        return hardware_interface::return_type::OK;
    }

    // 逐周期跳变钳制 (回跳/追赶根治)：JTC 样条输出必然连续，
    // 单周期 |Δcmd|>kMaxStepPerCycle 只可能来自 hold 重置/取消/恢复追赶，
    // 绝不可能是合法轨迹 (URDF 限速 2.5964 rad/s ⇒ 5ms 合法步长 ≤0.013 rad)。
    // 钳制后瞬跳变为额定速度内的有界缓动，正常轨迹永不触发。
    std::array<double, kNJoint> pt = hw_pos_cmd_;
    if (has_last_written_cmd_) {
        for (int i = 0; i < kNJoint; i++) {
            const double d = pt[i] - last_written_cmd_[i];
            if (d > kMaxStepPerCycle) pt[i] = last_written_cmd_[i] + kMaxStepPerCycle;
            else if (d < -kMaxStepPerCycle) pt[i] = last_written_cmd_[i] - kMaxStepPerCycle;
        }
    }

    double accepted_progress = 0.0;
    if (!commandOnAllowedPath(pt, accepted_progress)) {
        command_fault_.store(true, std::memory_order_relaxed);
        fault_cause_.store(2, std::memory_order_relaxed);
        rej_seq_.fetch_add(1, std::memory_order_relaxed);
        rej_cmd_ = pt;
        rej_ref_ = last_written_cmd_;
        rej_progress_ = accepted_progress;
        rej_site_ = 2;
        rej_seq_.fetch_add(1, std::memory_order_relaxed);
        return hardware_interface::return_type::OK;
    }
    const double progress_delta = accepted_progress - last_path_progress_;
    if (std::fabs(progress_delta) > 1e-5) {
        if (path_direction_ == 0 ||
            (last_path_progress_ > 0.99 && progress_delta < 0.0) ||
            (last_path_progress_ < 0.01 && progress_delta > 0.0)) {
            path_direction_ = progress_delta > 0.0 ? 1 : -1;
        }
        // 反向检测滞后 0.02（2026-07-25 由 0.002 放宽）：OMPL/RRTConnect
        // 路径在"线进度"坐标上非单调，实测返程中进度合法回弹 >0.002
        // （cause=3 误杀，快照实锤）。真正要抓的是 JTC 取消/恢复追赶产生
        // 的大幅回跳（≫0.02 进度 ≈ >0.01 rad），0.02 滞后仍可靠拦截。
        if ((path_direction_ > 0 && progress_delta < -0.02) ||
            (path_direction_ < 0 && progress_delta > 0.02)) {
            command_fault_.store(true, std::memory_order_relaxed);
            fault_cause_.store(3, std::memory_order_relaxed);
            rej_seq_.fetch_add(1, std::memory_order_relaxed);
            rej_cmd_ = pt;
            rej_ref_ = last_written_cmd_;
            rej_progress_ = accepted_progress;
            rej_site_ = 3;
            rej_seq_.fetch_add(1, std::memory_order_relaxed);
            return hardware_interface::return_type::OK;
        }
    }

    // 上采样 ×2（2026-07-24 晚，RIB 饥饿抖动根治）：JTC setpoint 100Hz，
    // 控制器按 ~5ms CAN 周期消费——100 pts/s 供给意味着每两个 CAN 周期
    // 就有一个空转（缓存为空）→ 机械臂走走停停（用户观测的"卡顿响声"）。
    // 每个 setpoint 展开为 [中点, 终点] 两个 RIB 点（200 pts/s），CAN 周期
    // 始终有新点；轨迹时长不变（每 setpoint 仍占 ~10ms 内容）。
    // 旧栈（ROS1 Python 200Hz / 自研 quintic 200Hz）同样是按消费率供给。
    if (has_last_written_cmd_) {
        std::array<double, kNJoint> mid;
        for (int i = 0; i < kNJoint; i++) {
            mid[i] = 0.5 * (last_written_cmd_[i] + pt[i]);
        }
        if (!setpoint_queue_.try_enqueue(mid)) {
            queue_overflow_.store(true, std::memory_order_relaxed);
            fault_cause_.store(4, std::memory_order_relaxed);
            return hardware_interface::return_type::OK;
        }
    }
    if (!setpoint_queue_.try_enqueue(pt)) {
        // 队满 (sender 连续 ~10s 停摆 = SDK 持续故障): 不允许丢点，
        // 置原子错误标志，由 read() 返回 ERROR (§3.3 问题 4)
        queue_overflow_.store(true, std::memory_order_relaxed);
        fault_cause_.store(4, std::memory_order_relaxed);
        return hardware_interface::return_type::OK;
    }
    setpoint_seq_.fetch_add(1, std::memory_order_relaxed);
    last_enqueue_ns_.store(now_ns, std::memory_order_relaxed);
    last_written_cmd_ = pt;
    last_path_progress_ = accepted_progress;
    has_last_written_cmd_ = true;
    return hardware_interface::return_type::OK;
}

// ══════════════════════════════════════════════════════════════════════
// sender 线程 — sendLoop 流控移植 (joint_trajectory_controller.cpp:395-554)
// ══════════════════════════════════════════════════════════════════════

void AuboRos2System::senderLoop()
{
    using namespace std::chrono;
    int rib = 0, ok = 0, fail = 0;
    int rib_peak = 0;          // RIB 峰值
    int rib_deplete = 0;       // RIB 耗尽连续计数
    int rib_high_count = 0;    // RIB≥350 (batch=1) 事件数
    int rib_warn_count = 0;    // RIB≥300 (batch=2) 事件数
    int diag_fail = 0;
    int consecutive_send_fail = 0;
    auto last_diag = steady_clock::now() - milliseconds(250);
    auto last_summary = steady_clock::now();
    double ema = 0, ema_max = 0;
    double ema_sum = 0; int ema_samples = 0;

    // 冻结检测状态 (§3.3 问题 1 对策 3)
    uint64_t last_seq = setpoint_seq_.load(std::memory_order_relaxed);
    int freeze_cycles = 0;
    bool freeze_stopped = false;
    bool safety_stopped = false;   // 安全闩锁一次性触发标记 (F3)

    // 发送失败的待重发批次：点绝不丢弃（丢点 = 轨迹跳变 = 控制器跟踪误差保护停机，
    // 2026-07-22 真机事故根因）。pending 为 FIFO 分段队列（大批次失败降级时
    // 拆成 ≤8 小批按序重发），非空时停止出队新点（RIB 门控自然背压）。
    std::deque<std::vector<aubo_robot_namespace::wayPoint_S>> pending;

    long pts_since_summary = 0;   // SEND_SUMMARY 窗口内成功发送点数

    RCLCPP_INFO(kLogger, "senderLoop started");

    while (threads_running_.load(std::memory_order_relaxed) && rclcpp::ok()) {
        const size_t avail = setpoint_queue_.size_approx();

        // ── 安全闩锁 (F3): SDK 安全事件或安全 IO 轮询命中 → 停止并清队 + traj_fault_ ──
        const bool any_fault = safety_fault_.load(std::memory_order_relaxed) ||
            command_fault_.load(std::memory_order_relaxed) ||
            traj_fault_.load(std::memory_order_relaxed);
        if (any_fault && !safety_stopped) {
            RCLCPP_ERROR(kLogger,
                "FAULT_STOP: safety/command/trajectory fault (rib=%d, cause=%d) — stop and drain",
                rib, fault_cause_.load(std::memory_order_relaxed));
            const uint32_t seq = rej_seq_.load(std::memory_order_relaxed);
            if (seq > 0 && seq % 2 == 0) {
                RCLCPP_ERROR(kLogger,
                    "rejected site=%d progress=%.4f cmd=[%.4f %.4f %.4f %.4f %.4f %.4f] ref=[%.4f %.4f %.4f %.4f %.4f %.4f]",
                    rej_site_, rej_progress_,
                    rej_cmd_[0], rej_cmd_[1], rej_cmd_[2], rej_cmd_[3], rej_cmd_[4], rej_cmd_[5],
                    rej_ref_[0], rej_ref_[1], rej_ref_[2], rej_ref_[3], rej_ref_[4], rej_ref_[5]);
            }
            hw_->stopMotion();
            std::array<double, kNJoint> dump;
            while (setpoint_queue_.try_dequeue(dump)) {}
            pending.clear();
            traj_fault_.store(true, std::memory_order_relaxed);
            safety_stopped = true;
        }

        // ── 冻结检测兜底: setpoint 连续 kFreezeCycles 周期无变化且 RIB>0
        //    → JTC 已取消/冻结但 RIB 仍有存量 (残余 = 水位×4.7ms) → stop + 排空 ──
        const uint64_t seq = setpoint_seq_.load(std::memory_order_relaxed);
        if (seq != last_seq) {
            last_seq = seq;
            freeze_cycles = 0;
            freeze_stopped = false;
        } else if (!freeze_stopped && rib > 0) {
            if (++freeze_cycles >= kFreezeCycles) {
                RCLCPP_WARN(kLogger,
                    "FREEZE_STOP: setpoint frozen %d cycles with rib=%d — stopMotion + drain queue + traj_fault_",
                    freeze_cycles, rib);
                hw_->stopMotion();   // 内部 control_mutex_ 保护, 锁外调用
                std::array<double, kNJoint> dump;
                while (setpoint_queue_.try_dequeue(dump)) {}
                pending.clear();   // stop 后禁止重发残留批次（2026-07-22 修复）
                traj_fault_.store(true, std::memory_order_relaxed);   // F2: 止动必须告知 JTC
                fault_cause_.store(7, std::memory_order_relaxed);
                freeze_stopped = true;
            }
        }

        // ── 反馈失新 = 观测降级，不是机械臂失控（2026-07-22 审查 F1）──
        // RIB 点在控制器侧照常执行；短暂失新期间 write() 继续入队（走廊有界），
        // >12s 才闩锁。这里只做可观测性告警，绝不 stopMotion（否则 JTC 续跑
        // 产生追赶跳变）。
        {
            const int64_t ok_ns = state_ok_ns_.load(std::memory_order_relaxed);
            const int64_t now_ns = duration_cast<nanoseconds>(
                steady_clock::now().time_since_epoch()).count();
            const int64_t age_ms = (ok_ns > 0) ? (now_ns - ok_ns) / 1000000 : 0;
            if (ok_ns > 0 && age_ms > 1000) {
                static int stale_warn = 0;
                if (++stale_warn % 100 == 1)
                    RCLCPP_WARN(kLogger,
                        "FEEDBACK_STALE %.1fs — motion continues on RIB buffer (latch at %.0fs)",
                        age_ms / 1000.0, kFeedbackFaultNs / 1e9);
            }
        }

        // RIB 查询 (锁外 — sender 独占查询)。流式期间每周期一查（v1.3.1
        // 写/查延迟 ~5ms，不存在带宽问题；Fix14 原味：水位必须当轮新鲜）。
        // 空闲（队列空 + RIB 空 + 无 pending）时完全不查：TCP2CAN 空闲期
        // 诊断查询极易撞超时（2026-07-22 实测），且 2026-07-24 v5/v6 两轮
        // 实验证实空闲期 2Hz 保温查询会诱发 status 连接 11-13s 级失新，
        // 比冷启动代价更严重。冷启动延迟改由轨迹起始保持段吸收。
        auto now = steady_clock::now();
        const bool streaming = (avail > 0) || (rib > 0) || !pending.empty();
        if (streaming &&
            (rib <= 0 ||
             duration_cast<milliseconds>(now - last_diag).count() >= 25)) {
            int r = -1;
            if (hw_->readDiagnosis(r)) {
                rib = r; last_diag = now; diag_fail = 0;
            } else if (++diag_fail >= 3) {
                RCLCPP_ERROR(kLogger, "RIB diagnosis failed 3 consecutive times");
                traj_fault_.store(true, std::memory_order_relaxed);
                fault_cause_.store(5, std::memory_order_relaxed);
            }
        }

        // RIB 峰值追踪 + 告警 (实测容量 1200 槽, §8.7; 带节流)
        if (rib > rib_peak) rib_peak = rib;
        if (rib >= kRibGateHigh) {
            static int rib_hi_cnt = 0;
            if (++rib_hi_cnt % 20 == 1)  // 20 cycle 节流 (~100ms @200Hz)
                RCLCPP_WARN(kLogger,
                    "RIB_HIGH rib=%d peak=%d — controller buffer near overflow (capacity 1200)",
                    rib, rib_peak);
        } else if (rib >= 250) {
            static int rib_warn_cnt = 0;
            if (++rib_warn_cnt % 50 == 1)  // 50 cycle 节流 (~250ms @200Hz)
                RCLCPP_WARN(kLogger,
                    "RIB_WARN rib=%d peak=%d — buffer filling, possible send delay",
                    rib, rib_peak);
        }

        // 自适应批量 (还原 ROS1 原始公式, 经 Fix1-Fix14 验证 — 逐字保留；
        // 上限由 8 参数化为 tcp2can_batch_max_，2026-07-24：大批次减少调用
        // 次数 = 更少撞停滞的机会，服务器拒绝时运行时降回 8，见失败分支)
        int need = std::max(2, (int)std::ceil((400 - rib) / 6.0));
        if (ema > 10) need = std::max(need, 4);
        if (ema > 14) need = std::max(need, 6);
        if (ema > 20) need = std::max(need, 8);
        need = std::min(need, tcp2can_batch_max_);

        // RIB≥300 最小批量持续发送 (不停不冷 — 参考 UR writeKeepalive; 溢出保险)
        if (rib >= kRibGateHigh) { need = 1; rib_high_count++; }
        else if (rib >= kRibGateWarn) { need = std::min(need, 2); rib_warn_count++; }

        // ── 唯一改动点 (§3.6): 目标工作水位带 [60,120] 槽 ──
        //   rib > 120: 缓灌/暂停 (本轮不发)
        //   60 ≤ rib ≤ 120: 限速 ~1 点/周期 ≈ 消费率 200Hz (速率匹配, §3.3 问题 3)
        //   rib < 60: need 不变 → 加速灌
        if (rib > kRibBandHigh) need = 0;
        else if (rib >= kRibBandLow) need = std::min(need, 1);

        size_t n = std::min(avail, (size_t)std::max(need, 0));

        // RIB 耗尽检测 (avail>0 但 rib==0 → 控制器空闲)
        if (avail > 0 && rib <= 0) {
            rib_deplete++;
            if (rib_deplete == 1)
                RCLCPP_INFO(kLogger,
                    "RIB_IDLE avail=%zu rib=%d — controller buffer empty, start filling",
                    avail, rib);
        } else {
            rib_deplete = 0;
        }

        if (n > 0 || !pending.empty()) {
            // 批次来源：优先重发 pending（失败保留的批次，FIFO 取首段），
            // 否则出队新点 (SPSC 无锁; sender 线程内允许堆分配)
            std::vector<aubo_robot_namespace::wayPoint_S> batch;
            if (!pending.empty()) {
                batch = std::move(pending.front());
                pending.pop_front();
            } else {
                batch.reserve(n);
                std::array<double, kNJoint> pt;
                while (batch.size() < n && setpoint_queue_.try_dequeue(pt)) {
                    aubo_robot_namespace::wayPoint_S wp{};
                    for (int i = 0; i < kNJoint; i++) wp.jointpos[i] = pt[i];
                    batch.push_back(wp);
                }
            }

            if (!batch.empty()) {
                auto t0 = steady_clock::now();
                bool sent = hw_->writeTrajectoryPoints(batch);
                if (!sent) {
                    // 发送路径也会撞 1s 整超时 (ret=10007, §8.7 流启动前两批实测):
                    // 按 stopMotion 同款模式 — 锁外 sleep 20ms 后重试一次
                    std::this_thread::sleep_for(milliseconds(20));
                    sent = hw_->writeTrajectoryPoints(batch);
                }
                auto el = duration_cast<microseconds>(steady_clock::now() - t0);
                double ms = el.count() / 1000.0;

                if (sent) {
                    consecutive_send_fail = 0;
                    rib += static_cast<int>(batch.size());  // 乐观增量：
                    // 两次 diag 查询（50ms）之间保持门控新鲜，下次查询校正
                    pts_since_summary += static_cast<long>(batch.size());
                    ema = (ema <= 0) ? ms : (0.9 * ema + 0.1 * ms);
                    if (ema > ema_max) ema_max = ema;
                    ema_sum += ms; ema_samples++;
                    ok++;

                    // 时序抖动告警: 单次发送延迟 > 50ms OR EMA > 30ms
                    if (ms > 50.0) {
                        RCLCPP_WARN(kLogger,
                            "TIMING_SPIKE send=%.1fms ema=%.1fms batch=%zu rib=%d — possible TCP congestion",
                            ms, ema, batch.size(), rib);
                    } else if (ema > 30.0 && ok % 20 == 1) {
                        RCLCPP_WARN(kLogger,
                            "TIMING_HIGH ema=%.1fms batch=%zu rib=%d — sustained send delay",
                            ema, batch.size(), rib);
                    }

                    // 每 200 批或每 2 秒打印摘要
                    auto since_summary = duration_cast<milliseconds>(
                        steady_clock::now() - last_summary).count();
                    if (ok % 200 == 0 || since_summary > 2000) {
                        const double pts_per_s = since_summary > 0
                            ? pts_since_summary * 1000.0 / since_summary : 0.0;
                        pts_since_summary = 0;
                        last_summary = steady_clock::now();
                        RCLCPP_INFO(kLogger,
                            "SEND_SUMMARY ok=%d fail=%d rib=%d peak=%d rate=%.0fpts/s "
                            "ema=%.1fms(avg=%.1f/max=%.1f) batch=%zu need=%d",
                            ok, fail, rib, rib_peak, pts_per_s, ema,
                            ema_samples > 0 ? ema_sum / ema_samples : 0, ema_max,
                            batch.size(), need);
                    }
                } else {
                    // 失败不丢点：批次移回 pending 队首，下轮重发（下轮起
                    // pending 优先、不再出队新点，RIB 门控自然背压）。
                    fail++;
                    if (batch.size() > 8) {
                        // 服务器拒绝大批次（批量上限是服务器状态相关的，
                        // 2026-07-24 实测）：拆成 ≤8 小批按序重发，
                        // 并把批量上限降回已验证的 8，本会话不再尝试大批次。
                        if (tcp2can_batch_max_ > 8) {
                            tcp2can_batch_max_ = 8;
                            RCLCPP_WARN(kLogger,
                                "TCP2CAN batch %zu rejected — falling back to batch_max=8 for this session",
                                batch.size());
                        }
                        for (size_t off = batch.size(); off > 0;) {
                            const size_t begin = off > 8 ? off - 8 : 0;
                            pending.push_front(std::vector<aubo_robot_namespace::wayPoint_S>(
                                std::make_move_iterator(batch.begin() + begin),
                                std::make_move_iterator(batch.begin() + off)));
                            off = begin;
                        }
                    } else {
                        pending.push_front(std::move(batch));
                    }
                    if (++consecutive_send_fail >= 3) {
                        RCLCPP_ERROR(kLogger, "TCP2CAN send failed 3 consecutive batches");
                        traj_fault_.store(true, std::memory_order_relaxed);
                        fault_cause_.store(6, std::memory_order_relaxed);
                    }
                    if (fail % 50 == 1)
                        RCLCPP_WARN(kLogger,
                            "SEND_FAIL #%d pending=%zu rib=%d — writeTrajectoryPoints failed, holding batch in pending (no points dropped)",
                            fail, pending.size(), rib);
                }
            }
        }

        // 自适应睡眠 (还原 ROS1 原始策略 — 逐字保留)。v1.3.1 写延迟 ~5ms，
        // 产能远超消费率，RIB 水位门控主导流量，无需外加节拍（40ms 节拍器
        // 是 v2.5.3 写延迟 ~41ms 时代的补救，2026-07-24 晚随换库移除）。
        if (avail > 40) std::this_thread::sleep_for(milliseconds(1));
        else if (rib < 200 && avail > 0) std::this_thread::sleep_for(milliseconds(1));
        else std::this_thread::sleep_for(milliseconds(4));
    }
    RCLCPP_INFO(kLogger, "senderLoop exit (ok=%d fail=%d rib_peak=%d)", ok, fail, rib_peak);
}

// ══════════════════════════════════════════════════════════════════════════
// status 线程 — 20Hz 轮询（2026-07-24 网卡驱动修复后复测：轮询 200Hz 持续
// 零失败；推送 33Hz 独立可用。旧"推送+TCP2CAN 必断链/轮询上限 64Hz/10Hz
// 安全工作点"均为降级通道下的经验值，已作废，见 docs/nic_driver_incident.md）
// ══════════════════════════════════════════════════════════════════════════

void AuboRos2System::statusLoop()
{
    using namespace std::chrono;
    double pos[kNJoint], vel[kNJoint];
    int safety_div = 0;
    while (threads_running_.load(std::memory_order_relaxed) && rclcpp::ok()) {
        // 20Hz 轮询：网卡驱动修复后（2026-07-24，docs/nic_driver_incident.md）
        // 实测 5ms/2ms 周期轮询 1200/2000 样本零失败；20Hz 留有充足余量。
        // 推送是否启用见 hw 层 registerCallbacks 的注释（并发长测判定）。
        if (hw_->readJointState(pos, vel)) {
            static bool first_poll_logged = false;
            if (!first_poll_logged) {
                first_poll_logged = true;
                RCLCPP_INFO(kLogger, "statusLoop: first joint-state poll succeeded (20Hz path alive)");
            }
            for (int i = 0; i < kNJoint; i++) {
                state_pos_[i].store(pos[i], std::memory_order_relaxed);
                state_vel_[i].store(vel[i], std::memory_order_relaxed);
            }
            state_valid_.store(true, std::memory_order_relaxed);
            state_ok_ns_.store(
                duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count(),
                std::memory_order_relaxed);
        }
        // 安全 IO 兜底轮询 ~5s (F3：事件推送可能丢失时仍能发现急停/保护停)。
        // 2026-07-24 实测：服务器全局串行处理请求，300ms 高频 IO 查询是
        // 关节状态反馈被饿死的主要负载来源之一（IO 查询自身也易撞 5s
        // 超时并链式阻塞 status_mutex_）。事件回调仍是主安全通道。
        if (allow_motion_commands_ && ++safety_div >= 50 && !safety_fault_.load(std::memory_order_relaxed)) {
            safety_div = 0;
            bool estop = false, pstop = false;
            if (hw_->readSafetyIOStatus(estop, pstop) && (estop || pstop)) {
                safety_fault_.store(true, std::memory_order_relaxed);
                fault_cause_.store(9, std::memory_order_relaxed);
                RCLCPP_ERROR(kLogger,
                    "SAFETY IO poll: emergency=%d protective=%d — safety_fault_ latched",
                    (int)estop, (int)pstop);
            }
        }
        std::this_thread::sleep_for(milliseconds(50));  // 20Hz：2026-07-24 网卡
        // 驱动修复后实测轮询 200Hz 持续零失败（旧"10Hz 上限"是降级通道下的
        // 经验值，已作废）；20Hz 是留足服务器余量的安全工作点。
    }
}

}  // namespace aubo_driver

PLUGINLIB_EXPORT_CLASS(aubo_driver::AuboRos2System, hardware_interface::SystemInterface)
