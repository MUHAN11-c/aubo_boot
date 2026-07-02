/*
 * 轨迹控制器 — 预计算完整轨迹 + 独立发送线程 (ROS1 publishWaypointToRobot 移植)
 */

#include "aubo_driver_ros2/joint_trajectory_controller.h"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <thread>

namespace aubo_driver {

JointTrajectoryController::JointTrajectoryController(
    std::shared_ptr<AuboHardwareInterface> hw,
    const std::vector<std::string>& joint_names,
    const rclcpp::NodeOptions& options)
    : rclcpp::Node("joint_trajectory_controller", options)
    , hw_(std::move(hw)), joint_names_(joint_names)
{
    RCLCPP_INFO(get_logger(), "Controller created (%zu joints, %d Hz)",
        joint_names_.size(), (int)(1.0/update_period_));
}

JointTrajectoryController::~JointTrajectoryController() { deactivate(); }

void JointTrajectoryController::configure()
{
    trajectory_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    update_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
        this, "/joint_trajectory_controller/follow_joint_trajectory",
        [this](auto u, auto g) { return handleGoal(u, g); },
        [this](auto h) { return handleCancel(h); },
        [this](auto h) { handleAccepted(h); },
        rcl_action_server_get_default_options(), trajectory_cb_group_);

    update_timer_ = create_wall_timer(
        std::chrono::microseconds((int)(update_period_*1e6)),
        [this] { update(); }, update_cb_group_);

    send_running_ = true;
    send_thread_ = std::thread(&JointTrajectoryController::sendLoop, this);

    RCLCPP_INFO(get_logger(), "Configured (precompute + send thread)");
}

void JointTrajectoryController::deactivate()
{
    send_running_ = false;                       // 通知 sendLoop 退出
    stopAndClear();                              // 停止运动 + 清空预计算
    if (send_thread_.joinable()) send_thread_.join();
    if (has_active_goal_) {
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
        result->error_string = "Controller deactivated during execution";
        active_goal_->abort(result);             // abort() — 对齐标准 JTC (PR #1517)
        has_active_goal_ = false;
        active_goal_.reset();
    }
    // leaveTcp2CanbusMode() 由 main.cpp 调用，不在此处
    if (update_timer_) update_timer_->cancel();
}

rclcpp_action::GoalResponse JointTrajectoryController::handleGoal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const FollowJointTrajectory::Goal> goal)
{
    const auto& traj = goal->trajectory;

    // 1. 轨迹非空
    if (traj.points.empty()) {
        RCLCPP_ERROR(get_logger(), "Rejected: empty trajectory");
        return rclcpp_action::GoalResponse::REJECT;
    }

    // 2. 关节名匹配
    for (const auto& name : traj.joint_names) {
        if (std::find(joint_names_.begin(), joint_names_.end(), name) == joint_names_.end()) {
            RCLCPP_ERROR(get_logger(), "Rejected: unknown joint '%s'", name.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    // 3. 字段维度一致 (每个 point 的 positions 大小 == joint_names 数量)
    for (size_t i = 0; i < traj.points.size(); ++i) {
        if (traj.points[i].positions.size() != traj.joint_names.size()) {
            RCLCPP_ERROR(get_logger(),
                "Rejected: point %zu positions size %zu != joint_names size %zu",
                i, traj.points[i].positions.size(), traj.joint_names.size());
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    // 4. time_from_start 单调递增
    for (size_t i = 1; i < traj.points.size(); ++i) {
        if (rclcpp::Duration(traj.points[i].time_from_start) <=
            rclcpp::Duration(traj.points[i - 1].time_from_start)) {
            RCLCPP_ERROR(get_logger(),
                "Rejected: non-increasing time_from_start at point %zu", i);
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    // 5. 轨迹未过期
    if (rclcpp::Time(traj.header.stamp).seconds() != 0.0) {
        auto end_time = rclcpp::Time(traj.header.stamp) +
                        rclcpp::Duration(traj.points.back().time_from_start);
        if (end_time < get_clock()->now()) {
            RCLCPP_ERROR(get_logger(), "Rejected: trajectory ends in the past");
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    RCLCPP_INFO(get_logger(), "ACCEPT goal: %zu pts, %.2fs, joints=%zu",
        traj.points.size(), rclcpp::Duration(traj.points.back().time_from_start).seconds(),
        traj.joint_names.size());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse JointTrajectoryController::handleCancel(
    const std::shared_ptr<GoalHandle> gh)
{
    RCLCPP_INFO(get_logger(), "Cancel requested — immediate stop");
    stopAndClear();                             // 硬件停止 + 清空预计算

    auto result = std::make_shared<FollowJointTrajectory::Result>();
    if (active_goal_ && active_goal_ == gh) {
        result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
        result->error_string = "Canceled by user";
        has_active_goal_ = false;
        active_goal_.reset();
    } else {
        // goal 已结束或不存在, 仍发送结果让 MoveIt2 的 waitForExecution 正常返回
        result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
        result->error_string = "Goal already completed";
    }

    // CRITICAL: 不能在 handleCancel 内部调用 gh->canceled()！
    // 此时 goal state 仍为 EXECUTING. rclcpp 在 handleCancel 返回 ACCEPT 后
    // 才会将状态转为 CANCELING, 只有在那之后才能调用 canceled()
    // → 延迟到 update() 线程 (200Hz, 下一个事件循环周期) 执行
    pending_cancel_gh_ = gh;
    pending_cancel_result_ = result;
    pending_cancel_flag_ = true;
    RCLCPP_INFO(get_logger(), "Cancel: deferred result pending (update thread will send canceled())");

    return rclcpp_action::CancelResponse::ACCEPT;
}

void JointTrajectoryController::handleAccepted(const std::shared_ptr<GoalHandle> gh)
{
    // PR2: 抢占活跃 goal (如果存在)
    if (has_active_goal_) {
        RCLCPP_WARN(get_logger(), "PREEMPT: cancelling active goal for new incoming action");
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
        result->error_string = "Current goal cancelled due to new incoming action.";

        // CRITICAL: 不能在此直接调用 active_goal_->canceled()！
        // 此时 goal state 仍为 EXECUTING，必须延迟到 update() 线程执行
        // (与 handleCancel 相同的 rclcpp 状态机限制)
        pending_cancel_gh_ = active_goal_;
        pending_cancel_result_ = result;
        pending_cancel_flag_ = true;

        has_active_goal_ = false;
        active_goal_.reset();
        RCLCPP_INFO(get_logger(), "PREEMPT: calling stopAndClear...");
        stopAndClear();  // 仅在抢占时停止 + 清空
        RCLCPP_INFO(get_logger(), "PREEMPT: stopAndClear done, deferred cancel pending");
    } else {
        // 无活跃 goal: 仅清空预计算，不调 stopMotion (机械臂已停止，阻塞 TCP 调用无意义)
        RCLCPP_INFO(get_logger(), "NEW_GOAL: no active goal, skipping stopMotion");
        std::lock_guard<std::mutex> lock(precomputed_mutex_);
        precomputed_.clear();
        precomputed_idx_ = 0;
    }

    // PR2: 读当前位置 + blendToFirstPoint
    auto traj = remapJointNames(gh->get_goal()->trajectory);

    double current_joints[6] = {0};
    bool has_current = hw_->readJointState(current_joints);

    int blend_count = 0;
    std::vector<aubo_robot_namespace::wayPoint_S> blend_pts;
    if (has_current) {
        blend_count = blendToFirstPoint(current_joints, traj.points.front(), blend_pts);
        if (blend_count > 0) {
            RCLCPP_INFO(get_logger(), "Blended %d transition points (150ms)", blend_count);
        }
    }

    // 预计算轨迹 (先算到局部 vector，避免长时间持锁阻塞 sendLoop)
    double T = rclcpp::Duration(traj.points.back().time_from_start).seconds();
    int n = std::max(1, (int)std::round(T / update_period_));

    std::vector<aubo_robot_namespace::wayPoint_S> local;
    local.reserve(blend_pts.size() + n);
    for (auto& wp : blend_pts) local.push_back(wp);

    double t = 0; size_t p = 0;
    for (int s = 0; s < n && p + 1 < traj.points.size(); s++, t += update_period_) {
        while (p + 1 < traj.points.size() &&
               t > rclcpp::Duration(traj.points[p + 1].time_from_start).seconds() + 1e-6)
            p++;
        if (p + 1 >= traj.points.size()) break;
        auto& L = traj.points[p], &C = traj.points[p + 1];
        double seg = rclcpp::Duration(C.time_from_start).seconds() -
                     rclcpp::Duration(L.time_from_start).seconds();
        if (seg <= 0) seg = update_period_;
        double ts = std::max(0.0, t - rclcpp::Duration(L.time_from_start).seconds());
        double pos[6], vel[6], acc[6];
        quinticInterpolate(L, C, ts, seg, pos, vel, acc);
        aubo_robot_namespace::wayPoint_S wp{};
        for (int i = 0; i < kNJoint; i++) wp.jointpos[i] = pos[i];
        local.push_back(wp);
    }

    // 短暂持锁交换 (sendLoop 仅在此时短暂等待)
    {
        std::lock_guard<std::mutex> lock(precomputed_mutex_);
        precomputed_.swap(local);
        precomputed_idx_ = 0;
    }

    // PR3: 解析容差
    readGoalTolerances(gh->get_goal());
    goal_hold_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);  // 重置等待计时器
    goal_first_check_pending_ = true;  // 重置首次到位检查标记

    // 保存目标点
    goal_target_point_ = traj.points.back();
    goal_target_ = traj; goal_target_.points = {traj.points.back()};

    active_goal_ = gh; has_active_goal_ = true;
    RCLCPP_INFO(get_logger(), "Accepted (%zu wp, %.2fs) → %zu pts (blended=%d)",
        traj.points.size(), T, precomputed_.size(), blend_count);
}

void JointTrajectoryController::update()
{
    // 延迟 cancel 结果执行 (从 handleCancel/handleAccepted 委托)
    // 参考: realtime_tools::RealtimeServerGoalHandle::runNonRealtime()
    //   - 在 handleCancel 返回 ACCEPT 后 (rclcpp 已将 goal state 从 EXECUTING→CANCELING)
    //     才能安全调用 canceled()
    //   - try-catch 兜底防止意外状态转换崩溃 (对齐标准 JTC 模式)
    if (pending_cancel_flag_) {
        pending_cancel_flag_ = false;
        if (pending_cancel_gh_) {
            try {
                pending_cancel_gh_->canceled(pending_cancel_result_);
                RCLCPP_INFO(get_logger(), "CANCEL: deferred result sent (%s)",
                    pending_cancel_result_->error_string.c_str());
            } catch (const rclcpp::exceptions::RCLErrorBase& e) {
                RCLCPP_ERROR(get_logger(), "CANCEL: deferred result failed — %s",
                    e.formatted_message.c_str());
            }
            pending_cancel_gh_.reset();
            pending_cancel_result_.reset();
        }
        return;  // skip normal processing this cycle
    }

    if (!has_active_goal_ || !hw_ || !hw_->isConnected()) return;

    // 安全检查 (每 250ms)
    static int sc = 0;
    if (++sc >= 50) {
        sc = 0;
        bool e, p;
        hw_->readSafetyIOStatus(e, p);
        if (e || p) {
            RCLCPP_ERROR(get_logger(), "ESTOP");
            abortActiveGoal(FollowJointTrajectory::Result::INVALID_GOAL);
            return;
        }
    }

    // 轨迹未发送完 → 不做到位检查 (sendLoop 负责发送)
    {
        std::lock_guard<std::mutex> lock(precomputed_mutex_);
        if (precomputed_idx_ < precomputed_.size()) return;
        // 轨迹刚发完 → 记录开始等待时刻
        if (goal_hold_start_.seconds() == 0 && precomputed_idx_ >= precomputed_.size()) {
            goal_hold_start_ = get_clock()->now();
            RCLCPP_INFO(get_logger(), "GOAL_WAIT: trajectory sent, waiting for robot to reach goal (tol=%.4frad, time_limit=%.1fs)",
                goal_tolerances_[0], goal_time_tolerance_ > 0 ? goal_time_tolerance_ : -1.0);
        }
    }

    // 发完所有点后, 50ms 一次目标检查
    static int gc = 0;
    if (++gc < 10) return;
    gc = 0;

    double c[6], vel[6];
    if (!hw_->readJointState(c, vel)) return;

    // 首次到位检查: 记录初始误差 (用于判断是否 RIB 丢点导致"跳到位")
    if (goal_first_check_pending_) {
        goal_first_check_pending_ = false;
        double max_err = 0; int max_j = 0;
        for (int i = 0; i < kNJoint; i++) {
            double e = std::fabs(c[i] - goal_target_point_.positions[i]);
            if (e > max_err) { max_err = e; max_j = i; }
        }
        RCLCPP_INFO(get_logger(),
            "GOAL_FIRST_CHECK max_err=%.4f(j%d) pos=[%.4f %.4f %.4f %.4f %.4f %.4f] "
            "target=[%.4f %.4f %.4f %.4f %.4f %.4f] elapsed=%.2fs",
            max_err, max_j,
            c[0], c[1], c[2], c[3], c[4], c[5],
            goal_target_point_.positions[0], goal_target_point_.positions[1],
            goal_target_point_.positions[2], goal_target_point_.positions[3],
            goal_target_point_.positions[4], goal_target_point_.positions[5],
            (get_clock()->now() - goal_hold_start_).seconds());
    }

    // goal_time_tolerance 超时检查
    if (goal_time_tolerance_ > 0.0) {
        double elapsed = (get_clock()->now() - goal_hold_start_).seconds();
        if (elapsed > goal_time_tolerance_) {
            auto result = std::make_shared<FollowJointTrajectory::Result>();
            result->error_code = FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
            result->error_string = "Aborted due to goal_time_tolerance exceeding by " +
                                   std::to_string(elapsed - goal_time_tolerance_) + " seconds";
            RCLCPP_WARN(get_logger(), "%s", result->error_string.c_str());
            active_goal_->abort(result);
            has_active_goal_ = false;
            active_goal_.reset();
            goal_hold_count_ = 0;
            return;
        }
    }

    // 到位检查 (per-joint 容差)
    bool pos_ok = withinGoalConstraints(c, goal_target_);
    bool velocity_ok = true;
    double max_vel = 0;
    for (int i = 0; i < kNJoint; i++) {
        if (std::fabs(vel[i]) > max_vel) max_vel = std::fabs(vel[i]);
        if (stopped_velocity_tolerance_ > 0.0 &&
            std::fabs(vel[i]) > stopped_velocity_tolerance_) {
            velocity_ok = false;
        }
    }

    if (pos_ok && velocity_ok) {
        if (++goal_hold_count_ >= kGoalHoldRequired) {
            auto result = std::make_shared<FollowJointTrajectory::Result>();
            result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
            result->error_string = "Goal successfully reached!";
            RCLCPP_INFO(get_logger(),
                "GOAL_REACHED hold=%d elapsed=%.2fs max_vel=%.4frad/s pos_err=[%.4f %.4f %.4f %.4f %.4f %.4f]",
                goal_hold_count_, (get_clock()->now() - goal_hold_start_).seconds(), max_vel,
                std::fabs(c[0] - goal_target_point_.positions[0]),
                std::fabs(c[1] - goal_target_point_.positions[1]),
                std::fabs(c[2] - goal_target_point_.positions[2]),
                std::fabs(c[3] - goal_target_point_.positions[3]),
                std::fabs(c[4] - goal_target_point_.positions[4]),
                std::fabs(c[5] - goal_target_point_.positions[5]));
            active_goal_->succeed(result);
            has_active_goal_ = false;
            active_goal_.reset();
            goal_hold_count_ = 0;
        }
    } else {
        // 每 1 秒输出目标监控日志 (方便核查到位收敛过程)
        static int diag_cnt = 0;
        if (++diag_cnt >= 20) {  // 50ms×20 = 1s
            diag_cnt = 0;
            double max_err = 0; int max_j = 0;
            for (int i = 0; i < kNJoint; i++) {
                double err = std::fabs(c[i] - goal_target_point_.positions[i]);
                if (err > max_err) { max_err = err; max_j = i; }
            }
            RCLCPP_INFO(get_logger(),
                "GOAL_MON pos_ok=%d vel_ok=%d hold=%d/%d max_err=%.4f(j%d) max_vel=%.4f elapsed=%.1fs",
                pos_ok, velocity_ok, goal_hold_count_, kGoalHoldRequired,
                max_err, max_j, max_vel,
                (get_clock()->now() - goal_hold_start_).seconds());
        }
        goal_hold_count_ = 0;
    }
}

// ========== sendLoop (ROS1 publishWaypointToRobot 移植) ==========

void JointTrajectoryController::sendLoop()
{
    using namespace std::chrono;
    int rib = 0, ok = 0, fail = 0;
    int rib_peak = 0;          // RIB 峰值
    int rib_deplete = 0;       // RIB 耗尽连续计数
    int rib_high_count = 0;    // RIB≥350 (batch=1) 事件数
    int rib_warn_count = 0;    // RIB≥300 (batch=2) 事件数
    auto last_diag = steady_clock::now() - milliseconds(250);
    auto last_summary = steady_clock::now();
    auto traj_start = steady_clock::now();
    size_t traj_total = 0, traj_sent = 0;
    double ema = 0, ema_max = 0;
    double ema_sum = 0; int ema_samples = 0;
    RCLCPP_INFO(rclcpp::get_logger("sendLoop"), "sendLoop started");

    while (send_running_ && rclcpp::ok()) {
        // RIB 查询 (锁外 — sendLoop 独占查询)
        size_t avail;
        {
            std::lock_guard<std::mutex> lock(precomputed_mutex_);
            avail = (precomputed_idx_ < precomputed_.size())
                ? precomputed_.size() - precomputed_idx_ : 0;
            // 轨迹边界检测: 新轨迹开始发送
            if (avail > 0 && traj_total == 0) {
                traj_total = precomputed_.size();
                traj_sent = precomputed_idx_;
                traj_start = steady_clock::now();
                read_latency_.reset();  // 重置 SDK 延迟统计
                send_latency_.reset();
                RCLCPP_INFO(rclcpp::get_logger("sendLoop"),
                    "TRAJ_START total=%zu idx=%zu", traj_total, traj_sent);
            }
            // 轨迹结束
            if (avail == 0 && traj_total > 0 && traj_sent >= traj_total) {
                auto elapsed = duration_cast<milliseconds>(steady_clock::now() - traj_start).count();
                RCLCPP_INFO(rclcpp::get_logger("sendLoop"),
                    "TRAJ_DONE sent=%zu/%zu elapsed=%ldms fail=%d "
                    "ema_avg=%.1fms ema_max=%.1fms rib_peak=%d rib_hi=%d rib_warn=%d",
                    traj_sent, traj_total, elapsed, fail,
                    ema_samples > 0 ? ema_sum / ema_samples : 0,
                    ema_max, rib_peak, rib_high_count, rib_warn_count);
                // 输出 SDK TCP 延迟分布报告
                RCLCPP_INFO(rclcpp::get_logger("sendLoop"),
                    "%s", read_latency_.report("READ_DIAG").c_str());
                RCLCPP_INFO(rclcpp::get_logger("sendLoop"),
                    "%s", send_latency_.report("WRITE_PTS").c_str());
                traj_total = traj_sent = 0;
                rib_peak = 0; ema_max = 0; ema_sum = 0; ema_samples = 0;
                rib_high_count = 0; rib_warn_count = 0;
            }
        }

        auto now = steady_clock::now();
        int diag_iv = (avail > 0) ? 0 : 250;
        if (rib <= 0 || duration_cast<milliseconds>(now - last_diag).count() >= diag_iv) {
            auto t_diag0 = steady_clock::now();
            int r; if (hw_->readDiagnosis(r)) { rib = r; last_diag = now; }
            auto t_diag_el = duration_cast<microseconds>(steady_clock::now() - t_diag0);
            read_latency_.record(t_diag_el.count() / 1000.0);  // 记录 readDiagnosis TCP 延迟
        }

        // RIB 峰值追踪 + 告警 (带节流, 对齐标准 JTC RCLCPP_WARN_THROTTLE 模式)
        if (rib > rib_peak) rib_peak = rib;
        if (rib >= 350) {
            static int rib_hi_cnt = 0;
            if (++rib_hi_cnt % 20 == 1)  // 20 cycle 节流 (~100ms @200Hz)
                RCLCPP_WARN(rclcpp::get_logger("sendLoop"),
                    "RIB_HIGH rib=%d peak=%d — controller buffer near overflow (capacity~400)", rib, rib_peak);
        } else if (rib >= 250) {
            static int rib_warn_cnt = 0;
            if (++rib_warn_cnt % 50 == 1)  // 50 cycle 节流 (~250ms @200Hz)
                RCLCPP_WARN(rclcpp::get_logger("sendLoop"),
                    "RIB_WARN rib=%d peak=%d — buffer filling, possible send delay", rib, rib_peak);
        }

        // 自适应批量 (还原 ROS1 原始公式, 经 Fix1-Fix14 验证)
        int need = std::max(2, (int)std::ceil((400 - rib) / 6.0));
        if (ema > 10) need = std::max(need, 4);
        if (ema > 14) need = std::max(need, 6);
        if (ema > 20) need = std::max(need, 8);
        need = std::min(need, 8);

        // RIB≥300 最小批量持续发送 (不停不冷 — 参考 UR writeKeepalive)
        if (rib >= 350) { need = 1; rib_high_count++; }
        else if (rib >= 300) { need = std::min(need, 2); rib_warn_count++; }

        size_t n = std::min(avail, (size_t)need);

        // RIB 耗尽检测 (avail>0 但 rib==0 → 控制器空闲)
        if (avail > 0 && rib <= 0) {
            rib_deplete++;
            if (rib_deplete == 1)
                RCLCPP_INFO(rclcpp::get_logger("sendLoop"),
                    "RIB_IDLE avail=%zu rib=%d — controller buffer empty, start filling", avail, rib);
        } else {
            rib_deplete = 0;
        }

        if (n > 0) {
            // batch 提取 (锁内 — 最小化持锁时间)
            std::vector<aubo_robot_namespace::wayPoint_S> batch;
            {
                std::lock_guard<std::mutex> lock(precomputed_mutex_);
                n = std::min(n, precomputed_.size() - precomputed_idx_);
                if (n > 0) {
                    batch.assign(precomputed_.begin() + precomputed_idx_,
                                 precomputed_.begin() + precomputed_idx_ + n);
                    precomputed_idx_ += n;
                    traj_sent = precomputed_idx_;
                }
            }

            if (!batch.empty()) {
                auto t0 = steady_clock::now();
                if (hw_->writeTrajectoryPoints(batch)) {
                    auto el = duration_cast<microseconds>(steady_clock::now() - t0);
                    double ms = el.count() / 1000.0;
                    send_latency_.record(ms);  // 记录 writeTrajectoryPoints TCP 延迟
                    ema = (ema <= 0) ? ms : (0.9 * ema + 0.1 * ms);
                    if (ema > ema_max) ema_max = ema;
                    ema_sum += ms; ema_samples++;
                    ok++;

                    // 时序抖动告警: 单次发送延迟 > 50ms OR EMA > 30ms
                    if (ms > 50.0) {
                        RCLCPP_WARN(rclcpp::get_logger("sendLoop"),
                            "TIMING_SPIKE send=%.1fms ema=%.1fms batch=%zu rib=%d — possible TCP congestion",
                            ms, ema, n, rib);
                    } else if (ema > 30.0 && ok % 20 == 1) {
                        RCLCPP_WARN(rclcpp::get_logger("sendLoop"),
                            "TIMING_HIGH ema=%.1fms batch=%zu rib=%d — sustained send delay", ema, n, rib);
                    }

                    // 每 200 批或每 2 秒打印摘要
                    auto since_summary = duration_cast<milliseconds>(steady_clock::now() - last_summary).count();
                    if (ok % 200 == 0 || since_summary > 2000) {
                        last_summary = steady_clock::now();
                        RCLCPP_INFO(rclcpp::get_logger("sendLoop"),
                            "SEND_SUMMARY ok=%d fail=%d rib=%d peak=%d ema=%.1fms(avg=%.1f/max=%.1f) "
                            "idx=%zu/%zu batch=%zu need=%d",
                            ok, fail, rib, rib_peak, ema,
                            ema_samples > 0 ? ema_sum / ema_samples : 0, ema_max,
                            traj_sent, traj_total, n, need);
                    }
                } else {
                    fail++;
                    RCLCPP_WARN(rclcpp::get_logger("sendLoop"),
                        "SEND_FAIL #%d batch=%zu rib=%d — writeTrajectoryPoints failed", fail, n, rib);
                }
            }
        }

        // 自适应睡眠 (还原 ROS1 原始策略)
        if (avail > 40) std::this_thread::sleep_for(milliseconds(1));
        else if (rib < 200 && avail > 0) std::this_thread::sleep_for(milliseconds(1));
        else std::this_thread::sleep_for(milliseconds(4));
    }
    RCLCPP_INFO(rclcpp::get_logger("sendLoop"), "sendLoop exit (ok=%d fail=%d rib_peak=%d)", ok, fail, rib_peak);
}

// ========== 插值 + 状态机 ==========

void JointTrajectoryController::quinticInterpolate(
    const trajectory_msgs::msg::JointTrajectoryPoint& last,
    const trajectory_msgs::msg::JointTrajectoryPoint& curr,
    double t, double T, double p[6], double v[6], double a[6])
{
    auto pad = [](const auto& vec, size_t n){
        std::array<double,6> o{}; for(size_t i=0;i<std::min(vec.size(),n);i++) o[i]=vec[i]; return o; };
    auto pl=pad(last.positions,kNJoint), vl=pad(last.velocities,kNJoint), al=pad(last.accelerations,kNJoint);
    auto pc=pad(curr.positions,kNJoint), vc=pad(curr.velocities,kNJoint), ac=pad(curr.accelerations,kNJoint);
    double T2=T*T,T3=T2*T,T4=T3*T,T5=T4*T, t2=t*t,t3=t2*t,t4=t3*t,t5=t4*t;
    for (int i=0;i<kNJoint;i++){
        double a1=vl[i], a2=0.5*al[i], h=pc[i]-pl[i];
        double a3=0.5/T3*(20*h-(8*vc[i]+12*vl[i])*T-(3*al[i]-ac[i])*T2);
        double a4=0.5/T4*(-30*h+(14*vc[i]+16*vl[i])*T+(3*al[i]-2*ac[i])*T2);
        double a5=0.5/T5*(12*h-6*(vc[i]+vl[i])*T+(ac[i]-al[i])*T2);
        p[i]=pl[i]+a1*t+a2*t2+a3*t3+a4*t4+a5*t5;
        v[i]=a1+2*a2*t+3*a3*t2+4*a4*t3+5*a5*t4;
        a[i]=2*a2+6*a3*t+12*a4*t2+20*a5*t3;
    }
}

bool JointTrajectoryController::withinGoalConstraints(
    const double c[6], const trajectory_msgs::msg::JointTrajectory& t) const
{
    if (t.points.empty()) return false;
    auto& g = t.points.back();
    for (int i = 0; i < kNJoint && i < (int)g.positions.size(); i++) {
        double tol = (i < kNJoint) ? goal_tolerances_[i] : goal_tolerance_;
        if (tol <= 0.0) continue;  // 容差为 0 表示不检查该关节
        if (std::fabs(c[i] - g.positions[i]) > tol) return false;
    }
    return true;
}

void JointTrajectoryController::abortActiveGoal(int error_code)
{
    if (active_goal_) {
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_code = error_code;
        result->error_string = "Goal aborted";
        active_goal_->abort(result);
        active_goal_.reset();
    }
    has_active_goal_ = false;
    goal_hold_count_ = 0;
    {
        std::lock_guard<std::mutex> lock(precomputed_mutex_);
        precomputed_.clear();
        precomputed_idx_ = 0;
    }
}

void JointTrajectoryController::stopAndClear()
{
    // 不设置 send_running_=false！
    // sendLoop 检测到 precomputed_ 为空后自然进入等待状态 (avail==0 → sleep 4ms 循环)
    // 仅 deactivate() 中才设置 send_running_=false 并 join 线程
    {
        std::lock_guard<std::mutex> lock(precomputed_mutex_);
        precomputed_.clear();
        precomputed_idx_ = 0;
    }
    if (hw_ && hw_->isConnected()) {
        hw_->stopMotion();  // SDK 硬件 stop + 丢弃 RIB
    }
}

// ========== blendToFirstPoint: quintic smoothstep 过渡 (C² 连续) ==========

int JointTrajectoryController::blendToFirstPoint(
    const double current[6],
    const trajectory_msgs::msg::JointTrajectoryPoint& target,
    std::vector<aubo_robot_namespace::wayPoint_S>& out)
{
    // 距离 < 0.01 rad → 不需要过渡
    double max_diff = 0;
    for (int i = 0; i < kNJoint; i++)
        max_diff = std::max(max_diff, std::fabs(current[i] - target.positions[i]));
    if (max_diff < 0.01) return 0;

    const int kBlendSteps = 30;  // 150ms @200Hz (update_period_ = 5ms)
    for (int s = 0; s < kBlendSteps; s++) {
        double t = (s + 1.0) / kBlendSteps;
        double s_curve = 10 * t * t * t - 15 * t * t * t * t + 6 * t * t * t * t * t;  // C2 连续
        aubo_robot_namespace::wayPoint_S wp{};
        for (int i = 0; i < kNJoint; i++)
            wp.jointpos[i] = current[i] + (target.positions[i] - current[i]) * s_curve;
        out.push_back(wp);
    }
    return kBlendSteps;
}

// ========== readGoalTolerances: 从 goal 解析 per-joint 容差 (PR3) ==========

void JointTrajectoryController::readGoalTolerances(
    const std::shared_ptr<const FollowJointTrajectory::Goal>& goal)
{
    // 初始化默认值
    for (int i = 0; i < kNJoint; i++) goal_tolerances_[i] = goal_tolerance_;

    // 从 goal 覆盖 per-joint 容差
    for (const auto& tol : goal->goal_tolerance) {
        auto it = std::find(joint_names_.begin(), joint_names_.end(), tol.name);
        if (it != joint_names_.end()) {
            size_t idx = std::distance(joint_names_.begin(), it);
            if (tol.position > 0.0) goal_tolerances_[idx] = tol.position;
        }
    }

    // goal_time_tolerance: 0.0=未指定(无时间限制), -1.0=erase(无时间限制), >0=使用指定值
    // 对齐标准 JTC tolerances.hpp resolve_tolerance_source(): erase→0.0, unspecified→default
    double gtt = rclcpp::Duration(goal->goal_time_tolerance).seconds();
    if (gtt > 0.0) {
        goal_time_tolerance_ = gtt;  // 使用 goal 指定的值
    } else {
        goal_time_tolerance_ = 0.0;  // 无时间限制 (仅 >0 才触发超时检查)
    }

    RCLCPP_DEBUG(get_logger(), "Tolerances: goal_time=%.3f, per_joint=[%.4f %.4f %.4f %.4f %.4f %.4f]",
        goal_time_tolerance_, goal_tolerances_[0], goal_tolerances_[1], goal_tolerances_[2],
        goal_tolerances_[3], goal_tolerances_[4], goal_tolerances_[5]);
}

trajectory_msgs::msg::JointTrajectory
JointTrajectoryController::remapJointNames(const trajectory_msgs::msg::JointTrajectory& t) const
{
    trajectory_msgs::msg::JointTrajectory o=t;
    std::map<std::string,size_t> m;
    for (size_t i=0;i<t.joint_names.size();i++) m[t.joint_names[i]]=i;
    for (auto& pt:o.points){
        auto P=pt.positions, V=pt.velocities, A=pt.accelerations;
        pt.positions.resize(kNJoint,0); pt.velocities.resize(kNJoint,0); pt.accelerations.resize(kNJoint,0);
        for (size_t j=0;j<joint_names_.size()&&j<kNJoint;j++){
            auto it=m.find(joint_names_[j]);
            if (it!=m.end()&&it->second<P.size()){
                pt.positions[j]=P[it->second];
                if (it->second<V.size()) pt.velocities[j]=V[it->second];
                if (it->second<A.size()) pt.accelerations[j]=A[it->second];
            }
        }
    }
    o.joint_names=joint_names_;
    return o;
}

}  // namespace aubo_driver
