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

    feedback_pub_ = create_publisher<FollowJointTrajectory::Feedback>("aubo/feedback_states", 1000);

    update_timer_ = create_wall_timer(
        std::chrono::microseconds((int)(update_period_*1e6)),
        [this] { update(); }, update_cb_group_);

    send_running_ = true;
    send_thread_ = std::thread(&JointTrajectoryController::sendLoop, this);

    RCLCPP_INFO(get_logger(), "Configured (precompute + send thread)");
}

void JointTrajectoryController::deactivate()
{
    send_running_ = false;
    if (send_thread_.joinable()) send_thread_.join();
    if (has_active_goal_) abortActiveGoal();
    if (update_timer_) update_timer_->cancel();
}

rclcpp_action::GoalResponse JointTrajectoryController::handleGoal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const FollowJointTrajectory::Goal> goal)
{
    if (goal->trajectory.points.empty()) return rclcpp_action::GoalResponse::REJECT;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse JointTrajectoryController::handleCancel(
    const std::shared_ptr<GoalHandle> gh) { (void)gh; return rclcpp_action::CancelResponse::ACCEPT; }

void JointTrajectoryController::handleAccepted(const std::shared_ptr<GoalHandle> gh)
{
    if (has_active_goal_) { RCLCPP_WARN(get_logger(),"Preempting"); abortActiveGoal(); }
    active_goal_ = gh; has_active_goal_ = true;

    auto traj = remapJointNames(gh->get_goal()->trajectory);
    precomputed_.clear(); precomputed_idx_ = 0;
    double T = rclcpp::Duration(traj.points.back().time_from_start).seconds();
    int n = std::max(1, (int)std::round(T / update_period_));
    precomputed_.reserve(n);

    double t=0; size_t p=0;
    for (int s=0; s<n && p+1<traj.points.size(); s++, t+=update_period_) {
        while (p+1<traj.points.size() && t>rclcpp::Duration(traj.points[p+1].time_from_start).seconds()+1e-6) p++;
        if (p+1>=traj.points.size()) break;
        auto& L=traj.points[p], &C=traj.points[p+1];
        double seg = rclcpp::Duration(C.time_from_start).seconds()-rclcpp::Duration(L.time_from_start).seconds();
        if (seg<=0) seg=update_period_;
        double ts = std::max(0.0, t-rclcpp::Duration(L.time_from_start).seconds());
        double pos[6],vel[6],acc[6];
        quinticInterpolate(L,C,ts,seg,pos,vel,acc);
        aubo_robot_namespace::wayPoint_S wp{};
        for (int i=0;i<kNJoint;i++) wp.jointpos[i]=pos[i];
        precomputed_.push_back(wp);
    }
    goal_target_ = traj; goal_target_.points = {traj.points.back()};
    RCLCPP_INFO(get_logger(), "Accepted (%zu wp, %.2fs) → %zu pts", traj.points.size(), T, precomputed_.size());
}

void JointTrajectoryController::update()
{
    if (!has_active_goal_ || !hw_ || !hw_->isConnected()) return;
    static int sc=0;
    if (++sc>=50) { sc=0; bool e,p; hw_->readSafetyIOStatus(e,p);
        if(e||p){ RCLCPP_ERROR(get_logger(),"ESTOP"); abortActiveGoal(); return; } }

    if (precomputed_idx_ < precomputed_.size()) return;

    // 发完所有点后, 50ms 一次目标检查
    static int gc=0;
    if (++gc<10) return;
    gc=0;
    static int first=0;
    if (!first) { RCLCPP_INFO(get_logger(), "Goal checking started (idx=%zu/%zu)", precomputed_idx_, precomputed_.size()); first=1; }
    double c[6];
    if (hw_->readJointState(c) && withinGoalConstraints(c, goal_target_)) {
        if (++goal_hold_count_ >= kGoalHoldRequired) {
            RCLCPP_INFO(get_logger(), "Goal reached");
            active_goal_->succeed(std::make_shared<FollowJointTrajectory::Result>());
            has_active_goal_=false; active_goal_.reset();
        }
    } else { goal_hold_count_=0; }
}

// ========== sendLoop (ROS1 publishWaypointToRobot 移植) ==========

void JointTrajectoryController::sendLoop()
{
    using namespace std::chrono;
    int rib=0, ok=0, fail=0;
    auto last_diag = steady_clock::now() - milliseconds(250);
    double ema=0;
    RCLCPP_INFO(rclcpp::get_logger("sendLoop"), "sendLoop started");

    while (send_running_ && rclcpp::ok()) {
        size_t avail = (precomputed_idx_<precomputed_.size()) ? precomputed_.size()-precomputed_idx_ : 0;

        // RIB (ROS1: 降频查, RIB≤0 立即查)
        auto now=steady_clock::now();
        int diag_iv = (avail>0) ? 120 : 250;
        if (rib<=0 || duration_cast<milliseconds>(now-last_diag).count()>=diag_iv) {
            int r; if (hw_->readDiagnosis(r)) { rib=r; last_diag=now; }
        }

        // RIB≥300 等消费, 每4ms重查, 失败→设0不卡死
        if (rib>=300) {
            std::this_thread::sleep_for(milliseconds(4));
            int r; last_diag=steady_clock::now();
            rib = hw_->readDiagnosis(r) ? r : 0;
            continue;
        }

        // 自适应批量 (ROS1: ceil((400-rib)/6), min2 max8, EMA补偿)
        int need = std::max(2, (int)std::ceil((400-rib)/6.0));
        if (ema>10) need=std::max(need,4);
        if (ema>14) need=std::max(need,6);
        if (ema>20) need=std::max(need,8);
        need = std::min(need, 8);
        size_t n = std::min(avail, (size_t)need);

        if (n>0) {
            std::vector<aubo_robot_namespace::wayPoint_S> batch(precomputed_.begin()+precomputed_idx_, precomputed_.begin()+precomputed_idx_+n);
            auto t0=steady_clock::now();
            if (hw_->writeTrajectoryPoints(batch)) {
                auto el = duration_cast<microseconds>(steady_clock::now()-t0);
                double ms = el.count()/1000.0;
                ema = (ema<=0) ? ms : (0.9*ema + 0.1*ms);
                ok++; precomputed_idx_+=n;
                if (ok<=5 || ok%50==0)
                    RCLCPP_INFO(rclcpp::get_logger("sendLoop"), "send #%d (%zu pts, RIB=%d, idx=%zu/%zu, ema=%.1fms)", ok, n, rib, precomputed_idx_, precomputed_.size(), ema);
            } else {
                fail++; if (fail<=3) RCLCPP_WARN(rclcpp::get_logger("sendLoop"), "send FAIL #%d", fail);
            }
        }

        // 自适应睡眠 (ROS1)
        if (avail>40) std::this_thread::sleep_for(milliseconds(1));
        else if (rib<200 && avail>0) std::this_thread::sleep_for(milliseconds(1));
        else std::this_thread::sleep_for(milliseconds(4));
    }
    RCLCPP_INFO(rclcpp::get_logger("sendLoop"), "sendLoop exit (ok=%d fail=%d)", ok, fail);
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
    for (int i=0;i<kNJoint&&i<(int)g.positions.size();i++)
        if (std::fabs(c[i]-g.positions[i]) > goal_tolerance_) return false;
    return true;
}

void JointTrajectoryController::abortActiveGoal()
{
    if (active_goal_){ auto g=active_goal_; active_goal_.reset();
        g->abort(std::make_shared<FollowJointTrajectory::Result>()); }
    has_active_goal_=false; goal_hold_count_=0;
    precomputed_.clear(); precomputed_idx_=0;
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
