/*
 * aubo_callback_monitor — SDK 回调 vs 话题轮询 对比。
 *
 * 数据源:
 *   源A(回调): RealTimeJointStatusCallback → /aubo/callback_joint_states
 *   源B(轮询): 订阅 /joint_states (已有, ~100Hz)
 *   源C(差分): 从源B位置差分估算速度
 *
 * 对比项 (只比两者都有的):
 *   位置: 回调 jointPosJ vs 轮询 position
 *   速度: 回调 jointSpeedMoto×减速比 vs 轮询差分估算
 *
 * 回调独有 (源A有,源B无):
 *   电流 电压 温度 目标位置 目标速度 目标电流 错误码
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <atomic>
#include <mutex>
#include <cmath>

#include "aubo_driver_ros2/aubo_hardware_interface.h"

// AUBO E5 减速比 (来自 aubo_driver.h)
// 大关节 J0-J2: 谐波减速器 121:1
// 小关节 J3-J5: 谐波减速器 101:1
// jointSpeedMoto 是电机侧 RPM, 转关节 rad/s:  vel * (2π/60) / ratio
static constexpr double kV2R[6] = {
    (2.0*M_PI/60.0)/121.0, (2.0*M_PI/60.0)/121.0, (2.0*M_PI/60.0)/121.0,  // J0-J2 大关节
    (2.0*M_PI/60.0)/101.0, (2.0*M_PI/60.0)/101.0, (2.0*M_PI/60.0)/101.0,  // J3-J5 小关节
};

namespace aubo_driver {

struct DataStats {
    std::atomic<uint64_t> count{0};
    std::atomic<double>   last_ts{0.0};
    std::atomic<double>   min_iv{1e9};
    std::atomic<double>   max_iv{0.0};
    double                sum_iv{0.0};
    double                latest[6]{};
    std::mutex            mux;
};

class AuboCallbackMonitor : public rclcpp::Node {
public:
    AuboCallbackMonitor(const rclcpp::NodeOptions& o = rclcpp::NodeOptions())
        : rclcpp::Node("aubo_callback_monitor", o)
    {
        cb_pub_  = this->create_publisher<sensor_msgs::msg::JointState>(
            "/aubo/callback_joint_states", 100);
        diff_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/aubo/polled_diff_vel", 100);
        poll_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::QoS(3000),
            [this](sensor_msgs::msg::JointState::ConstSharedPtr m) { onPoll(m); });
        report_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&AuboCallbackMonitor::report, this));
    }

    bool init(const std::string& host, int port) {
        hw_ = std::make_unique<AuboHardwareInterface>();
        if (!hw_->init(host, port)) return false;
        hw_->registerCallbacks(
            [this](int t, int, const std::string&) {
                if (t == (int)aubo_robot_namespace::RobotEvent_socketDisconnected)
                    RCLCPP_ERROR(get_logger(), "[EVENT] socket断开");
                event_cnt_++;
            },
            [this](const AuboHardwareInterface::JointFull& d) {
                onCbJoint(d);
            },
            [this](const aubo_robot_namespace::wayPoint_S&) { wp_cnt_++; },
            [this](double s) { onCbSpeed(s); }
        );
        start_ = steady_now();
        RCLCPP_INFO(get_logger(), "已注册 JointStatus+RoadPoint+EndSpeed+Event");
        return true;
    }

private:
    // ================================================================
    // 回调记录 (SDK 内部线程)
    // ================================================================
    void onCbJoint(const AuboHardwareInterface::JointFull& d) {
        // 频率统计
        uint64_t n = cb_.count.fetch_add(1);
        double now = steady_now();
        for (int i = 0; i < 6; i++)
            if (std::isnan(d.pos[i]) || std::fabs(d.pos[i]) > 2*M_PI) return;
        double prev = cb_.last_ts.exchange(now);
        if (n > 0 && prev > 0.0) updateIv(cb_, now - prev);
        { std::lock_guard lk(cb_.mux); for (int i=0;i<6;i++) cb_.latest[i]=d.pos[i]; }

        // 保存完整字段
        { std::lock_guard lk(full_mux_); full_ = d; }

        // 发布 (降频: 每5帧发1帧)
        if (n % 5 == 0) {
            double jv[6];
            for (int i=0;i<6;i++) jv[i] = d.vel[i] * kV2R[i];  // 电机RPM → 关节rad/s
            auto m = std::make_shared<sensor_msgs::msg::JointState>();
            m->header.stamp = this->now();
            m->name = names_;
            m->position.assign(d.pos, d.pos+6);
            m->velocity.assign(jv, jv+6);
            cb_pub_->publish(*m);
        }
    }

    void onCbSpeed(double s) {
        uint64_t n = spd_.count.fetch_add(1);
        double now = steady_now();
        if (std::isnan(s) || s<0 || s>10) return;
        double prev = spd_.last_ts.exchange(now);
        if (n>0 && prev>0) updateIv(spd_, now-prev);
        { std::lock_guard lk(spd_.mux); spd_.latest[0]=s; }
    }

    // ================================================================
    // 轮询订阅 (ROS2 线程)
    // ================================================================
    void onPoll(sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        int n = std::min((int)msg->position.size(), 6);
        if (!n) return;
        const double* pos = msg->position.data();
        uint64_t c = poll_.count.fetch_add(1);
        double now = steady_now();
        double pv = poll_.last_ts.exchange(now);
        if (c>0 && pv>0) updateIv(poll_, now-pv);
        { std::lock_guard lk(poll_.mux); for (int i=0;i<n;i++) poll_.latest[i]=pos[i]; }

        // 差分速度: 每帧都算+存, 报告直接用最新值
        { std::lock_guard lk(dmux_);
          if (has_lp_) { double dt=now-lp_ts_; if (dt>0.001) for (int i=0;i<n;i++) diff_vel_[i]=(pos[i]-lp_pos_[i])/dt; }
          for (int i=0;i<n;i++) lp_pos_[i]=pos[i]; lp_ts_=now; has_lp_=true; }

        // 每10帧发布一次差分速度话题
        if (c%10==0) {
            auto m = std::make_shared<sensor_msgs::msg::JointState>();
            m->header.stamp = this->now(); m->name=names_;
            m->position.assign(pos,pos+n); m->velocity.assign(diff_vel_,diff_vel_+n);
            diff_pub_->publish(*m);
            uint64_t dc=diff_.count.fetch_add(1);
            double dp=diff_.last_ts.exchange(now);
            if (dc>0&&dp>0) updateIv(diff_,now-dp);
        }
    }

    // ================================================================
    // 对比报告 (10s)
    // ================================================================
    void report() {
        double t = steady_now()-start_;
        if (t<0.5) return;

        auto iv = [](DataStats& s){
            uint64_t c=s.count.load();
            if (c<2) return std::make_tuple(0.,0.,0.);
            std::lock_guard lk(s.mux);
            double avg=s.sum_iv/(c-1)*1000;
            return std::make_tuple(s.min_iv.load()*1000, s.max_iv.load()*1000, avg);
        };
        uint64_t cn=cb_.count.load(), pn=poll_.count.load(), dn=diff_.count.load();
        auto [cmin,cmax,cavg]=iv(cb_);
        auto [pmin,pmax,pavg]=iv(poll_);
        auto [dmin,dmax,davg]=iv(diff_);

        // 取最新数据
        double cp[6]{}, pp[6]{}, cv[6]{}, dv6[6]{};
        { std::lock_guard lk(cb_.mux);  for (int i=0;i<6;i++) cp[i]=cb_.latest[i]; }
        { std::lock_guard lk(poll_.mux);for (int i=0;i<6;i++) pp[i]=poll_.latest[i]; }
        { std::lock_guard lk(full_mux_);for (int i=0;i<6;i++) cv[i]=full_.vel[i]*kV2R[i]; }
        { std::lock_guard lk(dmux_);for (int i=0;i<6;i++) dv6[i]=diff_vel_[i]; }

        double pdiff=0, vdiff=0;
        for (int i=0;i<6;i++){ double d=std::fabs(cp[i]-pp[i]); if(d>pdiff)pdiff=d; }
        for (int i=0;i<6;i++){ double d=std::fabs(cv[i]-dv6[i]); if(d>vdiff)vdiff=d; }

        // === 格式: 频率对比 ===
        RCLCPP_INFO(get_logger(),"══════ %ds 频率对比 ────────────────────────",(int)t);
        RCLCPP_INFO(get_logger()," 源A [SDK回调] %lu帧 %.1fHz 间隔 %5.1f/%5.1f/%5.1f ms",
            cn,cn/t,cmin,cmax,cavg);
        RCLCPP_INFO(get_logger()," 源B [话题轮询] %lu帧 %.1fHz 间隔 %5.1f/%5.1f/%5.1f ms",
            pn,pn/t,pmin,pdiff,pavg);
        RCLCPP_INFO(get_logger()," 源C [差分速度] %lu帧 %.1fHz (每10帧发布1帧差值)",
            dn,dn/t);

        // === 格式: 位置对比 (两者都有) ===
        RCLCPP_INFO(get_logger(),"── 位置对比 (两者都有) ──");
        RCLCPP_INFO(get_logger()," 回调: %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f rad",
            cp[0],cp[1],cp[2],cp[3],cp[4],cp[5]);
        RCLCPP_INFO(get_logger()," 轮询: %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f rad",
            pp[0],pp[1],pp[2],pp[3],pp[4],pp[5]);
        RCLCPP_INFO(get_logger()," 差值: %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f rad  max=%.6f",
            cp[0]-pp[0],cp[1]-pp[1],cp[2]-pp[2],cp[3]-pp[3],cp[4]-pp[4],cp[5]-pp[5],pmax);

        // === 格式: 速度对比 (两者都有, 回调是转换后的真关节速度) ===
        RCLCPP_INFO(get_logger(),"── 速度对比 (两者都有) ──");
        RCLCPP_INFO(get_logger()," 回调(编码器→rad/s): %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",
            cv[0],cv[1],cv[2],cv[3],cv[4],cv[5]);
        RCLCPP_INFO(get_logger()," 轮询(差分估算):      %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",
            dv6[0],dv6[1],dv6[2],dv6[3],dv6[4],dv6[5]);
        RCLCPP_INFO(get_logger()," 差值:                %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f  max=%.3f",
            cv[0]-dv6[0],cv[1]-dv6[1],cv[2]-dv6[2],cv[3]-dv6[3],cv[4]-dv6[4],cv[5]-dv6[5],vdiff);

        // === 格式: 回调独有数据 ===
        RCLCPP_INFO(get_logger(),"── 回调独有 (轮询无) ──");
        { std::lock_guard lk(full_mux_); auto& d=full_;
        RCLCPP_INFO(get_logger()," 电流(mA):  %7.0f %7.0f %7.0f %7.0f %7.0f %7.0f",
            d.cur[0],d.cur[1],d.cur[2],d.cur[3],d.cur[4],d.cur[5]);
        RCLCPP_INFO(get_logger()," 温度(°C):  %7.1f %7.1f %7.1f %7.1f %7.1f %7.1f",
            d.temp[0],d.temp[1],d.temp[2],d.temp[3],d.temp[4],d.temp[5]);
        RCLCPP_INFO(get_logger()," 目标位置:  %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f",
            d.tgt_pos[0],d.tgt_pos[1],d.tgt_pos[2],d.tgt_pos[3],d.tgt_pos[4],d.tgt_pos[5]);
        RCLCPP_INFO(get_logger()," 错误码:    %7d %7d %7d %7d %7d %7d",
            d.err[0],d.err[1],d.err[2],d.err[3],d.err[4],d.err[5]); }

        RCLCPP_INFO(get_logger(),"其他: EndSpeed=%lu RoadPoint=%lu Event=%lu",
            spd_.count.load(),wp_cnt_.load(),event_cnt_.load());

        // reset min/max
        cb_.min_iv=1e9;cb_.max_iv=0;poll_.min_iv=1e9;poll_.max_iv=0;
        diff_.min_iv=1e9;diff_.max_iv=0;spd_.min_iv=1e9;spd_.max_iv=0;
    }

    // ================================================================
    // 辅助
    // ================================================================
    static double steady_now(){
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    void updateIv(DataStats& s, double v){
        v=std::max(0.0001,std::min(v,10.));
        double o=s.min_iv.load(); while(v<o&&!s.min_iv.compare_exchange_weak(o,v)){}
        o=s.max_iv.load(); while(v>o&&!s.max_iv.compare_exchange_weak(o,v)){}
        {std::lock_guard lk(s.mux);s.sum_iv+=v;}
    }

    std::unique_ptr<AuboHardwareInterface> hw_; double start_{0};
    DataStats cb_, poll_, diff_, spd_;
    std::atomic<uint64_t> wp_cnt_{0}, event_cnt_{0};
    AuboHardwareInterface::JointFull full_; std::mutex full_mux_;
    double lp_pos_[6]{}, lp_ts_{0}; bool has_lp_{false};
    double diff_vel_[6]{}; std::mutex dmux_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cb_pub_, diff_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr poll_sub_;
    rclcpp::TimerBase::SharedPtr report_timer_;
    std::vector<std::string> names_{
        "shoulder_joint","upperArm_joint","foreArm_joint",
        "wrist1_joint","wrist2_joint","wrist3_joint"};
};

}  // namespace aubo_driver

int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    auto t=std::make_shared<rclcpp::Node>("_mp");
    t->declare_parameter<std::string>("server_host","169.254.10.98");
    t->declare_parameter<int>("server_port",8899);
    std::string h=t->get_parameter("server_host").as_string();
    int p=t->get_parameter("server_port").as_int(); t.reset();
    rclcpp::NodeOptions o; o.automatically_declare_parameters_from_overrides(true);
    auto m=std::make_shared<aubo_driver::AuboCallbackMonitor>(o);
    if(!m->init(h,p)){rclcpp::shutdown();return 1;}
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(m); exec.spin();
    RCLCPP_INFO(rclcpp::get_logger("monitor"),"Shutting down, leaving TCP2CAN...");
    m.reset(); rclcpp::shutdown();
}
