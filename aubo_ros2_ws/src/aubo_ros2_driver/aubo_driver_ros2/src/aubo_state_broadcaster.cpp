/*
 * aubo_state_broadcaster — 最优方案: 回调推送数据 + 保留必要的轮询。
 *
 * 数据源:
 *   RoadPointCallback (33Hz)      → joint_states, robot_status (关节角+笛卡尔位姿)
 *   JointStatusCallback (33Hz)    → aubo/feedback_states (编码器速度)
 *   readDiagnosis (50Hz 轮询)     → rib_status (RIB 缓冲量, SDK 无回调)
 *   readSafetyIOStatus (50Hz 轮询) → robot_status.enable (is_online + 安全状态)
 *   readFullIOStatus    (10Hz, 默认组串行化) → /robot_io_status (IO 状态持续发布)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <ivg_interfaces/msg/robot_status.hpp>
#include <ivg_interfaces/msg/robot_io_status.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <chrono>
#include <atomic>
#include <mutex>
#include <cmath>

#include "aubo_driver_ros2/aubo_hardware_interface.h"

namespace aubo_driver {

class AuboStateBroadcaster : public rclcpp::Node {
public:
    explicit AuboStateBroadcaster(
        std::shared_ptr<AuboHardwareInterface> hw,
        const rclcpp::NodeOptions& o = rclcpp::NodeOptions())
        : rclcpp::Node("aubo_state_broadcaster", o), hw_(std::move(hw))
    {
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "joint_states", 3000);
        feedback_pub_ = this->create_publisher<
            control_msgs::action::FollowJointTrajectory_Feedback>(
            "aubo/feedback_states", 1000);
        robot_status_pub_ = this->create_publisher<ivg_interfaces::msg::RobotStatus>(
            "/robot_status", 1000);
        rib_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "/aubo_driver/rib_status", 1000);
        robot_io_status_pub_ = this->create_publisher<ivg_interfaces::msg::RobotIOStatus>(
            "/robot_io_status", 1000);

        robot_status_msg_.header.frame_id = "base_link";
        rib_status_.data.resize(3);

        // ---- 注册回调 (替代 200Hz readJointState 轮询) ----
        hw_->registerCallbacks(
            nullptr,  // Event — 驱动内自行处理
            [this](const AuboHardwareInterface::JointFull& d) {
                onJointData(d);
            },
            [this](const aubo_robot_namespace::wayPoint_S& wp) {
                onWaypoint(wp);
            },
            nullptr   // EndSpeed — 暂不需要
        );

        // ---- 50Hz 保留轮询 (RIB + 安全 IO, SDK 无回调替代) ----
        poll_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&AuboStateBroadcaster::pollTick, this));

        // ---- 10Hz IO 状态轮询 (默认回调组, 与 pollTick 串行化避免 conn_status_ 并发) ----
        io_poll_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&AuboStateBroadcaster::pollIOTick, this));

        RCLCPP_INFO(this->get_logger(),
            "State broadcaster: callbacks+RIB(50Hz) + IO_status(10Hz)");
    }

private:
    // ============================================================
    // JointStatus 回调 (SDK 内部线程) — 编码器速度 + 跟踪误差
    // ============================================================
    void onJointData(const AuboHardwareInterface::JointFull& d) {
        std::lock_guard lk(fb_mux_);
        for (int i=0;i<6;i++) fb_pos_[i]=d.pos[i];
        for (int i=0;i<6;i++) fb_tgt_pos_[i]=d.tgt_pos[i];  // 目标位置 (jointTagPosJ)
        // 电机 RPM → 关节 rad/s
        for (int i=0;i<6;i++) fb_vel_[i]=d.vel[i]*kV2R[i];
        has_fb_=true;
    }

    // ============================================================
    // RoadPoint 回调 (SDK 内部线程) — 关节角+笛卡尔位姿
    // ============================================================
    void onWaypoint(const aubo_robot_namespace::wayPoint_S& wp) {
        // 关节角快照
        {
            std::lock_guard lk(js_mux_);
            for (int i=0;i<6;i++) js_pos_[i]=wp.jointpos[i];
            has_js_=true;
        }
        // 笛卡尔快照
        {
            std::lock_guard lk(rs_mux_);
            rs_x_=wp.cartPos.position.x;
            rs_y_=wp.cartPos.position.y;
            rs_z_=wp.cartPos.position.z;
            rs_w_=wp.orientation.w;
            rs_ox_=wp.orientation.x;
            rs_oy_=wp.orientation.y;
            rs_oz_=wp.orientation.z;
            has_rs_=true;
        }
    }

    // ============================================================
    // 50Hz 轮询 tick (RIB + 安全 IO)
    // ============================================================
    void pollTick() {
        // RIB 缓冲量
        int rib=0;
        if (hw_->readDiagnosis(rib)) {
            rib_status_.data[0]=rib;
            rib_status_.data[2]=1;
        } else {
            rib_status_.data[2]=0;
        }
        rib_pub_->publish(rib_status_);

        // 安全 IO + 连接状态 → enable（三者缺一不可）
        bool e_stop=false, p_stop=false;
        const bool online = hw_->isConnected();
        if (online && hw_->readSafetyIOStatus(e_stop, p_stop)) {
            robot_status_msg_.enable = !e_stop && !p_stop;
        } else {
            robot_status_msg_.enable = false;
        }

        // 发布 robot_status (用 RoadPoint 推送的最新笛卡尔)
        robot_status_msg_.header.stamp = this->now();
        robot_status_msg_.is_online = online;
        robot_status_msg_.in_motion = (rib>0);
        // planning_status 基于 RIB + 在线状态近似推导
        // 精确的 "planning"/"executing" 区分需要订阅 MoveIt action feedback (TODO)
        if (!online) {
            robot_status_msg_.planning_status = "idle";
        } else if (rib > 0) {
            robot_status_msg_.planning_status = "executing";
        } else {
            robot_status_msg_.planning_status = "idle";
        }
        { std::lock_guard lk(rs_mux_);
          if (has_rs_) {
              robot_status_msg_.cartesian_position_xyz.x=rs_x_;
              robot_status_msg_.cartesian_position_xyz.y=rs_y_;
              robot_status_msg_.cartesian_position_xyz.z=rs_z_;
              robot_status_msg_.cartesian_position.position.x=rs_x_;
              robot_status_msg_.cartesian_position.position.y=rs_y_;
              robot_status_msg_.cartesian_position.position.z=rs_z_;
              robot_status_msg_.cartesian_position.orientation.w=rs_w_;
              robot_status_msg_.cartesian_position.orientation.x=rs_ox_;
              robot_status_msg_.cartesian_position.orientation.y=rs_oy_;
              robot_status_msg_.cartesian_position.orientation.z=rs_oz_;
              const double w=rs_w_, x=rs_ox_, y=rs_oy_, z=rs_oz_;
              const double sinr_cosp = 2.0*(w*x + y*z);
              const double cosr_cosp = 1.0 - 2.0*(x*x + y*y);
              robot_status_msg_.cartesian_rpy.x = std::atan2(sinr_cosp, cosr_cosp);
              const double sinp = 2.0*(w*y - z*x);
              robot_status_msg_.cartesian_rpy.y = (std::abs(sinp)>=1.0) ? std::copysign(M_PI/2.0,sinp) : std::asin(sinp);
              const double siny_cosp = 2.0*(w*z + x*y);
              const double cosy_cosp = 1.0 - 2.0*(y*y + z*z);
              robot_status_msg_.cartesian_rpy.z = std::atan2(siny_cosp, cosy_cosp);
          } }
        { std::lock_guard lk(js_mux_);
          if (has_js_) for (int i=0;i<6;i++) {
              robot_status_msg_.joint_position_rad[i]=js_pos_[i];
              robot_status_msg_.joint_position_deg[i]=js_pos_[i]*180.0/M_PI;
          } }
        robot_status_pub_->publish(robot_status_msg_);

        // 发布 joint_states (用 RoadPoint 推送)
        {
            auto js=std::make_shared<sensor_msgs::msg::JointState>();
            js->header.stamp = this->now();
            js->name = names_;
            { std::lock_guard lk(js_mux_);
              if (has_js_) js->position.assign(js_pos_, js_pos_+6); }
            joint_state_pub_->publish(*js);
        }

        // 发布 feedback_states (用 JointStatus: 实际+目标+误差)
        {
            auto fb=std::make_shared<
                control_msgs::action::FollowJointTrajectory_Feedback>();
            fb->header.stamp = this->now();
            fb->joint_names = names_;
            { std::lock_guard lk(fb_mux_);
              if (has_fb_) {
                  fb->actual.positions.assign(fb_pos_,fb_pos_+6);
                  fb->actual.velocities.assign(fb_vel_,fb_vel_+6);
                  fb->desired.positions.assign(fb_tgt_pos_,fb_tgt_pos_+6);
                  // 跟踪误差 = desired - actual (rad)
                  double errors[6];
                  for (int i=0;i<6;i++) errors[i]=fb_tgt_pos_[i]-fb_pos_[i];
                  fb->error.positions.assign(errors,errors+6);
              } }
            feedback_pub_->publish(*fb);
        }
    }

    // ============================================================
    // 10Hz IO 状态轮询 (默认回调组, 与 pollTick 串行化, SDK conn_status_ 无并发风险)
    // ============================================================
    void pollIOTick() {
        ivg_interfaces::msg::RobotIOStatus io_msg;
        io_msg.header.stamp = this->now();
        if (hw_->readFullIOStatus(io_msg)) {
            robot_io_status_pub_->publish(io_msg);
        }
    }

    std::shared_ptr<AuboHardwareInterface> hw_;
    rclcpp::TimerBase::SharedPtr poll_timer_;
    rclcpp::TimerBase::SharedPtr io_poll_timer_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<control_msgs::action::FollowJointTrajectory_Feedback>::SharedPtr feedback_pub_;
    rclcpp::Publisher<ivg_interfaces::msg::RobotStatus>::SharedPtr robot_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr rib_pub_;
    rclcpp::Publisher<ivg_interfaces::msg::RobotIOStatus>::SharedPtr robot_io_status_pub_;

    ivg_interfaces::msg::RobotStatus robot_status_msg_;
    std_msgs::msg::Int32MultiArray rib_status_;

    // RoadPoint 快照
    double js_pos_[6]{}; bool has_js_{false}; std::mutex js_mux_;
    double rs_x_{},rs_y_{},rs_z_{},rs_w_{1},rs_ox_{},rs_oy_{},rs_oz_{}; bool has_rs_{false}; std::mutex rs_mux_;
    // JointStatus 快照
    double fb_pos_[6]{}, fb_tgt_pos_[6]{}, fb_vel_[6]{}; bool has_fb_{false}; std::mutex fb_mux_;

    static constexpr double kV2R[6] = {
        (2.0*M_PI/60.0)/121.0, (2.0*M_PI/60.0)/121.0, (2.0*M_PI/60.0)/121.0,
        (2.0*M_PI/60.0)/101.0, (2.0*M_PI/60.0)/101.0, (2.0*M_PI/60.0)/101.0};

    std::vector<std::string> names_{
        "shoulder_joint","upperArm_joint","foreArm_joint",
        "wrist1_joint","wrist2_joint","wrist3_joint"};
};

}  // namespace aubo_driver

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto hw = std::make_shared<aubo_driver::AuboHardwareInterface>();
    auto t = std::make_shared<rclcpp::Node>("_bc_params");
    t->declare_parameter<std::string>("server_host","169.254.10.98");
    t->declare_parameter<int>("server_port",8899);
    std::string host=t->get_parameter("server_host").as_string();
    int port=t->get_parameter("server_port").as_int(); t.reset();

    if (!hw->init(host,port)) {
        RCLCPP_FATAL(rclcpp::get_logger("state_bc"),"HW init failed");
        rclcpp::shutdown(); return 1;
    }

    rclcpp::NodeOptions o; o.automatically_declare_parameters_from_overrides(true);
    auto node=std::make_shared<aubo_driver::AuboStateBroadcaster>(hw,o);

    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(),2);
    exec.add_node(node); exec.spin();

    RCLCPP_INFO(rclcpp::get_logger("state_bc"),"Shutting down, leave TCP2CAN...");
    node.reset(); hw->shutdown(); hw.reset();
    rclcpp::shutdown();
}
