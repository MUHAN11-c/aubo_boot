/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017-2018, AUBO Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * (same as ROS1 aubo_driver - ported to ROS2)
 */

#include "aubo_driver_ros2/aubo_driver.h"
#include <sys/time.h>
#include <time.h>
#include <stdio.h>
#include <cstdlib>
#include <algorithm>
#include <chrono>
#include <atomic>
#include <functional>
#include <mutex>
#include <fstream>
#include <cctype>

namespace aubo_driver {

static std::atomic<int64_t> g_last_moveit_cb_steady_ms{0};

double MaxAcc[ARM_DOF] = {17.30878, 17.30878, 17.30878, 20.73676, 20.73676, 20.73676};
double MaxVelc[ARM_DOF] = {2.596177, 2.596177, 2.596177, 3.110177, 3.110177, 3.110177};

std::string AuboDriver::joint_name_[ARM_DOF] = {"shoulder_joint","upperArm_joint","foreArm_joint","wrist1_joint","wrist2_joint","wrist3_joint"};

std::ofstream file;
std::ofstream file_v;
char *t1[128] = {0};

static double time_from_start_to_sec(const builtin_interfaces::msg::Duration& t) {
    return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}

static void fillCartesianPoseAndRpy(demo_interface::msg::RobotStatus& msg,
                                    const aubo_robot_namespace::wayPoint_S& wp)
{
    msg.cartesian_position.position.x = wp.cartPos.position.x;
    msg.cartesian_position.position.y = wp.cartPos.position.y;
    msg.cartesian_position.position.z = wp.cartPos.position.z;

    const double w = wp.orientation.w;
    const double x = wp.orientation.x;
    const double y = wp.orientation.y;
    const double z = wp.orientation.z;
    msg.cartesian_position.orientation.w = w;
    msg.cartesian_position.orientation.x = x;
    msg.cartesian_position.orientation.y = y;
    msg.cartesian_position.orientation.z = z;

    msg.cartesian_position_xyz.x = msg.cartesian_position.position.x;
    msg.cartesian_position_xyz.y = msg.cartesian_position.position.y;
    msg.cartesian_position_xyz.z = msg.cartesian_position.position.z;

    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    const double roll = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp = 2.0 * (w * y - z * x);
    const double pitch = (std::abs(sinp) >= 1.0) ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);

    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    const double yaw = std::atan2(siny_cosp, cosy_cosp);

    msg.cartesian_rpy.x = roll;
    msg.cartesian_rpy.y = pitch;
    msg.cartesian_rpy.z = yaw;
}

AuboDriver::AuboDriver(int num)
  : rclcpp::Node("aubo_driver"),
    delay_clear_times(0), buffer_size_(400), io_flag_delay_(0.02), data_recieved_(false), data_count_(0),
    real_robot_exist_(false), emergency_stopped_(false), protective_stopped_(false), normal_stopped_(false),
    controller_connected_flag_(false), start_move_(false), control_mode_(aubo_driver::SendTargetGoal),
    rib_buffer_size_(0), jti(ARM_DOF, 1.0/200), jto(ARM_DOF), collision_class_(6),
    over_speed_flag_(false), last_time_from_start_(-1.0), joint_feedback_publish_thread_(nullptr)
{
    this->declare_parameter<int>("external_axis_number", num);
    int ext = this->get_parameter("external_axis_number").as_int();
    axis_number_ = 6 + ext;
    RCLCPP_INFO(this->get_logger(), "aubo_driver/external_axis_number: %d", ext);
    for(int i = 0; i < axis_number_; i++) {
        current_joints_[i] = 0;
        target_point_[i] = 0;
        if(i < 3) joint_ratio_[i] = BIG_MODULE_RATIO;
        else if(i < 6) joint_ratio_[i] = SMALL_MODULE_RATIO;
        else joint_ratio_[i] = 2 * M_PI / 10.05309632;
        jti.maxVelocity[i] = VMAX * joint_ratio_[i];
        jti.maxAcceleration[i] = AMAX * joint_ratio_[i];
        jti.maxJerk[i] = JMAX * joint_ratio_[i];
    }
    rs.robot_controller_ = ROBOT_CONTROLLER;
    rib_status_.data.resize(3);
    waypoint_vector_.clear();
    robot_status_msg_.header.frame_id = "base_link";

    joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 3000);
    joint_feedback_pub_ = this->create_publisher<control_msgs::action::FollowJointTrajectory_Feedback>("aubo/feedback_states", 1000);
    joint_target_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/aubo_driver/real_pose", 500);
    robot_status_pub_ = this->create_publisher<demo_interface::msg::RobotStatus>("/aubo_driver/robot_status", 1000);
    rib_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/aubo_driver/rib_status", 1000);
    cancle_trajectory_pub_ = this->create_publisher<std_msgs::msg::UInt8>("aubo_driver/cancel_trajectory", 100);
    
    io_pub_ = this->create_publisher<demo_interface::msg::RobotIOStatus>("/aubo_driver/io_states", 10);
    // 以功能域拆分 callback group：轨迹、500Hz 控制、50Hz 状态、服务、控制命令互相隔离
    trajectory_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    update_control_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    state_timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_cmd_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    io_srv_ = this->create_service<demo_interface::srv::SetRobotIO>(
        "/aubo_driver/set_io",
        std::bind(&AuboDriver::setIO, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_cb_group_);
    ik_srv_ = this->create_service<aubo_msgs::srv::GetIK>(
        "/aubo_driver/get_ik",
        std::bind(&AuboDriver::getIK, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_cb_group_);
    fk_srv_ = this->create_service<aubo_msgs::srv::GetFK>(
        "/aubo_driver/get_fk",
        std::bind(&AuboDriver::getFK, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_cb_group_);

    rclcpp::SubscriptionOptions control_sub_opts;
    control_sub_opts.callback_group = control_cmd_cb_group_;
    trajectory_execution_subs_ = this->create_subscription<std_msgs::msg::String>(
        "trajectory_execution_event", 10, std::bind(&AuboDriver::trajectoryExecutionCallback, this, std::placeholders::_1), control_sub_opts);
    robot_control_subs_ = this->create_subscription<std_msgs::msg::String>(
        "robot_control", 10, std::bind(&AuboDriver::robotControlCallback, this, std::placeholders::_1), control_sub_opts);
    // moveItController_cmd 与 timer 并行即可；订阅回调自身必须串行，避免 time_from_start 顺序被并发重入打乱
    rclcpp::SubscriptionOptions moveit_sub_opts;
    moveit_sub_opts.callback_group = trajectory_cb_group_;
    moveit_controller_subs_ = this->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
        "moveItController_cmd",
        rclcpp::QoS(20000),
        std::bind(&AuboDriver::moveItPosCallback, this, std::placeholders::_1),
        moveit_sub_opts);
    teach_subs_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "teach_cmd", 10, std::bind(&AuboDriver::teachCallback, this, std::placeholders::_1), control_sub_opts);
    moveAPI_subs_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "moveAPI_cmd", 10, std::bind(&AuboDriver::AuboAPICallback, this, std::placeholders::_1), control_sub_opts);
    controller_switch_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/aubo_driver/controller_switch", 10, std::bind(&AuboDriver::controllerSwitchCallback, this, std::placeholders::_1), control_sub_opts);

    std::string file_name = "/tmp/aubo_driver_ros2_jointpose.csv";
    remove(file_name.c_str());
    file.open(file_name, std::ios::out);
    std::string file_name_v = "/tmp/aubo_driver_ros2_jointpf.csv";
    remove(file_name_v.c_str());
    file_v.open(file_name_v, std::ios::out);

    send_to_robot_thread_ = new std::thread(&AuboDriver::publishWaypointToRobot, this);
}

AuboDriver::~AuboDriver()
{
    publish_thread_running_ = false;
    if (feed_to_ros_motion_thread_ && feed_to_ros_motion_thread_->joinable())
        feed_to_ros_motion_thread_->join();
    delete feed_to_ros_motion_thread_;
    feed_to_ros_motion_thread_ = nullptr;
    if (joint_feedback_publish_thread_ && joint_feedback_publish_thread_->joinable())
        joint_feedback_publish_thread_->join();
    delete joint_feedback_publish_thread_;
    joint_feedback_publish_thread_ = nullptr;
    if(control_option_ == aubo_driver::RosMoveIt)
        robot_send_service_.robotServiceLeaveTcp2CanbusMode();
    robot_send_service_.robotServiceLogout();
    robot_receive_service_.robotServiceLogout();
}

void AuboDriver::timerCallback()
{
    static auto last_timer_ts = std::chrono::steady_clock::now();
    static auto last_diag_poll_ts = std::chrono::steady_clock::now() - std::chrono::milliseconds(500);
    const auto timer_now = std::chrono::steady_clock::now();
    const double timer_delta_ms = std::chrono::duration<double, std::milli>(timer_now - last_timer_ts).count();
    last_timer_ts = timer_now;
    double sampled_waypoint_ms = -1.0;
    double sampled_diag_ms = -1.0;
    bool sampled_do_sdk_poll = false;
    static uint32_t waypoint_skip_counter = 0;
    if(controller_connected_flag_) {
        const bool do_sdk_poll = !start_move_ || ((++waypoint_skip_counter) % 5 == 0);
        sampled_do_sdk_poll = do_sdk_poll;
        if (do_sdk_poll) {
            const auto waypoint_t0 = std::chrono::steady_clock::now();
            int ret = robot_receive_service_.robotServiceGetCurrentWaypointInfo(rs.wayPoint_);
            const double waypoint_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - waypoint_t0).count();
            sampled_waypoint_ms = waypoint_ms;
            if(ret == aubo_robot_namespace::InterfaceCallSuccCode) {
                double joints[ARM_DOF];
                for(int i = 0; i < 6; i++) joints[i] = rs.wayPoint_.jointpos[i];
                setCurrentPosition(joints);
                const bool should_poll_diag = real_robot_exist_ && !start_move_;
                const auto diag_gap_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    timer_now - last_diag_poll_ts).count();
                const bool diag_due = (diag_gap_ms >= 200);
                if(should_poll_diag && diag_due) {
                    const auto diag_t0 = std::chrono::steady_clock::now();
                    robot_receive_service_.robotServiceGetRobotDiagnosisInfo(rs.robot_diagnosis_info_);
                    const double diag_ms = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - diag_t0).count();
                    sampled_diag_ms = diag_ms;
                    last_diag_poll_ts = timer_now;
                    rib_buffer_size_ = rs.robot_diagnosis_info_.macTargetPosDataSize;
                }
            } else if(ret == aubo_robot_namespace::ErrCode_SocketDisconnect) {
                if(!connectToRobotController())
                    RCLCPP_ERROR(this->get_logger(), "Cannot connect to the robot controller!");
            }
        }
        // 状态字段每周期更新（50Hz），不随 do_sdk_poll 跳过
        robot_status_msg_.header.stamp = this->now();
        robot_status_msg_.is_online = controller_connected_flag_;
        robot_status_msg_.enable = (real_robot_exist_ && rs.robot_diagnosis_info_.armPowerStatus != 0);
        robot_status_msg_.in_motion = start_move_;
        // planning_status 仅反映 MoveIt2 规划状态，由 trajectory_execution_event 驱动
        switch (planning_status_code_.load(std::memory_order_relaxed)) {
            case 1: robot_status_msg_.planning_status = "planning"; break;
            case 2: robot_status_msg_.planning_status = "executing"; break;
            case 3: robot_status_msg_.planning_status = "error"; break;
            default: robot_status_msg_.planning_status = "idle"; break;
        }
        for(int i = 0; i < 6 && i < axis_number_; i++) {
            robot_status_msg_.joint_position_rad[i] = current_joints_[i];
            robot_status_msg_.joint_position_deg[i] = current_joints_[i] * 180.0 / M_PI;
        }
        fillCartesianPoseAndRpy(robot_status_msg_, rs.wayPoint_);
        { std::lock_guard<std::mutex> lock(buf_queue_mutex_); rib_status_.data[0] = static_cast<int32_t>(buf_queue_.size()); }
        rib_status_.data[1] = control_mode_;
        rib_status_.data[2] = controller_connected_flag_ ? 1 : 0;
    } else {
        setCurrentPosition(target_point_);
        robot_status_msg_.is_online = false;
        robot_status_msg_.enable = false;
        robot_status_msg_.in_motion = false;
        robot_status_msg_.planning_status = "idle";
        robot_status_msg_.header.stamp = this->now();
        for (int i = 0; i < 6; ++i) {
            robot_status_msg_.joint_position_rad[i] = current_joints_[i];
            robot_status_msg_.joint_position_deg[i] = current_joints_[i] * 180.0 / M_PI;
        }
        robot_status_msg_.cartesian_position_xyz.x = 0.0;
        robot_status_msg_.cartesian_position_xyz.y = 0.0;
        robot_status_msg_.cartesian_position_xyz.z = 0.0;
        robot_status_msg_.cartesian_rpy.x = 0.0;
        robot_status_msg_.cartesian_rpy.y = 0.0;
        robot_status_msg_.cartesian_rpy.z = 0.0;
        robot_status_msg_.cartesian_position.position.x = 0.0;
        robot_status_msg_.cartesian_position.position.y = 0.0;
        robot_status_msg_.cartesian_position.position.z = 0.0;
        robot_status_msg_.cartesian_position.orientation.w = 1.0;
        robot_status_msg_.cartesian_position.orientation.x = 0.0;
        robot_status_msg_.cartesian_position.orientation.y = 0.0;
        robot_status_msg_.cartesian_position.orientation.z = 0.0;
        rib_status_.data[0] = 0;
        rib_status_.data[1] = control_mode_;
        rib_status_.data[2] = 0;
    }
    robot_status_pub_->publish(robot_status_msg_);
    rib_pub_->publish(rib_status_);

    if(control_mode_ == aubo_driver::SynchronizeWithRealRobot) {
        if(controller_connected_flag_) {
            memcpy(last_recieve_point_, current_joints_, sizeof(double) * axis_number_);
            memcpy(target_point_, current_joints_, sizeof(double) * axis_number_);
        } else {
            RCLCPP_INFO(this->get_logger(), "No connection to robot controller!");
        }
    } else if(control_mode_ == aubo_driver::SendTargetGoal) {
        if(control_option_ == aubo_driver::AuboAPI) {
            {
                std::lock_guard<std::mutex> lock(joints_mutex_);
                memcpy(target_point_, current_joints_, sizeof(double) * axis_number_);
            }
            auto joints = std::make_shared<std_msgs::msg::Float32MultiArray>();
            joints->data.resize(axis_number_);
            for(int i = 0; i < axis_number_; i++) joints->data[i] = static_cast<float>(target_point_[i]);
            joint_target_pub_->publish(*joints);
        }
    }
}

void AuboDriver::publishJointStateAndFeedbackLoop()
{
    const auto period = std::chrono::milliseconds(1000 / PUBLISH_RATE_);
    static bool has_prev = false;
    static double prev_pos[ARM_DOF] = {0.0};
    while (publish_thread_running_ && rclcpp::ok()) {
        auto loop_start = std::chrono::steady_clock::now();
        if (control_mode_ == aubo_driver::SendTargetGoal) {
            int axis = 0;
            double cur[ARM_DOF], tgt[ARM_DOF];
            bool conn = false;
            {
                std::lock_guard<std::mutex> lock(joints_mutex_);
                axis = axis_number_;
                conn = controller_connected_flag_;
                for (int i = 0; i < axis && i < ARM_DOF; i++) {
                    cur[i] = current_joints_[i];
                    tgt[i] = target_point_[i];
                }
            }
            sensor_msgs::msg::JointState joint_state;
            control_msgs::action::FollowJointTrajectory_Feedback joint_feedback;
            joint_state.header.stamp = this->now();
            joint_state.name.resize(axis);
            joint_feedback.joint_names.resize(axis);
            joint_state.position.resize(axis);
            joint_feedback.actual.positions.resize(axis);
            for (int i = 0; i < axis; i++) {
                joint_state.name[i] = joint_name_[i];
                joint_state.position[i] = conn ? cur[i] : tgt[i];
                joint_feedback.joint_names[i] = joint_name_[i];
                joint_feedback.actual.positions[i] = joint_state.position[i];
            }
            if (axis >= 6) {
                double max_jump = 0.0;
                int jump_joint = -1;
                if (has_prev) {
                    for (int i = 0; i < 6; i++) {
                        double d = std::fabs(joint_state.position[i] - prev_pos[i]);
                        if (d > max_jump) {
                            max_jump = d;
                            jump_joint = i;
                        }
                    }
                }
                (void)max_jump;
                (void)jump_joint;
                for (int i = 0; i < 6; i++) prev_pos[i] = joint_state.position[i];
                has_prev = true;
            }
            joint_states_pub_->publish(joint_state);
            joint_feedback.header.stamp = this->now();
            joint_feedback_pub_->publish(joint_feedback);
        }
        std::this_thread::sleep_until(loop_start + period);
    }
}

bool AuboDriver::roadPointCompare(double *point1, double *point2) {
    for(int i = 0; i < axis_number_; i++)
        if(fabs(point1[i] - point2[i]) >= THRESHHOLD) return true;
    return false;
}

double* AuboDriver::getCurrentPosition() { return current_joints_; }
void AuboDriver::setCurrentPosition(double *target) {
    std::lock_guard<std::mutex> lock(joints_mutex_);
    for(int i = 0; i < axis_number_; i++) current_joints_[i] = target[i];
}
double* AuboDriver::getTagrtPosition() { return target_point_; }
void AuboDriver::setTagrtPosition(double *target) {
    std::lock_guard<std::mutex> lock(joints_mutex_);
    for(int i = 0; i < axis_number_; i++) target_point_[i] = target[i];
}

bool AuboDriver::setRobotJointsByMoveIt()
{
    std::vector<PlanningState> ps_batch;
    {
        std::lock_guard<std::mutex> lock(buf_queue_mutex_);
        if(buf_queue_.empty()) {
            return false;
        }
        const size_t kMaxBufPopPerCycle = 8;
        ps_batch.reserve(kMaxBufPopPerCycle);
        while(!buf_queue_.empty() && ps_batch.size() < kMaxBufPopPerCycle) {
            ps_batch.push_back(buf_queue_.front());
            buf_queue_.pop();
        }
        buf_queue_size_.store(buf_queue_.size());
    }
    const PlanningState &ps = ps_batch.front();
    if(controller_connected_flag_) {
        if(emergency_stopped_) {
            start_move_ = false;
            clearBufQueue();
            return true;
        } else if(protective_stopped_ || normal_stopped_) {
            auto cancle = std::make_shared<std_msgs::msg::UInt8>();
            cancle->data = 1;
            cancle_trajectory_pub_->publish(*cancle);
            memcpy(&jti.currentPosition[0], ps.joint_pos_, axis_number_*sizeof(double));
            memcpy(&jti.currentVelocity[0], ps.joint_vel_, axis_number_*sizeof(double));
            memcpy(&jti.currentAcceleration[0], ps.joint_acc_, axis_number_*sizeof(double));
            memset(&jti.targetVelocity[0], 0, axis_number_*sizeof(double));
            otgVelocityModeParameterUpdate(jti);
            int resultValue = 0;
            while(resultValue != 1) {
                resultValue = otgVelocityModeResult(1, jto);
                double jointAngle[] = {jto.newPosition[0],jto.newPosition[1],jto.newPosition[2],jto.newPosition[3],jto.newPosition[4],jto.newPosition[5]};
                robot_send_service_.robotServiceSetRobotPosData2Canbus(jointAngle);
            }
            start_move_ = false;
            clearBufQueue();
            if(normal_stopped_) { normal_stopped_ = false; delay_clear_times = STOP_DELAY_CLEAR_TIMES; }
        } else {
            for (const auto &st : ps_batch) {
                JointPosWithTime jpt;
                for(int i = 0; i < 6; i++) jpt.joint_pos[i] = st.joint_pos_[i];
                jpt.time_from_start = st.time_from_start_;
                jpt.trajectory_epoch = st.trajectory_epoch_;
                ros_motion_queue_.enqueue(jpt);
            }
            file << ps.joint_pos_[0] << "," << ps.joint_pos_[1] << "," << ps.joint_pos_[2] << "," << ps.joint_pos_[3] << "," << ps.joint_pos_[4] << "," << ps.joint_pos_[5] << std::endl;
        }
    }
    setTagrtPosition(ps_batch.back().joint_pos_);
    return true;
}

void AuboDriver::controllerSwitchCallback(const std_msgs::msg::Int32::ConstSharedPtr msg)
{
    int controller_type = msg->data;
    if(controller_type == control_option_) {
        RCLCPP_INFO(this->get_logger(), "The controller type is: %s", (control_option_ == aubo_driver::AuboAPI) ? "robot-controller" : "ros-controller");
        return;
    }
    if(controller_type == aubo_driver::AuboAPI) {
        if(start_move_) {
            RCLCPP_WARN(this->get_logger(), "Try to switch after robot stops!");
            return;
        }
        int ret = robot_send_service_.robotServiceLeaveTcp2CanbusMode();
        if(ret == aubo_robot_namespace::InterfaceCallSuccCode) {
            RCLCPP_INFO(this->get_logger(), "Switches to robot-controller successfully");
            control_option_ = aubo_driver::AuboAPI;
        } else
            RCLCPP_INFO(this->get_logger(), "Failed to switch to robot-controller");
    } else if(controller_type == aubo_driver::RosMoveIt) {
        int ret = robot_send_service_.robotServiceEnterTcp2CanbusMode();
        if(ret == aubo_robot_namespace::InterfaceCallSuccCode) {
            RCLCPP_INFO(this->get_logger(), "Switches to ros-controller successfully");
            control_option_ = aubo_driver::RosMoveIt;
        } else
            RCLCPP_INFO(this->get_logger(), "Failed to switch to ros-controller.");
    } else
        RCLCPP_INFO(this->get_logger(), "Undefined controller type!");
}

void AuboDriver::moveItPosCallback(const trajectory_msgs::msg::JointTrajectoryPoint::ConstSharedPtr msg)
{
    const auto moveit_cb_t0 = std::chrono::steady_clock::now();
    const auto now_steady_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        moveit_cb_t0.time_since_epoch()).count();
    g_last_moveit_cb_steady_ms.store(now_steady_ms, std::memory_order_relaxed);
    std::lock_guard<std::mutex> cb_lock(moveit_cb_mutex_);
    double jointAngle[ARM_DOF];
    for(int i = 0; i < axis_number_; i++) jointAngle[i] = msg->positions[i];
    if(controller_connected_flag_) {
        if(roadPointCompare(jointAngle, last_recieve_point_)) {
            PlanningState ps;
            memcpy(ps.joint_pos_, jointAngle, sizeof(double) * axis_number_);
            if(msg->velocities.size() >= static_cast<size_t>(axis_number_))
                memcpy(ps.joint_vel_, msg->velocities.data(), sizeof(double) * axis_number_);
            if(msg->accelerations.size() >= static_cast<size_t>(axis_number_))
                memcpy(ps.joint_acc_, msg->accelerations.data(), sizeof(double) * axis_number_);
            ps.time_from_start_ = time_from_start_to_sec(msg->time_from_start);
            const bool new_trajectory =
                (ps.time_from_start_ <= 1e-9) ||
                (last_received_time_from_start_ >= 0.0 && ps.time_from_start_ < last_received_time_from_start_ - 1e-9);
            const uint64_t trajectory_epoch = new_trajectory
                ? (current_trajectory_epoch_.fetch_add(1, std::memory_order_relaxed) + 1)
                : current_trajectory_epoch_.load(std::memory_order_relaxed);
            ps.trajectory_epoch_ = trajectory_epoch;
            last_received_time_from_start_ = ps.time_from_start_;
            memcpy(last_recieve_point_, jointAngle, sizeof(double) * axis_number_);
            {
                std::lock_guard<std::mutex> lock(buf_queue_mutex_);
                if (new_trajectory) {
                    while (!buf_queue_.empty()) buf_queue_.pop();
                    buf_queue_size_.store(0);
                }
                buf_queue_.push(ps);
                size_t qsize = buf_queue_.size();
                buf_queue_size_.store(qsize);
                if (qsize == 1u)
                {
                    data_count_.store(0);  // 首点到达：对齐 50 次计数
                }
                if (new_trajectory)
                {
                    last_time_from_start_ = -1.0;
                    std::memset(last_joint_velc_.jointPara, 0, sizeof(last_joint_velc_.jointPara));
                }
                if(qsize > static_cast<size_t>(buffer_size_) && !start_move_ && delay_clear_times == 0) {
                    start_move_ = true;
                }
            }
        }
    } else {
        setTagrtPosition(jointAngle);
        rib_buffer_size_ = 0;
    }
}

void AuboDriver::trajectoryExecutionCallback(const std_msgs::msg::String::ConstSharedPtr msg)
{
    if(msg->data == "planning") {
        planning_status_code_.store(1, std::memory_order_relaxed);
    } else if(msg->data == "executing" || msg->data == "execute") {
        planning_status_code_.store(2, std::memory_order_relaxed);
    } else if(msg->data == "stop" || msg->data == "cancel") {
        RCLCPP_INFO(this->get_logger(), "trajectory execution status: stop");
        normal_stopped_ = true;
        planning_status_code_.store(0, std::memory_order_relaxed);
    } else if(msg->data == "error" || msg->data == "in_error") {
        planning_status_code_.store(3, std::memory_order_relaxed);
    }
}

void AuboDriver::robotControlCallback(const std_msgs::msg::String::ConstSharedPtr msg)
{
    if(msg->data == "powerOn") {
        aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
        memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));
        aubo_robot_namespace::ROBOT_SERVICE_STATE result;
        int ret = robot_send_service_.rootServiceRobotStartup(toolDynamicsParam, collision_class_, true, true, 1000, result);
        if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
            RCLCPP_INFO(this->get_logger(), "Initial success.");
        else
            RCLCPP_ERROR(this->get_logger(), "Initial failed.");
    }
}

void AuboDriver::clearBufQueue()
{
    std::lock_guard<std::mutex> lock(buf_queue_mutex_);
    while (!buf_queue_.empty()) buf_queue_.pop();
    buf_queue_size_.store(0);
}

void AuboDriver::updateControlStatus()
{
    static auto last_update_ts = std::chrono::steady_clock::now();
    const auto update_now = std::chrono::steady_clock::now();
    const double update_delta_ms = std::chrono::duration<double, std::milli>(update_now - last_update_ts).count();
    last_update_ts = update_now;
    size_t bq = buf_queue_size_.load(std::memory_order_relaxed);
    if (delay_clear_times > 0) {
        clearBufQueue();
        start_move_ = false;
        delay_clear_times--;
    }
    int c = data_count_.fetch_add(1) + 1;
    if (c == MAXALLOWEDDELAY) {
        data_count_.store(0);
        // 与 Noetic 对齐：100ms 后只要缓冲非空就可启动，避免等待过多点导致可见停顿
        if (bq > 0 && !start_move_ && delay_clear_times == 0)
            start_move_ = true;
    }
}

void AuboDriver::feedToRosMotionLoop()
{
    const auto period = std::chrono::milliseconds(5);   // 200Hz，与插值发布一致
    int empty_streak = 0;
    int idle_log_stride = 0;
    while (publish_thread_running_ && rclcpp::ok()) {
        auto loop_start = std::chrono::steady_clock::now();
        if (start_move_) {
            const int rib = rib_buffer_size_.load();
            const int feed_count = (rib < 200) ? 3 : 1;
            int batch = 0;
            int qsz_snapshot = static_cast<int>(ros_motion_queue_.size_approx());
            for (int i = 0; i < feed_count; i++) {
                if (!setRobotJointsByMoveIt()) break;
                batch++;
            }
            if(batch > 0) {
                empty_streak = 0;
                idle_log_stride = 0;
            } else if(ros_motion_queue_.size_approx() == 0) {
                empty_streak++;
                idle_log_stride++;
                if (idle_log_stride >= 100) {
                    idle_log_stride = 0;
                }
                const auto now_steady_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                const int64_t last_cb_ms = g_last_moveit_cb_steady_ms.load(std::memory_order_relaxed);
                const int64_t gap_since_cb_ms = (last_cb_ms > 0) ? (now_steady_ms - last_cb_ms) : -1;
                const bool rib_low = (rib <= 5);
                if (gap_since_cb_ms > 800 && empty_streak > 80 && rib_low) {
                    start_move_ = false;
                    empty_streak = 0;
                }
                if(empty_streak > 500) {
                    // 上游在 worker 周期切换中会出现秒级无回调窗口：仅在“长时间无回调”时退出 start_move_，
                    // 既避免短时欠供导致频繁停启，又能在长空窗时进入可控 idle 状态。
                    if (gap_since_cb_ms > 800) {
                        start_move_ = false;
                    }
                    // 与 Noetic 语义对齐：不因短时/间歇缺点直接退出运动态，避免频繁停启造成卡顿
                    empty_streak = 0;
                }
            } else {
                empty_streak = 0;
                idle_log_stride = 0;
            }
        }
        std::this_thread::sleep_until(loop_start + period);
    }
}

void AuboDriver::teachCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg)
{
    if(control_mode_ == aubo_driver::Teach && msg->data.size() >= 2) {
        (void)msg->data[0];
        (void)msg->data[1];
    }
}

void AuboDriver::AuboAPICallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg)
{
    if(control_mode_ != aubo_driver::SendTargetGoal || msg->data.size() < 7) return;
    double type = msg->data[0];
    if(type == 0) {
        double joints[6] = {msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5], msg->data[6]};
        if(roadPointCompare(joints, target_point_)) {
            memcpy(target_point_, joints, sizeof(double) * axis_number_);
            if(controller_connected_flag_) {
                int ret = robot_send_service_.robotServiceJointMove(joints, true);
                RCLCPP_INFO(this->get_logger(), "move to goal with API! %d", ret);
            }
        }
    }
}

bool AuboDriver::connectToRobotController()
{
    this->declare_parameter<std::string>("server_host", "127.0.0.1");
    server_host_ = this->get_parameter("server_host").as_string();
    RCLCPP_INFO(this->get_logger(), "server_host: %s", server_host_.c_str());

    int max_link_times = 5, count = 0;
    int ret1 = aubo_robot_namespace::InterfaceCallSuccCode;
    do {
        count++;
        ret1 = robot_send_service_.robotServiceLogin(server_host_.c_str(), server_port, "aubo", "123456");
    } while(ret1 != aubo_robot_namespace::InterfaceCallSuccCode && count < max_link_times);

    if(ret1 == aubo_robot_namespace::InterfaceCallSuccCode) {
        int ret2 = robot_receive_service_.robotServiceLogin(server_host_.c_str(), server_port, "aubo", "123456");
        controller_connected_flag_ = true;
        RCLCPP_INFO(this->get_logger(), "login success.");
        aubo_robot_namespace::wayPoint_S wp;
        robot_receive_service_.robotServiceGetCurrentWaypointInfo(wp);
        for(int i = 0; i < 6; i++) joint_filter_[i] = wp.jointpos[i];
        ret2 = robot_receive_service_.robotServiceGetIsRealRobotExist(real_robot_exist_);
        if(ret2 == aubo_robot_namespace::InterfaceCallSuccCode)
            RCLCPP_INFO(this->get_logger(), real_robot_exist_ ? "real robot exist." : "real robot does not exist.");
        robot_mac_size_service_.robotServiceLogin(server_host_.c_str(), server_port, "aubo", "123456");
        return true;
    } else {
        controller_connected_flag_ = false;
        RCLCPP_ERROR(this->get_logger(), "login failed.");
        return false;
    }
}

void AuboDriver::run()
{
    RCLCPP_INFO(this->get_logger(), "Start the driver!");
    int ret;
    if(connectToRobotController()) {
        ret = robot_send_service_.robotServiceEnterTcp2CanbusMode();
        if(ret == aubo_robot_namespace::InterfaceCallSuccCode) {
            RCLCPP_INFO(this->get_logger(), "Switches to ros-controller successfully");
            control_option_ = aubo_driver::RosMoveIt;
        } else if(ret == aubo_robot_namespace::ErrCode_ResponseReturnError) {
            robot_send_service_.robotServiceLeaveTcp2CanbusMode();
            ret = robot_send_service_.robotServiceEnterTcp2CanbusMode();
            if(ret == aubo_robot_namespace::InterfaceCallSuccCode) {
                RCLCPP_INFO(this->get_logger(), "Switches to ros-controller successfully");
                control_option_ = aubo_driver::RosMoveIt;
            } else {
                control_option_ = aubo_driver::AuboAPI;
                RCLCPP_WARN(this->get_logger(), "Failed to switch to ros-controller!");
            }
        } else {
            control_option_ = aubo_driver::AuboAPI;
            RCLCPP_WARN(this->get_logger(), "Failed to switch to ros-controller!");
        }
        ret = robot_receive_service_.robotServiceGetCurrentWaypointInfo(rs.wayPoint_);
        if(ret == aubo_robot_namespace::InterfaceCallSuccCode) {
            double joints[8];
            for(int i = 0; i < 6; i++) joints[i] = rs.wayPoint_.jointpos[i];
            setCurrentPosition(joints);
            setTagrtPosition(joints);
            auto robot_joints = std::make_shared<std_msgs::msg::Float32MultiArray>();
            robot_joints->data.resize(axis_number_);
            for(int i = 0; i < axis_number_; i++) robot_joints->data[i] = static_cast<float>(current_joints_[i]);
            joint_target_pub_->publish(*robot_joints);
        }
    }
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / TIMER_SPAN_),
        std::bind(&AuboDriver::timerCallback, this),
        state_timer_cb_group_);
    // 500Hz updateControlStatus 使用独立 callback group，与 50Hz timer/轨迹订阅并行（单节点 + MultiThreadedExecutor）
    update_control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(2),
        std::bind(&AuboDriver::updateControlStatus, this),
        update_control_cb_group_);
    mb_publish_thread_ = new std::thread(&AuboDriver::publishIOMsg, this);
    publish_thread_running_ = true;
    feed_to_ros_motion_thread_ = new std::thread(&AuboDriver::feedToRosMotionLoop, this);
    joint_feedback_publish_thread_ = new std::thread(&AuboDriver::publishJointStateAndFeedbackLoop, this);
}

void AuboDriver::publishIOMsg()
{
    rclcpp::Rate update_rate(50);
    const auto parse_io_pin = [](const std::string& io_name, int fallback) -> int {
        std::string digits;
        digits.reserve(io_name.size());
        for (const char ch : io_name) {
            if (std::isdigit(static_cast<unsigned char>(ch))) {
                digits.push_back(ch);
            }
        }
        if (!digits.empty()) {
            return std::atoi(digits.c_str());
        }
        return fallback;
    };

    while (rclcpp::ok()) {
        if (start_move_) {
            update_rate.sleep();
            continue;
        }
        demo_interface::msg::RobotIOStatus io_msg;
        io_msg.header.stamp = this->now();
        io_msg.header.frame_id = "base_link";
        io_msg.is_connected = controller_connected_flag_;
        std::vector<aubo_robot_namespace::RobotIoDesc> status_vector_in, status_vector_out;
        std::vector<aubo_robot_namespace::RobotIoType> io_type_in, io_type_out;
        io_type_in.push_back(aubo_robot_namespace::RobotBoardUserDI);
        io_type_out.push_back(aubo_robot_namespace::RobotBoardUserDO);
        robot_receive_service_.robotServiceGetBoardIOStatus(io_type_in, status_vector_in);
        robot_receive_service_.robotServiceGetBoardIOStatus(io_type_out, status_vector_out);
        for (size_t i = 6; i < status_vector_in.size(); i++) {
            const int pin = parse_io_pin(status_vector_in[i].ioName, static_cast<int>(i - 6));
            if (pin >= 0) {
                if (io_msg.digital_inputs.size() <= static_cast<size_t>(pin)) {
                    io_msg.digital_inputs.resize(static_cast<size_t>(pin) + 1, false);
                }
                io_msg.digital_inputs[static_cast<size_t>(pin)] = (status_vector_in[i].ioValue != 0);
            }
        }
        for (size_t i = 0; i < status_vector_out.size(); i++) {
            const int pin = parse_io_pin(status_vector_out[i].ioName, static_cast<int>(i));
            if (pin >= 0) {
                if (io_msg.digital_outputs.size() <= static_cast<size_t>(pin)) {
                    io_msg.digital_outputs.resize(static_cast<size_t>(pin) + 1, false);
                }
                io_msg.digital_outputs[static_cast<size_t>(pin)] = (status_vector_out[i].ioValue != 0);
            }
        }
        status_vector_in.clear();
        status_vector_out.clear();
        io_type_in.clear();
        io_type_out.clear();
        io_type_in.push_back(aubo_robot_namespace::RobotBoardControllerDI);
        io_type_out.push_back(aubo_robot_namespace::RobotBoardControllerDO);
        robot_receive_service_.robotServiceGetBoardIOStatus(io_type_in, status_vector_in);
        robot_receive_service_.robotServiceGetBoardIOStatus(io_type_out, status_vector_out);
        double digitalIn[30] = {0};
        for (size_t i = 0; i < status_vector_in.size(); i++) {
            if(status_vector_in[i].ioAddr < 30) digitalIn[status_vector_in[i].ioAddr] = status_vector_in[i].ioValue;
        }
        if(real_robot_exist_) {
            if(digitalIn[0] == 0 || digitalIn[8] == 0) emergency_stopped_ = true;
            else emergency_stopped_ = false;
            if(digitalIn[1] == 0 || digitalIn[9] == 0) protective_stopped_ = true;
            else protective_stopped_ = false;
        }
        status_vector_in.clear();
        status_vector_out.clear();
        io_type_in.clear();
        io_type_out.clear();
        io_type_in.push_back(aubo_robot_namespace::RobotBoardUserAI);
        io_type_out.push_back(aubo_robot_namespace::RobotBoardUserAO);
        robot_receive_service_.robotServiceGetBoardIOStatus(io_type_in, status_vector_in);
        robot_receive_service_.robotServiceGetBoardIOStatus(io_type_out, status_vector_out);
        for (size_t i = 0; i < status_vector_in.size(); i++) {
            const int pin = static_cast<int>(status_vector_in[i].ioAddr);
            if (pin >= 0) {
                if (io_msg.analog_inputs.size() <= static_cast<size_t>(pin)) {
                    io_msg.analog_inputs.resize(static_cast<size_t>(pin) + 1, 0.0f);
                }
                io_msg.analog_inputs[static_cast<size_t>(pin)] = static_cast<float>(status_vector_in[i].ioValue);
            }
        }
        for (size_t i = 0; i < status_vector_out.size(); i++) {
            const int pin = static_cast<int>(status_vector_out[i].ioAddr);
            if (pin >= 0) {
                if (io_msg.analog_outputs.size() <= static_cast<size_t>(pin)) {
                    io_msg.analog_outputs.resize(static_cast<size_t>(pin) + 1, 0.0f);
                }
                io_msg.analog_outputs[static_cast<size_t>(pin)] = static_cast<float>(status_vector_out[i].ioValue);
            }
        }
        status_vector_in.clear();
        status_vector_out.clear();
        robot_receive_service_.robotServiceGetAllToolDigitalIOStatus(status_vector_in);
        robot_receive_service_.robotServiceGetAllToolAIStatus(status_vector_out);
        for (size_t i = 0; i < status_vector_in.size(); i++) {
            const int pin = static_cast<int>(status_vector_in[i].ioAddr);
            if (pin >= 0) {
                if (io_msg.tool_io_status.digital_outputs.size() <= static_cast<size_t>(pin)) {
                    io_msg.tool_io_status.digital_outputs.resize(static_cast<size_t>(pin) + 1, false);
                }
                io_msg.tool_io_status.digital_outputs[static_cast<size_t>(pin)] = (status_vector_in[i].ioValue != 0);
            }
        }
        for (size_t i = 0; i < status_vector_out.size(); i++) {
            const int pin = static_cast<int>(status_vector_out[i].ioAddr);
            if (pin >= 0) {
                if (io_msg.tool_io_status.analog_inputs.size() <= static_cast<size_t>(pin)) {
                    io_msg.tool_io_status.analog_inputs.resize(static_cast<size_t>(pin) + 1, 0.0f);
                }
                io_msg.tool_io_status.analog_inputs[static_cast<size_t>(pin)] = static_cast<float>(status_vector_out[i].ioValue);
            }
        }
        io_pub_->publish(io_msg);
        update_rate.sleep();
    }
}

std::vector<aubo_robot_namespace::wayPoint_S> AuboDriver::tryPopWaypoint(int count)
{
    std::vector<aubo_robot_namespace::wayPoint_S> wayPointVector;
    aubo_robot_namespace::wayPoint_S wp;
    JointPosWithTime jpt;
    std::array<double, 6> joint, interpolation_joint;
    uint8_t same_point = 0;
    for(int n = 0; n < count; n++) {
        if(!ros_motion_queue_.try_dequeue(jpt)) break;
        if (jpt.trajectory_epoch < current_trajectory_epoch_.load(std::memory_order_relaxed)) {
            last_time_from_start_ = -1.0;
            for (int i = 0; i < 6; i++)
                last_joint_velc_.jointPara[i] = 0.0;
            continue;
        }
        joint = jpt.joint_pos;
        const double kNominalTimeStep = 0.005;  // simulator 以 200Hz 发布，驱动侧按固定步长估算速度/加速度更稳
        double actual_time_step = kNominalTimeStep;
        bool discontinuity = false;
        if(last_time_from_start_ >= 0.0) {
            double time_diff = jpt.time_from_start - last_time_from_start_;
            if (time_diff <= 0.0 || time_diff > 0.02) {
                discontinuity = true;
            }
            if(time_diff > 0.0 && time_diff <= 0.02)
                actual_time_step = time_diff;
            else
                actual_time_step = kNominalTimeStep;
        }
        last_time_from_start_ = jpt.time_from_start;
        if (discontinuity) {
            for (int i = 0; i < 6; i++)
                last_joint_velc_.jointPara[i] = 0.0;
        }
        same_point = 0;
        for(int i = 0; i < 6; i++) {
            if (fabs(joint[i] - joint_filter_[i]) < 0.00015) same_point |= (1 << i);
        }
        if(same_point != 0x3F) {
            double time_step_for_calc = (actual_time_step >= 0.003 && actual_time_step <= 0.02) ? actual_time_step : kNominalTimeStep;
            for(int i = 0; i < 6; i++) {
                target_joint_velc_.jointPara[i] = fabs(joint[i] - joint_filter_[i]) / time_step_for_calc;
                if(target_joint_velc_.jointPara[i] > MaxVelc[i]) {
                    over_speed_flag_ = true;
                    robot_send_service_.robotServiceLeaveTcp2CanbusMode();
                }
            }
            if(over_speed_flag_) {
                over_speed_flag_ = false;
                const double max_target_vel = *std::max_element(target_joint_velc_.jointPara, target_joint_velc_.jointPara + 6);
                int n_equalpart = static_cast<int>(ceil(max_target_vel / MaxVelc[0]));
                if(n_equalpart < 1) n_equalpart = 1;
                const int kMaxOverspeedSplitParts = 8;
                if (n_equalpart > kMaxOverspeedSplitParts) {
                    n_equalpart = kMaxOverspeedSplitParts;
                }
                interpolation_joint = joint_filter_;
                for(int i = 0; i < n_equalpart - 1; i++) {
                    for(int j = 0; j < 6; j++)
                        interpolation_joint[j] = interpolation_joint[j] + (joint[j] - joint_filter_[j]) / n_equalpart;
                    memcpy(wp.jointpos, interpolation_joint.data(), 6 * sizeof(double));
                    wayPointVector.push_back(wp);
                }
            }
            double time_step_acc = (actual_time_step >= 0.003 && actual_time_step <= 0.02) ? actual_time_step : kNominalTimeStep;
            for(int i = 0; i < 6; i++)
                joint_acc_.jointPara[i] = fabs(target_joint_velc_.jointPara[i] - last_joint_velc_.jointPara[i]) / time_step_acc;
            memcpy(wp.jointpos, joint.data(), 6 * sizeof(double));
            wayPointVector.push_back(wp);
            memcpy(last_joint_velc_.jointPara, target_joint_velc_.jointPara, sizeof(last_joint_velc_.jointPara));
            joint_filter_ = joint;
        }
    }
    return wayPointVector;
}

void AuboDriver::publishWaypointToRobot()
{
    std::vector<aubo_robot_namespace::wayPoint_S> wayPointVector;
    int current_macsz = rib_buffer_size_.load();
    const int expect_macsz = 400;
    // 关键修复：基础小批量发送，队列积压时自适应提速，避免长期欠供导致卡顿
    const int kBaseCntPerSend = 2;
    const int kMaxAdaptiveCntPerSend = 8;
    const int kMaxWaypointBatchSend = 16;
    int cnt = 0;
    auto last_diag_refresh = std::chrono::steady_clock::now() - std::chrono::milliseconds(200);
    double canbus_send_ms_ema = 0.0;

    while(rclcpp::ok()) {
        current_macsz = rib_buffer_size_.load();
        const size_t qsz = ros_motion_queue_.size_approx();
        const auto now = std::chrono::steady_clock::now();
        const auto diag_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_diag_refresh).count();
        // 关键修复：SDK 诊断查询本身存在 10~200ms 阻塞，发送热路径降频到 >=120ms，避免周期性卡顿
        const int diag_refresh_interval_ms = (qsz > 0) ? 120 : 250;
        const bool need_diag_refresh =
            (current_macsz <= 0) ||
            (diag_age_ms >= diag_refresh_interval_ms);
        if(need_diag_refresh) {
            aubo_robot_namespace::RobotDiagnosis pub_diag;
            const auto diag_call_start = std::chrono::steady_clock::now();
            const int diag_ret = robot_mac_size_service_.robotServiceGetRobotDiagnosisInfo(pub_diag);
            const double diag_call_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - diag_call_start).count();
            (void)diag_call_ms;
            if(0 == diag_ret) {
                rib_buffer_size_ = pub_diag.macTargetPosDataSize;
                current_macsz = pub_diag.macTargetPosDataSize;
                last_diag_refresh = now;
                if(current_macsz == 0)
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

        if(current_macsz < expect_macsz && 0 != qsz) {
            int desired_cnt = static_cast<int>(ceil(static_cast<double>(expect_macsz - current_macsz) / 6.0));
            if (qsz > 80) desired_cnt = std::max(desired_cnt, 4);
            if (qsz > 160) desired_cnt = std::max(desired_cnt, 6);
            if (qsz > 240) desired_cnt = std::max(desired_cnt, 8);
            int min_cnt_from_send_cost = kBaseCntPerSend;
            if (canbus_send_ms_ema > 10.0) min_cnt_from_send_cost = 4;
            if (canbus_send_ms_ema > 14.0) min_cnt_from_send_cost = 6;
            if (canbus_send_ms_ema > 20.0) min_cnt_from_send_cost = 8;
            desired_cnt = std::max(desired_cnt, min_cnt_from_send_cost);
            cnt = std::max(kBaseCntPerSend, std::min(kMaxAdaptiveCntPerSend, desired_cnt));
            wayPointVector = tryPopWaypoint(cnt);
            if(!wayPointVector.empty()) {
                if (wayPointVector.size() > static_cast<size_t>(kMaxWaypointBatchSend)) {
                    wayPointVector.resize(static_cast<size_t>(kMaxWaypointBatchSend));
                }
                const auto canbus_send_start = std::chrono::steady_clock::now();
                robot_mac_size_service_.robotServiceSetRobotPosData2Canbus(wayPointVector);
                const double canbus_send_ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - canbus_send_start).count();
                if (canbus_send_ms_ema <= 0.0)
                    canbus_send_ms_ema = canbus_send_ms;
                else
                    canbus_send_ms_ema = 0.9 * canbus_send_ms_ema + 0.1 * canbus_send_ms;
                current_macsz += static_cast<int>(wayPointVector.size()) * 6;
                rib_buffer_size_ = current_macsz;
            }
            wayPointVector.clear();
        }

        if (ros_motion_queue_.size_approx() > 60)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        else if (current_macsz < 200 && ros_motion_queue_.size_approx() > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        else
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
    }
}

int AuboDriver::checkTargetVelc(JointParam mTaget_JointAngle, JointParam mLast_JointAngle, JointVelcAccParam &mJointVelc)
{
    for(int i = 0; i < 6; i++) {
        mJointVelc.jointPara[i] = fabs(mTaget_JointAngle.jointPos[i] - mLast_JointAngle.jointPos[i]) / 0.005;
        if(mJointVelc.jointPara[i] > MaxVelc[i]) return -1;
    }
    return 0;
}

int AuboDriver::checkTargetAcc(JointVelcAccParam mLastJointVelc, JointVelcAccParam &mTargetJointVelc)
{
    (void)mLastJointVelc;
    (void)mTargetJointVelc;
    return 0;
}

void AuboDriver::setIO(const std::shared_ptr<demo_interface::srv::SetRobotIO::Request> req,
                       std::shared_ptr<demo_interface::srv::SetRobotIO::Response> resp)
{
    resp->success = false;
    resp->error_code = -1;
    resp->message = "未知 io_type";

    std::string io_type = req->io_type;
    std::transform(io_type.begin(), io_type.end(), io_type.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    const int io_index = req->io_index;
    const double value = req->value;

    if (io_type == "digital_output") {
        robot_send_service_.robotServiceSetBoardIOStatus(
            aubo_robot_namespace::RobotBoardUserDO, io_index + 32, value);
        std::this_thread::sleep_for(std::chrono::duration<double>(io_flag_delay_));
        resp->success = true;
        resp->error_code = 0;
        resp->message = "digital_output 设置成功";
    } else if (io_type == "analog_output") {
        robot_send_service_.robotServiceSetBoardIOStatus(
            aubo_robot_namespace::RobotBoardUserAO, io_index, value);
        std::this_thread::sleep_for(std::chrono::duration<double>(io_flag_delay_));
        resp->success = true;
        resp->error_code = 0;
        resp->message = "analog_output 设置成功";
    } else if (io_type == "tool_io") {
        if (value < 0.0) {
            robot_send_service_.robotServiceSetToolDigitalIOType(
                static_cast<aubo_robot_namespace::ToolDigitalIOAddr>(io_index), aubo_robot_namespace::IO_IN);
            std::this_thread::sleep_for(std::chrono::duration<double>(io_flag_delay_));
        } else {
            robot_send_service_.robotServiceSetToolDigitalIOType(
                static_cast<aubo_robot_namespace::ToolDigitalIOAddr>(io_index), aubo_robot_namespace::IO_OUT);
            std::this_thread::sleep_for(std::chrono::duration<double>(io_flag_delay_));
            robot_send_service_.robotServiceSetToolDOStatus(
                static_cast<aubo_robot_namespace::ToolDigitalIOAddr>(io_index),
                (value > 0.5) ? aubo_robot_namespace::IO_STATUS_VALID : aubo_robot_namespace::IO_STATUS_INVALID);
            std::this_thread::sleep_for(std::chrono::duration<double>(io_flag_delay_));
        }
        resp->success = true;
        resp->error_code = 0;
        resp->message = "tool_io 设置成功";
    } else if (io_type == "tool_analog_output") {
        robot_send_service_.robotServiceSetBoardIOStatus(
            aubo_robot_namespace::RobotToolAO, io_index, value);
        std::this_thread::sleep_for(std::chrono::duration<double>(io_flag_delay_));
        resp->success = true;
        resp->error_code = 0;
        resp->message = "tool_analog_output 设置成功";
    } else if (io_type == "tool_power") {
        robot_send_service_.robotServiceSetToolPowerVoltageType(
            static_cast<aubo_robot_namespace::ToolPowerType>(static_cast<int>(value)));
        resp->success = true;
        resp->error_code = 0;
        resp->message = "tool_power 设置成功";
    }
}

void AuboDriver::getFK(const std::shared_ptr<aubo_msgs::srv::GetFK::Request> req, std::shared_ptr<aubo_msgs::srv::GetFK::Response> resp)
{
    aubo_robot_namespace::wayPoint_S wayPoint;
    double joint[] = {req->joint[0], req->joint[1], req->joint[2], req->joint[3], req->joint[4], req->joint[5]};
    robot_send_service_.robotServiceRobotFk(joint, 6, wayPoint);
    resp->pos[0] = static_cast<float>(wayPoint.cartPos.position.x);
    resp->pos[1] = static_cast<float>(wayPoint.cartPos.position.y);
    resp->pos[2] = static_cast<float>(wayPoint.cartPos.position.z);
    resp->ori[0] = static_cast<float>(wayPoint.orientation.w);
    resp->ori[1] = static_cast<float>(wayPoint.orientation.x);
    resp->ori[2] = static_cast<float>(wayPoint.orientation.y);
    resp->ori[3] = static_cast<float>(wayPoint.orientation.z);
}

void AuboDriver::getIK(const std::shared_ptr<aubo_msgs::srv::GetIK::Request> req, std::shared_ptr<aubo_msgs::srv::GetIK::Response> resp)
{
    aubo_robot_namespace::wayPoint_S wayPoint;
    double joint[] = {req->ref_joint[0], req->ref_joint[1], req->ref_joint[2], req->ref_joint[3], req->ref_joint[4], req->ref_joint[5]};
    aubo_robot_namespace::Pos position;
    position.x = req->pos[0]; position.y = req->pos[1]; position.z = req->pos[2];
    aubo_robot_namespace::Ori ori;
    ori.w = req->ori[0]; ori.x = req->ori[1]; ori.y = req->ori[2]; ori.z = req->ori[3];
    robot_send_service_.robotServiceRobotIk(joint, position, ori, wayPoint);
    resp->joint[0] = static_cast<float>(wayPoint.jointpos[0]);
    resp->joint[1] = static_cast<float>(wayPoint.jointpos[1]);
    resp->joint[2] = static_cast<float>(wayPoint.jointpos[2]);
    resp->joint[3] = static_cast<float>(wayPoint.jointpos[3]);
    resp->joint[4] = static_cast<float>(wayPoint.jointpos[4]);
    resp->joint[5] = static_cast<float>(wayPoint.jointpos[5]);
}

}  // namespace aubo_driver
