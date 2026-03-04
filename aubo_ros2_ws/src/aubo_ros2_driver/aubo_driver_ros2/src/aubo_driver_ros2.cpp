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
#include <mutex>
#include <fstream>

namespace aubo_driver {

double MaxAcc[ARM_DOF] = {17.30878, 17.30878, 17.30878, 20.73676, 20.73676, 20.73676};
double MaxVelc[ARM_DOF] = {2.596177, 2.596177, 2.596177, 3.110177, 3.110177, 3.110177};

std::string AuboDriver::joint_name_[ARM_DOF] = {"shoulder_joint","upperArm_joint","foreArm_joint","wrist1_joint","wrist2_joint","wrist3_joint"};

std::ofstream file;
std::ofstream file_v;
char *t1[128] = {0};

void get_format_time_ms(char *str_time) {
    struct tm *tm_t;
    struct timeval time;
    gettimeofday(&time,NULL);
    tm_t = localtime(&time.tv_sec);
    if(NULL != tm_t) {
        sprintf(str_time,"%04d-%02d-%02d %02d:%02d:%02d.%03ld",
            tm_t->tm_year+1900, tm_t->tm_mon+1, tm_t->tm_mday,
            tm_t->tm_hour, tm_t->tm_min, tm_t->tm_sec, time.tv_usec/1000);
        return;
    }
}

static double time_from_start_to_sec(const builtin_interfaces::msg::Duration& t) {
    return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
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
    robot_status_pub_ = this->create_publisher<demo_interface::msg::RobotStatus>("robot_status", 1000);
    io_pub_ = this->create_publisher<aubo_msgs::msg::IOStates>("/aubo_driver/io_states", 10);
    rib_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/aubo_driver/rib_status", 1000);
    cancle_trajectory_pub_ = this->create_publisher<std_msgs::msg::UInt8>("aubo_driver/cancel_trajectory", 100);

    io_srv_ = this->create_service<aubo_msgs::srv::SetIO>("/aubo_driver/set_io", std::bind(&AuboDriver::setIO, this, std::placeholders::_1, std::placeholders::_2));
    ik_srv_ = this->create_service<aubo_msgs::srv::GetIK>("/aubo_driver/get_ik", std::bind(&AuboDriver::getIK, this, std::placeholders::_1, std::placeholders::_2));
    fk_srv_ = this->create_service<aubo_msgs::srv::GetFK>("/aubo_driver/get_fk", std::bind(&AuboDriver::getFK, this, std::placeholders::_1, std::placeholders::_2));

    trajectory_execution_subs_ = this->create_subscription<std_msgs::msg::String>("trajectory_execution_event", 10, std::bind(&AuboDriver::trajectoryExecutionCallback, this, std::placeholders::_1));
    robot_control_subs_ = this->create_subscription<std_msgs::msg::String>("robot_control", 10, std::bind(&AuboDriver::robotControlCallback, this, std::placeholders::_1));
    // moveItController_cmd 使用独立 callback group，与 50Hz timer / 500Hz update_control 并行，无需拆成多节点
    trajectory_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    update_control_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions moveit_sub_opts;
    moveit_sub_opts.callback_group = trajectory_cb_group_;
    moveit_controller_subs_ = this->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
        "moveItController_cmd",
        rclcpp::QoS(20000),
        std::bind(&AuboDriver::moveItPosCallback, this, std::placeholders::_1),
        moveit_sub_opts);
    teach_subs_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("teach_cmd", 10, std::bind(&AuboDriver::teachCallback, this, std::placeholders::_1));
    moveAPI_subs_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("moveAPI_cmd", 10, std::bind(&AuboDriver::AuboAPICallback, this, std::placeholders::_1));
    controller_switch_sub_ = this->create_subscription<std_msgs::msg::Int32>("/aubo_driver/controller_switch", 10, std::bind(&AuboDriver::controllerSwitchCallback, this, std::placeholders::_1));

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
    if(controller_connected_flag_) {
        int ret = robot_receive_service_.robotServiceGetCurrentWaypointInfo(rs.wayPoint_);
        if(ret == aubo_robot_namespace::InterfaceCallSuccCode) {
            double joints[ARM_DOF];
            for(int i = 0; i < 6; i++) joints[i] = rs.wayPoint_.jointpos[i];
            setCurrentPosition(joints);
            if(real_robot_exist_) {
                robot_receive_service_.robotServiceGetRobotDiagnosisInfo(rs.robot_diagnosis_info_);
                rib_buffer_size_ = rs.robot_diagnosis_info_.macTargetPosDataSize;
            }
            robot_status_msg_.is_online = controller_connected_flag_;
            robot_status_msg_.enable = (real_robot_exist_ && rs.robot_diagnosis_info_.armPowerStatus != 0);
            robot_status_msg_.in_motion = start_move_;
            robot_status_msg_.planning_status = (emergency_stopped_ || protective_stopped_) ? "stopped" : (start_move_ ? "moving" : "idle");
        } else if(ret == aubo_robot_namespace::ErrCode_SocketDisconnect) {
            if(!connectToRobotController())
                RCLCPP_ERROR(this->get_logger(), "Cannot connect to the robot controller!");
        }
        robot_status_msg_.header.stamp = this->now();
        for(int i = 0; i < 6 && i < axis_number_; i++)
            robot_status_msg_.joint_position_rad[i] = current_joints_[i];
        { std::lock_guard<std::mutex> lock(buf_queue_mutex_); rib_status_.data[0] = static_cast<int32_t>(buf_queue_.size()); }
        rib_status_.data[1] = control_mode_;
        rib_status_.data[2] = controller_connected_flag_ ? 1 : 0;
    } else {
        setCurrentPosition(target_point_);
        robot_status_msg_.is_online = false;
        robot_status_msg_.in_motion = false;
        robot_status_msg_.header.stamp = this->now();
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
    PlanningState ps;
    {
        std::lock_guard<std::mutex> lock(buf_queue_mutex_);
        if(buf_queue_.empty())
            return false;
        ps = buf_queue_.front();
        buf_queue_.pop();
        buf_queue_size_.store(buf_queue_.size());
    }
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
            JointPosWithTime jpt;
            for(int i = 0; i < 6; i++) jpt.joint_pos[i] = ps.joint_pos_[i];
            jpt.time_from_start = ps.time_from_start_;
            ros_motion_queue_.enqueue(jpt);
            file << ps.joint_pos_[0] << "," << ps.joint_pos_[1] << "," << ps.joint_pos_[2] << "," << ps.joint_pos_[3] << "," << ps.joint_pos_[4] << "," << ps.joint_pos_[5] << std::endl;
        }
    }
    setTagrtPosition(ps.joint_pos_);
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
            memcpy(last_recieve_point_, jointAngle, sizeof(double) * axis_number_);
            {
                std::lock_guard<std::mutex> lock(buf_queue_mutex_);
                buf_queue_.push(ps);
                size_t qsize = buf_queue_.size();
                buf_queue_size_.store(qsize);
                if (qsize == 1u)
                    data_count_.store(0);  // 首点到达：对齐 50 次计数
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
    if(msg->data == "stop") {
        RCLCPP_INFO(this->get_logger(), "trajectory execution status: stop");
        normal_stopped_ = true;
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
    size_t bq = buf_queue_size_.load(std::memory_order_relaxed);
    if (delay_clear_times > 0) {
        clearBufQueue();
        start_move_ = false;
        delay_clear_times--;
    }
    int c = data_count_.fetch_add(1) + 1;
    if (c == MAXALLOWEDDELAY) {
        data_count_.store(0);
        if (bq >= 50 && !start_move_ && delay_clear_times == 0)
            start_move_ = true;
    }
}

void AuboDriver::feedToRosMotionLoop()
{
    const auto period = std::chrono::milliseconds(5);   // 200Hz，与插值发布一致
    const int max_batch_per_cycle = 150;   // 单周期最多转 150 点，避免 RIB=0 时 ros_motion_queue_ 被送机线程一次抽空后长时间补不上
    int empty_streak = 0;
    while (publish_thread_running_ && rclcpp::ok()) {
        auto loop_start = std::chrono::steady_clock::now();
        if (start_move_) {
            int batch = 0;
            while (setRobotJointsByMoveIt() && batch < max_batch_per_cycle)
                batch++;
            if(batch > 0) {
                empty_streak = 0;
            } else if(ros_motion_queue_.size_approx() == 0) {
                empty_streak++;
                if(empty_streak > 500) {
                    start_move_ = false;
                    empty_streak = 0;
                }
            } else {
                empty_streak = 0;
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
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / TIMER_SPAN_), std::bind(&AuboDriver::timerCallback, this));
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
    while (rclcpp::ok()) {
        aubo_msgs::msg::IOStates io_msg;
        std::vector<aubo_robot_namespace::RobotIoDesc> status_vector_in, status_vector_out;
        std::vector<aubo_robot_namespace::RobotIoType> io_type_in, io_type_out;
        io_type_in.push_back(aubo_robot_namespace::RobotBoardUserDI);
        io_type_out.push_back(aubo_robot_namespace::RobotBoardUserDO);
        robot_receive_service_.robotServiceGetBoardIOStatus(io_type_in, status_vector_in);
        robot_receive_service_.robotServiceGetBoardIOStatus(io_type_out, status_vector_out);
        char num[2];
        for (size_t i = 6; i < status_vector_in.size(); i++) {
            aubo_msgs::msg::Digital digi;
            num[0] = status_vector_in[i].ioName[5];
            num[1] = status_vector_in[i].ioName[6];
            digi.pin = static_cast<uint8_t>(std::atoi(num));
            digi.state = (status_vector_in[i].ioValue != 0);
            io_msg.digital_in_states.push_back(digi);
        }
        for (size_t i = 0; i < status_vector_out.size(); i++) {
            aubo_msgs::msg::Digital digo;
            num[0] = status_vector_out[i].ioName[5];
            num[1] = status_vector_out[i].ioName[6];
            digo.pin = static_cast<uint8_t>(std::atoi(num));
            digo.state = (status_vector_out[i].ioValue != 0);
            io_msg.digital_out_states.push_back(digo);
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
            aubo_msgs::msg::Analog ana;
            ana.pin = static_cast<uint8_t>(status_vector_in[i].ioAddr);
            ana.domain = aubo_msgs::msg::Analog::VOLTAGE;
            ana.state = status_vector_in[i].ioValue;
            io_msg.analog_in_states.push_back(ana);
        }
        for (size_t i = 0; i < status_vector_out.size(); i++) {
            aubo_msgs::msg::Analog ana;
            ana.pin = static_cast<uint8_t>(status_vector_out[i].ioAddr);
            ana.domain = aubo_msgs::msg::Analog::VOLTAGE;
            ana.state = status_vector_out[i].ioValue;
            io_msg.analog_out_states.push_back(ana);
        }
        status_vector_in.clear();
        status_vector_out.clear();
        robot_receive_service_.robotServiceGetAllToolDigitalIOStatus(status_vector_in);
        robot_receive_service_.robotServiceGetAllToolAIStatus(status_vector_out);
        for (size_t i = 0; i < status_vector_in.size(); i++) {
            aubo_msgs::msg::Digital digo;
            digo.pin = static_cast<uint8_t>(status_vector_in[i].ioAddr);
            digo.state = (status_vector_in[i].ioValue != 0);
            io_msg.flag_states.push_back(digo);
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
        joint = jpt.joint_pos;
        double actual_time_step = 0.005;
        if(last_time_from_start_ >= 0.0) {
            double time_diff = jpt.time_from_start - last_time_from_start_;
            if(time_diff > 0.0 && time_diff < 1.0) actual_time_step = time_diff;
            else if(time_diff <= 0.0) actual_time_step = 0.005;
            else actual_time_step = 0.005;
        }
        last_time_from_start_ = jpt.time_from_start;
        same_point = 0;
        for(int i = 0; i < 6; i++) {
            if (fabs(joint[i] - joint_filter_[i]) < 0.00015) same_point |= (1 << i);
        }
        if(same_point != 0x3F) {
            double time_step_for_calc = (actual_time_step >= 0.005 && actual_time_step <= 1.0) ? actual_time_step : 0.005;
            for(int i = 0; i < 6; i++) {
                target_joint_velc_.jointPara[i] = fabs(joint[i] - joint_filter_[i]) / time_step_for_calc;
                if(target_joint_velc_.jointPara[i] > MaxVelc[i]) {
                    over_speed_flag_ = true;
                    robot_send_service_.robotServiceLeaveTcp2CanbusMode();
                }
            }
            if(over_speed_flag_) {
                over_speed_flag_ = false;
                std::sort(target_joint_velc_.jointPara, target_joint_velc_.jointPara + 6);
                int n_equalpart = static_cast<int>(ceil(target_joint_velc_.jointPara[5] / MaxVelc[0]));
                if(n_equalpart < 1) n_equalpart = 1;
                interpolation_joint = joint_filter_;
                for(int i = 0; i < n_equalpart - 1; i++) {
                    for(int j = 0; j < 6; j++)
                        interpolation_joint[j] = interpolation_joint[j] + (joint[j] - joint_filter_[j]) / n_equalpart;
                    memcpy(wp.jointpos, interpolation_joint.data(), 6 * sizeof(double));
                    wayPointVector.push_back(wp);
                }
            }
            double time_step_acc = (actual_time_step > 0.0001 && actual_time_step < 1.0) ? actual_time_step : 0.005;
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
    int current_macsz = 0;
    const int expect_macsz = 400;
    int cnt = 0;

    while(rclcpp::ok()) {
        aubo_robot_namespace::RobotDiagnosis pub_diag;
        if(0 == robot_mac_size_service_.robotServiceGetRobotDiagnosisInfo(pub_diag)) {
            rib_buffer_size_ = pub_diag.macTargetPosDataSize;
            current_macsz = pub_diag.macTargetPosDataSize;
            if(current_macsz == 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

        if(current_macsz < expect_macsz && 0 != ros_motion_queue_.size_approx()) {
            cnt = static_cast<int>(ceil(static_cast<double>(expect_macsz - current_macsz) / 6.0));
            wayPointVector = tryPopWaypoint(cnt);
            if(!wayPointVector.empty()) {
                robot_mac_size_service_.robotServiceSetRobotPosData2Canbus(wayPointVector);
            }
            wayPointVector.clear();
        }

        // RIB 低时缩短睡眠，尽快灌点，减少机械臂等点卡顿（日志证实 rib=0 时会出现停顿）
        int sleep_ms = (current_macsz < 50) ? 1 : 4;
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
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

void AuboDriver::setIO(const std::shared_ptr<aubo_msgs::srv::SetIO::Request> req, std::shared_ptr<aubo_msgs::srv::SetIO::Response> resp)
{
    resp->success = true;
    if (req->fun == 1) {
        robot_send_service_.robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotBoardUserDO, req->pin + 32, req->state);
        std::this_thread::sleep_for(std::chrono::duration<double>(io_flag_delay_));
    } else if (req->fun == 2) {
        robot_send_service_.robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotBoardUserAO, req->pin, req->state);
        std::this_thread::sleep_for(std::chrono::duration<double>(io_flag_delay_));
    } else if (req->fun == 3) {
        if(req->state == -1) {
            robot_send_service_.robotServiceSetToolDigitalIOType(static_cast<aubo_robot_namespace::ToolDigitalIOAddr>(req->pin), aubo_robot_namespace::IO_IN);
            std::this_thread::sleep_for(std::chrono::duration<double>(io_flag_delay_));
        } else {
            robot_send_service_.robotServiceSetToolDigitalIOType(static_cast<aubo_robot_namespace::ToolDigitalIOAddr>(req->pin), aubo_robot_namespace::IO_OUT);
            std::this_thread::sleep_for(std::chrono::duration<double>(io_flag_delay_));
            robot_send_service_.robotServiceSetToolDOStatus(static_cast<aubo_robot_namespace::ToolDigitalIOAddr>(req->pin), static_cast<aubo_robot_namespace::IO_STATUS>(req->state));
            std::this_thread::sleep_for(std::chrono::duration<double>(io_flag_delay_));
        }
    } else if (req->fun == 4) {
        robot_send_service_.robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotToolAO, req->pin, req->state);
        std::this_thread::sleep_for(std::chrono::duration<double>(io_flag_delay_));
    } else if (req->fun == 5) {
        robot_send_service_.robotServiceSetToolPowerVoltageType(static_cast<aubo_robot_namespace::ToolPowerType>(req->state));
    } else {
        resp->success = false;
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
