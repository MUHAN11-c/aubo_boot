/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017-2018, AUBO Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef AUBO_DRIVER_ROS2_AUBO_DRIVER_H_
#define AUBO_DRIVER_ROS2_AUBO_DRIVER_H_

#include <thread>
#include <mutex>
#include <atomic>
#include <string>
#include <fstream>
#include <iostream>
#include <sys/timeb.h>
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <aubo_msgs/srv/set_io.hpp>
#include <aubo_msgs/srv/get_fk.hpp>
#include <aubo_msgs/srv/get_ik.hpp>
#include <aubo_msgs/msg/io_states.hpp>
#include <aubo_msgs/msg/joint_trajectory_feedback.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <demo_interface/msg/robot_status.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "aubo_driver_ros2/AuboRobotMetaType.h"
#include "aubo_driver_ros2/serviceinterface.h"
#include "aubo_driver_ros2/readerwriterqueue.h"

#include "otg/otgnewslib.h"

#define MINIMUM_BUFFER_SIZE 300
#define ARM_DOF 8
#define MAXALLOWEDDELAY 50   // 与 ROS1 一致：50*2ms=100ms 启动延迟
#define server_port 8899
#define BIG_MODULE_RATIO 2 * M_PI / 60.0 / 121
#define SMALL_MODULE_RATIO 2 * M_PI / 60.0 / 101
#define VMAX 3000
#define AMAX 10000
#define JMAX 40000
#define STOP_DELAY_CLEAR_TIMES 100

namespace aubo_driver
{
    struct PlanningState
    {
        double joint_vel_[ARM_DOF];
        double joint_acc_[ARM_DOF];
        double joint_pos_[ARM_DOF];
        double time_from_start_;
    };
    enum ROBOT_CONTROLLER_MODE
    {
        ROBOT_CONTROLLER=0,
        ROS_CONTROLLER
    };
    enum ControlOption
    {
        AuboAPI = 0,
        RosMoveIt
    };
    enum ControMode
    {
        Teach = 0,
        SendTargetGoal,
        SynchronizeWithRealRobot
    };

    struct RobotState
    {
        aubo_robot_namespace::JointStatus joint_status_[ARM_DOF];
        aubo_robot_namespace::wayPoint_S wayPoint_;
        aubo_robot_namespace::RobotDiagnosis robot_diagnosis_info_;
        bool IsRealRobotExist;
        bool isRobotControllerConnected;
        ROBOT_CONTROLLER_MODE robot_controller_;
        aubo_robot_namespace::RobotState state_;
        aubo_robot_namespace::RobotErrorCode code_;
    };

    typedef struct
    {
        double jointPos[ARM_DOF];
    }JointParam;

    typedef struct
    {
        double jointPara[ARM_DOF];
    }JointVelcAccParam;

    class AuboDriver : public rclcpp::Node
    {
        public:
            AuboDriver(int num);
            ~AuboDriver();
            bool roadPointCompare(double *point1, double *point2);

            double* getCurrentPosition();
            void setCurrentPosition(double *target);
            double* getTagrtPosition();
            void setTagrtPosition(double *target);

            void updateControlStatus();
            void run();
            bool connectToRobotController();
            void setIO(const std::shared_ptr<aubo_msgs::srv::SetIO::Request> req, std::shared_ptr<aubo_msgs::srv::SetIO::Response> resp);
            void getFK(const std::shared_ptr<aubo_msgs::srv::GetFK::Request> req, std::shared_ptr<aubo_msgs::srv::GetFK::Response> resp);
            void getIK(const std::shared_ptr<aubo_msgs::srv::GetIK::Request> req, std::shared_ptr<aubo_msgs::srv::GetIK::Response> resp);

            const int UPDATE_RATE_ = 500;
            const int TIMER_SPAN_ = 50;       // 主定时器 50Hz（状态查询）
            const int PUBLISH_RATE_ = 50;     // 与 ROS1 一致
            const double THRESHHOLD = 0.000001;

        public:
            static std::string joint_name_[ARM_DOF];
            double joint_ratio_[ARM_DOF];
            int axis_number_;
            int buffer_size_;
            ServiceInterface robot_send_service_;
            ServiceInterface robot_receive_service_;
            std::vector<aubo_robot_namespace::wayPoint_S> waypoint_vector_;

            RobotState rs;
            std::queue<PlanningState>  buf_queue_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
            rclcpp::Publisher<control_msgs::action::FollowJointTrajectory_Feedback>::SharedPtr joint_feedback_pub_;
            rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_target_pub_;
            rclcpp::Publisher<demo_interface::msg::RobotStatus>::SharedPtr robot_status_pub_;
            rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr moveit_controller_subs_;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trajectory_execution_subs_;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_control_subs_;
            rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr teach_subs_;
            rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr moveAPI_subs_;
            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr controller_switch_sub_;
            rclcpp::Publisher<aubo_msgs::msg::IOStates>::SharedPtr io_pub_;

        private:
            void moveItPosCallback(const trajectory_msgs::msg::JointTrajectoryPoint::ConstSharedPtr msg);
            void trajectoryExecutionCallback(const std_msgs::msg::String::ConstSharedPtr msg);
            void robotControlCallback(const std_msgs::msg::String::ConstSharedPtr msg);
            void AuboAPICallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg);
            void teachCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg);
            void timerCallback();
            void publishJointStateAndFeedbackLoop();  // 独立线程固定 50Hz 发布，避免 timer 抖动
            void feedToRosMotionLoop();               // 独立线程固定 500Hz 从 buf_queue_ 取点，避免主线程 spin 延迟导致取点间隔抖动
            void controllerSwitchCallback(const std_msgs::msg::Int32::ConstSharedPtr msg);
            void publishIOMsg();
            std::vector<aubo_robot_namespace::wayPoint_S> tryPopWaypoint(int count);
            void publishWaypointToRobot();
            int checkTargetVelc(JointParam mTaget_JointAngle, JointParam mLast_JointAngle, JointVelcAccParam &mJointVelc);
            int checkTargetAcc(JointVelcAccParam mLastJointVelc, JointVelcAccParam &mTargetJointVelc);
            bool setRobotJointsByMoveIt();

            bool reverse_connected_;
            double last_recieve_point_[ARM_DOF];
            int control_option_;
            bool emergency_stopped_;
            bool protective_stopped_;
            bool normal_stopped_;
            bool data_recieved_;
            int data_count_;
            bool real_robot_exist_;
            bool controller_connected_flag_;
            bool start_move_;
            double current_joints_[ARM_DOF];
            double target_point_[ARM_DOF];
            JointTrajectoryInput jti;
            JointTrajectoryOutput jto;

            rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr rib_pub_;
            rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr cancle_trajectory_pub_;
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Service<aubo_msgs::srv::SetIO>::SharedPtr io_srv_;
            rclcpp::Service<aubo_msgs::srv::GetFK>::SharedPtr fk_srv_;
            rclcpp::Service<aubo_msgs::srv::GetIK>::SharedPtr ik_srv_;
            std::thread* mb_publish_thread_;
            std::thread* joint_feedback_publish_thread_;
            std::atomic<bool> publish_thread_running_{false};
            mutable std::mutex joints_mutex_;
            mutable std::mutex buf_queue_mutex_;      // 保护 buf_queue_（push/pop/size），与 feedToRosMotionLoop 线程共享
            std::thread* feed_to_ros_motion_thread_{nullptr};

            double io_flag_delay_;
            std::string server_host_;
            std::atomic<int> rib_buffer_size_;  // 控制器缓冲量，由 publishWaypointToRobot 写、updateControlStatus 读，原子避免竞态
            int control_mode_;
            int collision_class_;
            std_msgs::msg::Int32MultiArray rib_status_;
            demo_interface::msg::RobotStatus robot_status_msg_;

            int delay_clear_times;

            std::array<double, 6> ros_joint_pos_;
            struct JointPosWithTime {
                std::array<double, 6> joint_pos;
                double time_from_start;
            };
            moodycamel::ReaderWriterQueue<JointPosWithTime> ros_motion_queue_;
            aubo_robot_namespace::wayPoint_S waypoint_;
            std::array<double, 6> joint_filter_;
            double last_time_from_start_;

            ServiceInterface robot_mac_size_service_;
            std::thread* send_to_robot_thread_;
            JointVelcAccParam target_joint_velc_;
            JointVelcAccParam joint_acc_;
            JointVelcAccParam last_joint_velc_;
            bool over_speed_flag_;
    };
}

#endif /* AUBO_DRIVER_ROS2_AUBO_DRIVER_H_ */
