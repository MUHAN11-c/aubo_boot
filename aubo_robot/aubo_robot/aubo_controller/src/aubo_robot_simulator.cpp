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

/**
 * @file aubo_robot_simulator.cpp
 * @brief AUBO 机器人控制器仿真器
 * 
 * 该文件实现了 AUBO 机器人的控制器仿真器，用于在没有真实机器人的情况下
 * 测试和开发 MoveIt 运动规划功能。仿真器遵循 ROS-Industrial 机器人驱动规范。
 * 
 * 主要功能：
 * - 接收轨迹命令并模拟执行
 * - 发布关节状态和反馈信息
 * - 支持5次多项式轨迹插值
 * - 提供与真实控制器兼容的接口
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <time.h>
#include <string.h>
#include <vector>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <thread>
#include <algorithm>
#include <cmath>
#include <chrono>


#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "actionlib/server/action_server.h"
#include "actionlib/server/server_goal_handle.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "std_msgs/String.h"
#include <controller_manager/controller_manager.h>
#include <realtime_tools/realtime_publisher.h>


#define ARM_DOF 8  // 机器人自由度数量（6个关节 + 2个扩展轴）

namespace aubo_controller {

/**
 * @class MotionControllerSimulator
 * @brief 运动控制器仿真器类
 * 
 * 负责模拟机器人运动控制器的核心功能：
 * - 接收轨迹点并存储在缓冲区
 * - 使用5次多项式插值生成平滑运动轨迹
 * - 按指定更新频率执行运动
 * - 发布关节状态给 MoveIt
 */
class MotionControllerSimulator
{
public:
    int rib_buffer_size_;              // RIB（机器人接口缓冲区）大小
    bool controller_connected_flag_;    // 控制器连接标志
    boost::mutex mutex_;               // 互斥锁，保护共享数据

protected:
    ros::NodeHandle nh_;                                    // ROS 节点句柄
    double update_rate_;                                    // 运动更新频率（Hz），影响运动平滑度
    std::queue<trajectory_msgs::JointTrajectoryPoint> motion_buffer_;  // 运动轨迹点缓冲区（FIFO队列）
//    int num_joints_;
    std::string joint_names_[ARM_DOF];  // 关节名称数组
    double joint_positions_[ARM_DOF];   // 当前关节位置数组（弧度）
    std::string initial_joint_state_;   // 初始关节状态

    bool position_updated_flag_;  // 位置更新标志
    bool sig_shutdown_;           // 关闭信号标志
    bool sig_stop_;               // 停止信号标志

    const int MINIMUM_BUFFER_SIZE_ = 250;  // 最小缓冲区大小阈值

    ros::Publisher moveit_joint_state_pub_;    // 发布关节状态给 MoveIt
    ros::Subscriber update_joint_state_subs_;   // 订阅真实关节位置更新
    std::thread* motion_thread_;               // 运动控制线程指针

public:
    /**
     * @brief 构造函数
     * @param num_joints 关节数量
     * @param update_rate 运动更新频率（Hz），更高的频率产生更平滑的仿真运动
     * @param jointnames 关节名称数组指针
     * 
     * 初始化运动控制器仿真器，设置更新频率、关节名称，并启动运动控制线程
     */
    MotionControllerSimulator(int num_joints, double update_rate, std::string *jointnames)
    {
        // 运动循环更新频率（更高的更新频率产生更平滑的仿真运动）
        update_rate_ = update_rate;
        ROS_INFO("Setting motion update rate (hz): %f", update_rate_);

        // 复制关节名称
        for(int i = 0; i < ARM_DOF; i++)
            joint_names_[i] = jointnames[i];

//        Initialize motion buffer (contains joint position lists)
//        motion_buffer_.
        std::string def_joint_names[] = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
//        joint_names_ = ros::param::get("controller_joint_names",def_joint_names);
        double joint_state[num_joints];
//        initial_joint_state_ = ros::param::get("initial_joint_state", joint_state);

        // 初始化标志和缓冲区
        rib_buffer_size_ = 0;
        controller_connected_flag_ = false;
        position_updated_flag_ = false;
        sig_shutdown_ = false;
        sig_stop_ = false;

        // 发布关节状态给 MoveIt（用于控制命令）
        moveit_joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("moveit_controller_cmd", 2000);
        // 订阅真实关节位置更新（从真实机器人驱动获取）
        update_joint_state_subs_ = nh_.subscribe("real_pose", 50, &MotionControllerSimulator::updateJointStateCallback, this);

        // 启动运动控制工作线程
        motion_thread_ = new std::thread(boost::bind(&MotionControllerSimulator::motionWorker, this));

    }

    /**
     * @brief 更新关节状态回调函数
     * @param msg 包含真实关节位置的浮点数组消息
     * 
     * 从真实机器人驱动接收当前关节位置，仅在运动过程中更新
     */
    void updateJointStateCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
    {
        if(isInMotion())
        {
            for(int i = 0; i < ARM_DOF; i++)
            {
                joint_positions_[i] = msg->data[i];
            }
            ROS_DEBUG("Update current joint state successfully!");
        }
    }

    /**
     * @brief 添加运动轨迹点
     * @param point 轨迹点（包含位置、速度、加速度和时间信息）
     * 
     * 将新的轨迹点添加到运动缓冲区队列中
     * 注意：添加新轨迹到缓冲区时，需要处理加速度信息
     */
    void addMotionWaypoint(trajectory_msgs::JointTrajectoryPoint point)
    {
        // 当添加新轨迹到缓冲区时，这里需要处理加速度信息
        motion_buffer_.push(point);
    }

    /**
     * @brief 获取当前关节位置
     * @param joint_position 输出参数，用于存储关节位置数组
     * 
     * 线程安全地获取当前所有关节的位置值
     */
    void getJointPositions(double *joint_position)
    {
        mutex_.lock();
        memcpy(joint_position, joint_positions_, ARM_DOF*sizeof(double));
        mutex_.unlock();
    }


    /**
     * @brief 运动控制工作线程函数
     * 
     * 这是运动控制的核心函数，运行在独立线程中：
     * 1. 从缓冲区取出轨迹点
     * 2. 使用5次多项式插值生成平滑轨迹
     * 3. 按更新频率执行运动
     * 4. 发布关节状态
     * 
     * 使用5次多项式插值确保位置、速度、加速度的连续性：
     * p(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵
     */
    void motionWorker()
    {
        // 运动相关变量
        double move_duration, update_duration, T, T2, T3, T4, T5, tt, ti, t1, t2, t3, t4, t5;
        // 5次多项式系数数组（每个关节一组系数）
        double a1[ARM_DOF], a2[ARM_DOF], a3[ARM_DOF], a4[ARM_DOF], a5[ARM_DOF], h[ARM_DOF], intermediate_goal_point[ARM_DOF];
        
        ros::param::set("/driver_start","1");  // 设置驱动启动标志
//        self.positionUpdatedFlag = rospy.get_param('/IsRobotConnected', '0')
//               while self.positionUpdatedFlag == '0':
//                   rospy.sleep( self.update_rate/400)
//                   self.positionUpdatedFlag = rospy.get_param('/IsRobotConnected', '0')
        ROS_INFO("Starting motion worker in motion controller simulator");
        
        // 计算更新周期（秒）
        if(update_rate_ > 0)
            update_duration = 1.0 / update_rate_;
        
        trajectory_msgs::JointTrajectoryPoint last_goal_point, current_goal_point;

        // 初始化上一个目标点为当前关节位置
        mutex_.lock();
        memcpy(&last_goal_point.positions[0], joint_positions_, ARM_DOF*sizeof(double));
        mutex_.unlock();

        // 主循环：持续处理轨迹点直到收到关闭信号
        while(!sig_shutdown_)
        {
            try
            {
                // 从缓冲区取出下一个轨迹点
                current_goal_point = motion_buffer_.front();
                motion_buffer_.pop();
                
                // 检查时间顺序，如果时间不递增，使用绝对时间
                if(current_goal_point.time_from_start <= last_goal_point.time_from_start)
                    move_duration = current_goal_point.time_from_start.toSec();
                else
                {
                    // 计算时间间隔
                    T = current_goal_point.time_from_start.toSec() - last_goal_point.time_from_start.toSec();
                    
                    // 计算5次多项式的各项系数
                    // a1: 初始速度系数
                    memcpy(a1, &last_goal_point.velocities[0], ARM_DOF*sizeof(double));
                    T2 = T * T;   // T²
                    T3 = T2 * T;  // T³
                    T4 = T3 * T;  // T⁴
                    T5 = T4 * T;  // T⁵
                    
                    // 为每个关节计算5次多项式系数
                    for(int i = 0; i < ARM_DOF; i++)
                    {
                        a2[i] = 0.5 * last_goal_point.accelerations[i];  // 初始加速度系数
                        h[i] = current_goal_point.positions[i] - last_goal_point.positions[i];  // 位置差
                        
                        // 计算3次项系数（确保位置和速度连续性）
                        a3[i] = 0.5 / T3 * (20*h[i] - (8*current_goal_point.velocities[i] + 12*last_goal_point.velocities[i])*T - (3*last_goal_point.accelerations[i] - current_goal_point.accelerations[i])*T2);
                        
                        // 计算4次项系数（确保加速度连续性）
                        a4[i] = 0.5 / T4 * (-30*h[i] + (14*current_goal_point.velocities[i] + 16*last_goal_point.velocities[i])*T + (3*last_goal_point.accelerations[i] - 2*current_goal_point.accelerations[i])*T2);
                        
                        // 计算5次项系数（平滑过渡）
                        a5[i] = 0.5 / T5 * (12*h[i] - 6*(current_goal_point.velocities[i] + last_goal_point.velocities[i])*T + (current_goal_point.accelerations[i] - last_goal_point.accelerations[i])*T2);
                    }
                    
                    // 按更新频率插值生成中间轨迹点
                    if(update_rate_ > 0)
                    {
                        tt = last_goal_point.time_from_start.toSec();  // 当前时间
                        ti = tt;  // 起始时间
                        
                        // 在起始点和目标点之间插值
                        while (tt < current_goal_point.time_from_start.toSec())
                        {
                            // 计算相对时间
                            t1 = tt - ti;
                            t2 = t1 * t1;   // t²
                            t3 = t2 * t1;   // t³
                            t4 = t3 * t1;   // t⁴
                            t5 = t4 * t1;   // t⁵
                            
                            // 使用5次多项式计算每个关节的中间位置
                            // p(t) = p0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵
                            for(int i = 0; i < ARM_DOF; i++)
                                intermediate_goal_point[i] = last_goal_point.positions[i] + a1[i]*t1 + a2[i]*t2 + a3[i]*t3 + a4[i]*t4 + a5[i]*t5;
                            
                            // 移动到中间目标点并发布状态
                            tt += update_duration;
                            moveToTarget((int)(update_duration*1000), intermediate_goal_point);
                            jointStatePublisher();
                        }
                        move_duration = current_goal_point.time_from_start.toSec() - tt;
                    }
                }
                
                // 移动到最终目标点
                moveToTarget((int)(move_duration*1000), &current_goal_point.positions[0]);
                jointStatePublisher();
                last_goal_point = current_goal_point;

            }
            catch(...)
            {
                ROS_ERROR("Unexpected exception in generating control data!");
            }
        }
        ROS_DEBUG("Shutting down motion controller!");
    }

    /**
     * @brief 检查是否在运动中
     * @return true 如果运动缓冲区不为空（有轨迹在执行）
     */
    bool isInMotion()
    {
        return !motion_buffer_.empty();
    }

    /**
     * @brief 关闭运动控制器
     * 
     * 设置关闭信号，使运动工作线程退出循环
     */
    void shutdown()
    {
        sig_shutdown_ = true;
        ROS_DEBUG("Motion_Controller shutdown signaled!");
    }

    /**
     * @brief 立即停止运动
     * 
     * 清空运动缓冲区并设置停止信号，使当前运动立即停止
     */
    void stop()
    {
        ROS_DEBUG("Motion_Controller stop signaled!");
        mutex_.lock();
        // 清空所有待执行的轨迹点
        while (!motion_buffer_.empty())
            motion_buffer_.pop();
        sig_stop_ = true;
        mutex_.unlock();
    }

    /**
     * @brief 发布关节状态
     * 
     * 将当前关节位置发布到 MoveIt 控制命令话题
     * 用于 MoveIt 的轨迹执行控制
     */
    void jointStatePublisher()
    {
        try
        {
            sensor_msgs::JointState joint_state;
            joint_state.header.stamp = ros::Time::now();
            joint_state.name.resize(ARM_DOF);
            joint_state.position.resize(ARM_DOF);
            
            // 线程安全地获取关节位置
            mutex_.lock();
            for(int i = 0; i < ARM_DOF; i++)
            {
                joint_state.name[i] = joint_names_[i];
                joint_state.position[i] = joint_positions_[i];
            }
            mutex_.unlock();
            
            // 发布关节状态给 MoveIt
            moveit_joint_state_pub_.publish(joint_state);
        }
        catch (...)
        {
             ROS_ERROR("Unexpected exception in joint state publisher!!!");
        }
    }

    /**
     * @brief 移动到目标位置
     * @param dur 移动持续时间（毫秒）
     * @param point 目标关节位置数组
     * 
     * 更新关节位置到目标值，并处理缓冲区大小和停止信号
     */
    void moveToTarget(int dur, double *point)
    {
        // 如果缓冲区过大，等待一段时间（防止生成控制数据过快）
        while(rib_buffer_size_ > MINIMUM_BUFFER_SIZE_)
        {
            ROS_INFO("generate control data too fast!");
            std::this_thread::sleep_for(std::chrono::milliseconds(dur));
        }

        // 如果缓冲区为空且控制器未连接，等待（运动开始或没有机器人连接）
        if(rib_buffer_size_ == 0 && controller_connected_flag_ == false)
            std::this_thread::sleep_for(std::chrono::milliseconds(dur));

        // 线程安全地更新关节位置
        mutex_.lock();
        if(!sig_stop_)
        {
            // 复制目标位置到当前关节位置
            memcpy(joint_positions_, &point[0], ARM_DOF*sizeof(double));
            ROS_DEBUG("Moved to target position!");
        }
        else
        {
            // 如果收到停止信号，清除停止标志但不更新位置
            ROS_INFO("Stopping motion immediately, clearing stop signal");
            sig_stop_ = false;
        }
        mutex_.unlock();

    }

private:

};


/**
 * @class AuboRobotSimulatorNode
 * @brief AUBO 机器人仿真器节点类
 * 
 * 该类模拟 AUBO 机器人控制器，遵循 ROS-Industrial 机器人驱动规范：
 * http://www.ros.org/wiki/Industrial/Industrial_Robot_Driver_Spec
 * 
 * 主要功能：
 * - 接收轨迹命令（/joint_path_command）
 * - 发布关节状态（/joint_states）
 * - 发布轨迹执行反馈（/feedback_states）
 * - 与 MoveIt 集成，提供仿真环境
 * 
 * TODO: 当前仅支持最基本的运动接口
 * TODO: 待添加的接口：
 *   - 关节流式控制
 *   - 所有服务接口
 */
class AuboRobotSimulatorNode
{
public:

protected:
    const int JOINT_STATE_PUB_RATE = 50;  // 关节状态发布频率（Hz）
    ros::NodeHandle nh_;                  // ROS 节点句柄
    double pub_rate_;                     // 发布速率
    bool controller_enable_flag_;         // 控制器使能标志
    MotionControllerSimulator *motion_ctrl_;  // 运动控制器仿真器指针

    std::string joint_names_[ARM_DOF];    // 关节名称数组

    ros::Publisher joint_state_pub_;      // 关节状态发布器（/joint_states）
    ros::Publisher joint_feedback_pub_;   // 轨迹反馈发布器（/feedback_states）
    ros::Subscriber joint_path_sub_;      // 轨迹命令订阅器（/joint_path_command）
    ros::Subscriber plan_type_sub_;       // 规划类型订阅器（/rib_status）

    ros::Timer timer_;                    // 定时器，用于周期性发布反馈


    std::queue<trajectory_msgs::JointTrajectoryPoint> motion_buffer_;  // 运动缓冲区（未使用）
    double joint_positions_[];            // 关节位置数组（未使用）
    std::string initial_joint_state_;     // 初始关节状态（未使用）
    int rib_buffer_size_;                 // RIB 缓冲区大小（未使用）
    bool controller_connected_flag_;       // 控制器连接标志（未使用）
    bool position_updated_flag_;          // 位置更新标志（未使用）
    bool sig_shutdown_;                   // 关闭信号（未使用）
    bool sig_stop_;                       // 停止信号（未使用）

public:
    /**
     * @brief 构造函数
     * 
     * 初始化 AUBO 机器人仿真器节点：
     * - 从参数服务器获取关节名称
     * - 创建运动控制器仿真器（200Hz 更新频率）
     * - 设置话题发布器和订阅器
     * - 启动定时器用于周期性发布反馈
     */
    AuboRobotSimulatorNode()
    {
        // 初始化标志
        controller_connected_flag_ = true;
        controller_enable_flag_ = true;  // 默认使能控制器
        
        std::string joint_names_[ARM_DOF];
        std::string def_joint_names[] = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
        
        // 从参数服务器获取关节名称配置
        std::vector<std::string> jointNames;
        ros::param::get("controller_joint_names", jointNames);
        
        // 创建运动控制器仿真器（6个关节，200Hz更新频率）
        motion_ctrl_ = new MotionControllerSimulator(6, 200, &jointNames[0]);

        // 发布关节状态（供 robot_state_publisher 和 move_group 使用）
        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 100);
        
        // 发布轨迹执行反馈（供 aubo_joint_trajectory_action 使用）
        joint_feedback_pub_ = nh_.advertise<control_msgs::FollowJointTrajectoryFeedback>("feedback_states", 100);
        
        // 订阅轨迹命令（从 aubo_joint_trajectory_action 接收）
        joint_path_sub_ = nh_.subscribe("joint_path_command", 100, &AuboRobotSimulatorNode::trajectoryCallback, this);

        // 订阅 RIB 状态（从 aubo_driver 接收，用于同步缓冲区状态）
        plan_type_sub_ = nh_.subscribe("rib_status", 100, &AuboRobotSimulatorNode::ribStatusCallback, this);

//        joint_positions = new double[ARM_DOF];

        // 创建定时器，周期性发布关节反馈（50Hz）
        timer_ = nh_.createTimer(ros::Duration(1.0 / JOINT_STATE_PUB_RATE), &AuboRobotSimulatorNode::publishWorker, this);
        timer_.start();
    }

    /**
     * @brief 析构函数
     * 
     * 清理资源：停止定时器并删除运动控制器
     */
    ~AuboRobotSimulatorNode()
    {
        if(timer_.isValid())
            timer_.stop();
        if(motion_ctrl_ != NULL)
        {
            delete motion_ctrl_;
            motion_ctrl_ = NULL;
        }
    }

    /**
     * @brief 定时器回调函数 - 发布关节反馈
     * @param e 定时器事件
     * 
     * 周期性发布轨迹执行反馈信息，供 aubo_joint_trajectory_action 使用
     * 用于监控轨迹执行状态和进度
     */
    void publishWorker(const ros::TimerEvent& e)
    {
        if(controller_enable_flag_ == true /*&& motion_ctrl_.positionUpdatedFlag == true*/)
        {
            try
            {
                control_msgs::FollowJointTrajectoryFeedback joint_fb_msg;
                
                // 线程安全地获取当前关节位置
                motion_ctrl_->mutex_.lock();
                joint_fb_msg.header.stamp = ros::Time::now();
                joint_fb_msg.joint_names.resize(ARM_DOF);
                joint_fb_msg.actual.positions.resize(ARM_DOF);
                
                double joint_position[ARM_DOF];
                motion_ctrl_->getJointPositions(joint_position);
                
                // 填充反馈消息
                for(int i = 0; i < ARM_DOF; i++)
                {
                    joint_fb_msg.joint_names[i] = joint_names_[i];
                    joint_fb_msg.actual.positions[i] = joint_position[i];
                }
                
                // 发布反馈
                joint_feedback_pub_.publish(joint_fb_msg);
                motion_ctrl_->mutex_.unlock();
            }
            catch(...)
            {
                ROS_INFO("Unexpected exception in joint feedback state publisher!");
            }
        }
    }

    /**
     * @brief 轨迹命令回调函数
     * @param msg 接收到的关节轨迹消息
     * 
     * 处理从 aubo_joint_trajectory_action 接收的轨迹命令：
     * 1. 验证轨迹有效性
     * 2. 如果正在运动，停止当前轨迹（不支持轨迹拼接）
     * 3. 将轨迹点重新映射到控制器关节顺序
     * 4. 添加到运动控制器缓冲区执行
     */
    void trajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
    {
        // 如果轨迹为空或控制器未使能，不处理
        if(msg->points.size() == 0 || controller_enable_flag_ == false)
        {
           // 如果 JointTrajectory 为空或机器人由其他控制器控制，则不执行任何操作
        }
        else
        {
            ROS_DEBUG("handle joint_path_command message");
            try
            {
                ROS_INFO("Received trajectory with %d points, executing callback", msg->points.size());
                
                // 检查是否正在运动中
                if(motion_ctrl_->isInMotion())
                {
                    if(msg->points.size() > 0)
                        ROS_ERROR("Received trajectory while still in motion, trajectory splicing not supported");
                    else
                        ROS_DEBUG("Received empty trajectory while still in motion, stopping current trajectory");
                    // 停止当前运动
                    motion_ctrl_->stop();
                }
                else
                {
                    // 处理每个轨迹点
                    for(int i = 0; i < msg->points.size(); i++)
                    {
                        // 首先将轨迹点重新映射到控制器关节顺序，然后添加到控制器
                        trajectory_msgs::JointTrajectoryPoint point = msg->points[i];
                        
                        // 关节名称匹配和重新排序
                        // 将消息中的关节顺序映射到控制器期望的关节顺序
                        for(int j = 0; j < ARM_DOF; j++)
                        {
                            for(int k = 0; k < ARM_DOF; k++)
                            {
                                if(joint_names_[j] == msg->joint_names[k])
                                {
                                    // 找到匹配的关节，复制位置、速度、加速度
                                    point.positions[j] = msg->points[i].positions[k];
                                    point.velocities[j] = msg->points[i].velocities[k];
                                    point.accelerations[j] = msg->points[i].accelerations[k];
                                    break;
                                }
                            }
                        }
                        // 添加到运动控制器缓冲区
                        motion_ctrl_->addMotionWaypoint(point);
                    }
                }
            }
            catch(...)
            {
                ROS_DEBUG("Unexpected exception while adding control point to the controller!");
            }
        }
        ROS_DEBUG("Exiting trajectory callback");
    }

    /**
     * @brief RIB 状态回调函数
     * @param msg RIB 状态消息（整数数组）
     * 
     * 从 aubo_driver 接收 RIB（机器人接口缓冲区）状态信息：
     * - data[0]: 缓冲区大小
     * - data[1]: 控制器连接状态（1=ROS控制，0=其他控制）
     * - data[2]: 控制器连接标志
     * 
     * 用于同步仿真器与真实驱动器的状态
     */
    void ribStatusCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
    {
        try
        {
            // 检查控制器连接状态
            if(msg->data[1] == 1)
            {
                controller_connected_flag_ = true;
                ROS_DEBUG("The robot is controlled by the ros controller!");
            }
            else
            {
                controller_connected_flag_ = false;
                 ROS_DEBUG("The robot is not controlled by the ros controller!");
            }
            
            // 更新缓冲区大小和连接标志
            motion_ctrl_->rib_buffer_size_ = msg->data[0];
            motion_ctrl_->controller_connected_flag_ = msg->data[2];
        }
        catch(...)
        {
            ROS_DEBUG("Unexpected exception while parsing the aubo driver message!");
        }
    }

private:

};
}

/**
 * @brief 主函数
 * 
 * 初始化 ROS 节点并启动 AUBO 机器人仿真器
 * 使用异步spinner处理回调，允许多线程并发处理
 */
int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "aubo_simulator_controller");
    ros::NodeHandle nh;

    // 创建 AUBO 机器人仿真器节点（在构造函数中完成所有初始化）
    aubo_controller::AuboRobotSimulatorNode simulator_node;

    // 启动异步spinner（3个线程）处理回调函数
    // 异步spinner允许回调函数在独立线程中执行，不会阻塞主线程
    ros::AsyncSpinner spinner(3);
    spinner.start();

    // 等待关闭信号
    ros::waitForShutdown();

    exit(0);
}
