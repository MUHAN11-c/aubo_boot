/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_MOVE_TO_POSE_SERVER_H_
#define DEMO_DRIVER_MOVE_TO_POSE_SERVER_H_

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <demo_interface/MoveToPose.h>
#include <string>
#include <cstdint>

namespace demo_driver
{

/**
 * @brief 错误码枚举
 * 定义所有可能的错误代码，用于标识不同类型的错误
 */
enum class MoveToPoseErrorCode : int32_t
{
    // ========== 成功 ==========
    SUCCESS = 0,                    ///< 操作成功

    // ========== 参数验证错误 (范围: -1 到 -99) ==========
    INVALID_VELOCITY_FACTOR = -1,    ///< 速度缩放因子无效，必须在 0.0 到 1.0 之间
    INVALID_ACCELERATION_FACTOR = -2,///< 加速度缩放因子无效，必须在 0.0 到 1.0 之间

    // ========== 系统初始化错误 (范围: -100 到 -199) ==========
    MOVEIT_NOT_INITIALIZED = -100,  ///< MoveIt 接口未初始化
    IK_COMPUTATION_FAILED = -101,   ///< 逆运动学计算失败，无法将目标位姿转换为关节角度

    // ========== 异常错误 (范围: -200 到 -299) ==========
    EXCEPTION_OCCURRED = -200,      ///< 发生异常，通常是运行时错误

    // ========== 轨迹检查错误 (范围: -300 到 -399) ==========
    TRAJECTORY_EMPTY = -300,         ///< 轨迹为空，没有轨迹点
    TRAJECTORY_JOINT_NAMES_EMPTY = -301, ///< 轨迹关节名称为空
    TRAJECTORY_POINT_SIZE_MISMATCH = -302, ///< 轨迹点大小不匹配，关节数量不一致
    TRAJECTORY_OVERSPEED = -303      ///< 轨迹超速，检测到关节速度超过限制
};

/**
 * @brief 移动到目标位姿服务服务器类
 * 提供 MoveIt 运动规划服务，支持关节空间和笛卡尔空间规划
 */
class MoveToPoseServer
{
public:
    MoveToPoseServer();  // 构造函数
    ~MoveToPoseServer(); // 析构函数

    void spin();  // 主循环，保持节点运行

private:
    // ROS 节点句柄
    ros::NodeHandle nh_;           // 公共节点句柄
    ros::NodeHandle private_nh_;  // 私有节点句柄（用于参数）

    // MoveIt 接口
    moveit::planning_interface::MoveGroupInterface* move_group_;  // MoveIt 运动组接口
    moveit::planning_interface::PlanningSceneInterface* planning_scene_interface_;  // 规划场景接口

    // 服务服务器
    ros::ServiceServer move_to_pose_service_;  // 移动到目标位姿服务

    // 服务回调函数
    bool moveToPoseCallback(demo_interface::MoveToPose::Request& req,
                           demo_interface::MoveToPose::Response& res);

    // 辅助函数
    bool moveToPose(const geometry_msgs::Pose& target_pose, 
                    bool use_joints,
                    float velocity_factor,
                    float acceleration_factor,
                    int32_t& error_code,
                    std::string& message);
    
    /**
     * @brief 检查轨迹是否超速
     * @param plan MoveIt 规划结果
     * @param error_code 输出错误代码
     * @param message 输出错误消息
     * @return 如果轨迹超速返回 false，否则返回 true
     * 
     * 使用与 aubo_driver 相同的逻辑检查速度：
     * 速度 = |Δjoint| / 0.005 (固定时间间隔)
     * 与关节最大速度限制比较
     */
    bool checkTrajectoryOverspeed(const moveit::planning_interface::MoveGroupInterface::Plan& plan,
                                  int32_t& error_code,
                                  std::string& message);

    // 参数
    std::string planning_group_name_;  // 规划组名称
    std::string base_frame_;           // 基础坐标系名称
    std::string end_effector_link_;    // 末端执行器链接名称
};

} // namespace demo_driver

#endif // DEMO_DRIVER_MOVE_TO_POSE_SERVER_H_

