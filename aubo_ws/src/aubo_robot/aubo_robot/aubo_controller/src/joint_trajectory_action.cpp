/**
 * @file joint_trajectory_action.cpp
 * @brief AUBO 机器人关节轨迹动作服务器实现
 * 
 * 本文件实现了 ROS-Industrial 标准的关节轨迹动作服务器，作为 MoveIt 和机器人控制器之间的接口适配器。
 * 
 * 主要功能：
 * 1. 接收来自 MoveIt 的 FollowJointTrajectoryAction 目标（通过 actionlib）
 * 2. 将轨迹命令发布到机器人驱动节点（通过 topic: joint_path_command）
 * 3. 订阅控制器状态反馈（topic: feedback_states）和机器人状态（topic: robot_status）
 * 4. 监控轨迹执行状态，检查是否达到目标约束
 * 5. 处理紧急停止、保护停止等异常情况
 * 6. 实现看门狗机制，检测控制器是否响应
 * 
 * 工作流程：
 * - MoveIt 规划轨迹 -> 发送 FollowJointTrajectoryAction 目标
 * - 本节点接收目标 -> 验证关节名称和轨迹有效性
 * - 发布轨迹到 joint_path_command topic -> 机器人驱动节点接收并执行
 * - 驱动节点发布反馈 -> 本节点检查是否达到目标约束
 * - 达到目标且机器人停止 -> 返回成功结果给 MoveIt
 * 
 * @author AUBO Robotics
 * @date 2017-2018
 * 
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

#include "joint_trajectory_action.h"
#include "utils.h"
#include "utils_.h"
#include "param_utils.h"
#include "utils.h"
#include <map>

namespace industrial_robot_client
{
namespace joint_trajectory_action
{
// ========== 静态常量定义 ==========
const double JointTrajectoryAction::WATCHDOG_PERIOD_ = 1.0;  ///< 看门狗周期（秒），用于检测控制器是否响应
const double JointTrajectoryAction::DEFAULT_GOAL_THRESHOLD_ = 0.002;  ///< 默认目标阈值（弧度），用于判断是否到达目标位置

/**
 * @brief 构造函数：初始化关节轨迹动作服务器
 * 
 * @param controller_name 控制器名称，用于创建 action server（例如："aubo_e5_controller/follow_joint_trajectory"）
 * 
 * 初始化流程：
 * 1. 创建 action server，绑定目标回调和取消回调
 * 2. 从参数服务器读取目标阈值（goal_threshold）
 * 3. 从 robot_description 获取关节名称列表
 * 4. 过滤掉空关节名称（控制器不支持的关节）
 * 5. 创建发布者：发布轨迹命令到 joint_path_command
 * 6. 创建订阅者：订阅控制器状态反馈、机器人状态、轨迹执行事件
 * 7. 创建看门狗定时器，用于检测控制器响应
 * 8. 启动 action server
 */
JointTrajectoryAction::JointTrajectoryAction(std::string controller_name) :
    action_server_(node_, controller_name, boost::bind(&JointTrajectoryAction::goalCB, this, _1),
                   boost::bind(&JointTrajectoryAction::cancelCB, this, _1), false), has_active_goal_(false),
                       controller_alive_(false), has_moved_once_(false)
{
  ros::NodeHandle pn("~");

  // 从参数服务器读取目标阈值，如果不存在则使用默认值 0.002 弧度
  pn.param("constraints/goal_threshold", goal_threshold_, DEFAULT_GOAL_THRESHOLD_);

  // 从 robot_description 参数中获取控制器关节名称列表
  if (!industrial_utils::param::getJointNames("controller_joint_names", "robot_description", joint_names_))
    ROS_ERROR("Failed to initialize joint_names.");

  // 控制器关节名称参数可能包含空字符串（表示控制器不支持的关节）
  // 这些空字符串需要被移除，因为轨迹动作应该忽略这些关节
  std::remove(joint_names_.begin(), joint_names_.end(), std::string());
  ROS_INFO_STREAM("Filtered joint names to " << joint_names_.size() << " joints");

  // 创建发布者：发布轨迹命令到机器人驱动节点
  pub_trajectory_command_ = node_.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 100);
  
  // 创建订阅者：订阅控制器状态反馈（由机器人驱动节点发布）
  sub_trajectory_state_ = node_.subscribe("feedback_states", 1, &JointTrajectoryAction::controllerStateCB, this);
  
  // 创建订阅者：订阅机器人状态（紧急停止、保护停止、运动状态等）
  sub_robot_status_ = node_.subscribe("robot_status", 1, &JointTrajectoryAction::robotStatusCB, this);
  
  // 创建订阅者：订阅轨迹执行事件（用于外部停止轨迹执行）
  trajectory_execution_subs_ = node_.subscribe("trajectory_execution_event", 1, &JointTrajectoryAction::trajectoryExecutionCallback,this);

  // 创建看门狗定时器：如果控制器在 WATCHDOG_PERIOD_ 秒内没有响应，将中止当前目标
  // 第三个参数 true 表示单次触发（one-shot），需要手动重启
  watchdog_timer_ = node_.createTimer(ros::Duration(WATCHDOG_PERIOD_), &JointTrajectoryAction::watchdog, this, true);
  
  // 启动 action server，开始接收来自 MoveIt 的目标
  action_server_.start();
}

JointTrajectoryAction::~JointTrajectoryAction()
{
}

/**
 * @brief 轨迹执行事件回调函数
 * 
 * 当外部节点（如用户程序）发布轨迹执行事件时，此回调函数被触发。
 * 主要用于处理外部停止请求。
 * 
 * @param msg 轨迹执行事件消息，如果 data == "stop" 则停止当前轨迹执行
 */
void JointTrajectoryAction::trajectoryExecutionCallback(const std_msgs::String::ConstPtr &msg)
{
    if(msg->data == "stop")
    {
        ROS_INFO("trajectory execution status: stop1");
        // 将当前目标标记为中止（aborted），并清除活动目标标志
        active_goal_.setAborted();
        has_active_goal_ = false;
    }
}

/**
 * @brief 机器人状态回调函数
 * 
 * 当机器人驱动节点发布机器人状态消息时，此回调函数被触发。
 * 主要用于检测紧急停止、保护停止等异常状态，并相应地中止轨迹执行。
 * 
 * @param msg 机器人状态消息，包含：
 *   - e_stopped: 紧急停止状态
 *   - in_error: 保护停止/错误状态
 *   - in_motion: 机器人运动状态
 */
void JointTrajectoryAction::robotStatusCB(const industrial_msgs::RobotStatusConstPtr &msg)
{
  // 缓存机器人状态，供后续使用（例如在检查目标完成时使用）
  last_robot_status_ = msg;
  
  // 更新 has_moved_once_ 标志：如果机器人曾经运动过，则保持为 true；
  // 否则，检查当前状态是否为运动状态
  has_moved_once_ = has_moved_once_ ? true : (last_robot_status_->in_motion.val == industrial_msgs::TriState::TRUE);

  // 检查紧急停止或保护停止状态
  if (msg->e_stopped.val == 1 /*isEmergencyStopped()*/|| msg->in_error.val == 1 /*isProtectiveStopped()*/)
  {
      if (msg->e_stopped.val == 1)
      {
          ROS_INFO("Emergency stop pressed!");
      }
      else
      {
           ROS_INFO("Robot is protective stopped!");
      }
      
      // 如果有活动目标，则中止轨迹执行
      if(has_active_goal_)
      {
          ROS_INFO("Aborting trajectory");
          active_goal_.setAborted();
          has_active_goal_ = false;
      }
  }
}

/**
 * @brief 看门狗定时器回调函数
 * 
 * 如果控制器在 WATCHDOG_PERIOD_ 秒内没有发布状态反馈，此函数被触发。
 * 这表明控制器可能已经停止响应或出现故障，需要中止当前目标。
 * 
 * 注意：此定时器是单次触发的（one-shot），每次收到控制器反馈时会重启定时器。
 * 
 * @param e 定时器事件（包含时间戳等信息）
 */
void JointTrajectoryAction::watchdog(const ros::TimerEvent &e)
{
  // 调试日志：如果从未收到过控制器状态消息
  if (!last_trajectory_state_)
  {
    ROS_DEBUG("Waiting for subscription to joint trajectory state");
  }

  ROS_WARN("Trajectory state not received for %f seconds", WATCHDOG_PERIOD_);
  controller_alive_ = false;

  // 如果控制器看起来不活跃，则中止活动目标
  if (has_active_goal_)
  {
    // 如果订阅者从未建立连接，last_trajectory_state_ 将为 null
    if (!last_trajectory_state_)
    {
      ROS_WARN("Aborting goal because we have never heard a controller state message.");
    }
    else
    {
      ROS_WARN_STREAM("Aborting goal because we haven't heard from the controller in " << WATCHDOG_PERIOD_ << " seconds");
    }
    abortGoal();
  }
}

/**
 * @brief Action Server 目标回调函数
 * 
 * 当 MoveIt 或其他节点发送 FollowJointTrajectoryAction 目标时，此函数被调用。
 * 
 * 处理流程：
 * 1. 检查控制器是否活跃（是否收到过反馈）
 * 2. 验证轨迹是否为空
 * 3. 验证关节名称是否匹配
 * 4. 如果有活动目标，先取消当前目标
 * 5. 接受新目标，保存轨迹，计算检查时间点
 * 6. 发布轨迹命令到机器人驱动节点
 * 
 * @param gh 目标句柄，包含轨迹信息和目标约束
 */
void JointTrajectoryAction::goalCB(const JointTractoryActionServer::GoalHandle &gh)
{
  ROS_INFO("Received new goal");
  JointTractoryActionServer::GoalHandle tmp_gh = const_cast<JointTractoryActionServer::GoalHandle&>(gh);

  // 如果还没有收到远程控制器的反馈，拒绝所有目标
  // 这确保只有在控制器正常工作时才接受目标
  if (!controller_alive_)
  {
    ROS_ERROR("Joint trajectory action rejected: waiting for (initial) feedback from controller");
    control_msgs::FollowJointTrajectoryResult rslt;
    rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    tmp_gh.setRejected(rslt, "Waiting for (initial) feedback from controller");

    // 已经拒绝，无需继续处理
    return;
  }

  // 检查轨迹是否为空
  if (!tmp_gh.getGoal()->trajectory.points.empty())
  {
    // 检查目标轨迹的关节名称是否与控制器期望的关节名称匹配
    if (industrial_utils::isSimilar(joint_names_, tmp_gh.getGoal()->trajectory.joint_names))
    {
      // 如果已有活动目标，先取消当前目标
      if (has_active_goal_)
      {
        ROS_WARN("Received new goal, canceling current goal");
        abortGoal();
      }

      // 接受新目标
      tmp_gh.setAccepted();
      active_goal_ = tmp_gh;
      has_active_goal_ = true;
      
      // 计算开始检查目标完成的时间点：轨迹执行到一半时开始检查
      // 这样可以避免在轨迹刚开始执行时就检查完成状态
      time_to_check_ = ros::Time::now() +
          ros::Duration(active_goal_.getGoal()->trajectory.points.back().time_from_start.toSec() / 2.0);
      has_moved_once_ = false;

      ROS_INFO("Publishing trajectory");

      // 保存当前轨迹并发布到机器人驱动节点
      current_traj_ = active_goal_.getGoal()->trajectory;
      pub_trajectory_command_.publish(current_traj_);

    }
    else
    {
      // 关节名称不匹配，拒绝目标
      ROS_ERROR("Joint trajectory action failing on invalid joints");
      control_msgs::FollowJointTrajectoryResult rslt;
      rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
      tmp_gh.setRejected(rslt, "Joint names do not match");
    }
  }
  else
  {
    // 轨迹为空，拒绝目标
    ROS_ERROR("Joint trajectory action failed on empty trajectory");
    control_msgs::FollowJointTrajectoryResult rslt;
    rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    tmp_gh.setRejected(rslt, "Empty trajectory");
  }

  // 添加一些信息性日志消息，指示不支持的目标约束
  // 这些约束在目标消息中可能被指定，但 ROS-Industrial 驱动不支持
  if (tmp_gh.getGoal()->goal_time_tolerance.toSec() > 0.0)
  {
    ROS_WARN_STREAM("Ignoring goal time tolerance in action goal, may be supported in the future");
  }
  if (!tmp_gh.getGoal()->goal_tolerance.empty())
  {
    ROS_WARN_STREAM(
        "Ignoring goal tolerance in action, using paramater tolerance of " << goal_threshold_ << " instead");
  }
  if (!tmp_gh.getGoal()->path_tolerance.empty())
  {
    ROS_WARN_STREAM("Ignoring goal path tolerance, option not supported by ROS-Industrial drivers");
  }
}

/**
 * @brief Action Server 取消回调函数
 * 
 * 当 MoveIt 或其他节点请求取消当前目标时，此函数被调用。
 * 
 * 处理流程：
 * 1. 验证取消请求是否针对当前活动目标
 * 2. 发布空轨迹命令到控制器（停止机器人）
 * 3. 将目标标记为已取消
 * 4. 清除活动目标标志
 * 
 * @param gh 要取消的目标句柄
 */
void JointTrajectoryAction::cancelCB(const JointTractoryActionServer::GoalHandle &gh)
{
  JointTractoryActionServer::GoalHandle tmp_gh = const_cast<JointTractoryActionServer::GoalHandle&>(gh);
  ROS_DEBUG("Received action cancel request");
  
  // 检查取消请求是否针对当前活动目标
  if (active_goal_ == gh)
  {
    // 停止控制器：发布空轨迹命令
    // 机器人驱动节点收到空轨迹后会停止当前运动
    trajectory_msgs::JointTrajectory empty;
    empty.joint_names = joint_names_;
    pub_trajectory_command_.publish(empty);

    // 将当前目标标记为已取消
    tmp_gh.setCanceled();
    has_active_goal_ = false;
  }
  else
  {
    ROS_WARN("Active goal and goal cancel do not match, ignoring cancel request");
  }
}

/**
 * @brief 控制器状态回调函数
 * 
 * 当机器人驱动节点发布关节轨迹反馈消息时，此回调函数被触发。
 * 这是检查目标完成状态的核心函数。
 * 
 * 处理流程：
 * 1. 更新控制器活跃状态，重启看门狗定时器
 * 2. 检查是否有活动目标
 * 3. 验证关节名称是否匹配
 * 4. 检查是否到达检查时间点（轨迹执行到一半）
 * 5. 检查是否达到目标约束
 * 6. 检查机器人是否已停止运动
 * 7. 如果满足所有条件，标记目标为成功
 * 
 * @param msg 关节轨迹反馈消息，包含当前关节位置、速度、加速度等信息
 */
void JointTrajectoryAction::controllerStateCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg)
{
  ROS_DEBUG("Checking controller state feedback");
  
  // 缓存最新的轨迹状态
  last_trajectory_state_ = msg;
  controller_alive_ = true;

  // 重启看门狗定时器：收到反馈说明控制器正常响应
  watchdog_timer_.stop();
  watchdog_timer_.start();

  // 如果没有活动目标，忽略反馈
  if (!has_active_goal_)
  {
    //ROS_DEBUG("No active goal, ignoring feedback");
    return;
  }
  
  // 如果当前轨迹为空，忽略反馈
  if (current_traj_.points.empty())
  {
    ROS_DEBUG("Current trajectory is empty, ignoring feedback");
    return;
  }

  // 验证控制器反馈的关节名称是否与期望的关节名称匹配
  if (!industrial_utils::isSimilar(joint_names_, msg->joint_names))
  {
    ROS_ERROR("Joint names from the controller don't match our joint names.");
    return;
  }

  // 如果机器人还没有运动过，且当前时间还未到达检查时间点，则等待
  // 这样可以避免在轨迹刚开始执行时就检查完成状态
  if (!has_moved_once_ && (ros::Time::now() < time_to_check_))
  {
    ROS_DEBUG("Waiting to check for goal completion until halfway through trajectory");
    return;
  }

  // ========== 检查目标约束 ==========
  // 检查机器人是否已经到达目标约束范围内，并且已经停止运动
  
  ROS_DEBUG("Checking goal constraints");
  if (withinGoalConstraints(last_trajectory_state_, current_traj_))
  {
    // 如果机器人状态可用，进行额外的运动停止检查
    // 因为控制器目标可能仍在移动，需要确认机器人实际已停止
    if (last_robot_status_)
    {
      // 当前机器人驱动在收到新轨迹时如果仍在运动，会调用运动停止
      // 如果驱动没有发布运动状态（即旧驱动），这仍然可以工作，但会发出警告
      if (last_robot_status_->in_motion.val == industrial_msgs::TriState::FALSE)
      {
        // 在目标约束内，已停止运动，返回成功
        ROS_INFO("Inside goal constraints, stopped moving, return success for action");
        active_goal_.setSucceeded();
        has_active_goal_ = false;
      }
      else if (last_robot_status_->in_motion.val == industrial_msgs::TriState::UNKNOWN)
      {
        // 在目标约束内，但运动状态未知，仍然返回成功（兼容旧驱动）
        ROS_INFO("Inside goal constraints, return success for action");
        ROS_WARN("Robot status in motion unknown, the robot driver node and controller code should be updated");
        active_goal_.setSucceeded();
        has_active_goal_ = false;
      }
      else
      {
        // 在目标约束内，但机器人仍在运动，继续等待
        ROS_INFO("Within goal constraints but robot is still moving");
      }
    }
    else
    {
      // 机器人状态未发布，仍然返回成功（兼容旧驱动）
      ROS_INFO("Inside goal constraints, return success for action");
      ROS_WARN("Robot status is not being published the robot driver node and controller code should be updated");
      active_goal_.setSucceeded();
      has_active_goal_ = false;
    }
  }
}

/**
 * @brief 中止当前目标
 * 
 * 当检测到错误（如看门狗超时、紧急停止等）时，调用此函数中止当前目标。
 * 
 * 注意：当前实现中，发布空轨迹命令的代码被注释掉了。
 * 这可能是因为在某些情况下，直接中止目标而不发送停止命令更安全。
 */
void JointTrajectoryAction::abortGoal()
{
  // 停止控制器：发布空轨迹命令
  // 注意：当前代码中这行被注释掉了，可能是有意为之
  trajectory_msgs::JointTrajectory empty;
//  pub_trajectory_command_.publish(empty);

  // 将当前目标标记为已中止
  active_goal_.setAborted();
  has_active_goal_ = false;
}

/**
 * @brief 向映射表中插入键值对
 * 
 * 辅助函数，用于将关节名称和对应的值插入到映射表中。
 * 
 * @param key 关节名称
 * @param value 关节值（位置、速度等）
 * @param mappings 目标映射表
 * @return true 如果插入成功，false 如果键已存在
 */
bool mapInsert(const std::string & key, double value, std::map<std::string, double> & mappings)
{
  bool rtn = false;

  std::pair<std::map<std::string, double>::iterator, bool> insert_rtn;

  // 尝试插入键值对
  insert_rtn = mappings.insert(std::make_pair(key, value));

  // insert 返回的第二个值是布尔值（true 表示成功，false 表示键已存在）
  if (!insert_rtn.second)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << "::Failed to insert item into map with key: " << key);
    rtn = false;
  }
  else
  {
    rtn = true;
  }
  return rtn;

}


/**
 * @brief 将键值对向量转换为映射表
 * 
 * 辅助函数，用于将关节名称向量和值向量转换为映射表，方便后续查找。
 * 
 * @param keys 关节名称向量
 * @param values 关节值向量（位置、速度等）
 * @param mappings 输出的映射表
 * @return true 如果转换成功，false 如果键和值的大小不匹配或插入失败
 */
bool toMap(const std::vector<std::string> & keys, const std::vector<double> & values,
           std::map<std::string, double> & mappings)
{
  bool rtn;

  mappings.clear();

  // 检查键和值的大小是否匹配
  if (keys.size() == values.size())
  {
    rtn = true;

    // 逐个插入键值对
    for (size_t i = 0; i < keys.size(); ++i)
    {
      rtn = mapInsert(keys[i], values[i], mappings);
      if (!rtn)
      {
        break;
      }
    }

  }
  else
  {
    ROS_ERROR_STREAM(__FUNCTION__ << "::keys size: " << keys.size()
                     << " does not match values size: " << values.size());

    rtn = false;
  }

  return rtn;
}

/**
 * @brief 检查两个映射表中的值是否在指定范围内
 * 
 * 辅助函数，用于检查当前关节位置是否在目标位置的容差范围内。
 * 
 * @param keys 关节名称向量（用于遍历）
 * @param lhs 左侧映射表（例如：当前关节位置）
 * @param rhs 右侧映射表（例如：目标关节位置）
 * @param full_range 完整范围（容差），实际比较时使用 half_range = full_range / 2.0
 * @return true 如果所有关节的值都在范围内，false 否则
 */
bool isWithinRange(const std::vector<std::string> & keys, const std::map<std::string, double> & lhs,
                   const std::map<std::string, double> & rhs, double full_range)
{
  bool rtn = false;

  // 检查大小是否匹配
  if ((keys.size() != rhs.size()) || (keys.size() != lhs.size()))
  {
    ROS_ERROR_STREAM(__FUNCTION__ << "::Size mistmatch ::lhs size: " << lhs.size() <<
                     " rhs size: " << rhs.size() << " key size: " << keys.size());

    rtn = false;
  }
  else
  {
    // 计算半范围：虽然会导致一些精度损失，但足够使用
    double half_range = full_range / 2.0;
    rtn = true; // 假设在范围内，在下面的循环中捕获异常

    // 如果向量为空，此循环不会运行，结果为 true
    for (size_t i = 0; i < keys.size(); ++i)
    {
      // 检查每个关节的差值是否超过半范围
      if (fabs(lhs.at(keys[i]) - rhs.at(keys[i])) > fabs(half_range))
      {
        rtn = false;
        break;
      }
    }

  }

  return rtn;
}


/**
 * @brief 检查两个向量对是否在指定范围内（重载版本）
 * 
 * 辅助函数的重载版本，接受向量而不是映射表。
 * 先将向量转换为映射表，然后调用另一个 isWithinRange 函数。
 * 
 * @param lhs_keys 左侧关节名称向量
 * @param lhs_values 左侧关节值向量
 * @param rhs_keys 右侧关节名称向量
 * @param rhs_values 右侧关节值向量
 * @param full_range 完整范围（容差）
 * @return true 如果所有关节的值都在范围内，false 否则
 */
bool isWithinRange(const std::vector<std::string> & lhs_keys, const std::vector<double> & lhs_values,
                   const std::vector<std::string> & rhs_keys, const std::vector<double> & rhs_values, double full_range)
{
  bool rtn = false;
  std::map<std::string, double> lhs_map;
  std::map<std::string, double> rhs_map;
  
  // 检查关节名称是否相似（顺序可能不同，但内容相同）
  if (industrial_utils::isSimilar(lhs_keys, rhs_keys))
  {
    // 将向量转换为映射表
    if (toMap(lhs_keys, lhs_values, lhs_map) && toMap(rhs_keys, rhs_values, rhs_map))
    {
      // 调用映射表版本的 isWithinRange
      rtn = isWithinRange(lhs_keys, lhs_map, rhs_map, full_range);
    }
  }
  else
  {
    ROS_ERROR_STREAM(__FUNCTION__ << "::Key vectors are not similar");
    rtn = false;
  }
  return rtn;
}

/**
 * @brief 检查是否在目标约束范围内
 * 
 * 核心函数，用于判断机器人当前关节位置是否在目标轨迹最后一个点的容差范围内。
 * 
 * 算法：
 * 1. 获取轨迹的最后一个点（目标点）
 * 2. 比较当前关节位置（来自反馈消息）和目标关节位置
 * 3. 使用 goal_threshold_ 作为容差，检查每个关节的差值是否在容差范围内
 * 
 * @param msg 关节轨迹反馈消息，包含当前关节位置
 * @param traj 目标轨迹，取其最后一个点作为目标位置
 * @return true 如果所有关节都在目标约束范围内，false 否则
 */
bool JointTrajectoryAction::withinGoalConstraints(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg,
                                                  const trajectory_msgs::JointTrajectory & traj)
{
  bool rtn = false;
  
  // 检查轨迹是否为空
  if (traj.points.empty())
  {
    ROS_WARN("Empty joint trajectory passed to check goal constraints, return false");
    rtn = false;
  }
  else
  {
    // 获取轨迹的最后一个点（目标点）
    int last_point = traj.points.size() - 1;

    // 调试代码（已注释）：打印当前关节位置和目标关节位置
//    std::cout<<"last_trajectory_state"<<last_trajectory_state_->joint_names[0]<<","<<last_trajectory_state_->joint_names[1]<<","<<last_trajectory_state_->joint_names[2]
//                                                    <<","<<last_trajectory_state_->joint_names[3]<<","<<last_trajectory_state_->joint_names[4]
//                                                    <<","<<last_trajectory_state_->joint_names[5]<<","<<last_trajectory_state_->joint_names[6]<<std::endl;
//    std::cout<<"traj.joint_names"<<traj.joint_names[0]<<","<<traj.joint_names[1]<<","<<traj.joint_names[2]
//                                                    <<","<<traj.joint_names[3]<<","<<traj.joint_names[4]
//                                                    <<","<<traj.joint_names[5]<<","<<traj.joint_names[6]<<std::endl;


//    ROS_INFO("laset position,%f,%f,%f,%f,%f,%f,%f",last_trajectory_state_->actual.positions[0],last_trajectory_state_->actual.positions[1],last_trajectory_state_->actual.positions[2],last_trajectory_state_->actual.positions[3],
//            last_trajectory_state_->actual.positions[4],last_trajectory_state_->actual.positions[5],last_trajectory_state_->actual.positions[6]);

//    ROS_INFO("current position,%f,%f,%f,%f,%f,%f,%f",traj.points[last_point].positions[0],traj.points[last_point].positions[1],traj.points[last_point].positions[2],traj.points[last_point].positions[3],
//            traj.points[last_point].positions[4],traj.points[last_point].positions[5],traj.points[last_point].positions[6]);

    // 检查当前关节位置是否在目标位置的容差范围内
    // 使用 goal_threshold_ 作为容差（例如 0.002 弧度）
    if (industrial_robot_client::utils::isWithinRange(last_trajectory_state_->joint_names,
                                                      last_trajectory_state_->actual.positions, traj.joint_names,
                                                      traj.points[last_point].positions, goal_threshold_))
    {
      rtn = true;
    }
    else
    {
      rtn = false;
    }
  }
  return rtn;
}

} //joint_trajectory_action
} //industrial_robot_client


/**
 * @brief 主函数：启动关节轨迹动作服务器节点
 * 
 * 此节点应该在机器人描述（robot_description）加载后启动。
 * 
 * 工作流程：
 * 1. 初始化 ROS 节点
 * 2. 从参数服务器获取机器人名称（/robot_name）
 * 3. 等待机器人描述启动（robot_name 不为空）
 * 4. 根据机器人名称确定控制器名称（action server 的命名空间）
 * 5. 创建 JointTrajectoryAction 实例
 * 6. 运行节点（进入 ROS 事件循环）
 * 
 * 支持的机器人型号：
 * - aubo_i3, aubo_i5, aubo_i5l, aubo_i7, aubo_i10, aubo_i16, aubo_i20
 * - aubo_e3, aubo_e5
 * - aubo_iS7, aubo_iS20
 * 
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 0 正常退出
 */
/** This node should be loaded after the robot description**/
using industrial_robot_client::joint_trajectory_action::JointTrajectoryAction;
int main(int argc, char** argv)
{
  // 初始化 ROS 节点
  ros::init(argc, argv, "aubo_joint_follow_action");

  std::string robot_name, controller_name;
  
  // 从参数服务器获取机器人名称
  ros::param::get("/robot_name", robot_name);
  
  // 等待机器人描述启动：robot_name 参数必须被设置
  while(robot_name == "")
  {
    sleep(1);
    ROS_INFO("Waiting for the robot description to start up!");
    ros::param::get("/robot_name", robot_name);
  }
  
  // 根据机器人名称确定控制器名称（action server 的命名空间）
  // 控制器名称格式：{robot_name}_controller/follow_joint_trajectory
  if(robot_name == "aubo_i5")
      controller_name = "aubo_i5_controller/follow_joint_trajectory";
  else if(robot_name == "aubo_i3")
          controller_name = "aubo_i3_controller/follow_joint_trajectory";
  else if(robot_name == "aubo_i7")
          controller_name = "aubo_i7_controller/follow_joint_trajectory";
  else if(robot_name == "aubo_i10")
          controller_name = "aubo_i10_controller/follow_joint_trajectory";
  else if(robot_name == "aubo_i5l")
          controller_name = "aubo_i5l_controller/follow_joint_trajectory";
  else if(robot_name == "aubo_i16")
          controller_name = "aubo_i16_controller/follow_joint_trajectory";
  else if(robot_name == "aubo_e5")
          controller_name = "aubo_e5_controller/follow_joint_trajectory";
  else if(robot_name == "aubo_e3")
          controller_name = "aubo_e3_controller/follow_joint_trajectory";
  else if(robot_name == "aubo_i20")
          controller_name = "aubo_i20_controller/follow_joint_trajectory";
  else if(robot_name == "aubo_iS20")
          controller_name = "aubo_is20_controller/follow_joint_trajectory";
  else if(robot_name == "aubo_iS7")
          controller_name = "aubo_iS7_controller/follow_joint_trajectory";          
          
  // 创建关节轨迹动作服务器实例
  JointTrajectoryAction action(controller_name);
  
  // 运行节点：进入 ROS 事件循环，处理回调函数
  action.run();

  return 0;
}
