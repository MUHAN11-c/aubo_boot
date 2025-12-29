/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017-2018, AUBO Robotics
 * All rights reserved.
 *
 *
 *  Author: zhaoyu
 *  email : zhaoyu@aubo-robotics.cn
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
 * @file MoveGroupInterface_To_Noetic.cpp
 * @brief AUBO 机器人 MoveIt 运动规划演示程序
 * 
 * 该文件展示了如何使用 MoveIt 的 MoveGroupInterface API 进行各种类型的运动规划。
 * 包含10个完整的示例，涵盖从基础规划到高级场景管理的所有功能。
 * 
 * 主要功能：
 * 1. 位姿目标规划 - 规划并移动到目标位姿
 * 2. 关节空间规划 - 直接控制关节角度
 * 3. 路径约束规划 - 在保持姿态约束下规划
 * 4. 笛卡尔路径规划 - 末端执行器直线运动
 * 5. 碰撞对象管理 - 添加、移除碰撞对象
 * 6. 避障规划 - 在有障碍物的环境中规划
 * 7. 对象附加/分离 - 模拟抓取和释放
 * 
 * 使用前准备：
 * 1. 启动 move_group 节点
 * 2. 打开 RViz 并加载配置
 * 3. 确保机器人模型已正确加载
 */

// MoveIt 核心接口
#include <moveit/move_group_interface/move_group_interface.h>  // 运动规划组接口
#include <moveit/planning_scene_interface/planning_scene_interface.h>  // 规划场景接口

// MoveIt 消息类型
#include <moveit_msgs/DisplayRobotState.h>  // 机器人状态显示
#include <moveit_msgs/DisplayTrajectory.h>  // 轨迹显示

#include <moveit_msgs/AttachedCollisionObject.h>  // 附加碰撞对象
#include <moveit_msgs/CollisionObject.h>  // 碰撞对象

// 可视化工具
#include <moveit_visual_tools/moveit_visual_tools.h>  // MoveIt 可视化工具
#include <tf/LinearMath/Quaternion.h>  // 四元数工具（用于欧拉角转换）



/**
 * @brief 主函数
 * 
 * 初始化 ROS 节点和 MoveIt 接口，执行10个运动规划示例
 */
int main(int argc, char** argv)
{
  // 初始化 ROS 节点
  ros::init(argc, argv, "MoveGroupInterface_To_Noetic");
  ros::NodeHandle node_handle;

  // 启动异步spinner（单线程），用于处理回调函数
  // 异步spinner允许回调在独立线程中执行，不会阻塞主线程
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // ========================================================================
  // 初始化 MoveIt 接口
  // ========================================================================
  
  // 定义规划组名称（必须与 SRDF 文件中定义的规划组名称一致）
  static const std::string PLANNING_GROUP = "manipulator_e5";

  // 创建 MoveGroup 接口对象
  // MoveGroupInterface 是 MoveIt 的核心接口，用于运动规划和控制
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  
  // 设置位姿参考坐标系（所有位姿目标都相对于此坐标系）
  move_group.setPoseReferenceFrame("base_link");

  // 创建规划场景接口对象
  // PlanningSceneInterface 用于管理规划场景中的碰撞对象
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // 获取关节模型组信息
  // JointModelGroup 包含规划组中所有关节的模型信息
  const robot_state::JointModelGroup* joint_model_group = 
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // ========================================================================
  // 初始化可视化工具
  // ========================================================================
  
  // 创建可视化工具对象（用于在 RViz 中显示标记）
  namespace rvt = rviz_visual_tools;  // 命名空间别名
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  
  // 清除所有之前的可视化标记
  visual_tools.deleteAllMarkers();

  // 加载远程控制工具（在 RViz 中显示控制按钮）
  visual_tools.loadRemoteControl();

  // 创建文本显示位置（在 base_link 上方 1.2 米处）
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.2;
  
  // 发布文本标记到 RViz
  visual_tools.publishText(text_pose, "AUBO Demo", rvt::RED, rvt::XLARGE);
  visual_tools.trigger();  // 触发显示更新

  // ========================================================================
  // 输出基本信息
  // ========================================================================
  
  // 获取并输出规划坐标系（通常是 base_link）
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // 获取并输出末端执行器链接名称（通常是 wrist3_Link）
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // 等待用户交互（在 RViz 的 RvizVisualToolsGui 窗口中点击 'next' 按钮继续）
  visual_tools.prompt("Press 'next'1 in the RvizVisualToolsGui window to start the demo");
  
  // ========================================================================
  // 示例 0: 移动到 Home 位置（初始化）
  // ========================================================================
  // 功能：设置并移动到预定义的初始关节位置
  // 用途：在开始演示前，将机器人移动到安全且已知的初始状态

  try {
    // 设置速度缩放因子
    move_group.setMaxVelocityScalingFactor(0.1);
    // 设置加速度缩放因子
    move_group.setMaxAccelerationScalingFactor(0.1);
  } catch (const std::exception& e) {
    ROS_ERROR_NAMED("tutorial", "设置速度/加速度缩放因子失败: %s", e.what());
  }
  
  std::vector<double> home_position;
  home_position.push_back(-0.001255);   // 关节1（shoulder_joint）
  home_position.push_back(-0.148822);  // 关节2（upperArm_joint）
  home_position.push_back(-1.406503);  // 关节3（foreArm_joint）
  home_position.push_back(0.311441);    // 关节4（wrist1_joint）
  home_position.push_back(-1.571295);  // 关节5（wrist2_joint）
  home_position.push_back(-0.002450);  // 关节6（wrist3_joint）
  
  // 设置关节空间目标（直接指定每个关节的角度）
  move_group.setJointValueTarget(home_position);
  
  // 执行运动（plan + execute，自动规划并执行）
  // 验证速度缩放因子是否生效：记录执行时间
  ros::Time start_time = ros::Time::now();
  move_group.move();
  ros::Time end_time = ros::Time::now();
  double execution_time = (end_time - start_time).toSec();
  ROS_INFO_NAMED("tutorial", "移动到 Home 位置耗时: %.2f 秒 (速度缩放因子 0.1 生效时，时间会明显增加)", 
                 execution_time);


  // ========================================================================
  // 示例 1: 位姿目标规划（Pose Goal Planning）
  // ========================================================================
  // 功能：规划并移动到指定的末端执行器位姿
  // 应用场景：需要精确控制末端执行器位置和姿态的任务
  // 特点：在笛卡尔空间（任务空间）中规划，MoveIt 自动进行逆运动学求解

  // 步骤1: 设置目标位姿
  // 使用 RPY（Roll-Pitch-Yaw）欧拉角方式定义姿态，然后转换为四元数
  tf::Quaternion q;
  q.setRPY(3.14, 0, -1.57);  // Roll=180°, Pitch=0°, Yaw=-90°（单位：弧度）

  // 定义目标位姿（相对于 base_link 坐标系）
  geometry_msgs::Pose target_pose1;
  target_pose1.position.x = -0.4;   // X 坐标（米）
  target_pose1.position.y = -0.3;   // Y 坐标（米）
  target_pose1.position.z = 0.30;   // Z 坐标（米）
  
  // 将四元数赋值给位姿（四元数表示旋转，避免万向锁问题）
  target_pose1.orientation.x = q.x();
  target_pose1.orientation.y = q.y();
  target_pose1.orientation.z = q.z();
  target_pose1.orientation.w = q.w();

  // 设置位姿目标（MoveIt 会自动进行逆运动学求解）
  move_group.setPoseTarget(target_pose1);

  // 步骤2: 进行运动规划（仅规划，不执行）
  // Plan 对象包含完整的轨迹信息（位置、速度、加速度等）
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == 
                 moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", 
                 success ? "Success" : "FAILED");


  // 步骤3: 在 RViz 中可视化规划结果
  visual_tools.deleteAllMarkers();  // 清除之前的标记
  visual_tools.publishAxisLabeled(target_pose1, "pose1");  // 显示目标位姿的坐标轴
  visual_tools.publishText(text_pose, "AUBO Pose Goal Example1", 
                          rvt::RED, rvt::XLARGE);
  // 发布轨迹线（在 RViz 中显示规划的路径）
  // 参数1: 轨迹信息（包含所有轨迹点）
  // 参数2: 关节模型组（用于可视化机器人模型）
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();  // 触发显示更新

  // 步骤4: 执行规划好的轨迹
  move_group.execute(my_plan);
  // 步骤5: 返回 Home 位置
  move_group.setJointValueTarget(home_position);
  move_group.move();

  // 等待用户交互，继续下一个示例
  visual_tools.prompt("Press 'next'2 in the RvizVisualToolsGui window to continue the demo");







  // ========================================================================
  // 示例 2: 关节空间目标规划（Joint Space Goal Planning）
  // ========================================================================
  // 功能：直接控制关节角度，在关节空间中规划
  // 应用场景：需要精确控制单个或多个关节角度的任务
  // 特点：不需要逆运动学求解，规划速度快，但末端位置不直观

  // 步骤1: 获取当前机器人状态
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

  // 步骤2: 获取当前规划组中所有关节的角度值
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // 步骤3: 修改关节1的角度（旋转90度）
  // 注意：关节索引从0开始，joint_group_positions[0] 对应第一个关节
  joint_group_positions[0] = -1.57;  // -90度（单位：弧度，约等于 -π/2）
  
  // 设置关节空间目标（直接指定每个关节的角度）
  move_group.setJointValueTarget(joint_group_positions);

  // 步骤4: 进行规划
  success = (move_group.plan(my_plan) == 
            moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", 
                 success ? "success" : "FAILED");

  // 步骤5: 可视化规划结果
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "AUBO Joint Space Goal Example2", 
                          rvt::RED, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  // 步骤6: 执行规划
  move_group.execute(my_plan);

  // 步骤7: 恢复关节1到0度位置
  joint_group_positions[0] = 0;  // 0度（单位：弧度）
  move_group.setJointValueTarget(joint_group_positions);
  move_group.move();

  visual_tools.prompt("Press 'next'3 in the RvizVisualToolsGui window to continue the demo");


  // ========================================================================
  // 示例 3: 路径约束规划（Path Constraints Planning）
  // ========================================================================
  // 功能：在保持末端执行器姿态约束的情况下规划路径
  // 应用场景：需要保持末端执行器姿态不变的任务（如倒水、焊接、打磨等）
  // 特点：规划时间较长，但能确保末端执行器在整个路径中保持指定姿态

  // 步骤1: 设置目标姿态（RPY: Roll=180°, Pitch=0°, Yaw=-90°）
  q.setRPY(3.14, 0, -1.57);

  // 步骤2: 定义方向约束（Orientation Constraint）
  // 约束末端执行器在整个运动过程中保持指定的姿态
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "wrist3_Link";      // 要约束的链接名称（末端执行器）
  ocm.header.frame_id = "base_link";   // 参考坐标系

  // 设置目标姿态（四元数）
  ocm.orientation.w = q.w();
  ocm.orientation.x = q.x();
  ocm.orientation.y = q.y();
  ocm.orientation.z = q.z();
  
  // 设置各轴的容差（允许的姿态偏差，单位：弧度）
  ocm.absolute_x_axis_tolerance = 0.2;  // X轴容差（约11.5度）
  ocm.absolute_y_axis_tolerance = 0.2;  // Y轴容差（约11.5度）
  ocm.absolute_z_axis_tolerance = 0.2;  // Z轴容差（约11.5度）
  ocm.weight = 1.0;  // 约束权重（1.0表示完全约束）

  // 步骤3: 将约束添加到约束集合中
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  
  // 步骤4: 设置路径约束（MoveIt 会在规划时考虑这些约束）
  move_group.setPathConstraints(test_constraints);



  // 步骤5: 设置起始位置
  robot_state::RobotState start_state(*move_group.getCurrentState());
  geometry_msgs::Pose start_pose2;
  start_pose2.position.x = -0.4;   // 起始位置 X
  start_pose2.position.y = 0.05;   // 起始位置 Y
  start_pose2.position.z = 0.54;   // 起始位置 Z
  // 设置起始姿态（将 tf::Quaternion 转换为 geometry_msgs::Quaternion）
  start_pose2.orientation.x = q.x();
  start_pose2.orientation.y = q.y();
  start_pose2.orientation.z = q.z();
  start_pose2.orientation.w = q.w();

  // 机械臂首先需要运动到起始位置
  move_group.setPoseTarget(start_pose2);
  move_group.move();

  // 步骤6: 设置起始状态（用于可视化轨迹显示）
  // 通过逆运动学求解起始位姿对应的关节角度
  start_state.setFromIK(joint_model_group, start_pose2);
  move_group.setStartState(start_state);

  // 步骤7: 设置目标位姿（在保持姿态约束下移动到目标位置）
  geometry_msgs::Pose target_pose3_1;
  target_pose3_1.position.x = -0.4;   // 目标位置 X（与起始位置相同）
  target_pose3_1.position.y = -0.19;   // 目标位置 Y（向前移动）
  target_pose3_1.position.z = 0.41;    // 目标位置 Z（向下移动）
  // 设置目标姿态（将 tf::Quaternion 转换为 geometry_msgs::Quaternion）
  target_pose3_1.orientation.x = q.x();
  target_pose3_1.orientation.y = q.y();
  target_pose3_1.orientation.z = q.z();
  target_pose3_1.orientation.w = q.w();
  move_group.setPoseTarget(target_pose3_1);

  // 步骤8: 增加规划时间
  // 约束规划比普通规划更复杂，需要更长的规划时间
  // 默认规划时间为5秒，增加到20秒可以提高成功率
  move_group.setPlanningTime(20.0);

  // 步骤9: 进行约束规划
  success = (move_group.plan(my_plan) == 
            moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", 
                 success ? "success" : "FAILED");

  // 步骤10: 可视化规划结果
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(start_pose2, "start");      // 显示起始位姿
  visual_tools.publishAxisLabeled(target_pose3_1, "goal");   // 显示目标位姿
  visual_tools.publishText(text_pose, "AUBO Constrained Goal Example3", 
                          rvt::RED, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  // 步骤11: 执行规划
  move_group.execute(my_plan);

  // 步骤12: 返回起始位置
  move_group.setPoseTarget(start_pose2);
  move_group.move();

  visual_tools.prompt("next step 4");

  // 步骤13: 清除路径约束（重要：约束会影响后续规划，使用完后应清除）
  move_group.clearPathConstraints();







  // ========================================================================
  // 示例 4: 笛卡尔路径规划（Cartesian Path Planning）
  // ========================================================================
  // 功能：计算末端执行器在笛卡尔空间中的直线插值路径
  // 应用场景：需要精确控制末端执行器路径的任务（如切割、涂胶、焊接等）
  // 特点：末端执行器在路径点之间沿直线移动，路径可预测

  // 步骤1: 定义路径点（Waypoints）
  // 路径点定义了末端执行器需要经过的一系列位置
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(start_pose2);  // 第一个路径点（起始位置）

  geometry_msgs::Pose target_pose3 = start_pose2;

  // 第二个路径点：向下移动 0.2 米
  target_pose3.position.z -= 0.2;
  waypoints.push_back(target_pose3);

  // 第三个路径点：向右移动 0.15 米（在当前位置基础上）
  target_pose3.position.y -= 0.15;
  waypoints.push_back(target_pose3);

  // 第四个路径点：向上移动 0.2 米，向左移动 0.2 米，向前移动 0.2 米
  target_pose3.position.z += 0.2;
  target_pose3.position.y += 0.2;
  target_pose3.position.x -= 0.2;
  waypoints.push_back(target_pose3);

  // 步骤2: 设置速度和加速度缩放因子
  // 降低机器人运动速度和加速度（0.5 表示最大值的50%）
  // 注意：这是关节速度和加速度的缩放，不是末端执行器的速度/加速度
  move_group.setMaxVelocityScalingFactor(0.5);      // 速度缩放因子：50%
  move_group.setMaxAccelerationScalingFactor(0.5);  // 加速度缩放因子：50%（使启动和停止更平滑）

  // 步骤3: 计算笛卡尔路径
  moveit_msgs::RobotTrajectory trajectory;
  const double eef_step = 0.01;       // 插值步长（1厘米，即每1cm生成一个轨迹点）

  // 计算笛卡尔插值路径
  // 返回值 fraction: 0~1 表示成功计算的路径百分比，-1 表示错误
  // 例如：fraction=0.8 表示80%的路径可以成功计算，20%可能因为碰撞或不可达而失败
  // 注意：jump_threshold 参数在 ROS Noetic 中已被弃用，使用新版本API
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan (Cartesian path) (%.2f%% achieved)", 
                 fraction * 100.0);

  // 步骤4: 可视化路径
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "AUBO Cartesian Path Example4", 
                          rvt::RED, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);  // 显示路径点连线
  // 为每个路径点显示坐标轴
  for (std::size_t i = 0; i < waypoints.size(); ++i)
  {
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  }
  visual_tools.trigger();

  // 步骤5: 执行轨迹
  my_plan.trajectory_ = trajectory;  // 将计算的轨迹赋值给规划对象
  move_group.execute(my_plan);

  // 步骤6: 返回 Home 位置
  move_group.setJointValueTarget(home_position);
  move_group.move();

  visual_tools.prompt("Press 'next'5 ADD a Object in the RvizVisualToolsGui window to continue the demo");








  // ========================================================================
  // 示例 5: 添加碰撞对象（Adding Collision Objects）
  // ========================================================================
  // 功能：在规划场景中添加障碍物，用于避障规划
  // 应用场景：模拟真实环境中的障碍物，测试避障规划能力
  // 特点：支持多种几何形状（BOX、SPHERE、CYLINDER等）

  // 步骤1: 定义碰撞对象1（障碍物盒子）
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();  // 参考坐标系
  collision_object.id = "box1";  // 对象唯一标识符

  // 定义几何形状（盒子）
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;  // 类型：盒子
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;  // 长度（X方向，米）
  primitive.dimensions[1] = 0.05; // 宽度（Y方向，米）
  primitive.dimensions[2] = 0.4;  // 高度（Z方向，米）

  // 设置盒子的位置和姿态
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;  // 无旋转（单位四元数）
  box_pose.position.x = -0.3;     // X 坐标
  box_pose.position.y = 0.2;     // Y 坐标
  box_pose.position.z = 0.54;    // Z 坐标

  // 将几何形状和位姿添加到碰撞对象
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;  // 操作类型：添加

  // 步骤2: 定义碰撞对象2（桌面）
  moveit_msgs::CollisionObject collision_object2;
  collision_object2.header.frame_id = move_group.getPlanningFrame();
  collision_object2.id = "box2";

  // 定义桌面几何形状（大而薄的盒子）
  shape_msgs::SolidPrimitive primitive2;
  primitive2.type = primitive.BOX;
  primitive2.dimensions.resize(3);
  primitive2.dimensions[0] = 1.7;  // 长度（米）
  primitive2.dimensions[1] = 1.7;  // 宽度（米）
  primitive2.dimensions[2] = 0.05; // 高度（米，很薄）

  // 设置桌面位置（在地面上）
  geometry_msgs::Pose box_pose2;
  box_pose2.orientation.w = 1.0;  // 无旋转
  box_pose2.position.x = 0.0;     // 原点
  box_pose2.position.y = 0.0;     // 原点
  box_pose2.position.z = 0.0;     // 地面高度

  collision_object2.primitives.push_back(primitive2);
  collision_object2.primitive_poses.push_back(box_pose2);
  collision_object2.operation = collision_object2.ADD;

  // 步骤3: 将所有碰撞对象添加到规划场景
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  collision_objects.push_back(collision_object2);
  
  // 添加到规划场景（MoveIt 会自动进行碰撞检测）
  planning_scene_interface.addCollisionObjects(collision_objects);

  // 步骤4: 可视化状态
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "AUBO Add object Example5", rvt::RED, rvt::XLARGE);
  visual_tools.trigger();

  // 等待用户确认碰撞对象已显示在 RViz 中
  visual_tools.prompt("Press 'next'6 in the RvizVisualToolsGui window to once the collision object appears in RViz");



  // ========================================================================
  // 示例 6: 避障运动（Obstacle Avoidance Movement）
  // ========================================================================
  // 功能：在有障碍物的环境中规划路径，自动避开碰撞对象
  // 应用场景：真实工作环境中的运动规划，需要考虑环境障碍物
  // 特点：MoveIt 自动进行碰撞检测，规划器会寻找无碰撞路径

  // 步骤1: 设置目标位姿（目标位置在障碍物后方，需要绕过障碍物）
  q.setRPY(1.77, -0.59, -1.79);  // 设置目标姿态（RPY 欧拉角，单位：弧度）

  // 设置当前状态为起始状态
  move_group.setStartState(*move_group.getCurrentState());
  
  // 定义目标位姿
  geometry_msgs::Pose another_pose;
  another_pose.orientation.x = q.x();
  another_pose.orientation.y = q.y();
  another_pose.orientation.z = q.z();
  another_pose.orientation.w = q.w();
  another_pose.position.x = -0.37;  // 目标位置 X
  another_pose.position.y = 0.6;     // 目标位置 Y（在障碍物后方）
  another_pose.position.z = 0.4;    // 目标位置 Z
  move_group.setPoseTarget(another_pose);

  // 步骤2: 进行避障规划
  // MoveIt 会自动检测与碰撞对象的碰撞，规划器会寻找绕过障碍物的路径
  success = (move_group.plan(my_plan) == 
            moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 6 (pose goal move around cuboid) %s", 
                 success ? "SUCCESS" : "FAILED");

  // 步骤3: 可视化规划结果
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "AUBO Obstacle Goal Example6", rvt::RED, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  // 步骤4: 执行规划（机器人会自动绕过障碍物）
  move_group.execute(my_plan);

  // 步骤5: 返回 Home 位置
  move_group.setJointValueTarget(home_position);
  move_group.move();
  
  visual_tools.prompt("next step 7 attach the collision object to the robot");



  // ========================================================================
  // 示例 7: 附加对象到机器人（Attach Object to Robot）
  // ========================================================================
  // 功能：将碰撞对象附加到机器人，模拟抓取操作
  // 应用场景：模拟机器人抓取物体后，物体随机器人一起移动
  // 特点：附加的对象会随机器人运动，并参与碰撞检测

  ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
  
  // 将碰撞对象附加到机器人（对象会随机器人移动）
  move_group.attachObject(collision_object.id);

  visual_tools.publishText(text_pose, "AUBO Object attached to robot Example7", 
                          rvt::RED, rvt::XLARGE);
  visual_tools.trigger();

  visual_tools.prompt("Press 'next'8 in the RvizVisualToolsGui window to once the collision object attaches to the robot");

  // ========================================================================
  // 示例 8: 从机器人分离对象（Detach Object from Robot）
  // ========================================================================
  // 功能：将附加的对象从机器人分离，模拟释放操作
  // 应用场景：模拟机器人放置物体
  // 特点：分离后对象恢复为静态碰撞对象

  ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
  
  // 将对象从机器人分离（对象恢复为静态）
  move_group.detachObject(collision_object.id);

  visual_tools.publishText(text_pose, "AUBO Object detached from robot Example8", 
                          rvt::RED, rvt::XLARGE);
  visual_tools.trigger();

  visual_tools.prompt("Press 'next'9 in the RvizVisualToolsGui window to once the collision object detaches to the robot");




  // ========================================================================
  // 示例 9: 移除碰撞对象（Removing Collision Objects）
  // ========================================================================
  // 功能：从规划场景中移除碰撞对象
  // 应用场景：清理规划场景，移除不再需要的障碍物
  // 特点：移除后对象不再参与碰撞检测

  // 移除障碍物 box1
  ROS_INFO_NAMED("tutorial", "Remove the object from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "AUBO Object1 removed Example9", rvt::RED, rvt::XLARGE);
  visual_tools.trigger();

  // ========================================================================
  // 示例 10: 随机运动（Random Motion）- 已注释
  // ========================================================================
  // 功能：随机生成目标并执行运动（用于测试和演示）
  // 警告：随机运动不可预测，使用时需注意安全，建议在仿真中使用
  // 注意：此示例已被注释，如需使用请取消注释

  // 以下代码已注释，如需使用请取消注释
  // visual_tools.deleteAllMarkers();
  // visual_tools.publishText(text_pose, "AUBO Random Move Example 10", rvt::RED, rvt::XLARGE);
  // visual_tools.trigger();
  //
  // for(int i = 0; i < 30; i++)
  // {
  //   move_group.setRandomTarget();  // 设置随机目标
  //   move_group.move();              // 执行运动
  //   ROS_INFO_NAMED("tutorial", "Random Moving %d:", i);
  // }
  // visual_tools.prompt("Press 'next'10 : Robot Random Moving");

  // ========================================================================
  // 清理工作
  // ========================================================================
  
  // 移除桌面（box2）
  ROS_INFO_NAMED("tutorial", "Remove the desktop from the world");
  object_ids.push_back(collision_object2.id);
  planning_scene_interface.removeCollisionObjects(object_ids);
  
  // 显示完成信息
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, " Finish ", rvt::RED, rvt::XLARGE);
  visual_tools.trigger();

  // ========================================================================
  // 程序结束
  // ========================================================================
  ros::shutdown();
  return 0;
}
