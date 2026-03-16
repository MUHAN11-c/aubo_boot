# Grasp 运动控制 C++ 迁移 — 最终计划

## 一、目标与架构

将 `grasp_motion_controller.py` 和 `publish_grasps_client.py` 的机械臂运动控制迁移为 C++，在 `demo_driver` 包中：

- **基类**：`MoveitGripperIoBase`（仅将 `move_group_` 改为 protected）
- **子类**：`PublishGraspsClientWorker`，继承基类并实现抓取放置逻辑
- **数据流**：`graspnet_demo_points_node` 发布 `PoseArray` → `PublishGraspsClientWorker` 订阅并循环执行抓取放置

---

## 二、主流程（完整）

```
main():
  1. waitForServices()           // 等待 Aubo SetIO
  2. 注册 SIGINT 处理              // Ctrl+C → shutdown_requested
  3. executor.spin() 后台线程     // 订阅回调持续运行
  4. clearGraspWindow()          // 循环前清空陈旧数据（必须）
  5. while (rclcpp::ok() && !shutdown_requested && (max_cycles<0 || cycle_count<max_cycles)):
       success = runOneCycle()
       发布 status_topic
       if !success: sleep(fail_retry_delay_sec); fail_count++
       else: cycle_count++; success_count++
       sleep(cycle_delay_sec)
  6. onShutdown()                // 回安全位、开夹爪
  7. 销毁节点
```

---

## 三、runOneCycle 单次周期（9 步）

| 步骤 | 操作 | 失败处理 |
|------|------|----------|
| 1 | waitForGraspWindowReady()，加锁检查窗口 | 超时返回 false |
| 2 | selectBestFromWindow()，加锁；prefer_vertical 分支；**选后移除已选 pose** | 空返回 false |
| 3 | buildGraspToEndEffectorTransform + applyTransformationToPose | — |
| 4 | runGraspApproach(transformed_pose)，4 点笛卡尔，自实现 scaleTrajectoryTime | fraction<1 或点数>60 返回 false |
| 5 | setGripperIo(gripper_io_index, true) | 记录错误码 |
| 6 | runArcPath('z', lift_offset) | 记录 fraction |
| 7 | moveToPose(place_pose) 或 moveToJoints(place_joints) | 记录 MoveIt 错误码 |
| 8 | setGripperIo(gripper_io_index, false) | 记录错误码 |
| 9 | moveToHome() 或 moveToJoints(home_joints) | — |

---

## 四、runGraspApproach 逻辑（不回退）

1. pose_for_plan = applyGraspZFlip180(pose_ee)
2. current_pose = move_group_->getCurrentPose(eef_link)
3. 构建 4 waypoints：当前 → XY+安全高度(同姿态) → 同位置(抓取姿态 quatSameHemisphere) → 目标 Z
4. computeCartesianPath，重试 3 次，间隔 0.5 s
5. 若 fraction<1 或 点数>60：返回 false
6. scaleTrajectoryTime(vel) 后 execute

常量：kCartesianMaxPointsForExecution=60，kArcPathMaxRetries=3，kArcPathRetryDelaySec=0.5。scaleTrajectoryTime 需在子类内实现。

---

## 五、Python 逻辑与修正（必须一致）

- **_QUAT_Z_180**：(0,0,1,0)
- **applyGraspZFlip180**：ori * _QUAT_Z_180（右乘）
- **quatSameHemisphere**：dot≥0 返回 q，dot<0 返回 -q
- **gripper_tip 变换**：T_base_target = T_base_grasp * T_grasp_local，[2][3]=-grasp_z_offset
- **verticalityScore**：abs(dot(z_axis, [0,0,-1]))
- **不允许回退关节空间**：fraction<1 或 点数>60 时直接返回 false

---

## 六、文件与修改清单

| 操作 | 路径 |
|------|------|
| 修改 | `include/demo_driver/moveit_gripper_io_base.h`（move_group_ → protected） |
| 新建 | `include/demo_driver/publish_grasps_client_worker.h` |
| 新建 | `src/publish_grasps_client_worker.cpp` |
| 修改 | `CMakeLists.txt`（新增 publish_grasps_client_worker_node） |

---

## 七、参数（完整）

| 参数 | 默认 | 说明 |
|------|------|------|
| prefer_vertical | true | 选最垂直抓取 |
| grasp_z_offset | 0.15 | gripper_tip→end_effector 沿 z 补偿 (m) |
| height_above | 0.05 | 抓取点上方安全高度 (m) |
| joint_velocity_scaling | 0.15 | 关节空间速度缩放 |
| joint_acceleration_scaling | 0.1 | 关节空间加速度缩放 |
| grasp_poses_topic | grasp_poses_base | 抓取位姿话题 |
| wait_poses_timeout_sec | 30.0 | 等待窗口超时 (s) |
| grasp_window_size | 5 | 窗口大小 |
| min_groups_before_pick | 3 | 至少 M 组后再选 |
| gripper_io_index | 7 | 夹爪 IO pin |
| lift_offset | 0.2 | 抬起高度 (m) |
| place_mode | "pose" | "pose" 或 "joints" |
| place_pose | 7×double | (x,y,z,qx,qy,qz,qw) |
| place_joints | 6×double | 放置关节角 (rad) |
| cycle_delay_sec | 1.0 | 每周期后等待 (s) |
| fail_retry_delay_sec | 同 cycle_delay | 失败后额外等待 (s) |
| max_cycles | -1 | 最大周期数，-1 无限 |
| status_topic | grasp_place_status | 状态监控话题 |

---

## 八、建议实现项

| 项 | 说明 |
|----|------|
| **窗口优化** | 选后从窗口移除已选 pose |
| **异常处理** | 关键步骤 RCLCPP_ERROR + 错误码 |
| **状态监控** | 每周期后发布 status_topic（cycle_count、success_count、fail_count、last_step） |
| **优雅退出** | SIGINT → shutdown_requested → onShutdown（回安全位、开夹爪） |
| **循环前清理** | clearGraspWindow() 必须调用 |

---

## 九、类接口摘要

```cpp
class PublishGraspsClientWorker : public MoveitGripperIoBase {
  // 公开：run(), runOneCycle(), clearGraspWindow(), onShutdown()
  // 私有：runGraspApproach(), quatSameHemisphere(), applyGraspZFlip180(),
  //       verticalityScore(), selectBestFromWindow(), waitForGraspWindowReady(),
  //       applyTransformationToPose(), buildGraspToEndEffectorTransform()
  // 成员：_grasp_groups_window, window_mutex_, _latest_grasp_poses
  // 继承：moveToJoints, setGripperIo, runArcPath, moveToHome 等
};
```

---

## 十、依赖

- geometry_msgs（含 PoseArray）
- moveit_core, moveit_ros_planning_interface
- Eigen（通过 MoveIt）
- aubo_msgs（SetIO 服务）
