# graspnet_ros2

点云 6-DOF 抓取检测与 MoveIt 接近。推理后端是 vendored GraspNet-baseline 权重 + 纯 torch 点云算子，**不用 AnyGrasp SDK / 许可证**，也不依赖 open3d、graspnetAPI、CUDA 扩展编译。

不进 `harvest_system.launch.py`、不进 lifecycle、不订 `peach_interfaces`。与采摘栈共用驱动层（相机 / TF / MoveIt），独立 launch。

## 分层

| 层 | 模块 | 职责 |
|----|------|------|
| 推理纯核 | `grasp_core.GraspNetInference.get_grasp(points)` | 工作区过滤 → 采样 → 前向 → 碰撞/NMS/top-K（无 rclpy） |
| 检测节点 | `graspnet_demo_points_node` | PointCloud2 → MarkerArray / PoseArray / TF |
| 执行客户端 | `publish_grasps_client` | 选优 + TCP 补偿 + MoveIt 接近（须外部 `move_group`） |

权重默认 `share/graspnet_ros2/models/checkpoint-rs.tar`。

## 启动

```bash
# 检测（默认不拉相机；点云已由 bringup 提供）
ros2 launch graspnet_ros2 graspnet_detect.launch.py

# 检测 + 接近（须 move_group 已起；授权真机运动后才执行）
ros2 launch graspnet_ros2 graspnet_grasp.launch.py
```

采集默认待命：`ros2 service call /graspnet_capture_control std_srvs/srv/SetBool "{data: true}"` 开始一组。

点云 `/camera/depth_registered/points`（frame 兜底 `camera_depth_optical_frame`）。本 launch **不**起相机或手眼 TF。

规划组默认 `manipulator_e5`，末端 `tcp`。未授权不得真机运动。
