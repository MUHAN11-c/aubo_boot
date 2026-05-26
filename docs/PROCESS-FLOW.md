# IVG 流程执行细节文档

> **面向**: 开发者 / 调试者 | **最后更新**: 2026-05-15
>
> 本文档记录 IVG 系统从启动到执行的完整流程细节。配合 `start_aubo_new_driver.sh` 阅读。

---

## 一、启动流程总览

```
[0]  colcon build
[1]  机械臂核心 (MoveIt2 + Dashboard + StateBroadcaster + JTC)
[2]  Demo Driver 服务 (依赖 move_group)
[15] IVG Web 网关   (提前启动，仅依赖 build)
[16] rosbag 录制    (提前启动，仅需 ROS 2)
[3]  相机节点 → [4] 相机控制 → [5] 图像桥接   (并行组)
[6]  手眼标定 → [7] VPE → [14] FastAPI Web     (依赖相机)
[8]  GraspNet (依赖相机点云 + TF)
[9]  抓取Worker [10] 夹爪快换 [11] 咖啡拉花 [12] GraspNet循环 (并行组)
[13] 综合校验
```

### 时序图（关键依赖链）

```
时间轴 →

[0] ████████████████████████ 构建 (30-120s)
[1] ████████████████ 机械臂核心启动 ... 持续运行
    │ move_group 就绪 (active_wait 30s)
    ├── [2] ████ Demo Driver 服务
    ├── [15] ██ Web 网关 (并行)         ... 持续运行
    └── [16] ██ rosbag (并行)            ... 持续运行
         │
    ───── 相机组 ─────
    [3] ████ 相机节点 → /camera/color/image_raw 就绪
    [4] ██ 相机控制 (并行)
    [5] ██ 图像桥接 (并行)
         │
    ───── 视觉组 ─────
    [6] ████ 手眼标定 → TF 发布
    [7] ████ VPE → /estimate_pose 服务
    [14] ██ FastAPI Web (并行)
         │
    [8] ██████████ GraspNet → /grasp_poses_base
         │
    ───── Worker 组 (并行) ─────
    [9] ██ 抓取Worker [10] ██ 快换 [11] ██ 拉花 [12] ██ 循环
         │
    [13] ██ 综合校验 → 完成
```

---

## 二、每步详细说明

### [0] colcon build

- **条件**: `SKIP_BUILD!=1`
- **首次运行**: 不 source `install/setup.bash`（文件不存在时跳过）
- **后续运行**: source 后再 build（加速增量编译）
- **RobotWebTools**: build 完成后运行 `build_robotwebtools.sh`（导出 ros3djs/roslibjs/ros2djs 构建产物）
- **失败处理**: `set -e` 会终止脚本

### [1] 机械臂核心

**入口**: `ros2 launch aubo_moveit_config aubo_new_driver.launch.py server_host:=${AUBO_IP}`

**TCP 探测（launch 文件内 OpaqueFunction）**:
1. 尝试 `socket.connect((server_host, 8899))`，超时 2s
2. **TCP 可达** → `MODE=real`: `aubo_dashboard + aubo_state_broadcaster + joint_trajectory_controller` (C++ SDK)
3. **TCP 不可达** → `MODE=sim`: `ros2_control_node + joint_state_broadcaster + joint_trajectory_controller` (mock_components)

**共享节点**（两种模式都启动）:
- `aubo_mode` (Python): 将 `aubo_driver_mode` 发布到 `/aubo/mode` (transient_local, 每5s重发)
- `robot_state_publisher`: `/joint_states` + `/tf` + `/tf_static`
- `move_group` (MoveIt2): OMPL 规划器 `planning_time=15.0`, `max_planning_attempts=10`
- `rviz2` (延迟 12s)

**Dashboard 生命周期激活**（后台脚本，30s 超时）:
```
轮询 ros2 node list | grep /aubo_dashboard → configure → activate
```

**等待**: `move_group` 节点就绪 (active_wait node "/move_group" 30s)

### [2] Demo Driver 服务

**入口**: `ros2 launch aubo_moveit_config demo_driver_services.launch.py`

**启动节点** (5 个 C++ 服务，共享 `moveit_velocity_scaling_factor=0.1`):
| 服务 | 节点 |
|------|------|
| `/plan_trajectory` | `plan_trajectory_server` |
| `/execute_trajectory` | `execute_trajectory_server` |
| `/get_current_state` | `get_current_state_server` |
| `/set_speed_factor` | `set_speed_factor_server` |
| `/move_to_pose` | `set_robot_pose_server` |

**等待**: `/execute_trajectory` 服务就绪 (active_wait service 20s)

### [15] IVG Web 网关 (提前启动)

**入口**: `ros2 launch aubo_ros2_web_dashboard web_dashboard.launch.py`

**4 个子进程**:
1. `rosbridge_websocket` (`:9090`) — ROS ↔ WebSocket 桥接，最大消息 64MB
2. `tf2_web_republisher` — TF 变换 Web 发布
3. `web_video_server` (`:8089`) — MJPEG/快照 HTTP 服务
4. `FastAPI 网关` (`:8090`) — 统一入口：静态文件 + 反向代理 + BFF API

**等待**: Web Dashboard `/health` (active_wait http 20s)

### [3][4][5] 相机栈

| 步骤 | Launch | 节点 | 输出话题 |
|------|--------|------|---------|
| [3] | `percipio_camera.launch.py` | PercipioCameraNodeDriver (组件容器) | `/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/depth_registered/points` |
| [4] | `camera_control.launch.py` | camera_control_node | `/camera_status` (1Hz) |
| [5] | `image_data_bridge.launch.py` | image_data_bridge_node | `/image_data` |

**默认参数**: SN=`207000152740`, IP=`169.254.10.110`, 彩色 640x480 YUV, 深度 640x480

**等待**: `/camera/color/image_raw` (15s), `/camera_status` (10s)

### [6][7][14] 视觉栈

**手眼标定 [6]**: 订阅 `/camera/color/image_raw` + `/camera/color/camera_info`, 发布 TF `base_link→camera_link`。Web 服务在 `:8080`。

**VPE [7]**: 订阅彩色+深度图像，提供 `/estimate_pose` 服务 (ivg_interfaces/EstimatePose)。

**FastAPI Web [14]**: 绑定 `host:port` (默认 `127.0.0.1:8088`)，提供 VPE Web UI。

**等待**: `/estimate_pose` 服务 (15s)

### [8] GraspNet

**入口**: `ros2 launch graspnet_ros2 graspnet_demo_points_with_tf.launch.py`

**节点**:
- `hand_eye_static_tf_node`: 发布 `wrist3_Link→camera_frame` TF (手眼矩阵)
- `static_transform_publisher`: `camera_frame→camera_link` (旋转 `1.5708 -1.5708 0`)
- `graspnet_demo_points_node`: 订阅 `/camera/depth_registered/points` → 推理 → 发布 `/grasp_poses_base` (PoseArray), `grasp_markers` (MarkerArray)

**模型**: `graspnet-baseline/logs/log_kn/checkpoint-rs.tar`

**等待**: `/grasp_poses_base` 话题 (20s)

### [9][10][11][12] Worker 组

| 步骤 | Launch | 核心服务 |
|------|--------|---------|
| [9] | `execute_grasp_pose_worker.launch.py` | `/execute_single_grasp`, `/loop_grasp_control` |
| [10] | `gripper_swap_worker.launch.py` | `/change_tool`, `/run_gripper_swap`, `/get_current_tool` |
| [11] | `latte_io.launch.py` | `/set_latte_do2`, `/set_latte_do4` |
| [12] | `publish_grasps_client_worker_node` (ros2 run) | `/publish_grasps_worker_loop_control` |

**工具快换链路**: `/change_tool` → `gripper_swap_worker` → `/set_robot_io`（快换 IO）→ `moveToDockApproach` → **`/scene_detach`（提前卸碰撞）** → `releaseTool` → `pickTool` → **`/tool_changer_status` → `scene_attach_worker`**（`/attached_collision_object` + `/planning_scene` world REMOVE）→ `moveToHome`

**等待**: `/run_gripper_swap` (10s), `/change_tool` (5s), `/set_latte_do2` (5s)

### [13] 综合校验

列出关键服务 + HTTP 健康检查:
- `ros2 service list | grep` 验证
- `curl /health` (FastAPI + Web Dashboard)

---

## 三、实机/仿真自动检测

**代码位置**: `aubo_ros2_ws/src/aubo_ros2_driver/aubo_moveit_config/launch/aubo_new_driver.launch.py` — `_check_robot_reachable()` OpaqueFunction

```
socket.create_connection((server_host, 8899), timeout=2.0)
  │
  ├─ TCP 可达 → MODE = "real"
  │   └─ 启动 aubo_driver_ros2 C++ 节点
  │       ├── aubo_dashboard (LifecycleNode, :8899)
  │       ├── aubo_state_broadcaster (200Hz 回调)
  │       └── joint_trajectory_controller (TCP2CAN 轨迹流)
  │
  └─ TCP 不可达 → MODE = "simulation"
      └─ 启动 ros2_control + mock_components/GenericSystem
```

**影响**: 模式通过 `/aubo/mode` 话题广播 (transient_local QoS)，影响 `/set_io` 服务的实际行为：
- 真机: 通过 SDK 发送 IO 指令
- 仿真: IO 调用静默跳过 (gripper_swap_worker 通过 `/aubo/mode` 自动检测)

---

## 四、感知管道数据流

```
相机 (Percipio FM830)
  │
  ├── /camera/color/image_raw ────→ 手眼标定 (ArUco检测) ──→ base→camera TF
  ├── /camera/color/image_raw ────→ VPE (模板匹配) ──→ /estimate_pose
  ├── /camera/depth/image_raw ────→ VPE (深度投影)
  └── /camera/depth_registered/points ──→ GraspNet (PointNet++) ──→ /grasp_poses_base
                                              │
                                              ▼
                                    publish_grasps_client_worker
                                    (窗口收集 → 最优选择 → Z翻转)
                                              │
                                              ▼
                                    execute_grasp_pose_worker
                                    (接近 → 握持 → 抬升 → 放置)
```

---

## 五、抓取执行链路 (runOneCycle 9 步)

**代码位置**: `aubo_ros2_driver/demo_driver/src/execute_grasp_pose_worker.cpp`

```
1. moveToHome("camera_pose")          ← MoveIt 规划到预定义关节角
2. buildGraspPose()                  ← 从参数或 VPE 结果构建目标位姿
3. 变换 gripper_tip → end_effector   ← TF 链: gripper_tip → wrist3_Link
4. openGripper()                     ← set_io(IO_GRIPPER=6, true)
5. graspApproach()                   ← 6 航点笛卡尔路径: 接近→下降
   ↓ (Z 安全下限 0.19m, 重试 3 次)
6. closeGripper()                    ← set_io(IO_GRIPPER=6, false)
7. liftUp(Z + lift_offset=0.2)       ← 垂直抬升
8. moveToPlace()                     ← home + 偏移 (y=-0.2, z=-0.15)
9. openGripper() → returnToHome()    ← 松开工件，回起始位
```

**关键参数**:
- `joint_velocity_scaling=0.7`, `joint_acceleration_scaling=0.3`
- `cartesian_max_points=40` (每段笛卡尔路径最多 40 个航点)
- `joint_cartesian_switch_delay_sec=0.2` (关节→笛卡尔模式切换延迟)

---

## 六、工具快换完整链路

**代码位置**: `aubo_ros2_ws/src/tool_changer/src/gripper_swap_worker.cpp`（`changeToTool`）喵~

```
/change_tool srv (tool_id)
  │
  ▼
gripper_swap_worker::changeToTool(target)
  ├── （若有当前工具）moveToDockApproach(current)
  ├── updateSceneAttachment(current.id, false)  → 服务 /scene_detach
  ├── releaseTool(current)  （笛卡尔 + IO）
  ├── publishToolStatus(false)   ← 仅 release 成功后：清空 current_tool_
  ├── moveToDockApproach(target)
  ├── pickTool(target)
  ├── publishToolStatus(true)    ← pick 成功后立即发布（写入 target，不等 home）
  │       └→ scene_attach_worker.onToolStatus()
  │             /attached_collision_object: detach 旧 + attach 新
  │             /planning_scene: world REMOVE attached_tool_<id>（清除残留）
  └── moveToHome()
```

**配置来源**: `aubo_ros2_ws/src/tool_changer/config/tools.yaml` — 每种工具的 `dock_approach_joints` 和 `trajectory` 策略喵~

**PlanningScene 同步**（`aubo_ros2_ws/src/tool_changer/src/scene_attach_worker.cpp`）：工具网格 **ADD/REMOVE** 经 **`/attached_collision_object`**；detach 后可能残留在 **world** 的同名 `attached_tool_<id>` 经 **`/planning_scene`** 增量 diff（`world.collision_objects` REMOVE）清除喵~

---

## 七、关键调试命令

```bash
# === 进程状态 ===
ros2 node list                           # 所有运行中的节点
ros2 node info /move_group               # 节点详情 (话题/服务/订阅)

# === 服务诊断 ===
ros2 service list | grep -E 'grasp|tool|latte|estimate'
ros2 service call /change_tool ivg_interfaces/srv/ChangeTool "{tool_id: gripper0}"
ros2 service call /get_current_tool ivg_interfaces/srv/GetCurrentTool

# === 话题监控 ===
ros2 topic list                          # 所有话题
ros2 topic hz /joint_states              # 发布频率
ros2 topic echo /aubo/mode --once        # 驱动模式

# === TF 树验证 ===
ros2 run tf2_tools view_frames           # 生成 TF 树 PDF
ros2 run tf2_ros tf2_echo base_link wrist3_Link

# === 生命周期 (Dashboard) ===
ros2 lifecycle list /aubo_dashboard      # 当前状态
ros2 lifecycle set /aubo_dashboard configure
ros2 lifecycle set /aubo_dashboard activate

# === 日志 ===
ros2 run rqt_console rqt_console         # GUI 日志查看器
tail -f ~/.ros/log/latest/*.log          # 命令行日志

# === Web 健康检查 ===
curl http://127.0.0.1:8090/health
curl http://127.0.0.1:8088/health
```

---

## 八、常见故障模式与恢复

| 症状 | 可能原因 | 排查步骤 |
|------|---------|---------|
| `[1]` 步骤后启动卡住 | `move_group` 崩溃或 TF 断链 | `ros2 node list \| grep move_group`, 查 `~/.ros/log/` |
| `[3]` 相机话题超时 | 相机掉线或 SN 不匹配 | `ping 169.254.10.110`, 检查 `dmesg \| grep usb` |
| `[8]` GraspNet 超时 | GPU 显存不足 / CUDA 错误 | `nvidia-smi`, 检查模型文件存在 |
| `/estimate_pose` 返回空 | 模板不匹配或标定偏移 | 检查 VPE Web UI 模板库, 重做手眼标定 |
| `/change_tool` 失败 | 工具未在 dock 位 / IO 故障 | `ros2 service call /get_current_tool`, 手动回 home |
| Dashboard 页面空白 | 前端 JS 加载失败或 rosbridge 未就绪 | `curl http://127.0.0.1:8090/health`, 浏览器 F12 Console |
| 抓取位姿投影不显示 | TF 变换缺失 / CameraInfo 未收到 | `ros2 topic echo /camera/color/camera_info --once` |
| terminator 标签页闪退 | LD_LIBRARY_PATH 缺少库路径 | `ldd` 检查可执行文件, 查看标签页报错 |

---

*最后更新: 2026-05-15 | 基于 `start_aubo_new_driver.sh` v2.0 + 全部 launch 文件源码分析*
