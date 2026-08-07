# peach_motion

## 简介

套袋桃采摘的**任务编排包**：消费 peach_pose_ros2 的感知主输出
`BagGraspCandidateArray`，把逐帧候选经空间配准归并为持久目标档案
（TargetRegistry），维护 NEW/OBSERVING/READY/EXECUTING/DONE/FAILED/LOST
状态机与防重复调度；`~/pick` action 为六阶段全量执行器（轨道预接近 →
帧门控弧线 → 融合等待 → 可达性验证 → 可选插入 → 退出），调度侧按
confidence×0.6 + 近距×0.4 排序逐个执行，全程落盘（三文件 + rosbag2）。

## 使用方法

```bash
# MoveIt 为源码 overlay（/opt/ros/jazzy 无 moveit 包），先 source
source ~/ws_moveit/install/setup.bash
colcon build --packages-select peach_motion
source install/setup.bash
# 另开终端：AUBO E5 机械臂（sim 闭环 / 真机 hardware_mode:=real）
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=sim camera_enabled:=false
ros2 launch peach_motion mission.launch.py       # 任务编排 + pick 执行
```

话题与服务：

| 方向 | 名称 | 类型 | 说明 |
|------|------|------|------|
| sub | `/peach_pose_node/grasp_candidates` | `BagGraspCandidateArray` | 感知候选灌注册表 |
| pub | `~/registry`（mission_manager） | `TargetRegistry` | 目标档案快照（2 Hz） |
| pub | `~/stats`（mission_manager） | `std_msgs/String` | JSON 统计（状态/计数/下一目标） |
| pub | `~/stage_events`（pick_executor） | `std_msgs/String` | 阶段迁移 JSON |
| pub | `~/planned_orbit`（pick_executor） | `MarkerArray` | 弧线路径 LINE_STRIP |
| pub | `~/execution_trace`（pick_executor） | `PoseStamped` | 每 waypoint 实际 TCP 位姿 |
| action | `~/pick`（pick_executor） | `peach_pose_msgs/Pick` | 六阶段拾取执行 |

参数见 `config/mission.yaml`（mission_manager / pick_executor 两节，全部
中文行内注释；`execute_insert` 默认 false=仅可达性验证）。

## 执行逻辑

1. `mission_node`（console_scripts 入口）同进程组合 `MissionManager` 与
   `PickExecutor`，多线程执行器（MoveIt 阻塞等待需并发线程）。
2. `MissionManager` 工作线程跑任务循环：**GLOBAL_SCAN** 按 `observe_poses`
   逐个位姿 plan&execute + 停留 scan_duration_sec（订阅回调持续灌注册表）
   → **READY_LOOP** 按 `ready_order`（confidence×0.6 + 近距×0.4）取队首，
   mark_executing → 发 pick goal → 结果回写 DONE/FAILED+原因并记事件 →
   队列清空（全部 DONE/FAILED/LOST）→ **SITE_DONE** 写 summary 停 bag。
3. `PickExecutor._on_execute` 六阶段：**APPROACH_PLAN**（orbit_planner
   生成球面弧线：起点=当前 TCP、终点=目标 −standoff·轴、TCP Z 轴恒指目标，
   失败重试一次 arc+15°）→ **ARC_APPROACH**（逐 waypoint plan&execute，
   FrameGate 帧门控：每点执行后等新一帧 candidates，超时
   frame_gate_timeout_sec 放行）→ **REFINE_WAIT**（等 refined 支撑帧 ≥
   min_support_frames 或超时；跳变 > target_jump_dist → FAILED(moved)）→
   **FINAL_APPROACH**（plan_only 可达性验证，失败 FAILED(unreachable)，
   成功记轴-重力夹角指标）→ **INSERT**（execute_insert=true 才沿轴以
   5 mm 小步进位姿序列逐段 plan&execute 插 travel）→ **RETREAT**（回预
   接近点）。任意阶段取消 → 停发 + RETREAT → CANCELED。
4. `SessionStore`：`peach_records/<时间戳>/` 落 targets.jsonl /
   events.jsonl / summary.json；record=true 时 subprocess 拉起
   `ros2 bag record`（话题清单见 data_store.BAG_TOPICS），SITE_DONE 收尾。
5. 纯逻辑层可测：`TargetRegistry`（配准/状态机）、`orbit_planner`（纯
   numpy）、`FrameGate`（假时钟可测）；`PickExecutor` 经 moveit_factory /
   tcp_provider 注入假实现做流程单测。

## 软件框架

```
peach_motion/
├── peach_motion/
│   ├── target_registry.py   # TargetRegistry/TargetRecord + ready_order（纯逻辑）
│   ├── orbit_planner.py     # 球面弧线预接近路径点（纯 numpy）
│   ├── moveit_client.py     # MoveGroup 位姿约束 plan&execute + 失败分类
│   ├── data_store.py        # SessionStore：三文件 + rosbag2 录制（纯逻辑）
│   ├── mission_manager.py   # MissionManager：巡视/调度工作线程 + 快照发布
│   ├── pick_executor.py     # PickExecutor：六阶段执行器（可注入假 MoveIt/TF）
│   └── mission_node.py      # 入口编排（MultiThreadedExecutor）
├── config/mission.yaml      # mission_manager / pick_executor 两节参数
├── launch/mission.launch.py # 标准 Node() 启动
└── test/                    # lint + registry/store/orbit/gate/调度/执行器流程
```

依赖：rclpy、peach_pose_msgs、geometry_msgs、std_msgs、builtin_interfaces、
python3-numpy；MoveIt（moveit_msgs/shape_msgs）仅运行时经源码 overlay
提供（moveit_client 延迟 import，单测与离线逻辑不需要）。
