# 测试轮归集：e2e25r_pick1_170449

- 归集时间：2026-09-17T17:36:00
- 代码基线：git 5fbb8d9
- 运行窗口（本地时区）：17:05:01 → 17:07:05（含前后缓冲；由事件时间戳推断）

## 逐目标结果（ledger）

| target | outcome | failure_code | elapsed_s | 段耗时 |
|--------|---------|--------------|-----------|--------|
| target_1 | 0 |  | 8.56 | 局部重建完成：3 帧，5451 点（少于推荐 5 帧）；重叠 mean=1.0mm p95=5.8mm；TSDF 523 点 / mesh 937 顶点（累计积分 0.01s）；refit ACCEPT（袋模型 2 视） |

## 感知/重建关键事件（perception_data，时间戳升序）

```
17:05:01.617 global_targets_locked    
17:05:01.627 reconstruction_linked    
17:05:02.843 frame_skipped            skip[missing_mask] 缺少所选 target_id 的同时间戳掩膜
17:05:02.952 frame_accepted           
17:05:02.967 frame_accepted           
17:05:02.979 frame_skipped            skip[same_stamp] 缓存帧未更新（与上次采帧同帧），请等下一帧
17:05:03.037 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.0881 rad/s > 0.03
17:05:03.302 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.5070 rad/s > 0.03
17:05:03.474 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.6190 rad/s > 0.03
17:05:03.799 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.6729 rad/s > 0.03
17:05:03.979 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.6750 rad/s > 0.03
17:05:04.306 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.6708 rad/s > 0.03
17:05:04.477 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.6926 rad/s > 0.03
17:05:04.803 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.6853 rad/s > 0.03
17:05:04.939 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.6770 rad/s > 0.03
17:05:05.299 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.7113 rad/s > 0.03
17:05:05.488 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.6677 rad/s > 0.03
17:05:05.798 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.6605 rad/s > 0.03
17:05:05.920 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.6822 rad/s > 0.03
17:05:06.302 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.6345 rad/s > 0.03
17:05:06.473 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.6356 rad/s > 0.03
17:05:06.807 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.6034 rad/s > 0.03
17:05:06.960 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.5734 rad/s > 0.03
17:05:07.302 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.5153 rad/s > 0.03
17:05:07.472 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.4966 rad/s > 0.03
17:05:07.801 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.3940 rad/s > 0.03
17:05:07.948 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.4054 rad/s > 0.03
17:05:08.312 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.3029 rad/s > 0.03
17:05:08.476 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.3185 rad/s > 0.03
17:05:08.802 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.3522 rad/s > 0.03
17:05:08.939 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.3367 rad/s > 0.03
17:05:09.542 frame_accepted           
17:05:09.743 reconstruction_finalized 
17:06:05.146 target_dropped           anchor_drop_timeout（目标丢失超时）
```

## ROS 2 自动日志（~/.ros/log，窗口内 launch 目录）

### 2026-09-17-17-03-32-990257-mu-MS-7E34-247795
来源：`/home/mu/.ros/log/2026-09-17-17-03-32-990257-mu-MS-7E34-247795/launch.log`；关键行（ERROR/WARN/碰撞/规划失败/阶段门）：
```
1789636567.1294339 [WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
1789636567.4694998 [ERROR] [peach_lifecycle_flag_bridge-15]: process has died [pid 247870, exit code -2, cmd '/home/mu/Desktop/aubo_e5_jazzy_ws/install/peach_bringup/lib/peach_bringup/peach_lifecycle_flag_bridge --ros-args -r __node:=peach_lifecycle_flag_bridge'].
1789636568.2122269 [ERROR] [peach_harvester-11]: process has died [pid 247866, exit code -2, cmd '/home/mu/Desktop/aubo_e5_jazzy_ws/install/peach_harvester/lib/peach_harvester/peach_harvester --ros-args -r __ns:=/ --params-file /tmp/launch_params_v1gl51e5 --params-file /tmp/launch_params_3npxcjgz --params-file /tmp/launch_params_1wlk68vx --params-file /tmp/launch_params_w7cjc2tq --params-file /tmp/launch_params_5kln58n4'].
1789636568.8599727 [ERROR] [component_container-8]: process has died [pid 247863, exit code -6, cmd '/opt/ros/jazzy/lib/rclcpp_components/component_container --ros-args -r __node:=camera_container -r __ns:=/camera'].
1789636570.9220293 [ERROR] [move_group-6]: process has died [pid 247861, exit code -11, cmd '/opt/ros/jazzy/lib/moveit_ros_move_group/move_group --ros-args --params-file /tmp/launch_params_go4b4y_n --params-file /tmp/launch_params_cn4fbu3w'].
1789636572.4302356 [ERROR] [peach_observability-13]: process[peach_observability-13] failed to terminate '5' seconds after receiving 'SIGINT', escalating to 'SIGTERM'
1789636577.4255855 [ERROR] [peach_observability-13]: process[peach_observability-13] failed to terminate '10.0' seconds after receiving 'SIGTERM', escalating to 'SIGKILL'
1789636577.4780240 [ERROR] [peach_observability-13]: process has died [pid 247868, exit code -9, cmd '/home/mu/Desktop/aubo_e5_jazzy_ws/install/peach_observability/lib/peach_observability/peach_observability --ros-args -r __node:=peach_observability -r __ns:=/ --params-file /tmp/launch_params_xph_emsx'].
```

## 会话 bag

- `runs/session_20260917_170333`：✗ 无 /rosout（栈早于 2026-09-17 改动）
- `runs/session_20260917_171626`：✗ 无 /rosout（栈早于 2026-09-17 改动）；报告 `runs/session_20260917_171626/bag_report.md`

