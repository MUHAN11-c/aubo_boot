# 测试轮归集：field_pregrasp_20260918_1055

- 归集时间：2026-09-18T11:08:59
- 代码基线：git 529f237
- 运行窗口（本地时区）：10:56:49 → 10:57:57（含前后缓冲；由事件时间戳推断）

## 逐目标结果（ledger）

| target | outcome | failure_code | elapsed_s | 段耗时 |
|--------|---------|--------------|-----------|--------|
| target_0 | 2 | skipped_unreachable | 15.042 | {'reconfirm': 0.255, 'approach_insert': 6.535} |

## 感知/重建关键事件（perception_data，时间戳升序）

```
10:56:49.259 global_targets_locked    
10:56:49.264 reconstruction_linked    
10:56:50.346 frame_skipped            skip[missing_mask] 缺少所选 target_id 的同时间戳掩膜
10:56:50.477 frame_accepted           
10:56:50.492 frame_accepted           
10:56:50.521 frame_skipped            skip[near_duplicate] 近重复视角不积分：平移 0.0 mm / 旋转 0.00 deg
10:56:50.604 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.2281 rad/s > 0.03
10:56:50.742 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.4147 rad/s > 0.03
10:56:51.100 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.7517 rad/s > 0.03
10:56:51.244 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.7973 rad/s > 0.03
10:56:51.602 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.7735 rad/s > 0.03
10:56:51.789 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.7724 rad/s > 0.03
10:56:52.103 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.7590 rad/s > 0.03
10:56:52.217 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.7942 rad/s > 0.03
10:56:52.602 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.7911 rad/s > 0.03
10:56:52.762 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.7330 rad/s > 0.03
10:56:53.105 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.7361 rad/s > 0.03
10:56:53.275 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.6822 rad/s > 0.03
10:56:53.604 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.6698 rad/s > 0.03
10:56:53.776 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.6605 rad/s > 0.03
10:56:54.114 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.5723 rad/s > 0.03
10:56:54.288 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.5931 rad/s > 0.03
10:56:54.603 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.5868 rad/s > 0.03
10:56:54.762 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.5267 rad/s > 0.03
10:56:55.103 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.4635 rad/s > 0.03
10:56:55.276 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.4106 rad/s > 0.03
10:56:55.604 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.3940 rad/s > 0.03
10:56:55.797 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.3743 rad/s > 0.03
10:56:56.104 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.2934 rad/s > 0.03
10:56:56.264 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.2856 rad/s > 0.03
10:56:56.604 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.2363 rad/s > 0.03
10:56:56.758 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.1272 rad/s > 0.03
10:56:57.325 frame_accepted           
10:56:57.491 reconstruction_finalized 
```

## ROS 2 自动日志（~/.ros/log，窗口内 launch 目录）

### 2026-09-18-10-54-57-349468-mu-MS-7E34-30624
来源：`/home/mu/.ros/log/2026-09-18-10-54-57-349468-mu-MS-7E34-30624/launch.log`；关键行（ERROR/WARN/碰撞/规划失败/阶段门）：
```
```

## 会话 bag

- `runs/session_20260918_105458`：✓ 含 /rosout（节点日志可回放：ros2 bag play 后 echo /rosout）；报告 `runs/session_20260918_105458/bag_report.md`
- `runs/session_20260918_110648`：✗ 无 /rosout（栈早于 2026-09-17 改动）

