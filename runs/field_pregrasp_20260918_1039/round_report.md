# 测试轮归集：field_pregrasp_20260918_1039

- 归集时间：2026-09-18T11:08:59
- 代码基线：git 529f237
- 运行窗口（本地时区）：10:40:15 → 10:41:23（含前后缓冲；由事件时间戳推断）

## 逐目标结果（ledger）

| target | outcome | failure_code | elapsed_s | 段耗时 |
|--------|---------|--------------|-----------|--------|
| target_0 | 2 | skipped_unreachable | 14.981 | {'reconfirm': 0.079, 'approach_insert': 6.526} |

## 感知/重建关键事件（perception_data，时间戳升序）

```
10:40:15.404 global_targets_locked    
10:40:15.406 reconstruction_linked    
10:40:16.633 frame_skipped            skip[missing_mask] 缺少所选 target_id 的同时间戳掩膜
10:40:16.780 frame_accepted           
10:40:16.790 frame_accepted           
10:40:16.809 frame_skipped            skip[same_stamp] 缓存帧未更新（与上次采帧同帧），请等下一帧
10:40:16.876 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.1089 rad/s > 0.03
10:40:17.080 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.3795 rad/s > 0.03
10:40:17.175 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.6128 rad/s > 0.03
10:40:17.360 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.7994 rad/s > 0.03
10:40:17.677 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.7258 rad/s > 0.03
10:40:17.857 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.7870 rad/s > 0.03
10:40:18.172 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.7735 rad/s > 0.03
10:40:18.334 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.7600 rad/s > 0.03
10:40:18.683 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.8056 rad/s > 0.03
10:40:18.836 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.7579 rad/s > 0.03
10:40:19.170 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.7164 rad/s > 0.03
10:40:19.350 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.6802 rad/s > 0.03
10:40:19.677 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.6874 rad/s > 0.03
10:40:19.851 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.6491 rad/s > 0.03
10:40:20.171 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.6636 rad/s > 0.03
10:40:20.320 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.6283 rad/s > 0.03
10:40:20.677 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.5557 rad/s > 0.03
10:40:20.861 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.5423 rad/s > 0.03
10:40:21.178 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.4707 rad/s > 0.03
10:40:21.336 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.4801 rad/s > 0.03
10:40:21.685 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.4334 rad/s > 0.03
10:40:21.845 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.4023 rad/s > 0.03
10:40:22.175 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.3390 rad/s > 0.03
10:40:22.340 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.2810 rad/s > 0.03
10:40:22.675 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.2899 rad/s > 0.03
10:40:22.827 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.2891 rad/s > 0.03
10:40:23.176 frame_skipped            skip[robot_not_static] 机器人未静止：最大关节速度 0.0874 rad/s > 0.03
10:40:23.426 frame_accepted           
10:40:23.512 reconstruction_finalized 
```

## ROS 2 自动日志（~/.ros/log，窗口内 launch 目录）

### 2026-09-18-10-37-56-393924-mu-MS-7E34-22365
来源：`/home/mu/.ros/log/2026-09-18-10-37-56-393924-mu-MS-7E34-22365/launch.log`；关键行（ERROR/WARN/碰撞/规划失败/阶段门）：
```
```

## 会话 bag

（窗口内未找到 session 目录）

