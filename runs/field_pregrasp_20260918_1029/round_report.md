# 测试轮归集：field_pregrasp_20260918_1029

- 归集时间：2026-09-18T11:08:59
- 代码基线：git 529f237
- 运行窗口（本地时区）：10:31:26 → 10:32:30（含前后缓冲；由事件时间戳推断）

## 逐目标结果（ledger）

（无 outcomes——批在派发前即终止或被取消）

## 感知/重建关键事件（perception_data，时间戳升序）

```
10:31:26.962 global_targets_locked    
10:31:26.968 reconstruction_linked    
10:31:28.288 frame_skipped            skip[missing_mask] 缺少所选 target_id 的同时间戳掩膜
10:31:28.403 frame_accepted           
10:31:28.418 frame_accepted           
10:31:28.429 frame_skipped            skip[near_duplicate] 近重复视角不积分：平移 0.0 mm / 旋转 0.00 deg
10:31:28.432 frame_skipped            skip[near_duplicate] 近重复视角不积分：平移 0.0 mm / 旋转 0.00 deg
10:31:28.513 frame_skipped            skip[near_duplicate] 近重复视角不积分：平移 0.0 mm / 旋转 0.00 deg
10:31:28.716 frame_skipped            skip[near_duplicate] 近重复视角不积分：平移 0.0 mm / 旋转 0.00 deg
10:31:29.224 frame_skipped            skip[near_duplicate] 近重复视角不积分：平移 0.0 mm / 旋转 0.00 deg
10:31:29.717 frame_skipped            stale_frame
10:31:30.221 frame_skipped            stale_frame
10:31:30.710 frame_skipped            skip[missing_mask] 缺少所选 target_id 的同时间戳掩膜
```

## ROS 2 自动日志（~/.ros/log，窗口内 launch 目录）

（窗口内未找到 launch 目录——检查 --log-root）
## 会话 bag

- `runs/session_20260918_103757`：✓ 含 /rosout（节点日志可回放：ros2 bag play 后 echo /rosout）

