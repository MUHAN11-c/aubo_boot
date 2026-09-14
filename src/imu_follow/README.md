# imu_follow — IMU 姿态跟随（可选独立工具包）

订 `/imu/data`（serial_imu），把 enable 时刻起的 IMU 体轴姿态增量（死区 →
符号映射 → 锥限幅 → 平滑）叠加到参考 TCP 姿态上（位置钉死参考点），经
**MoveIt Servo**（官方实时方案，`motion.backend=servo` 默认）或 FJT 流式
（真机透传备选）下发。不是 peach 包：不随 `harvest_system` 起、不进
lifecycle、不改只读 bringup、不订 peach 话题。

## 双后端

- **servo（默认）**：节点对当前 TF 姿态闭环，姿态/位置误差 P 控制成
  `TwistStamped`（EE 系 speed_units）发 `moveit_servo/delta_twist_cmds`；
  Servo 以 100 Hz 增量 IK 流式输出 JTC 话题（奇异缩放/碰撞减速/平滑内建）。
  enable 时自动 `switch_command_type(TWIST)` + 确保未暂停（此版 servo 不切
  类型会拒收）；twist 发布用 **BEST_EFFORT** 匹配其订阅（可靠发布收不到）。
- **fjt（真机透传备选）**：`/compute_ik` 解关节、单步钳制后流式 FJT 动作
  （mock `/joint_trajectory_controller/...`；真机
  `/aubo_passthrough_trajectory_controller/...`）。servo 输出是 JTC 话题，
  透传控制器只有 FJT 动作口——真机要么用本后端要么加桥接。

## 门与安全

- `motion.enabled` 默认 **false**：只发布 `~/target_pose` 与
  `~/command_twist` 供检查，不发运动。真机使用须另行人工授权（AGENTS 红线）。
- 角速度上限 `execution.max_omega_rad_s`（0.5）；位置保持小增益防漂移。
- 自动 disable：IMU / 关节状态断流、连续 IK 失败（fjt）；disable 时 servo
  补一帧零速刹车、fjt 取消在途 goal。节点退出后 servo 因指令超时自停。

## 前置

- bringup（mock 硬件 + JTC + move_group + RViz）在线；**冷启动全零位参考
  IK/碰撞不可行，先导到 `global_photo_pose`** 再 enable。
- `/imu/data` 有数据（真 USB 或假流话题）。

## 用法（mock）

```bash
# 1) 起 mock 底座
ros2 launch aubo_e5_bringup bringup.launch.py \
  hardware_mode:=mock camera_enabled:=false
# 2) 导到拍照位（冷启动必做；关节值=SRDF global_photo_pose）
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: \
  [shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, \
  wrist3_joint], points: [{positions: [0.425083, 0.195177, 1.677740, \
  1.461739, -0.500161, 0.038621], time_from_start: {sec: 6}}]}}"
# 3) 起 servo + 跟随节点（默认只算不发）
ros2 launch imu_follow imu_follow_servo.launch.py
# 4) 假 IMU（无 USB 时；真 IMU 用默认 /imu/data 不用此步）
ros2 topic pub -r 20 /imu_data_fake sensor_msgs/msg/Imu \
  '{header: {frame_id: imu_link}, orientation: {w: 1.0}}'
# 5) 采参考开始跟随；开门下发（enable 会自动激活 servo）
ros2 service call /imu_follow/enable std_srvs/srv/Trigger
ros2 topic echo /imu_follow/target_pose --once    # dry 检查
ros2 param set /imu_follow motion.enabled true
ros2 topic echo /moveit_servo/status --once       # 0=No warnings
# 停（servo 补零速刹车）
ros2 service call /imu_follow/disable std_srvs/srv/Trigger
```

FJT 后端：`ros2 launch imu_follow imu_follow.launch.py` +
`ros2 param set /imu_follow motion.backend fjt`。

## 接口

| 名字 | 含义 |
|------|------|
| `/imu/data`（或假流） | 输入（订；勿与他源混流，双流会被平滑成中间值） |
| `/joint_states` | 当前关节（订；新鲜度与 fjt 种子） |
| `~/enable` / `~/disable` | `std_srvs/Trigger`：采参考开始（自动激活 servo）/ 停止 |
| `~/target_pose` | `PoseStamped`（base_link）：平滑后 TCP 目标（位置=参考） |
| `~/command_twist` | `TwistStamped`（tcp 系）：P 控制输出（dry 镜像） |
| `/moveit_servo/delta_twist_cmds` | servo 输入（BEST_EFFORT；开门时发） |
| `/moveit_servo/status` | servo 状态（0=No warnings） |
| `execution.follow_joint_trajectory_action` | fjt 后端动作（mock JTC / 真机透传） |

参数全量与中文注释：`config/imu_follow.yaml`（节点）与
`config/moveit_servo.yaml`（servo；此版参数名自带 `moveit_servo.` 前缀）。
改默认值须与 `imu_follow/params.py` 同改，键名冻结。姿态数学纯核
`imu_follow/follow_core.py`（表驱动测试 `test/test_core.py`，零 ROS）。

## 已知边界

- MoveIt Servo 在 **ws_moveit 覆盖层**（2.14 源码快照，apt 没有）——本机
  起栈须带该铺层（testing.md §4）。
- KDL/伺服节拍：跟随节点 20 Hz 输入，Servo 100 Hz 输出；真机透传走 fjt
  后端（~20 Hz 流式替换 goal，未经真机验证，现场先小锥低拍）。
- mock 冷启动关节全零：参考位姿 IK 无解（error_code=-31），先导拍照位。
- 体轴符号映射在大角度下是「手感」近似；方向以实机手感调 `follow.invert_*`。
- 位置锁死参考点：本包只跟姿态，不做位置跟随（servo 后端有小增益防漂移）。
