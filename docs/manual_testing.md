# AUBO E5 手动测试操作手册

更新日期：2026-07-25。当前配置：SDK **v1.3.1**（TCP2CAN 流式）、
controller_manager 100Hz、推送 33Hz + 轮询 20Hz 双路反馈。

> **运动约束**：只允许 `home` 与 `camera_pose` 两个授权点位及其连线走廊
> （±0.02 rad）。走廊外命令会被硬件路径门拒绝并闩锁（cause=2），属正常保护。

## 0. 每次开机后必做（需密码）

governor 与 NIC offload 设置**重启后不持久**，必须重跑：

```bash
sudo /home/mu/Desktop/aubo_e5_jazzy_ws/src/aubo_e5_bringup/scripts/configure_realtime.sh
```

验证（应全 PASS，PREEMPT_RT 行 WARN 可忽略）：

```bash
/home/mu/Desktop/aubo_e5_jazzy_ws/install/aubo_e5_bringup/lib/aubo_e5_bringup/realtime_preflight.sh
```

## 1. 环境（每个新终端）

```bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

## 2. 启动系统

```bash
ros2 launch aubo_e5_bringup bringup.launch.py \
  use_mock_hardware:=false enable_real_hardware:=true \
  allow_motion_commands:=true server_host:=169.254.10.98 \
  camera_enabled:=false hand_eye_enabled:=false start_moveit:=true
```

启动成功的标志（launch 日志）：

- `on_activate: TCP2CAN + threads started (seed pos=[...])`
- `first JointStatus push received` + `first joint-state poll succeeded`
- `Authorized services ready: /aubo/move_home and /aubo/move_camera_pose`

重启系统时：先在 launch 终端 `Ctrl+C` 一次，等 ~10s 干净退出
（看到 `senderLoop exit` + `Hardware interface shutdown`）再启动。
**切忌 `kill -9`**——SDK 未登出会在控制器侧留僵尸会话，污染后续测试。

## 3. M1：授权点位运动（直接 JTC）

```bash
ros2 service call /aubo/move_camera_pose std_srvs/srv/Trigger
ros2 service call /aubo/move_home std_srvs/srv/Trigger
```

成功标志：`Goal reached, success!` + `Reached authorized pose`。

## 4. M2/M3：规划管线（move_group）

`tools/m2_m3_retest.py [ompl|pilz] [home|camera_pose] [速度缩放0~1]`

```bash
aubo_py3.12/bin/python tools/m2_m3_retest.py ompl camera_pose 0.3   # M2: OMPL+TOTG+Ruckig
aubo_py3.12/bin/python tools/m2_m3_retest.py pilz home 0.3          # M3: Pilz PTP（确定性）
aubo_py3.12/bin/python tools/m2_m3_retest.py pilz camera_pose 0.8   # 快速档，可对比观感
```

成功标志：`execution error_code=1 (1=SUCCESS)`。

## 5. 观察与判读

```bash
ros2 topic echo --once /joint_states    # 当前关节角（home ≈ [0,-0.033,1.236,-0.368,1.570,0] 按字母序见 SRDF）
ros2 control list_controllers           # 均应 active
ros2 control list_hardware_components   # AuboE5System 应 active
```

launch 日志关键行：

- **健康**：`SEND_SUMMARY ... rate≈180-300pts/s rib=60-150 ema≈5-8ms fail=0`
- **异常**：`FAULT_STOP ... cause=N`，原因码：
  1=反馈失新 2=走廊越界 3=路径反向 4=队列溢出 5=RIB诊断连败
  6=发送连败 7=设定点冻结 8=SDK安全事件 9=安全IO急停
  （cause=2/3 会附带 `rejected site=... cmd=[...] ref=[...]` 快照）

## 6. 通道基线探针（先停 bringup——TCP2CAN 独占）

```bash
./build/diagnostics/runtime_probe 169.254.10.98 600 100   # 读路径：0 失败、max <50ms 为健康
./build/diagnostics/tcp2can_probe 169.254.10.98 30 8      # v2.5.3 对照探针
./build/diagnostics/full_test                              # SDK 全量 37 用例（含运动，约 5 分钟）
./build/diagnostics/full_test --skip-motion --skip-coexist # 只读部分

# v1.3.1 探针（当前插件同款库，需 LD_LIBRARY_PATH）：
V=/home/mu/aubo_boot/aubo_ros2_ws/src/aubo_ros2_driver/aubo_driver_ros2
LD_LIBRARY_PATH=$V/lib/lib64/aubocontroller:$V/lib/lib64/log4cplus:$V/lib/lib64/config:$V/lib/lib64/protobuf \
  ./build/diagnostics/tcp2can_probe_v131 169.254.10.98 30 8
# 健康基线：write avg ~5ms、0 失败、产能 ~200 pts/s
```

## 7. 关闭

launch 终端 `Ctrl+C` **一次**（SIGINT 走完整生命周期：stopMotion →
leaveTcp2Canbus → logout）。看到 `Hardware interface shutdown` 即干净退出。

## 异常处置顺序

1. 卡顿/延迟重现 → 重跑 `configure_realtime.sh`（offload 又开了）
2. 仍异常 → 重启控制柜（清服务器侧状态）
3. 仍异常 → 按 `docs/nic_driver_incident.md` 运维指引排查
   （ethtool 驱动版本/offload、dkms 模块、探针基线）

## 相关文档

- `docs/aubo_sdk_2_5_3_ros2_control_test_report.md`——测试与修复全程记录
- `docs/nic_driver_incident.md`——网卡问题与运维排障
- `docs/realtime_setup.md`——实时权限/内核配置

## RViz2 手动运动分析

在 RViz2 里拖拽目标、Plan & Execute 的同时，被动录制并按四大类标准
（准确性/平稳性/平滑性/实时性）逐段量化分析。

1. 启动 sim（默认带 MoveIt + RViz2）：

   ```bash
   ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=sim
   ```

2. 另开终端运行分析器（real 模式步骤相同，启动命令换成 real）：

   ```bash
   aubo_py3.12/bin/python tools/motion_analyzer.py rec /tmp/rviz_ana
   ```

3. 在 RViz2 MotionPlanning 面板拖拽目标位姿 → **Plan & Execute**
   （可重复多次，每次执行自动切为一个运动段；段切分规则：
   ||q̇|| > 0.01 rad/s 记段开始，< 0.01 持续 0.5s 记段结束）。

4. 分析器终端 `Ctrl-C`，打印逐段四组对照表（每项 PASS/FAIL/INFO），
   并写出每段 CSV：`<prefix>_seg<N>_joints.csv`（t、各关节 pos/vel）、
   `<prefix>_seg<N>_rib.csv`（t、rib_level）。

判定阈值含义与出处见 `docs/ur_motion_evaluation_standards.md`；
默认阈值可用命令行调整（`--goal-tol` / `--path-tol` / `--goal-time-tol` /
`--sparc-min` 等，`-h` 查看全部）。

注意：分析器取段开始前 2s 内最后一条 `/display_planned_path` 作标称轨迹；
RViz2 每次 Plan & Execute 都会发布该话题。若绕过 RViz2 直接发轨迹
（如 `tools/passthrough_traj_client.py`），标称对照项会自动跳过并注明
"无标称轨迹"。
