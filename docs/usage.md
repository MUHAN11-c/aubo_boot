# AUBO E5 使用命令手册（Passthrough 架构）

适用于本工作区 2026-07-27 之后的 passthrough 架构。所有命令默认已执行：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
source install/setup.bash
```

## 1. 构建

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release     # 全量构建
colcon build --packages-select aubo_e5_controllers        # 只构建某个包
```

## 2. 启动（三种运行模式）

```bash
# mock：GenericSystem + 标准 JTC（回归测试用，无 passthrough）
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=mock

# sim：板级模拟器 + passthrough 全链路闭环（无真机开发/验证）
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=sim

# real：真机（已取消 RT 预检，普通内核直接启动）
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=real robot_ip:=169.254.10.98
```

可叠加参数（任意模式可组合）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `moveit_enabled` | true | MoveIt move_group + rviz2（默认开启；底层调试可 `:=false` 关闭） |
| `camera_enabled` | false | 启动 percipio 相机 + 外参发布 |
| `hand_eye_enabled` | false | 启动手眼标定服务 |
| `hand_eye_web_enabled` | true | 手眼标定 Web 界面（仅 `hand_eye_enabled:=true` 时生效） |

```bash
# 完整工作单元（真机 + 相机 + 标定；MoveIt + rviz2 默认已开启）
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=real \
  robot_ip:=169.254.10.98 camera_enabled:=true hand_eye_enabled:=true
```

只起 MoveIt + rviz2（不起硬件/ros2_control，纯规划调试/可视化用）——MoveIt 的
launch 在 `aubo_e5_moveit_config` 里（唯一文件 `moveit.launch.py`，整体启动
move_group + rviz2），bringup 只是 include 它：

```bash
# 默认自带 robot_state_publisher + joint_state_publisher_gui（standalone_state_publishers:=true），
# 用 jsp_gui 摆位、RViz 里 Plan 看规划效果（无硬件执行）
ros2 launch aubo_e5_moveit_config moveit.launch.py

# 换控制器映射（默认 controllers.yaml/passthrough；mock 回归用 controllers_mock.yaml）
ros2 launch aubo_e5_moveit_config moveit.launch.py controllers_file:=controllers_mock.yaml
```

## 3. 控制器管理

```bash
ros2 control list_controllers                 # 查看控制器状态
ros2 control list_hardware_interfaces         # 查看接口（含 GPIO）
ros2 control set_controller_state aubo_io_controller inactive
```

注：三种模式下加载的控制器互不相同（mock 只有标准 JTC，sim/real 只有
passthrough + IO），两套轨迹控制器从不共存，因此不存在"切到另一套轨迹
控制器"的操作；控制器替换需改 launch 配置后重启。

## 4. 轨迹执行

### 4.1 命令行直接发 goal（冒烟用）

```bash
ros2 action send_goal /aubo_passthrough_trajectory_controller/follow_joint_trajectory \
control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint],
    points: [
      {positions: [0.1, 0, 0, 0, 0, 0], velocities: [0,0,0,0,0,0], accelerations: [0,0,0,0,0,0], time_from_start: {sec: 2}}
    ]
  }
}"
```

### 4.2 分析工具（tools/，参考 UR example_move.py 写法）

```bash
cd tools
# 项目 Python 统一用 aubo_py3.12 venv（见 AGENTS.md 第 2 节）
PY=../aubo_py3.12/bin/python

# 轨迹测试客户端：以当前位置为基准的小幅安全轨迹
$PY passthrough_traj_client.py wave_shoulder 3     # 肩关节往返 3 次
$PY passthrough_traj_client.py wave_all            # 六关节依次摆动
$PY passthrough_traj_client.py sine_shoulder       # 正弦密集路点（压测重采样/流控）

# 运动分析工具（单文件、单窗口图文报告：左曲线右文字，同图存 PNG）
# 数据默认保存在项目内 test_results/<时间戳>_<轨迹名|rec>_*（PNG+CSV）；
# 也可显式给 out_prefix 指定其他路径。
# run 模式：发内置轨迹并录制 joint_states + RIB 水位，输出执行分析
$PY motion_analyzer.py run sine_shoulder
#   报告：墙钟/标称时长比、终点误差、RIB min/max/mean、joint_states 频率，
#         曲线为位置跟踪(实际 vs 标称)/速度/逐关节跟踪误差/RIB 水位
#   输出：test_results/<时间戳>_sine_shoulder_{report.png,joints.csv,rib.csv}
#   选项：--no-gui 不弹窗只存 PNG（无显示环境）；--real 真机模式
#         （RIB 按执行期 >0 且 <400 判 PASS/FAIL；sim 无发送侧水位流控，
#          瞬时入队超 400 属预期，只记 INFO）

# rec 模式：RViz 拖拽 Plan & Execute 被动录制，工具自动切段，Ctrl-C 后
# 所有段汇总进同一个图文窗口，逐段按 A 准确性/B 平稳性/C 平滑性/D 实时性 量化
$PY motion_analyzer.py rec                              # Ctrl-C 汇总
#   输出：test_results/<时间戳>_rec_report.png（全部段一张图）、
#         _seg<N>_joints.csv、_seg<N>_rib.csv

# FK/IK 自洽性检查（仅 real 模式，走 dashboard 服务）
$PY fk_ik_check.py
```

### 4.3 MoveIt / RViz

```bash
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=sim   # MoveIt + rviz2 默认开启
# RViz MotionPlanning 面板 Plan & Execute；或代码侧 move_group 接口。
# 管线：ompl（默认，TOTG+Ruckig 平滑）、pilz_industrial_motion_planner（PTP/LIN/CIRC）。
```

MoveIt 的控制器映射随 `hardware_mode` 自动选择（bringup 传 `controllers_file`）：

- sim / real → `config/controllers.yaml`（`aubo_passthrough_trajectory_controller`）
- mock → `config/controllers_mock.yaml`（标准 `joint_trajectory_controller`，官方 ros2_control 链路）

单独启动 moveit.launch.py 时默认为 passthrough 映射，可用
`controllers_file:=controllers_mock.yaml` 覆盖。

## 5. IO 与状态

```bash
# 设置板载用户 DO（fun: 1=板载DO 2=板载AO 3=工具DO 4=工具AO 5=工具电源）
ros2 service call /aubo_io_controller/set_io aubo_msgs/srv/SetIO "{fun: 1, pin: 3, state: 1.0}"

# 状态监视
ros2 topic echo --once /aubo_io_controller/io_states      # 全部 IO 状态
ros2 topic echo --once /aubo_io_controller/robot_status   # 上电/急停/运动中/错误
ros2 topic echo --once /aubo_io_controller/rib_status     # RIB 水位 + 队列
ros2 topic hz /joint_states                               # 发布率 ~200Hz（随 controller_manager update_rate；
                                                          # 数据内容刷新受 SDK 推送速率限制，hz 看不出来）
```

注：sim 插件不模拟板载 IO 写回，`set_io` 返回 success=false 属预期。

## 6. Dashboard 服务（仅 real 模式）

```bash
ros2 service call /aubo_dashboard/startup std_srvs/srv/Trigger            # 上电初始化
ros2 service call /aubo_dashboard/shutdown std_srvs/srv/Trigger           # 断电
ros2 service call /aubo_dashboard/release_brake std_srvs/srv/Trigger      # 松刹车
ros2 service call /aubo_dashboard/stop std_srvs/srv/Trigger               # 停止（保留队列语义）
ros2 service call /aubo_dashboard/fast_stop std_srvs/srv/Trigger          # 快速停
ros2 service call /aubo_dashboard/collision_recover std_srvs/srv/Trigger  # 碰撞恢复
ros2 service call /aubo_dashboard/set_payload aubo_msgs/srv/SetPayload "{payload: 2.5}"
```

## 7. 真机分阶段验证流程（务必按序）

```bash
# 第 1 步：只核对状态（不上电、不运动）。对照示教器核对 6 关节角度。
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=real robot_ip:=<IP>
ros2 topic echo --once /joint_states

# 第 2 步：上电（或示教器手动上电）
ros2 service call /aubo_dashboard/startup std_srvs/srv/Trigger

# 第 3 步：低速小轨迹（RViz 速度因子拉到 0.1，或用工具的小幅轨迹）
aubo_py3.12/bin/python tools/passthrough_traj_client.py wave_shoulder

# 第 4 步：取消行为验证（执行中 Ctrl+C action client 或发新 goal 抢占）
aubo_py3.12/bin/python tools/passthrough_traj_client.py sine_shoulder &  # 长轨迹
aubo_py3.12/bin/python tools/passthrough_traj_client.py wave_shoulder    # 抢占，前者应 CANCELED

# 第 5 步：MoveIt 整机轨迹 + 分析（--real 启用 RIB 水位 PASS/FAIL 判定）
aubo_py3.12/bin/python tools/motion_analyzer.py run wave_all --real
# RViz 手动拖拽的运动段用 rec 模式逐段分析（A/B/C/D 四类 + 汇总图文窗口）
# 数据默认存 test_results/<时间戳>_*
aubo_py3.12/bin/python tools/motion_analyzer.py rec
```

真机验收指标：执行期点吞吐率 ≈200 点/s；RIB 不饿死（>0）不溢出（<400）；
墙钟/标称时长比 ≈1.0；终点误差 < goal_tolerance_rad(0.02)。

## 8. 手眼标定 Web 界面

```bash
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=real robot_ip:=<IP> \
  hand_eye_enabled:=true        # 浏览器打开 http://127.0.0.1:8088（仅回环地址）
```

页面分区与数据：

- **标定流程**：预检/规划/运动/采样/求解/完成步骤条；求解算法下拉可选
  `auto`（5 方法竞选取最优，默认）或指定 `tsai/park/horaud/andreff/daniilidis`，
  预检与执行都按所选方法求解（服务端参数 `solver_method` 为默认方法）。
- **机器人状态**：模式/急停/驱动上电/可运动/运动中/错误码 + 安全回路 LED
  （数据来自 `/aubo_io_controller/robot_status`、`io_states`）。
- **末端位姿**：腕部 wrist3_Link 与相机 camera_link 的 xyz/rpy/quat（TF，10Hz）。
- **关节状态表**：6 关节位置/速度/电流/温度/跟随误差/错误码
  （`/joint_states` + `/aubo_io_controller/joint_status`；温度>60°C、
  |跟随误差|>0.05rad、错误码≠0 标红）。
- **位姿采样矩阵**：点击已采样位姿格展开逐帧观测（重投影 RMS、腕部位姿、
  板在相机系位姿、MAD 留/剔标记）+ 重投影 RMS 时序图（1.0px 门限线）。
- **求解面板**：AX=XB 链式模型/残差/Huber 精化公式说明、5 方法打分表、
  精化统计（收敛/cost/迭代数）、逐样本一致性残差图（3mm/0.5° 门限线）、
  base_from_target 估计；历史候选点"详情"可回看同等明细。
- mock 模式无 io 控制器，机器人状态区显示"无数据"属预期；sim 无相机不能
  完整标定，但状态面板全部可用。

## 9. 常用排障

| 现象 | 排查 |
|---|---|
| `libprotobuf.so.9` 找不到 | 确认经 launch 启动（RPATH 已配置）；手动跑二进制需 `LD_LIBRARY_PATH` 指向 `install/aubo_e5_hardware/lib/aubo_e5_hardware/vendor` |
| SDK 读不到配置/日志 | ros2_control_node 与 dashboard 的 cwd 必须是各自 share 目录（launch 已设置） |
| goal 被拒 | 关节名拼写/数量（必须 6 个权威关节名）、points 的 velocities/accelerations 数组长度 |
| 抢占后卡死 | 不应出现（蓝本已修复）；抓 `/aubo_io_controller/rib_status` 与 ros2_control_node 日志 |
| sim 里 set_io 返回 false | 预期行为，sim 不模拟板载 IO 写回 |
| `on_error summary: read_error_reason=1 (push never arrived)` 但此前运行正常 | 2026-07-29 起已修复：该报错原为 `RealtimeThreadSafeBox::try_get()` 撞锁（best-effort try_lock 返回 nullopt）被误判为无数据；现 `read()` 回退上一帧缓存，真断流仍由 200ms `state_timeout_ms` 超时（reason=2）兜底。汇总里的 `read_box_misses` 是撞锁计数，>0 属正常 |
| `read_error_reason=2 (push stale)` | SDK 推送链路真断了（>200ms 无新帧）：查网卡 offload/governor（每次开机必做，见 AGENTS.md §9）与 `docs/nic_driver_incident.md` |
