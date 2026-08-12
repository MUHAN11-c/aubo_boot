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

launch 参数一览（每个参数都带中文说明，完整列表与默认值以
`ros2 launch aubo_e5_bringup bringup.launch.py --show-args` 为准）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `hardware_mode` | real | `mock`/`sim`/`real`（choices 限定，拼错直接抛错） |
| `robot_ip` | 169.254.10.98 | 真机控制器 IP；mock/sim 不用 |
| `moveit_enabled` | true | MoveIt move_group + rviz2（底层调试可 `:=false` 关闭） |
| `camera_enabled` | true | 启动 percipio 相机（percipio_camera.launch.py） |
| `extrinsics_enabled` | true | 手眼外参静态 TF（extrinsics_publisher：wrist3_Link→camera_link） |
| `hand_eye_enabled` | false | 启动手眼标定流程（采集/求解） |
| `hand_eye_web_enabled` | false | 标定 Web 界面（仅 `hand_eye_enabled:=true` 时生效） |

注意默认组合即"真机 + 相机 + 外参 + MoveIt"。sim/mock 模式若未接相机，
建议显式 `camera_enabled:=false`，否则 percipio 驱动找不到设备会持续报错
（不拖垮其他节点，但刷屏且易误判）。

```bash
# 完整工作单元（真机 + 相机 + 外参 + 标定 + Web；相机/外参/MoveIt 默认已开）
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=real \
  robot_ip:=169.254.10.98 hand_eye_enabled:=true hand_eye_web_enabled:=true
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
# 也可用位置参数 prefix 显式指定其他输出前缀。
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
# 设置板载用户 DO（fun: 1=板载DO(pin 0..15) 2=板载AO(0..3) 3=工具DO(0..1) 4=工具AO(0..1)；
# srv 里虽定义了 5=工具电源，但 aubo_io 契约无对应接口，控制器有意不支持）
ros2 service call /aubo_io_controller/set_io aubo_msgs/srv/SetIO "{fun: 1, pin: 3, state: 1.0}"

# 状态监视
ros2 topic echo --once /aubo_io_controller/io_states      # 全部 IO 状态
ros2 topic echo --once /aubo_io_controller/robot_status   # 上电/急停/运动中/错误
ros2 topic echo --once /aubo_io_controller/rib_status     # RIB 水位 + 队列
ros2 topic echo --once /aubo_io_controller/joint_status   # 各关节电流/温度/跟随误差
ros2 topic hz /joint_states                               # 发布率 ~200Hz（随 controller_manager update_rate；
                                                          # 数据内容刷新受 SDK 推送速率限制，hz 看不出来）
```

注：sim 插件不模拟板载 IO 写回，`set_io` 返回 success=false 属预期。

## 6. Dashboard 服务（仅 real 模式）

```bash
# /aubo_dashboard/startup 仅为冻结驱动兼容接口；项目流程禁止调用，上电由用户现场手动完成
ros2 service call /aubo_dashboard/shutdown std_srvs/srv/Trigger           # 断电
ros2 service call /aubo_dashboard/release_brake std_srvs/srv/Trigger      # 松刹车
ros2 service call /aubo_dashboard/stop std_srvs/srv/Trigger               # 停止（保留队列语义）
ros2 service call /aubo_dashboard/fast_stop std_srvs/srv/Trigger          # 快速停
ros2 service call /aubo_dashboard/collision_recover std_srvs/srv/Trigger  # 碰撞恢复
ros2 service call /aubo_dashboard/set_payload aubo_msgs/srv/SetPayload "{payload: 2.5}"

# 正/逆运动学（SDK 侧求解；fk_ik_check.py 即用这两个服务做自洽性检查）
# get_fk：6 关节角(rad) → pos[xyz] + ori[四元数 w,x,y,z]
ros2 service call /aubo_dashboard/get_fk aubo_msgs/srv/GetFK "{joint: [0, -1.57, 1.57, 0, 1.57, 0]}"
# get_ik：ref_joint 为求解种子/参考位形，pos + ori(四元数 w,x,y,z) → 6 关节角
ros2 service call /aubo_dashboard/get_ik aubo_msgs/srv/GetIK \
  "{ref_joint: [0, -1.57, 1.57, 0, 1.57, 0], pos: [0.4, 0.0, 0.4], ori: [1, 0, 0, 0]}"
```

## 7. 真机分阶段验证流程（务必按序）

```bash
# 第 0 步：清残留进程（每次启动 launch/测试/探针之前必做）。相机与 SDK 通道
# 都是独占的，且直接 kill launch 会留下孤儿 component_container/节点进程，
# 旧进程跑的是构建前的旧二进制，会让"改动已生效"的判断失真。
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run'   # 有残留按 PID 补杀

# 第 1 步：只核对状态（不上电、不运动）。对照示教器核对 6 关节角度。
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=real robot_ip:=<IP>
ros2 topic echo --once /joint_states

# 第 2 步：用户通过示教器或控制柜手动上电；确认完成后才允许继续
# 禁止脚本、测试、launch 或代理调用 /aubo_dashboard/startup

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
  hand_eye_enabled:=true hand_eye_web_enabled:=true
# 浏览器打开 http://127.0.0.1:8088（仅回环地址；host/port 为 web_gateway 节点参数，
# host 非回环会直接拒绝启动）
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

## 9. 场景重建与桃子位姿（venv 节点）

aubo_scene_recon、peach_pose_ros2 与 peach_reconstruction_ros2 依赖
open3d/torch，节点为标准
console_scripts 入口，启动器 shebang 在构建期由 setup.py 的
`options.build_scripts.executable` 指向工作区 `aubo_py3.12/bin/python`
（不存在则回退构建解释器）；launch 用标准 `Node()` 按名引用，无解释器
参数。venv 依赖可用 `aubo_py3.12/bin/pip install -r requirements.txt` 复现。

### 9.1 场景重建（aubo_scene_recon）

recon.launch.py 只起融合节点，相机需单独就绪——经 bringup
（`camera_enabled` 默认 true）或单独 `ros2 launch percipio_camera
percipio_camera.launch.py`。相机默认开启彩色点云，输出
`/camera/depth_registered/points`，与节点默认输入话题一致。

```bash
ros2 launch aubo_scene_recon recon.launch.py                       # open3d 点云累加（默认后端）
ros2 launch aubo_scene_recon recon.launch.py backend:=tsdf         # RGB-D TSDF
# launch 参数：reconstruction_params_file（默认包内 config/reconstruction.yaml）、save_dir
# （空 → 回退 <进程CWD>/recon_maps，工作区根运行即 recon_maps/）、
# pointcloud_topic、backend；ros2 launch ... --show-args 可查

# 运行期操作
ros2 topic echo --once /recon/map_cloud                            # 融合地图点云（RViz 可视化）
ros2 service call /recon_fusion_node/reset std_srvs/srv/Trigger    # 清空重建状态
ros2 service call /recon_fusion_node/save  std_srvs/srv/Trigger    # 保存地图到 save_dir
```

### 9.2 桃子位姿估计（peach_pose_ros2）

订阅 `/camera/color/image_raw` + `/camera/depth/image_raw` +
`/camera/color/camera_info`（均由 percipio 驱动默认提供），输出坐标系经
TF（含 extrinsics_publisher 外参）变到 `base_link`——因此真机使用时
bringup 的 `camera_enabled` 与 `extrinsics_enabled` 都要开着（均默认开）。

```bash
ros2 launch peach_pose_ros2 peach_pose.launch.py    # 无 launch 参数

# 输出（RViz 观察）：/peach_pose_node/grasp_candidates（抓取候选）、
#   ~/detections、~/masks、~/markers、~/debug_image、~/detection_cloud
```

### 9.3 套袋桃连续局部 TSDF（peach_reconstruction_ros2）

本包使用图像时间戳对应的精确 TF，以 Robot FK + 手眼外参为相机绝对位姿，
Open3D ICP 只做有界小修正；合格帧在机械臂连续慢速运动中立即积分，无需停稳。
机械臂只允许操作者在硬件或示教器上手动上电，禁止调用
`/aubo_dashboard/startup`。

```bash
# peach_pose_ros2 已运行且检测到目标后启动；launch 默认加载 reconstruction.yaml
ros2 launch peach_reconstruction_ros2 reconstruction.launch.py

# RViz2 速度缩放保持 0.01；连续获取 12～24 帧时观察
ros2 topic echo /peach/reconstruction/diagnostics
ros2 topic hz /camera/depth/image_raw

# 覆盖充分后由用户完成、保存
ros2 service call /peach_reconstruction_node/finalize_reconstruction std_srvs/srv/Trigger '{}'
ros2 service call /peach_reconstruction_node/save_session std_srvs/srv/Trigger '{}'
```

RViz2 添加 `/peach/reconstruction/tsdf_cloud`（PointCloud2）与
`/peach/reconstruction/markers`（MarkerArray）；后者包含相机轨迹、
finalize 后的 TSDF 网格和 refined 轴。完整参数与判据见包级 README。

### 9.4 主动视觉靠近与抓取（peach_approach_grasp）

默认模式只生成候选视点并调用 MoveIt 规划，不发送运动。BehaviorTree.CPP 读取
`config/harvest_tree.xml` 编排质量门和分支；球面主动视点仍采用首段 PTP、后续短 LIN
逐位闭环补观测。最终抓取由 MTC 执行“OMPL 无碰到入口 → Cartesian 精化轴插入 →
工具 IO → 独立 Cartesian 原轴撤回”。完整质量门、安全限制见包级 README。

```bash
ros2 launch peach_approach_grasp approach_grasp.launch.py
ros2 service call /peach_approach_grasp_node/start_cycle std_srvs/srv/Trigger '{}'
ros2 service call /peach_approach_grasp_node/query_state std_srvs/srv/Trigger '{}'
```

RViz2 添加 `/peach_approach_grasp_node/planned_views`（MarkerArray）查看候选视点；
Web 数据台显示 `/peach_approach_grasp_node/status`。真机运动默认关闭，禁止跳过
README 中的人工 arm、0.05 速度和无工具观察运动验证阶段。
RViz2 添加 Motion Planning Tasks 面板可查看 MTC 分阶段 solution。升级后必须重启
bringup/move_group，使 `ExecuteTaskSolutionCapability` 生效；不得为此重启当前真机进程。

### 9.5 无相机冒烟：数据集回放（tools/peach_dataset_replayer.py）

回放工具已独立到 tools/（不随 colcon 构建），发布与相机驱动同名的话题
（`/camera/color/image_raw`、`/camera/depth/image_raw`、
`/camera/color/camera_info`），内联 Percipio 棋盘内参（`--fx/--fy/--cx/--cy`
可覆盖），供 peach_pose 全链路冒烟：

```bash
# 终端 1：回放（默认数据集 src/peach_pose_ros2/data/dataset，--limit 默认 3 对）
aubo_py3.12/bin/python tools/peach_dataset_replayer.py --loop
aubo_py3.12/bin/python tools/peach_dataset_replayer.py --dataset <根> --limit 10 --rate 0.5

# 终端 2：peach_pose 节点。回放深度为真实毫米（不同于真机 raw 值），
# 需把 depth_scale_unit 从默认 0.25 改为 1.0：
ros2 launch peach_pose_ros2 peach_pose.launch.py
ros2 param set /peach_pose_node depth_scale_unit 1.0
```

## 10. 测试与 lint

15 个项目包已接入 lint 测试（percipio_camera 为厂商代码、
peach_moveit_config 为 Setup Assistant 生成包，均未接入），
业务测试用 venv 解释器手动跑：

```bash
# 全包 lint + 已接入的测试（10 个 CMake 包 ament_lint_auto：
# copyright/cpplint/uncrustify/lint_cmake/xmllint；5 个 Python 包 pytest 模板
# test_flake8/test_pep257）
colcon test && colcon test-result --verbose

# 业务测试（须先 source /opt/ros/jazzy/setup.bash——lint 用例与节点 import
# 都依赖 ROS 的 Python 路径；再进各包目录用 venv 跑）
cd src/aubo_hand_eye_calibration && ../../aubo_py3.12/bin/python -m pytest test/ -q  # 15 例 + 2 lint
cd src/aubo_scene_recon && ../../aubo_py3.12/bin/python -m pytest test/ -q            # open3d 在 venv
cd src/peach_pose_ros2 && PYTHONPATH=peach_pose_ros2:$PYTHONPATH ../../aubo_py3.12/bin/python -m pytest test/ -q
cd src/peach_reconstruction_ros2 && PYTHONPATH=.:../peach_pose_ros2:$PYTHONPATH ../../aubo_py3.12/bin/python -m pytest test/ -q
```

注：`colcon test` 走系统 python3（无 open3d），aubo_scene_recon 的
test_pc_utils 会被 conftest 自动跳过（collect_ignore），属预期；venv 内
装了 open3d，上述手动命令才是全量。

单个 lint 工具可直接对路径使用（先 source ROS 环境，工具在
/opt/ros/jazzy/bin）：

```bash
ament_uncrustify src/aubo_e5_controllers/src/xxx.cpp     # C++ 格式（100 列）
ament_cpplint    src/aubo_e5_controllers/src/xxx.cpp     # C++ 风格
ament_flake8     tools/motion_analyzer.py                # Python（99 列、单引号）
ament_lint_cmake src/aubo_e5_bringup/CMakeLists.txt      # CMake
ament_xmllint    src/aubo_msgs/package.xml               # XML
```

## 11. SDK 诊断探针（diagnostics/，独立 CMake 构建）

链接 vendored SDK v1.3.1 的底层探针，全部零运动，用于排查 SDK 推送/
TCP2CAN 链路（如网卡驱动停滞，见 `docs/nic_driver_incident.md`）：

```bash
# 一键构建 + 按序运行 + 绘图（真机；ABI/轮询延迟/推送频率/RIB 水位）
./diagnostics/run_tests.sh [host] [quick|full]
#   host 默认 169.254.10.98；quick ≈30s，full（默认）≈90s
#   输出 diagnostics/results/*.csv + *.png（每次运行覆盖最新）
#   注意：tcp2can 探针会进入 TCP2CAN 独占模式（结束自动恢复），期间示教器运动暂停

# 实时监测：与 bringup（sim/real）+ RViz2 并行运行，只订阅
# /joint_states + rib_status + joint_status，不新增 SDK 连接、对控制通道零干扰
aubo_py3.12/bin/python diagnostics/live_monitor.py            # 实时窗口 + 落盘 CSV
aubo_py3.12/bin/python diagnostics/live_monitor.py --no-gui   # 无头记录，Ctrl-C 出汇总 PNG
```

## 12. 常用排障

| 现象 | 排查 |
|---|---|
| `libprotobuf.so.9` 找不到 | 确认经 launch 启动（RPATH 已配置）；手动跑二进制需 `LD_LIBRARY_PATH` 指向 `install/aubo_e5_hardware/lib/aubo_e5_hardware/vendor` |
| SDK 读不到配置/日志 | ros2_control_node 与 dashboard 的 cwd 必须是各自 share 目录（launch 已设置） |
| 相机 `Open device fail -1014`，或改动"没生效" | 旧进程残留：设备与 SDK 通道均独占，且旧进程跑的是构建前二进制。`pgrep -af 'ros2 launch\|component_container\|extrinsics_publisher\|ros2 run'` 后按 PID 补杀（kill launch 会留孤儿节点），再重启 |
| goal 被拒 | 检查 6 个权威关节名、各点数组长度、NaN/Inf、严格递增的 `time_from_start`；passthrough 不支持 effort/path tolerance，携带时会拒绝 |
| 抢占后卡死 | 不应出现（蓝本已修复）；抓 `/aubo_io_controller/rib_status` 与 ros2_control_node 日志 |
| sim 里 set_io 返回 false | 预期行为，sim 不模拟板载 IO 写回 |
| sim/mock 启动后 percipio 刷屏报错 | 默认 `camera_enabled:=true` 会拉起相机驱动；未接相机时显式 `camera_enabled:=false` |
| 激活新外参后 TF 没变 | tf2 静态 TF 不接受同发布者覆盖：`/hand_eye_extrinsics_publisher/reload` 重读后旧订阅者仍拿旧值，须重启 extrinsics_publisher 进程（或整个标定 launch） |
| venv 跑节点/测试报 `ModuleNotFoundError: rcl_interfaces`（或 `ament_*`） | 先 `source /opt/ros/jazzy/setup.bash`：venv 是 `--system-site-packages`，但 ROS 的 Python 路径由 setup.bash 注入 PYTHONPATH，未 source 时 import 不到 |
| `import cv2` 报 `numpy.core.multiarray failed to import` / `import cv_bridge` 段错误 | venv 混入了 numpy 2.x：`aubo_py3.12/bin/pip install 'numpy==1.26.4'`；并确认未在 venv 装 opencv-python/scipy（会 shadow 系统 apt 版），锁定见根目录 `requirements.txt` |
| `colcon test` 里 aubo_scene_recon 的 test_pc_utils 被跳过 | 预期：colcon test 走系统 python3（无 open3d），conftest 自动 collect_ignore；全量跑用 venv 手动 pytest（见第 10 节） |
| `on_error summary: read_error_reason=1 (push never arrived)` 但此前运行正常 | 2026-07-29 起已修复：该报错原为 `RealtimeThreadSafeBox::try_get()` 撞锁（best-effort try_lock 返回 nullopt）被误判为无数据；现 `read()` 回退上一帧缓存，真断流仍由 200ms `state_timeout_ms` 超时（reason=2）兜底。汇总里的 `read_box_misses` 是撞锁计数，>0 属正常 |
| `read_error_reason=2 (push stale)` | SDK 推送链路真断了（>200ms 无新帧）：查网卡 offload/governor（每次开机必做，见 AGENTS.md §9）与 `docs/nic_driver_incident.md` |
# 桃子采摘任务中心

商业化联动、自动/维护所有权、Web 手动调试按钮、动态使能和类型化接口见
[`docs/peach_harvest_operations.md`](peach_harvest_operations.md)。统一业务栈入口：

```bash
ros2 launch peach_harvest_orchestrator harvest_system.launch.py
```
