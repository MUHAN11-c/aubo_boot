# ivg_sim

IVG 抓取验证 Gazebo Harmonic 桌面场景（**独立包**：不改 ivg_graspnet /
ivg_pose_estimation / peach_* 任何文件；检测节点经参数化启动 = 组合）。

桌面 + 10 YCB 对象杂乱摆位 + rgbd 相机（tilt 可调），点云经光学系中继喂
`ivg_graspnet` 检测节点，评分工具对 GT 对象位姿核验抓取覆盖率。
**2026-09-30 起含机械臂抓取执行闭环**（AUBO E5 + sim 两指夹爪 + gz_ros2_control
+ MoveIt：检测选目标 → IK → transit/descend/close/lift → gz 实测对象位移核验）。

## 运行链

```
config/table_layout.yaml（唯一事实源：桌/相机/摆位/tilt）
  └─ generate_table_world ──► worlds/ivg_table.sdf + ivg_table.manifest.yaml（GT）
       └─ gz sim -s -r（无头；GZ_SIM_RESOURCE_PATH 含 models/worlds）
            └─ parameter_bridge（点云）→ cloud_relay（帧名→camera_depth_optical_frame）
                 └─ /camera/depth_registered/points ──► graspnet_demo_points_node（零改动）
静态 TF base_link←camera_depth_optical_frame（layout.camera_optical_tf 单源推导）
  └─ grasp_poses_base ──► score_grasps（覆盖率/最近距离/垂直度，门=40%）

# 抓取执行栈（ivg_grasp_sim.launch.py，独立 ROS 域防同机串扰）：
gz sim（重力世界 ivg_table_grasp.sdf，对象可被搬动）
  + aubo_e5 spawn（urdf/aubo_e5_ivg_sim.xacro：厂商臂+基座+两指爪
    + gz_ros2_control；world link 由 arm_description.strip_world 剥除）
  + JTC/JSB/夹爪三控制器 + move_group（管线/限位/IK 复用 aubo_e5_moveit_config）
  └─ execute_grasp：检测选目标 → 残差闭环 → transit（两段高位）→
     密集路点下降 → 两段闭合 → 抬升 → gz 位移核验（Δz/Δxy + 判定）
```

## 用法（工作区根，Jazzy → 工作区 → aubo_py3.12 venv 依次 source）

```bash
# 恢复 YCB meshes（首次，~118MB，不入 git）
src/ivg_sim/models/fetch_ycb.sh

# ── A. 检测评分（零重力门世界，同 2026-09-29 基线）──
ros2 launch ivg_sim ivg_table_sim.launch.py
ros2 run ivg_sim score_grasps | tee runs/ivg_sim_score.txt

# ── B. 机械臂抓取执行（重力世界；gui:=true 带 Gazebo 窗口）──
export ROS_DOMAIN_ID=98   # 独立域：避开同机 peach 会话的 RViz/move_group 串扰
ros2 launch ivg_sim ivg_grasp_sim.launch.py gui:=true
# 另一终端（系统 python，无 venv）：
ros2 run ivg_sim execute_grasp --target potted_meat_can
```

要点：

- **运行链须激活 venv**：launch 以 `python3 -m ivg_graspnet.graspnet_node`
  起检测节点（install 入口 shebang 是系统 python、无 torch——历史现状）。
  `execute_grasp` 相反须用**系统 python**（moveit/ROS 消息面）。
- **换场景**：`ros2 run ivg_sim generate_table_world --jitter --seed 7`
  （同 seed 同布局）后重启 launch；GT manifest 随世界一起再生。
  `--gravity 9.81` 生成抓取执行变体 `ivg_table_grasp.sdf`（对象受重力；
  零重力门世界不被覆盖——检测确定性是评分门的前提）。
- **相机轴系**：gz 点云在传感器体系（spike 实测，`layout.GZ_OPTICAL_CONVENTION
  = 'body'`：x=视轴、y 左、z 上）；工作区裁剪按此写（`config/sim_graspnet.yaml`）。
- **桌沿假抓取**：桌面侧棱（|y|≈0.35）会被当 ledge 抓——workspace 已收紧到
  y±0.30 排除；换更大桌面时记得同步。
- **帧约定（执行栈）**：cell 根帧=`world`（桌面中心原点）；检测链
  `base_frame:=world`（CLI 覆盖，检测门世界仍用 yaml 默认 base_link）；
  机械臂 `base_link` 摆放单源 `arm_description.SPAWN_XYZ`（gz spawn 与
  静态 TF 同值）。

## 评分门与基线（2026-09-29，RTX 3090）

| 配置 | 抓取数 | 覆盖率 | 命中最近距离 | 垂直度 |
|------|--------|--------|--------------|--------|
| graspnet_torch，正视俯视（tilt 0），topK 20 | 20 | 30–50% | ≤0.11 m | 0.06–0.47 |
| graspnet_torch，tilt 35°，topK 40 | 34 | 40% | ≤0.11 m | **0.85–0.94** |

门（coverage ≥0.4）= 当前默认后端实测基线下界（tilt 35° 40%、正视俯视
30–50%）；倾斜视角显著改善 approach 垂直度。改进后端/场景后应上调门并在
此表追加基线。contact_graspnet 后端可经 `backend:=contact_graspnet` 对比
（其 0.6 置信度阈值在稀疏场景可能零提案，属模型行为）。

## 机械臂执行现状（2026-09-30）

闭环全链已通：spawn/控制器/MoveIt/检测/选抓/transit/下降/闭合/抬升/位移
核验（23 单测全绿；对象每次都被可靠触碰，判定器输出 Δz/Δxy）。**未竟**：
夹持保持未达成——对象被触碰后滑开（位移随修收敛 9.5→5.4cm：transit 扫桌
已修、IK 残差 7mm 已闭环、指轨已对齐、指速已降，最后一环疑在闭合接触
物理/掌体下探）。已修清单与挂账见 docs/testing-log 风格的提交说明。

## 许可

YCB 模型 CC BY 4.0（`models/LICENSE_YCB`，meshes 不入库、fetch 脚本恢复）；
包体 Apache-2.0。不进 harvest_system / lifecycle；沿用 IVG 的 CI 忽略待遇。

## 不做什么

- 不改任何其他包文件（检测节点组合式启动、参数走本包 `config/sim_graspnet.yaml`）；
  机械臂 ros2_control 声明在本包 xacro 内（驱动栈 `aubo_e5.ros2_control.xacro`
  只读不触碰）
- 臂的规划/IK 走 move_group（OMPL transit 因目标采样异常暂以两段 JTC 高位
  transit 替代，见 execute_grasp 注释）；物理闭环抓持调优为下一轮工作
- 不仿 ivg_pose_estimation（其输入是图像+软触发，phase-2 可选）
