# ivg_sim

IVG 抓取验证 Gazebo Harmonic 桌面场景（**独立包**：不改 ivg_graspnet /
ivg_pose_estimation / peach_* 任何文件；检测节点经参数化启动 = 组合）。

桌面 + 10 YCB 对象杂乱摆位 + rgbd 相机（tilt 可调），点云经光学系中继喂
`ivg_graspnet` 检测节点，评分工具对 GT 对象位姿核验抓取覆盖率。

## 运行链

```
config/table_layout.yaml（唯一事实源：桌/相机/摆位/tilt）
  └─ generate_table_world ──► worlds/ivg_table.sdf + ivg_table.manifest.yaml（GT）
       └─ gz sim -s -r（无头；GZ_SIM_RESOURCE_PATH 含 models/worlds）
            └─ parameter_bridge（点云）→ cloud_relay（帧名→camera_depth_optical_frame）
                 └─ /camera/depth_registered/points ──► graspnet_demo_points_node（零改动）
静态 TF base_link←camera_depth_optical_frame（layout.camera_optical_tf 单源推导）
  └─ grasp_poses_base ──► score_grasps（覆盖率/最近距离/垂直度，门=40%）
```

## 用法（工作区根，Jazzy → 工作区 → aubo_py3.12 venv 依次 source）

```bash
# 恢复 YCB meshes（首次，~118MB，不入 git）
src/ivg_sim/models/fetch_ycb.sh

# 起整栈（无头 gz + 桥 + 中继 + TF + 检测节点[venv python3]）
ros2 launch ivg_sim ivg_table_sim.launch.py
#   gui:=true 带 GUI；backend:=contact_graspnet 换后端；detect:=false 只起场景

# 另一终端：触发一次采集并评分（stdout 报告，可 tee 留档）
ros2 run ivg_sim score_grasps | tee runs/ivg_sim_score.txt
```

要点：

- **运行链须激活 venv**：launch 以 `python3 -m ivg_graspnet.graspnet_node`
  起检测节点（install 入口 shebang 是系统 python、无 torch——历史现状）。
- **换场景**：`ros2 run ivg_sim generate_table_world --jitter --seed 7`
  （同 seed 同布局）后重启 launch；GT manifest 随世界一起再生。
- **相机轴系**：gz 点云在传感器体系（spike 实测，`layout.GZ_OPTICAL_CONVENTION
  = 'body'`：x=视轴、y 左、z 上）；工作区裁剪按此写（`config/sim_graspnet.yaml`）。
- **桌沿假抓取**：桌面侧棱（|y|≈0.35）会被当 ledge 抓——workspace 已收紧到
  y±0.30 排除；换更大桌面时记得同步。
- **零重力世界**：对象冻结在摆位，GT 与 manifest 精确一致（检测基准要确定性；
  动态交互验证属后续轮）。

## 评分门与基线（2026-09-29，RTX 3090）

| 配置 | 抓取数 | 覆盖率 | 命中最近距离 | 垂直度 |
|------|--------|--------|--------------|--------|
| graspnet_torch，正视俯视（tilt 0），topK 20 | 20 | 30–50% | ≤0.11 m | 0.06–0.47 |
| graspnet_torch，tilt 35°，topK 40 | 34 | 40% | ≤0.11 m | **0.85–0.94** |

门（coverage ≥0.4）= 当前默认后端实测基线下界（tilt 35° 40%、正视俯视
30–50%）；倾斜视角显著改善 approach 垂直度。改进后端/场景后应上调门并在
此表追加基线。contact_graspnet 后端可经 `backend:=contact_graspnet` 对比
（其 0.6 置信度阈值在稀疏场景可能零提案，属模型行为）。

## 许可

YCB 模型 CC BY 4.0（`models/LICENSE_YCB`，meshes 不入库、fetch 脚本恢复）；
包体 Apache-2.0。不进 harvest_system / lifecycle；沿用 IVG 的 CI 忽略待遇。

## 不做什么

- 不改任何其他包文件（检测节点组合式启动、参数走本包 `config/sim_graspnet.yaml`）
- 不做机械臂仿真执行闭环（aubo_description 无 gazebo 标签；gz_ros2_control
  化属独立后续轮）
- 不仿 ivg_pose_estimation（其输入是图像+软触发，phase-2 可选）
