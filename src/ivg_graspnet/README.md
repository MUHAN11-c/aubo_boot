# ivg_graspnet

点云 6-DOF 抓取检测与 MoveIt 接近。**双后端可换**（`backend` 参数，2026-09-29
前沿化轮）：`graspnet_torch`（默认，vendored GraspNet-baseline 权重 + 纯
torch 点云算子，**不用 AnyGrasp SDK / 许可证**，不依赖 open3d、
graspnetAPI、CUDA 扩展编译）与 `contact_graspnet`（vendored
contact_graspnet_pytorch，Contact-GraspNet 移植）。

不进 `harvest_system.launch.py`、不进 lifecycle、不订 `peach_interfaces`。与采摘栈共用驱动层（相机 / TF / MoveIt），独立 launch。

## 分层

| 层 | 模块 | 职责 |
|----|------|------|
| 后端（模型可换） | `backends/`（注册表 + `GraspBackend` 协议） | 点云 → 原始 GraspList：`graspnet_torch`（前向+解码+Z-180 约定）、`contact_graspnet` |
| 模型无关后处理 | `postprocess.run_postprocess` | 开口裁剪 → 碰撞 → NMS → top-K；`GripperGeometry` 夹爪几何单源（无 rclpy） |
| 推理会话 | `inference_session.InferenceSession` | device 解析 / fp16 autocast / 显存摘要 |
| 数据契约 | `grasp_core.GraspList` | 17 维抓取向量 + 旋转列约定（approach/width/height） |
| 检测节点 | `graspnet_demo_points_node` | PointCloud2 → MarkerArray / PoseArray / TF；推理独立回调组 |
| 执行客户端 | `publish_grasps_client` | 选优 + TCP 补偿 + MoveIt 接近（须外部 `move_group`） |

- **权重与耦合超参随行**：`models/checkpoint-rs.tar` + 同名
  `checkpoint-rs.yaml` manifest（`num_view/num_angle/num_depth/...`）——
  换 checkpoint 只换两个文件，零改代码。contact_graspnet 权重随
  `contact_graspnet_lib/`（vendored，AMENT_IGNORE）携带。
- **行为等价门**：`test/test_backends.py::test_pipeline_matches_pre_refactor_golden`
  守卫重构前后同输入逐位一致（golden 基准 `test/data/golden_grasps.npz`）。
- **执行端约定**：`apply_grasp_z_flip`（默认 true）对齐后端
  `BackendInfo.approach_flip_z180`——graspnet_torch 需翻转；
  contact_graspnet 为 false（**待真机方向核验**，先在 RViz/Marker 确认
  approach 再授权执行）。MoveIt 超时自动 `cancel_goal_async`。
- **opt-in 使能门**：`require_enable`（默认 false 保持旧流程）为 true 时
  须先 `ros2 service call .../enable_grasp_motion std_srvs/srv/SetBool "{data: true}"`
  才会下发运动。
- 依赖：torch/torchvision/scipy 在 `aubo_py3.12` venv（requirements.txt
  钉版）；contact_graspnet 另需 `trimesh`（已钉）。vendor 两处最小补丁
  （pyrender 惰化、torch≥2.6 weights_only numpy 白名单）带
  `[ivg vendor patch]` 注释。

## 启动

```bash
# 检测（默认不拉相机；点云已由 bringup 提供）
ros2 launch ivg_graspnet graspnet_detect.launch.py
# 换后端：backend:=contact_graspnet（执行端同步 apply_grasp_z_flip:=false）

# 检测 + 接近（须 move_group 已起；授权真机运动后才执行）
ros2 launch ivg_graspnet graspnet_grasp.launch.py
```

采集默认待命：`ros2 service call /graspnet_capture_control std_srvs/srv/SetBool "{data: true}"` 开始一组。

点云 `/camera/depth_registered/points`（frame 兜底 `camera_depth_optical_frame`）。本 launch **不**起相机或手眼 TF。

规划组默认 `manipulator_e5`，末端 `tcp`。未授权不得真机运动。

## 许可

- 默认档 `graspnet_torch`：无第三方模型依赖（MIT 包体 + 随库权重）。
- `contact_graspnet`：NVIDIA 非商用系许可（vendor `License.pdf`），
  **内部研究/教学使用**；商用须移除该后端（默认档不受影响）。

评估基线：`campaign/20260929_ivg_refactor/report.md`（等价门 PASS；
contact_graspnet 在 vendor 真实场景 221 抓取）。
