# ivg_pose_estimation

模板匹配 6D 估姿（旁路，非采摘）。IDL 走 `ivg_interfaces`，不进 `harvest_system` / lifecycle。

## 分段流水线（2026-09-29 前沿化轮）

```
Preprocessor(深度带连通域) → FeatureExtractor
  → Segmenter（分割段，可换）      → Matcher（匹配段，可换）     → PoseEstimator.estimate_pose
     depth_band（默认，轻量兜底）     geometric（默认，距离/IoU）
     rembg_u2net（u2net 显著性）     dinov2_template（DINOv2 嵌入
     mobile_sam（MobileSAM 点提示）    检索，FoundPose/CNOS 范式）
```

- 段选型在 `config/pose_estimation.yaml`（`segmenter.backend` / `matcher.backend`），
  Web 调试面板的 `use_rembg` 开关运行时覆盖分割档。
- `dinov2_template` 首次使用对每工件模板库离线建嵌入索引
  （`<模板目录>/.dinov2_index.npz`，模板更新自动重建；DINOv2 权重经
  torch.hub 首次下载缓存至 `~/.cache/torch/hub`）。
- 单位单源：`camera.depth_scale`（Percipio 0.00025）、
  `calibration.translation_unit`（mm|m|auto）；深度换算只在边界发生。
- `EstimatePose` 请求/响应带 `std_msgs/Header`（响应 stamp=图像采集时刻、
  frame=`base_link`；TF 同 stamp 查询，失败回退 latest+WARN）。

## 启动

相机与手眼 TF 由本区 bringup / `extrinsics_publisher` 提供。

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --packages-select ivg_interfaces ivg_pose_estimation
source install/setup.bash

ros2 launch ivg_pose_estimation ivg_pose_estimation.launch.py
# 另终端 Web（默认 http://127.0.0.1:8089/，与手眼标定网关 8088 错开）
ros2 launch ivg_pose_estimation ivg_pose_estimation_web.launch.py
```

订 `/camera/{color,depth}/image_raw`；软触发发 `std_msgs/String` 到 `/camera/soft_trigger`。`T_B_C` 查 `base_link` ← `camera_color_optical_frame`。

Web 的运动/IO/抓取 HTTP 返回 **501**。真机请走 harvest 8090 调试操作面或 `ivg_graspnet`（须授权）。

**Web 写端点认证**（2026-09-29 起）：`/exit`、`/api/save_template_pose`、
`/api/standardize_template`、`/api/debug/update_params`、
`/api/debug/save_thresholds` 需认证——同源 UI 自动携带 `vpe_auth`
SameSite=Strict cookie（零改动）；curl 带 `-H "X-Auth-Token: <token>"`
（token 打印在启动日志；环境变量 `VPE_WEB_TOKEN` 可固定）。跨域 CORS
已收紧为同源。

模板根：launch `template_root` → `VPE_TEMPLATE_ROOT` → app_config.json（仅 Web 侧）→ `ivg_pose_estimation/templates`。

数学工具（四元数/旋转矩阵/RPY）统一来自 `scipy.spatial.transform.Rotation`；rembg 抠图与 MobileSAM 分割走 `pipeline/segmenters/`（依赖 venv 内 rembg/onnxruntime/ultralytics，未装则对应档自动旁路/降级）。`u2net.onnx`（约 168MB）与 `mobile_sam.pt`（约 39MB）**不随 git 分发**：执行 `models/fetch_u2net.sh` / `models/fetch_mobile_sam.sh`（后者优先拷贝仓内 `peach_harvester` 副本，零网络；u2net 也可首用时 rembg/pooch 下载）。运行时 `U2NET_HOME` 指向包内 `models/`。

许可：MobileSAM/SAM2 Apache-2.0；ultralytics AGPL-3.0（库级内部研究使用）；DINOv2 Apache-2.0；rembg MIT（u2net 权重随上游）。商用替换：默认档（depth_band+geometric）无第三方模型依赖。

活文档：[docs/architecture.md](../../../docs/architecture.md) §3 旁路、[docs/io.md](../../../docs/io.md) §8。评估基线：`campaign/20260929_ivg_refactor/report.md`。
