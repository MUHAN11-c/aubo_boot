# visual_pose_estimation_python

模板匹配 6D 估姿（旁路，非采摘）。IDL 走 `ivg_interfaces`，不进 `harvest_system` / lifecycle。

## 启动

相机与手眼 TF 由本区 bringup / `extrinsics_publisher` 提供。

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --packages-select ivg_interfaces ivg_utils visual_pose_estimation_python
source install/setup.bash

ros2 launch visual_pose_estimation_python visual_pose_estimation_python.launch.py
# 另终端 Web（默认 http://127.0.0.1:8088/）
ros2 launch visual_pose_estimation_python visual_pose_estimation_web.launch.py
```

订 `/camera/{color,depth}/image_raw`；软触发发 `std_msgs/String` 到 `/camera/soft_trigger`。`T_B_C` 查 `base_link` ← `camera_color_optical_frame`。

Web 的运动/IO/抓取 HTTP 返回 **501**。真机请走 harvest 8090 调试操作面或 `graspnet_ros2`（须授权）。

模板根：launch `template_root` → `VPE_TEMPLATE_ROOT` → app_config.json（仅 Web 侧）→ `visual_pose_estimation/templates`。

数学工具（四元数/旋转矩阵/RPY）统一来自 `ivg_utils.math`；rembg 抠图走进程内 `rembg_processor`（依赖 venv 内 rembg + onnxruntime，未装则该开关自动旁路）。运行时 `U2NET_HOME` 指向包内 `models/`，不写 `~/.u2net`。`u2net.onnx`（约 168MB）超过 Gitee/GitHub 单文件上限，**不随 git 分发**；本机已有则保留，clone 后执行 `models/fetch_u2net.sh`（或首次抠图时 rembg/pooch 下载）。

活文档：[docs/architecture.md](../../../docs/architecture.md) §3 旁路、[docs/io.md](../../../docs/io.md) §8。
