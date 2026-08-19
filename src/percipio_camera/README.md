# percipio_camera

图漾工业相机驱动（厂商代码）。本工作区只项目化 launch 默认与内参。

## 输出（采摘订阅）

- `/camera/color/image_raw`
- `/camera/depth/image_raw`（须与彩图配准）
- `/camera/color/camera_info`

感知 `depth_scale_unit` 默认 `0.25`（Percipio）。深度单位错了，几何全错。

入口多为 `percipio_rgbd.launch.py` / `percipio_camera.launch.py`，由 bringup 在 `camera_enabled:=true` 时 include。不要 pip 装 opencv-python 替代系统 OpenCV。
