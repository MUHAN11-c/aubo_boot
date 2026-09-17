# peach_stereo

PS800-E1 **主机侧单图案立体深度**相机节点：`percipio_camera` 的替代前端，
输出**话题同构**的 RGB-D，感知/重建零改动接入。依据与数据见
`docs/testing-log.md` 09-16 六个条目（根因 → 解锁 → 单图案验证 → 实时演示 → 需求评估）。

## 工作原理

```
解锁激光(关自动控制+拉功率) → 彩色640x480 + 双目IR 1280x960 同帧组(~13.7 组/s)
→ stereoRectify 校正 → SGBM(半分辨率 ~13ms) → 深度(左IR系, mm)
→ TYMapDepthImageToColorCoordinate 配准到彩色几何
→ /camera/color/image_raw + /camera/depth/image_raw(uint16×0.25mm) + camera_info
```

- **帧组硬件同步**：彩色/左 IR/右 IR 同时间戳（纪元微秒，TIME_SYNC=HOST），RGB-D 时间对齐天然满足感知 `sync_slop 0.05s`。
- **深度口径与 percipio 一致**：uint16 × `depth_scale_unit 0.25`(mm)，感知端默认参数直接可用。
- **质量-帧率旋钮**：`avg_k`（1→~13.7fps/1.49mm；4→~3.4fps；实测 k=5≈0.96mm@0.8m，优于设备 18 图案单帧 1.13mm）。
- 相比设备端深度（2.43fps）：**~5.6 倍帧率**，代价是单图案噪声 +30%（对 3–8mm 尺度的感知管线裕度充足，见评估条目）。

## 话题（namespace=camera，与 percipio_camera 对齐）

| 话题 | 类型 | 说明 |
|------|------|------|
| `color/image_raw` (+`/compressed`) | bgr8 | 彩色，与深度同帧组时间戳 |
| `depth/image_raw` (+`/compressed`) | mono16 | **已配准到彩色几何**，单位 0.25mm |
| `color/camera_info` | CameraInfo | 按彩色流分辨率折算的 K |
| `depth/debug_color` | bgr8 | 2Hz JET 伪彩调试流（可关） |

静态 TF：`camera_link → camera_color_frame → camera_color_optical_frame`（与 percipio 一致）。

## 使用

```bash
colcon build --packages-select peach_stereo
# 与 percipio_camera 互斥（相机独占）：先停 percipio 相机栈
ros2 launch peach_stereo stereo_camera.launch.py
ros2 topic hz /camera/depth/image_raw   # 预期 ~13.7 Hz
```

参数见 `config/stereo_camera.yaml`（nav2 式：部署值即事实源，键名冻结）。

## 边界与注意

- **激光满功率持续点亮**（`laser_power: 100`）：长时间运行的热管理未验证；停栈自动复位（auto=1/50）。台架连续观察为宜，田间长时间挂机前先做温升确认。
- 依赖 `percipio_camera` 源码树 vendored 的 camport4 SDK（头文件未随包安装，CMake 直接引用源码路径）；运行时 `.so` 由 `install/percipio_camera/lib` 解析。
- 大消息投递：raw RELIABLE 大图在本机 FastDDS 下有坍塌现象（testing-log 09-16 条目）；本节点经 image_transport 发布，自带 `/compressed` 通道，感知侧改订 compressed 可解。
- 未进 harvest_system 默认栈（需真机验证后接入）；当前作为独立相机前端并行存在。
- 深度工作范围：设备端额定 0.4–0.8m；本链路 SGBM 理论下限 ~0.27m（近距光学质量未标定）。
