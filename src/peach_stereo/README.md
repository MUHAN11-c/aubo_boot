# peach_stereo

PS800-E1 **主机侧单图案立体深度**相机节点：`percipio_camera` 的替代前端，
输出**话题同构**的 RGB-D，感知/重建零改动接入。依据与数据见
`docs/testing-log.md` 09-16/09-17 各条目（根因 → 解锁 → 单图案验证 → 实时演示 → 需求评估 → 感知 A/B）。

## PS800-E1 规格档案（本机实测，2026-09-16/17）

| 项 | 值 | 来源 |
|----|-----|------|
| 型号 / SN | Percipio PS800-E1 / 207000152740 | 设备直读 |
| 深度原理 | 散斑结构光双目（每帧 18 幅图案，image number=18 且 (18+1)/2×match5=47.5 顶满算力约束 <48） | SDK 直读 + 官方参数文档 |
| 固件 / SDK | tycam R3.6.49 / camport4 4.2.10，legacy GigE2.0 协议 | 日志 |
| 官方深度帧率 | **0.8 fps @ 全部分辨率**（1280×960/640×480/320×240 同值）；渲染延迟 1781 ms | en.percipio.xyz 产品页 |
| 实测深度帧率（设备端） | 2.43 fps（411.5 ms/帧，设备日志 `got one frame` 间隔证实；官方口径=IR 采集率÷图案数） | 台架 |
| 官方深度量程/精度 | 400–800 mm；Z 0.51 mm@500 mm、XY 1.14 mm@500 mm | 产品页 |
| 彩色 | yuyv 可选 640×480–2560×1920；**单流 15.00 fps**；不设档位默认 2560×1920（会拖死帧组到 2.5 gps） | 台架 |
| 双目 IR | 1280×960 mono8；独立模式采集率随曝光 14.8–25 fps；**L+R 同帧硬件同步** | 台架 |
| 立体几何 | 基线 **62.2 mm**，f≈1098 px @1280 宽；右相机外参=相对左 IR（头文件注释明示） | 设备标定直读 |
| 本链路深度 | 主机 SGBM：**13.5–13.7 fps**；时域噪声 1.49 mm@0.8 m（k=1）/0.96 mm（k=5），设备端对照 1.13 mm；与设备深度同目标互证差 **5 mm** | 台架 |
| 本链路近距下限 | SGBM 理论 ≈0.27 m（f·B/numDisp），低于设备额定 0.4 m；近距光学质量未标定 | 计算 |
| 激光投射器 | auto ctrl=1/power=50 默认；**独立 IR 模式须手动解锁**（auto=0+power=100，否则输出非光信号底噪）；laser 设置跨连接自动复位、**曝光值跨连接残留** | 台架 |
| 帧组时间戳 | TIME_SYNC=HOST 后彩色/L/R 同微秒（纪元 µs），RGB-D 对齐天然成立 | 台架 |
| 深度口径 | uint16 × 0.25 mm（与感知 `depth_scale_unit: 0.25` 对齐，零配置） | 设计 |
| 已知限制 | jpeg 插件编不了 mono16（`/compressed` 空载荷），深度压缩通道用 `compressedDepth`（12 字节容器头）或 `zstd`；raw RELIABLE 大图本机 FastDDS 投递坍塌 | 台架 |
| 无效参数（本机实测勿再调） | image number / SGPM 相位数 / 深度分辨率 / frame_rate 请求 / 深度模式 IR 曝光——均不改深度采集帧率 | 09-16 全实测 |

## 工作原理

```
解锁激光(关自动控制+拉功率) → 彩色640x480 + 双目IR 1280x960 同帧组(~13.7 组/s)
→ stereoRectify 校正 → SGBM(半分辨率 ~13ms) → 深度(左IR系, mm)
→ TYMapDepthImageToColorCoordinate 配准到彩色几何
→ 与 percipio 同构：color/image_raw + depth/image_raw(uint16×0.25mm)
   + {color,depth}/camera_info + depth_registered/points
```

- **帧组硬件同步**：彩色/左 IR/右 IR 同时间戳（纪元微秒，TIME_SYNC=HOST），RGB-D 时间对齐天然满足感知 `sync_slop 0.05s`。
- **深度口径与 percipio 一致**：uint16 × `depth_scale_unit 0.25`(mm)，感知端默认参数直接可用。
- **质量-帧率旋钮**：`avg_k`（1→~13.7fps/1.49mm；4→~3.4fps；实测 k=5≈0.96mm@0.8m，优于设备 18 图案单帧 1.13mm）。
- 相比设备端深度（2.43fps）：**~5.6 倍帧率**，代价是单图案噪声 +30%（对 3–8mm 尺度的感知管线裕度充足，见评估条目）。

## 话题（namespace=camera，与 percipio_camera 对齐）

| 话题 | 类型 | 说明 |
|------|------|------|
| `color/image_raw` | bgr8 | 彩色，与深度同帧组时间戳 |
| `depth/image_raw` | 16UC1 | **已配准到彩色几何**，单位 0.25mm |
| `color/camera_info` | CameraInfo | 按彩色流分辨率折算的 K |
| `depth/camera_info` | CameraInfo | 与彩色同 K（配准后），frame=`camera_depth_optical_frame` |
| `depth_registered/points` | PointCloud2 | 配准彩色点云（与 percipio `color_point_cloud_enable` 同名） |

静态 TF：`camera_link → {color,depth}_frame → 各 optical`（与 percipio 同名链；depth 帧 identity 等价）。

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
- 大消息投递：raw RELIABLE 大图在本机 FastDDS 下有坍塌现象（testing-log 09-16）。Jazzy `image_transport` 无法按话题关插件；整栈 catch-all 会订 jpeg/compressedDepth，16UC1 与 bgr8 交叉编码每帧 ERROR。本节点只发 raw Image（感知订 raw）。
- 默认不进 harvest_system（`camera_frontend` 默认 percipio）。整栈切 stereo：`camera_enabled:=true camera_frontend:=stereo`。
- 深度工作范围：设备端额定 0.4–0.8m；本链路 SGBM 理论下限 ~0.27m（近距光学质量未标定）。
