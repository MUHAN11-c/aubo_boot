# IVG2.0 大文件清单

本文档记录因超出 Git 限制（>100MB）或 `.gitignore` 规则排除的大文件位置与用途，供新环境部署时参考喵~

> 最后更新: 2026-05-13

---

## 1. 模型权重文件

| 文件 | 大小 | 路径 | 用途 |
|------|------|------|------|
| `yolo26x.pt` | 114M | `aubo_ros2_ws/src/latte_imitation/` | YOLOv26 超大模型（拉花模仿学习） |
| `yolo26m.pt` | 43M | `aubo_ros2_ws/src/latte_imitation/` | YOLOv26 中模型（拉花模仿学习） |
| `yolo26n.pt` | 5.3M | `aubo_ros2_ws/src/latte_imitation/` | YOLOv26 小模型（拉花模仿学习） |
| `yolo26n.pt` | 5.3M | `aubo_ros2_ws/src/vision_perception/` | YOLOv26 小模型（感知检测） |
| `yolo26n-obb.pt` | 5.7M | `aubo_ros2_ws/src/vision_perception/` | YOLOv26 OBB 旋转框模型 |
| `best.pt / last.pt` | 43M×2 | `aubo_ros2_ws/src/latte_imitation/runs/` | YOLO 训练产物 |
| `checkpoint-rs.tar` | 12M | `aubo_ros2_ws/src/graspnet_ros2/graspnet-baseline/logs/log_kn/` | GraspNet 训练检查点 |

**获取方式**：
- YOLO 模型：运行 `yolo predict` 或从 Ultralytics 自动下载
- GraspNet checkpoint：需从原始训练环境导出

---

## 2. AUBO SDK 文件

### 2.1 已纳入 Git 的编译依赖 (.so)

以下 `.so` 文件已提交到 Git，新电脑 clone 后可直接编译喵~

| 文件 | 大小 | 路径 |
|------|------|------|
| `libauborobotcontroller.so*` (4 个版本) | 7.1M×4 | `lib/lib64/aubocontroller/` |
| `liblog4cplus-1.2.so*` (2 个版本) | ~1M | `lib/lib64/log4cplus/` |
| `libconfig*.so*` (4 个版本) | ~1M | `lib/lib64/config/` |
| `libprotobuf*.so*` (3 个版本) | 13M | `lib/lib64/protobuf/` |
| `libotgLib.a` | 767K | `lib/lib64/` |

### 2.2 未纳入 Git 的大文件

| 文件 | 大小 | 路径 | 用途 | 获取方式 |
|------|------|------|------|---------|
| `libauborobotcontroller.a` | 15M | `lib/lib64/aubocontroller/` | SDK 静态库 (不需要，.so 已足够) | 如有需要，从 AUBO SDK 包提取 |
| `libauborobotcontroller.so*` | 12M×5 | `lib/lib32/` | SDK 32-bit (不参与编译) | 32-bit 系统才需要 |
| `doc/aubo_sdk/` | 25M | `doc/aubo_sdk/` | SDK 参考库 (不参与编译) | AUBO 原厂 SDK 包 |
| `doc/SDK测试demo/` | ~80M | `doc/SDK测试demo/` | SDK 测试 demo 与依赖 | AUBO 原厂 SDK 包 |

> **注意**：`.bak` 备份文件和 `.a`/`.la` 静态库文件已从版本管理中排除（`.gitignore` 规则）。编译只需要 `.so` 动态库喵~

---

## 3. rosbag 录包数据

| 文件 | 大小 | 路径 | 用途 |
|------|------|------|------|
| `ivg_session_0.db3` | 1.5G | `aubo_ros2_ws/rosbags/ivg_session/` | 最新一次 session 录包 |

**获取方式**：由 `start_aubo_new_driver.sh` 步骤 16 自动录制，可通过 `SKIP_ROSBAG=1` 跳过

---

## 4. 视频与模板资源

| 文件 | 大小 | 路径 | 用途 |
|------|------|------|------|
| `d822ee8ef5...mp4` | 68M | `aubo_ros2_ws/src/coffee_latte_demo/resource/` | 咖啡拉花演示视频 |
| `video_新手咖啡拉花练习顺序_0.mp4` | 31M | `aubo_ros2_ws/src/vision_perception/resource/` | 感知训练视频 |
| `3211242785.7z` | 28M | `aubo_ros2_ws/src/visual_pose_estimation/templates/` | 视觉模板压缩包 |

**获取方式**：
- 视频：需从原始录制设备获取
- 模板：需从 VPE 模板库导出

---

## 5. 其它大文件

| 文件 | 大小 | 路径 | 用途 |
|------|------|------|------|
| `xarm.dae` | 14M | `aubo_ros2_ws/src/ros_arm_tutorials/xarm_description/urdf/` | xArm DAE 网格模型（教学用） |
| `browse.vc.db` | 989M | `.vscode/` | VS Code IntelliSense 缓存（IDE 自动生成） |
| `_ext.cpython-310-*.so` | 29M | `graspnet_ros2/graspnet-baseline/pointnet2/` | PointNet++ CUDA 扩展（pip install -e . 编译产物） |
| `knn_pytorch.cpython-310-*.so` | 14M | `graspnet_ros2/graspnet-baseline/knn/` | KNN CUDA 扩展（pip install -e . 编译产物） |
| `chessboard_img.bmp` | 19M×3 | `aubo_ros2_ws/legacy/hand_eye_calibration_tool/data/` | 标定板 BMP 图像（遗留） |
| `export.log` | 5.2M | `aubo_ros2_ws/legacy/urdf4.2812/` | URDF 导出日志（遗留） |

**获取方式**：
- xarm.dae：ROS 教程包自带，`apt install ros-humble-xarm-description` 或从源码获取
- CUDA 扩展：在新环境运行 `pip3 install -e .` 重新编译（`graspnet-baseline/` 下 4 个子包）
- VS Code 缓存：IDE 自动生成，无需手动管理

---

## 部署检查清单

新环境部署时，确认以下大文件已就位：

- [ ] AUBO SDK `libauborobotcontroller.so.1.3.1` 及其依赖 `.so` 放在 `lib/lib64/{aubocontroller,log4cplus,config,protobuf}/`
- [ ] YOLO 模型权重放在工作空间根目录（`yolo26n.pt`, `yolo26n-obb.pt`）
- [ ] GraspNet 模型 checkpoint 放在 `graspnet-baseline/logs/log_kn/`
- [ ] 视频/模板资源按需从备份恢复
- [ ] GraspNet CUDA 扩展重新编译（`pip3 install -e .`）
